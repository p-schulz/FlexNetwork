#include "Mesh/FlexRailMeshBuilder.h"

#include "DynamicMesh/DynamicMesh3.h"
#include "DynamicMeshEditor.h"
#include "Operations/MeshBoolean.h"
#include "RoadTypeProfile.h"

using namespace UE::Geometry;

namespace
{
	void AppendTriangleChecked(FDynamicMesh3& Mesh, int32 A, int32 B, int32 C)
	{
		if (A != B && B != C && A != C)
		{
			Mesh.AppendTriangle(A, B, C);
		}
	}

	TArray<FFlexCurveFrame> ExtendEndFrames(TConstArrayView<FFlexCurveFrame> Source, float Overlap)
	{
		TArray<FFlexCurveFrame> Result;
		Result.Reserve(Source.Num());
		for (const FFlexCurveFrame& Frame : Source)
		{
			Result.Add(Frame);
		}
		if (Result.Num() >= 2 && Overlap > 0.f)
		{
			Result[0].Position -= Result[0].Tangent.GetSafeNormal() * Overlap;
			Result.Last().Position += Result.Last().Tangent.GetSafeNormal() * Overlap;
		}
		return Result;
	}

	FDynamicMesh3 BuildSweptSolid(TConstArrayView<FFlexCurveFrame> SourceFrames, float CenterOffset,
		float BottomHalfWidth, float TopHalfWidth, float BottomHeight, float TopHeight, float EndOverlap)
	{
		FDynamicMesh3 Mesh;
		const TArray<FFlexCurveFrame> Frames = ExtendEndFrames(SourceFrames, EndOverlap);
		if (Frames.Num() < 2 || TopHeight - BottomHeight <= KINDA_SMALL_NUMBER)
		{
			return Mesh;
		}

		TArray<TStaticArray<int32, 4>> Rings;
		Rings.SetNum(Frames.Num());
		for (int32 FrameIndex = 0; FrameIndex < Frames.Num(); ++FrameIndex)
		{
			const FFlexCurveFrame& Frame = Frames[FrameIndex];
			const FVector Center = Frame.Position + Frame.Right * CenterOffset;
			const FVector Points[4] = {
				Center - Frame.Right * BottomHalfWidth + Frame.Up * BottomHeight,
				Center + Frame.Right * BottomHalfWidth + Frame.Up * BottomHeight,
				Center + Frame.Right * TopHalfWidth + Frame.Up * TopHeight,
				Center - Frame.Right * TopHalfWidth + Frame.Up * TopHeight
			};
			for (int32 PointIndex = 0; PointIndex < 4; ++PointIndex)
			{
				Rings[FrameIndex][PointIndex] = Mesh.AppendVertex(FVector3d(Points[PointIndex]));
			}
		}

		for (int32 FrameIndex = 0; FrameIndex + 1 < Frames.Num(); ++FrameIndex)
		{
			for (int32 Side = 0; Side < 4; ++Side)
			{
				const int32 NextSide = (Side + 1) % 4;
				AppendTriangleChecked(Mesh, Rings[FrameIndex][Side], Rings[FrameIndex][NextSide], Rings[FrameIndex + 1][NextSide]);
				AppendTriangleChecked(Mesh, Rings[FrameIndex][Side], Rings[FrameIndex + 1][NextSide], Rings[FrameIndex + 1][Side]);
			}
		}

		const int32 StartCenter = Mesh.AppendVertex((Mesh.GetVertex(Rings[0][0]) + Mesh.GetVertex(Rings[0][1])
			+ Mesh.GetVertex(Rings[0][2]) + Mesh.GetVertex(Rings[0][3])) * 0.25);
		const int32 EndCenter = Mesh.AppendVertex((Mesh.GetVertex(Rings.Last()[0]) + Mesh.GetVertex(Rings.Last()[1])
			+ Mesh.GetVertex(Rings.Last()[2]) + Mesh.GetVertex(Rings.Last()[3])) * 0.25);
		for (int32 Side = 0; Side < 4; ++Side)
		{
			const int32 NextSide = (Side + 1) % 4;
			AppendTriangleChecked(Mesh, StartCenter, Rings[0][NextSide], Rings[0][Side]);
			AppendTriangleChecked(Mesh, EndCenter, Rings.Last()[Side], Rings.Last()[NextSide]);
		}
		return Mesh;
	}

	void AppendDisconnected(FDynamicMesh3& Target, const FDynamicMesh3& Source)
	{
		FDynamicMeshEditor Editor(&Target);
		FMeshIndexMappings Mappings;
		Editor.AppendMesh(&Source, Mappings);
	}

	bool CombineSolid(FDynamicMesh3& Aggregate, FDynamicMesh3&& Solid)
	{
		if (Solid.TriangleCount() == 0)
		{
			return true;
		}
		if (Aggregate.TriangleCount() == 0)
		{
			Aggregate = MoveTemp(Solid);
			return true;
		}
		if (!Aggregate.GetBounds().Intersects(Solid.GetBounds()))
		{
			AppendDisconnected(Aggregate, Solid);
			return true;
		}

		FDynamicMesh3 UnionMesh;
		FMeshBoolean Boolean(&Aggregate, &Solid, &UnionMesh, FMeshBoolean::EBooleanOp::Union);
		Boolean.SnapTolerance = 0.01;
		Boolean.bWeldSharedEdges = true;
		Boolean.bSimplifyAlongNewEdges = false;
		if (!Boolean.Compute() || UnionMesh.TriangleCount() == 0)
		{
			// A malformed source span must not make otherwise valid rails disappear.
			AppendDisconnected(Aggregate, Solid);
			return false;
		}
		Aggregate = MoveTemp(UnionMesh);
		return true;
	}

	void ConvertToSection(const FDynamicMesh3& Mesh, UMaterialInterface* Material, FFlexMeshSectionData& OutSection)
	{
		OutSection = FFlexMeshSectionData();
		OutSection.Material = Material;
		OutSection.bEnableCollision = true;
		TArray<int32> VertexMap;
		VertexMap.Init(INDEX_NONE, Mesh.MaxVertexID());
		for (const int32 VertexId : Mesh.VertexIndicesItr())
		{
			VertexMap[VertexId] = OutSection.Vertices.Add(FVector(Mesh.GetVertex(VertexId)));
			OutSection.Normals.Add(FVector::ZeroVector);
			const FVector Position = OutSection.Vertices.Last();
			OutSection.UV0.Add(FVector2D(Position.X, Position.Y) * 0.01f);
			OutSection.Tangents.Add(FProcMeshTangent(FVector::ForwardVector, false));
			OutSection.VertexColors.Add(FColor::White);
		}

		for (const int32 TriangleId : Mesh.TriangleIndicesItr())
		{
			const FIndex3i Triangle = Mesh.GetTriangle(TriangleId);
			const int32 A = VertexMap[Triangle.A];
			const int32 B = VertexMap[Triangle.B];
			const int32 C = VertexMap[Triangle.C];
			if (A == INDEX_NONE || B == INDEX_NONE || C == INDEX_NONE)
			{
				continue;
			}
			OutSection.Triangles.Add(A);
			OutSection.Triangles.Add(B);
			OutSection.Triangles.Add(C);
			const FVector FaceNormal = FVector::CrossProduct(OutSection.Vertices[B] - OutSection.Vertices[A],
				OutSection.Vertices[C] - OutSection.Vertices[A]);
			OutSection.Normals[A] += FaceNormal;
			OutSection.Normals[B] += FaceNormal;
			OutSection.Normals[C] += FaceNormal;
		}

		for (int32 VertexIndex = 0; VertexIndex < OutSection.Vertices.Num(); ++VertexIndex)
		{
			const FVector Normal = OutSection.Normals[VertexIndex].GetSafeNormal(UE_SMALL_NUMBER, FVector::UpVector);
			OutSection.Normals[VertexIndex] = Normal;
			FVector Tangent = FVector::CrossProduct(FVector::UpVector, Normal).GetSafeNormal();
			if (Tangent.IsNearlyZero())
			{
				Tangent = FVector::ForwardVector;
			}
			OutSection.Tangents[VertexIndex] = FProcMeshTangent(Tangent, false);
		}
	}
}

bool FFlexRailMeshBuilder::BuildRailMesh(TConstArrayView<FFlexRailSweepInput> Sweeps,
	const URoadTypeProfile* Profile, FFlexMeshSectionData& OutSection)
{
	OutSection = FFlexMeshSectionData();
	if (!Profile || !Profile->bIsRailProfile || Sweeps.IsEmpty())
	{
		return false;
	}

	FDynamicMesh3 OuterRails;
	FDynamicMesh3 GrooveCutters;
	const float BaseHalfWidth = FMath::Max(Profile->RailWidth * 0.5f, 0.5f);
	const float TopHalfWidth = FMath::Clamp(Profile->RailTopWidth * 0.5f, 0.5f, BaseHalfWidth);
	const float RailHeight = FMath::Max(Profile->RailHeight, 0.5f);
	const float HalfRailCenterSpacing = (Profile->RailGauge + Profile->RailWidth) * 0.5f;
	const float GrooveHalfWidth = FMath::Clamp(Profile->RailGrooveWidth * 0.5f, 0.25f, TopHalfWidth - 0.1f);
	const float GrooveInwardOffset = FMath::Clamp(Profile->RailGrooveInwardOffset, 0.f,
		FMath::Max(0.f, TopHalfWidth - GrooveHalfWidth - 0.1f));
	const float GrooveBottom = FMath::Clamp(RailHeight - Profile->RailGrooveDepth, 0.1f, RailHeight - 0.1f);
	const float CutterTop = RailHeight + FMath::Max(Profile->RailBooleanOverlap, 0.1f);

	for (const FFlexRailSweepInput& Sweep : Sweeps)
	{
		if (Sweep.Frames.Num() < 2)
		{
			continue;
		}
		for (const FRoadLaneDescriptor& Track : Profile->Lanes)
		{
			if (!Track.IsRail())
			{
				continue;
			}
			const float TrackCenter = Profile->GetLaneLateralOffset(Track);
			for (const float RailSide : { -1.f, 1.f })
			{
				const float RailCenter = TrackCenter + RailSide * HalfRailCenterSpacing;
				CombineSolid(OuterRails, BuildSweptSolid(Sweep.Frames, RailCenter, BaseHalfWidth, TopHalfWidth,
					0.f, RailHeight, Profile->RailBooleanOverlap));
				if (Profile->bUseGroovedRailProfile)
				{
					// Shift toward the track center. This leaves the right rail's wider shoulder on
					// its right and the left rail's wider shoulder on its left, as on tram rails.
					const float CutterCenter = RailCenter - RailSide * GrooveInwardOffset;
					CombineSolid(GrooveCutters, BuildSweptSolid(Sweep.Frames, CutterCenter, GrooveHalfWidth,
						GrooveHalfWidth, GrooveBottom, CutterTop, Profile->RailBooleanOverlap * 2.f));
				}
			}
		}
	}

	if (OuterRails.TriangleCount() == 0)
	{
		return false;
	}

	FDynamicMesh3 FinalMesh = MoveTemp(OuterRails);
	if (Profile->bUseGroovedRailProfile && GrooveCutters.TriangleCount() > 0)
	{
		FDynamicMesh3 DifferenceMesh;
		FMeshBoolean Difference(&FinalMesh, &GrooveCutters, &DifferenceMesh, FMeshBoolean::EBooleanOp::Difference);
		Difference.SnapTolerance = 0.01;
		Difference.bWeldSharedEdges = true;
		Difference.bSimplifyAlongNewEdges = false;
		if (Difference.Compute() && DifferenceMesh.TriangleCount() > 0)
		{
			FinalMesh = MoveTemp(DifferenceMesh);
		}
	}

	ConvertToSection(FinalMesh, Profile->RoadMaterial.Get(), OutSection);
	return !OutSection.IsEmpty();
}
