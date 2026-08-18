#include "Elements/PCGFlexNetworkNodes.h"

#include "PCGComponent.h"
#include "PCGContext.h"
#include "PCGParamData.h"
#include "Data/PCGDynamicMeshData.h"
#include "Metadata/PCGMetadata.h"
#include "DynamicMesh/DynamicMesh3.h"
#include "DynamicMesh/DynamicMeshAttributeSet.h"
#include "DynamicMesh/DynamicMeshOverlay.h"
#include "FlexNetworkSubsystem.h"
#include "FlexNetworkSettings.h"
#include "FlexNetworkSegmentActor.h"
#include "Mesh/FlexMeshSectionData.h"
#include "Mesh/FlexRoadMeshBuilder.h"
#include "RoadTypeProfile.h"

using UE::Geometry::FDynamicMesh3;

namespace
{
	const FName MeshPin = TEXT("Meshes");
	const FName InfoPin = TEXT("Info");
	const FName SurfacesPin = TEXT("Surfaces");
	const FName CrosswalksPin = TEXT("Crosswalks");
	const FName CornersPin = TEXT("SidewalkCorners");
	const FName IslandsPin = TEXT("CornerIslands");

	struct FFlexPCGSource
	{
		UFlexNetworkSubsystem* Network = nullptr;
		AFlexNetworkSegmentActor* SegmentActor = nullptr;
	};

	FFlexPCGSource ResolveSource(FPCGContext* Context)
	{
		FFlexPCGSource Result;
		UObject* SourceObject = Context && Context->ExecutionSource.IsValid() ? Context->ExecutionSource.GetObject() : nullptr;
		Result.SegmentActor = Cast<AFlexNetworkSegmentActor>(SourceObject);
		if (!Result.SegmentActor)
		{
			if (const UActorComponent* Component = Cast<UActorComponent>(SourceObject))
			{
				Result.SegmentActor = Cast<AFlexNetworkSegmentActor>(Component->GetOwner());
			}
		}
		UWorld* World = SourceObject ? SourceObject->GetWorld() : nullptr;
		Result.Network = World ? World->GetSubsystem<UFlexNetworkSubsystem>() : nullptr;
		return Result;
	}

	bool ToDynamicMesh(const FFlexMeshSectionData& Section, FDynamicMesh3& OutMesh)
	{
		if (Section.IsEmpty()) return false;
		// UPCGDynamicMeshData's material array defines the available slots, while this per-triangle
		// attribute actually selects a slot. Explicitly writing slot 0 makes the profile material
		// survive DynamicMesh PCG operations/spawning instead of falling back to a default material.
		OutMesh.EnableAttributes();
		OutMesh.Attributes()->EnableMaterialID();
		UE::Geometry::FDynamicMeshMaterialAttribute* MaterialIds = OutMesh.Attributes()->GetMaterialID();
		UE::Geometry::FDynamicMeshNormalOverlay* Normals = OutMesh.Attributes()->PrimaryNormals();
		UE::Geometry::FDynamicMeshUVOverlay* UVs = OutMesh.Attributes()->PrimaryUV();
		for (const FVector& Vertex : Section.Vertices) OutMesh.AppendVertex(Vertex);
		for (int32 Index = 0; Index + 2 < Section.Triangles.Num(); Index += 3)
		{
			const int32 A = Section.Triangles[Index], B = Section.Triangles[Index + 1], C = Section.Triangles[Index + 2];
			if (OutMesh.IsVertex(A) && OutMesh.IsVertex(B) && OutMesh.IsVertex(C) && A != B && B != C && A != C)
			{
				const int32 TriangleId = OutMesh.AppendTriangle(A, B, C);
				if (TriangleId >= 0)
				{
					MaterialIds->SetValue(TriangleId, 0);
					if (Section.Normals.IsValidIndex(A) && Section.Normals.IsValidIndex(B) && Section.Normals.IsValidIndex(C))
					{
						Normals->SetTriangle(TriangleId, UE::Geometry::FIndex3i(
							Normals->AppendElement(FVector3f(Section.Normals[A])),
							Normals->AppendElement(FVector3f(Section.Normals[B])),
							Normals->AppendElement(FVector3f(Section.Normals[C]))));
					}
					if (Section.UV0.IsValidIndex(A) && Section.UV0.IsValidIndex(B) && Section.UV0.IsValidIndex(C))
					{
						UVs->SetTriangle(TriangleId, UE::Geometry::FIndex3i(
							UVs->AppendElement(FVector2f(Section.UV0[A])),
							UVs->AppendElement(FVector2f(Section.UV0[B])),
							UVs->AppendElement(FVector2f(Section.UV0[C]))));
					}
				}
			}
		}
		return OutMesh.TriangleCount() > 0;
	}

	void AddMesh(FPCGContext* Context, const FFlexMeshSectionData& Section, FName Pin, const FString& IdTag)
	{
		FDynamicMesh3 Mesh;
		if (!ToDynamicMesh(Section, Mesh)) return;
		UPCGDynamicMeshData* Data = FPCGContext::NewObject_AnyThread<UPCGDynamicMeshData>(Context);
		Data->Initialize(MoveTemp(Mesh), TArray<UMaterialInterface*>{ Section.Material.Get() });
		FPCGTaggedData& Tagged = Context->OutputData.TaggedData.Emplace_GetRef();
		Tagged.Data = Data;
		Tagged.Pin = Pin;
		Tagged.Tags.Add(IdTag);
	}

	UPCGParamData* MakeInfo(FPCGContext* Context, FPCGMetadataAttribute<FString>*& Id, FPCGMetadataAttribute<double>*& Length)
	{
		UPCGParamData* Data = FPCGContext::NewObject_AnyThread<UPCGParamData>(Context);
		Id = Data->MutableMetadata()->CreateAttribute<FString>(TEXT("FlexId"), FString(), false, false);
		Length = Data->MutableMetadata()->CreateAttribute<double>(TEXT("LengthCm"), 0.0, false, false);
		return Data;
	}

	void AddInfo(FPCGContext* Context, UPCGParamData* Data)
	{
		FPCGTaggedData& Tagged = Context->OutputData.TaggedData.Emplace_GetRef();
		Tagged.Data = Data;
		Tagged.Pin = InfoPin;
	}

	void AppendChamferedCurbSpan(FFlexMeshSectionData& Section, const FVector& StartEdge, const FVector& EndEdge,
		const FVector& StartOutward, const FVector& EndOutward, const FVector& StartUp, const FVector& EndUp,
		float Width, float Height, float Chamfer)
	{
		const FVector Tangent = (EndEdge - StartEdge).GetSafeNormal();
		if (Tangent.IsNearlyZero() || Width <= KINDA_SMALL_NUMBER || Height <= KINDA_SMALL_NUMBER) return;
		const FVector O0 = StartOutward.GetSafeNormal(UE_SMALL_NUMBER, FVector::RightVector);
		const FVector O1 = EndOutward.GetSafeNormal(UE_SMALL_NUMBER, O0);
		const FVector U0 = StartUp.GetSafeNormal(UE_SMALL_NUMBER, FVector::UpVector);
		const FVector U1 = EndUp.GetSafeNormal(UE_SMALL_NUMBER, U0);
		const float C = FMath::Clamp(Chamfer, 0.f, FMath::Min(Width * 0.5f, Height));

		auto MakeRing = [Width, Height, C](const FVector& Edge, const FVector& Out, const FVector& Up)
		{
			TArray<FVector> Ring;
			Ring.Reserve(6);
			Ring.Add(Edge);
			Ring.Add(Edge + Out * Width);
			Ring.Add(Edge + Out * Width + Up * (Height - C));
			Ring.Add(Edge + Out * (Width - C) + Up * Height);
			Ring.Add(Edge + Out * C + Up * Height);
			Ring.Add(Edge + Up * (Height - C));
			return Ring;
		};
		const TArray<FVector> A = MakeRing(StartEdge, O0, U0);
		const TArray<FVector> B = MakeRing(EndEdge, O1, U1);
		FVector CenterA = FVector::ZeroVector, CenterB = FVector::ZeroVector;
		for (const FVector& P : A) CenterA += P;
		for (const FVector& P : B) CenterB += P;
		CenterA /= A.Num(); CenterB /= B.Num();

		for (int32 i = 0; i < A.Num(); ++i)
		{
			const int32 j = (i + 1) % A.Num();
			const FVector DesiredNormal = (((A[i] + A[j]) * 0.5f) - CenterA).GetSafeNormal();
			const FVector CandidateNormal = FVector::CrossProduct(B[i] - A[i], B[j] - A[i]);
			if (FVector::DotProduct(CandidateNormal, DesiredNormal) >= 0.f)
			{
				Section.AppendQuad(A[i], B[i], B[j], A[j], DesiredNormal, Tangent,
					FVector2D(0, 0), FVector2D(1, 0), FVector2D(1, 1), FVector2D(0, 1));
			}
			else
			{
				Section.AppendQuad(A[i], A[j], B[j], B[i], DesiredNormal, Tangent,
					FVector2D(0, 0), FVector2D(0, 1), FVector2D(1, 1), FVector2D(1, 0));
			}
		}

		// Each sampled span is a complete scaled/chamfered cube. Closed ends make isolated curb
		// runs valid too; neighboring spans merely share a coplanar internal cap.
		for (int32 i = 0; i < A.Num(); ++i)
		{
			const int32 j = (i + 1) % A.Num();
			Section.AppendTriangle(CenterA, A[j], A[i], -Tangent, O0, FVector2D(.5f, .5f), FVector2D(1, 1), FVector2D(0, 1));
			Section.AppendTriangle(CenterB, B[i], B[j], Tangent, O1, FVector2D(.5f, .5f), FVector2D(0, 1), FVector2D(1, 1));
		}
	}

	void AppendRoadCurbs(FFlexMeshSectionData& Section, const TArray<FFlexCurveFrame>& Frames, float RoadHalfWidth, float Width, float Height, float Chamfer)
	{
		for (int32 i = 0; i + 1 < Frames.Num(); ++i)
		{
			const FFlexCurveFrame& A = Frames[i];
			const FFlexCurveFrame& B = Frames[i + 1];
			AppendChamferedCurbSpan(Section, A.Position - A.Right * RoadHalfWidth, B.Position - B.Right * RoadHalfWidth,
				-A.Right, -B.Right, A.Up, B.Up, Width, Height, Chamfer);
			AppendChamferedCurbSpan(Section, A.Position + A.Right * RoadHalfWidth, B.Position + B.Right * RoadHalfWidth,
				A.Right, B.Right, A.Up, B.Up, Width, Height, Chamfer);
		}
	}

	bool GenerateSegments(FPCGContext* Context, bool bSidewalks)
	{
		const FFlexPCGSource Source = ResolveSource(Context);
		UFlexNetworkSubsystem* Network = Source.Network;
		// This helper is a free function rather than an IPCGElement member, so the regular
		// PCGE_LOG macro cannot call IPCGElement::ShouldLog(). Use the explicit-context form.
		if (!Network) { PCGE_LOG_C(Error, GraphAndLog, Context, NSLOCTEXT("FlexNetworkPCG", "NoWorld", "No FlexNetwork world subsystem is available.")); return true; }
		FPCGMetadataAttribute<FString>* IdAttr = nullptr; FPCGMetadataAttribute<double>* LengthAttr = nullptr;
		UPCGParamData* Info = MakeInfo(Context, IdAttr, LengthAttr);
		for (const TPair<FFlexSegmentId, FFlexRoadSegment>& Pair : Network->GetAllSegments())
		{
			if (Source.SegmentActor && Pair.Key != Source.SegmentActor->SegmentId) continue;
			FFlexSegmentMeshResult Result;
			if (!Network->BuildSegmentMeshResult(Pair.Key, Result)) continue;
			const FString Id = Pair.Key.ToString();
			AddMesh(Context, bSidewalks ? Result.Sidewalks : Result.Roadway, MeshPin, FString::Printf(TEXT("FlexSegmentId:%s"), *Id));
			const int64 Key = Info->FindOrAddMetadataKey(FName(*Id));
			IdAttr->SetValue(Key, Id); LengthAttr->SetValue(Key, Pair.Value.GetLength());
		}
		AddInfo(Context, Info);
		return true;
	}
}

TArray<FPCGPinProperties> UPCGFlexRoadMeshesSettings::OutputPinProperties() const { return { FPCGPinProperties(MeshPin, EPCGDataType::DynamicMesh), FPCGPinProperties(InfoPin, EPCGDataType::Param) }; }
TArray<FPCGPinProperties> UPCGFlexSidewalkMeshesSettings::OutputPinProperties() const { return { FPCGPinProperties(MeshPin, EPCGDataType::DynamicMesh), FPCGPinProperties(InfoPin, EPCGDataType::Param) }; }
TArray<FPCGPinProperties> UPCGFlexIntersectionMeshesSettings::OutputPinProperties() const { return { FPCGPinProperties(SurfacesPin, EPCGDataType::DynamicMesh), FPCGPinProperties(CrosswalksPin, EPCGDataType::DynamicMesh), FPCGPinProperties(CornersPin, EPCGDataType::DynamicMesh), FPCGPinProperties(IslandsPin, EPCGDataType::DynamicMesh), FPCGPinProperties(InfoPin, EPCGDataType::Param) }; }
TArray<FPCGPinProperties> UPCGFlexCurbMeshesSettings::OutputPinProperties() const { return { FPCGPinProperties(MeshPin, EPCGDataType::DynamicMesh), FPCGPinProperties(InfoPin, EPCGDataType::Param) }; }
FPCGElementPtr UPCGFlexRoadMeshesSettings::CreateElement() const { return MakeShared<FPCGFlexRoadMeshesElement>(); }
FPCGElementPtr UPCGFlexSidewalkMeshesSettings::CreateElement() const { return MakeShared<FPCGFlexSidewalkMeshesElement>(); }
FPCGElementPtr UPCGFlexIntersectionMeshesSettings::CreateElement() const { return MakeShared<FPCGFlexIntersectionMeshesElement>(); }
FPCGElementPtr UPCGFlexCurbMeshesSettings::CreateElement() const { return MakeShared<FPCGFlexCurbMeshesElement>(); }
bool FPCGFlexRoadMeshesElement::ExecuteInternal(FPCGContext* Context) const { return GenerateSegments(Context, false); }
bool FPCGFlexSidewalkMeshesElement::ExecuteInternal(FPCGContext* Context) const { return GenerateSegments(Context, true); }

bool FPCGFlexIntersectionMeshesElement::ExecuteInternal(FPCGContext* Context) const
{
	const FFlexPCGSource Source = ResolveSource(Context);
	UFlexNetworkSubsystem* Network = Source.Network;
	if (!Network) { PCGE_LOG(Error, GraphAndLog, NSLOCTEXT("FlexNetworkPCG", "NoWorldIntersection", "No FlexNetwork world subsystem is available.")); return true; }
	FPCGMetadataAttribute<FString>* IdAttr = nullptr; FPCGMetadataAttribute<double>* UnusedLength = nullptr;
	UPCGParamData* Info = MakeInfo(Context, IdAttr, UnusedLength);
	FPCGMetadataAttribute<int32>* ApproachCount = Info->MutableMetadata()->CreateAttribute<int32>(TEXT("ApproachCount"), 0, false, false);
	FPCGMetadataAttribute<int32>* SharpTurnCount = Info->MutableMetadata()->CreateAttribute<int32>(TEXT("SharpTurnCount"), 0, false, false);
	FPCGMetadataAttribute<double>* MaxTurnAngle = Info->MutableMetadata()->CreateAttribute<double>(TEXT("MaxTurnAngleDegrees"), 0.0, false, false);
	for (const TPair<FFlexNodeId, FFlexRoadNode>& Pair : Network->GetAllNodes())
	{
		if (!Pair.Value.IsJunction()) continue;
		if (Source.SegmentActor && !Pair.Value.ConnectedSegments.Contains(Source.SegmentActor->SegmentId)) continue;
		if (Source.SegmentActor)
		{
			// Every segment actor uses the same configured graph. Give each junction deterministic
			// ownership to the lowest connected ID so three/four coincident copies are not emitted.
			FFlexSegmentId Owner = Source.SegmentActor->SegmentId;
			for (FFlexSegmentId Candidate : Pair.Value.ConnectedSegments)
			{
				if (Candidate.Index < Owner.Index || (Candidate.Index == Owner.Index && Candidate.Generation < Owner.Generation))
				{
					Owner = Candidate;
				}
			}
			if (Owner != Source.SegmentActor->SegmentId) continue;
		}
		FFlexJunctionMeshResult Result;
		if (!Network->BuildJunctionMeshResult(Pair.Key, Result)) continue;
		const FString Id = Pair.Key.ToString();
		const FString Tag = FString::Printf(TEXT("FlexNodeId:%s"), *Id);
		AddMesh(Context, Result.Surface, SurfacesPin, Tag);
		AddMesh(Context, Result.Crosswalks, CrosswalksPin, Tag);
		AddMesh(Context, Result.SidewalkCorners, CornersPin, Tag);
		AddMesh(Context, Result.CornerIslands, IslandsPin, Tag);
		const int64 Key = Info->FindOrAddMetadataKey(FName(*Id));
		int32 NumSharpTurns = 0;
		double MaximumTurnAngle = 0.0;
		if (const FFlexJunctionData* Junction = Network->GetJunctionData(Pair.Key))
		{
			for (const FFlexLaneConnector& Connector : Junction->LaneConnectors)
			{
				NumSharpTurns += Connector.bSharpTurn ? 1 : 0;
				MaximumTurnAngle = FMath::Max(MaximumTurnAngle, static_cast<double>(Connector.TurnAngleDegrees));
			}
		}
		IdAttr->SetValue(Key, Id);
		UnusedLength->SetValue(Key, 0.0);
		ApproachCount->SetValue(Key, Pair.Value.ConnectedSegments.Num());
		SharpTurnCount->SetValue(Key, NumSharpTurns);
		MaxTurnAngle->SetValue(Key, MaximumTurnAngle);
	}
	AddInfo(Context, Info);
	return true;
}

bool FPCGFlexCurbMeshesElement::ExecuteInternal(FPCGContext* Context) const
{
	const UPCGFlexCurbMeshesSettings* Settings = Context->GetInputSettings<UPCGFlexCurbMeshesSettings>();
	check(Settings);
	const FFlexPCGSource Source = ResolveSource(Context);
	UFlexNetworkSubsystem* Network = Source.Network;
	if (!Network)
	{
		PCGE_LOG(Error, GraphAndLog, NSLOCTEXT("FlexNetworkPCG", "NoWorldCurb", "No FlexNetwork world subsystem is available."));
		return true;
	}

	FPCGMetadataAttribute<FString>* IdAttr = nullptr;
	FPCGMetadataAttribute<double>* LengthAttr = nullptr;
	UPCGParamData* Info = MakeInfo(Context, IdAttr, LengthAttr);
	const float SampleStep = GetDefault<UFlexNetworkSettings>()->ArcLengthSampleStep;

	if (Settings->bGenerateRoadCurbs)
	{
		for (const TPair<FFlexSegmentId, FFlexRoadSegment>& Pair : Network->GetAllSegments())
		{
			if (Source.SegmentActor && Pair.Key != Source.SegmentActor->SegmentId) continue;
			const FFlexRoadSegment& Segment = Pair.Value;
			if (!Segment.Profile || Segment.Profile->CurbHeight <= KINDA_SMALL_NUMBER || !Segment.ArcLengthTable.IsValid()) continue;
			float TrimStart = 0.f, TrimEnd = 0.f;
			if (!Network->GetSegmentTrimRange(Pair.Key, TrimStart, TrimEnd)) continue;
			const FFlexRoadNode* StartNode = Network->GetNode(Segment.StartNodeId);
			const TArray<FFlexCurveFrame> Frames = FFlexRoadMeshBuilder::BuildFramesForRange(Segment.Curve, Segment.ArcLengthTable,
				StartNode ? StartNode->UpVector : FVector::UpVector, SampleStep, TrimStart, TrimEnd);
			FFlexMeshSectionData Curbs;
			Curbs.Material = Segment.Profile->CurbMaterial ? Segment.Profile->CurbMaterial : Segment.Profile->SidewalkMaterial;
			AppendRoadCurbs(Curbs, Frames, Segment.Profile->GetRoadwayHalfWidth(), Settings->CurbWidth, Segment.Profile->CurbHeight, Settings->ChamferSize);
			const FString Id = Pair.Key.ToString();
			AddMesh(Context, Curbs, MeshPin, FString::Printf(TEXT("FlexCurbSegmentId:%s"), *Id));
			const int64 Key = Info->FindOrAddMetadataKey(FName(*(FString(TEXT("Curb_")) + Id)));
			IdAttr->SetValue(Key, Id);
			LengthAttr->SetValue(Key, FMath::Max(0.f, TrimEnd - TrimStart) * 2.0);
		}
	}

	if (Settings->bGenerateJunctionCurbs)
	{
		for (const TPair<FFlexNodeId, FFlexRoadNode>& Pair : Network->GetAllNodes())
		{
			const FFlexJunctionData* Junction = Network->GetJunctionData(Pair.Key);
			if (!Junction || Junction->PolygonBoundary.Num() < 3) continue;
			if (Source.SegmentActor)
			{
				if (!Pair.Value.ConnectedSegments.Contains(Source.SegmentActor->SegmentId)) continue;
				FFlexSegmentId Owner = Source.SegmentActor->SegmentId;
				for (FFlexSegmentId Candidate : Pair.Value.ConnectedSegments)
				{
					if (Candidate.Index < Owner.Index || (Candidate.Index == Owner.Index && Candidate.Generation < Owner.Generation)) Owner = Candidate;
				}
				if (Owner != Source.SegmentActor->SegmentId) continue;
			}

			const URoadTypeProfile* Profile = nullptr;
			for (FFlexSegmentId SegmentId : Pair.Value.ConnectedSegments)
			{
				if (const FFlexRoadSegment* Segment = Network->GetSegment(SegmentId); Segment && Segment->Profile)
				{
					Profile = Segment->Profile;
					break;
				}
			}
			if (!Profile || Profile->CurbHeight <= KINDA_SMALL_NUMBER) continue;
			FFlexMeshSectionData Curbs;
			Curbs.Material = Profile->CurbMaterial ? Profile->CurbMaterial : Profile->SidewalkMaterial;
			FVector Centroid = FVector::ZeroVector;
			for (const FVector& P : Junction->PolygonBoundary) Centroid += P;
			Centroid /= Junction->PolygonBoundary.Num();
			double TotalLength = 0.0;
			for (int32 i = 0; i < Junction->PolygonBoundary.Num(); ++i)
			{
				const bool bIsCurb = !Junction->PolygonEdgeIsCurbLine.IsValidIndex(i) || Junction->PolygonEdgeIsCurbLine[i];
				if (!bIsCurb) continue;
				const FVector& A = Junction->PolygonBoundary[i];
				const FVector& B = Junction->PolygonBoundary[(i + 1) % Junction->PolygonBoundary.Num()];
				FVector Outward = FVector::VectorPlaneProject((A + B) * 0.5f - Centroid, Pair.Value.UpVector).GetSafeNormal();
				if (Outward.IsNearlyZero()) Outward = FVector::CrossProduct((B - A).GetSafeNormal(), Pair.Value.UpVector).GetSafeNormal();
				AppendChamferedCurbSpan(Curbs, A, B, Outward, Outward, Pair.Value.UpVector, Pair.Value.UpVector,
					Settings->CurbWidth, Profile->CurbHeight, Settings->ChamferSize);
				TotalLength += FVector::Distance(A, B);
			}
			const FString Id = Pair.Key.ToString();
			AddMesh(Context, Curbs, MeshPin, FString::Printf(TEXT("FlexCurbNodeId:%s"), *Id));
			const int64 Key = Info->FindOrAddMetadataKey(FName(*(FString(TEXT("JunctionCurb_")) + Id)));
			IdAttr->SetValue(Key, Id);
			LengthAttr->SetValue(Key, TotalLength);
		}
	}

	AddInfo(Context, Info);
	return true;
}
