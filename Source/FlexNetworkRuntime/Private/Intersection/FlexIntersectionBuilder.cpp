#include "Intersection/FlexIntersectionBuilder.h"
#include "Math/FlexBezierMath.h"
#include "Math/FlexGeometry2D.h"
#include "Math/FlexTriangulation.h"
#include "Mesh/FlexRoadMeshBuilder.h"
#include "Algo/Sort.h"
#include "Algo/Accumulate.h"

namespace
{
	/** Outward-from-node unit tangent (3D) for one approach. */
	FVector GetOutwardTangent(const FFlexJunctionApproachInput& Approach)
	{
		const FVector Tangent = Approach.bNodeIsSegmentEnd
			? FFlexBezierMath::EvaluateDerivative(Approach.Curve, 1.f)
			: FFlexBezierMath::EvaluateDerivative(Approach.Curve, 0.f);
		const FVector Normalized = Tangent.GetSafeNormal(UE_SMALL_NUMBER, FVector::ForwardVector);
		return Approach.bNodeIsSegmentEnd ? -Normalized : Normalized;
	}

	struct FLocalBasis
	{
		FVector AxisX;
		FVector AxisY;
		FVector Up;

		FVector2D To2D(const FVector& WorldOffset) const { return FVector2D(FVector::DotProduct(WorldOffset, AxisX), FVector::DotProduct(WorldOffset, AxisY)); }
		FVector To3D(const FVector& Origin, const FVector2D& Local) const { return Origin + AxisX * Local.X + AxisY * Local.Y; }
	};

	FLocalBasis MakeLocalBasis(const FVector& Up)
	{
		FLocalBasis Basis;
		Basis.Up = Up.GetSafeNormal(UE_SMALL_NUMBER, FVector::UpVector);
		FVector AxisX = FVector::VectorPlaneProject(FVector::ForwardVector, Basis.Up);
		if (AxisX.IsNearlyZero())
		{
			AxisX = FVector::VectorPlaneProject(FVector::RightVector, Basis.Up);
		}
		Basis.AxisX = AxisX.GetSafeNormal(UE_SMALL_NUMBER, FVector::ForwardVector);
		Basis.AxisY = FVector::CrossProduct(Basis.Up, Basis.AxisX).GetSafeNormal(UE_SMALL_NUMBER, FVector::RightVector);
		return Basis;
	}

	struct FSortedApproach
	{
		int32 ApproachIndex = INDEX_NONE;
		FVector2D Direction2D = FVector2D::ZeroVector;
		float Angle = 0.f;
		float OuterExtent = 0.f;
	};

	bool IsDrivableLane(const FRoadLaneDescriptor& Lane)
	{
		return Lane.IsDrivable() && Lane.Direction != EFlexLaneDirection::None;
	}

	// Physical direction of travel for a lane entering (Incoming) or leaving (Outgoing) the
	// junction at this approach is determined purely by which end of the segment touches the
	// node -- Forward/Backward on the lane just picks which of those two roles it plays.
	void GetLaneRoles(const FRoadLaneDescriptor& Lane, bool bNodeIsSegmentEnd, bool& bOutIncoming, bool& bOutOutgoing)
	{
		bOutIncoming = false;
		bOutOutgoing = false;
		if (!IsDrivableLane(Lane))
		{
			return;
		}

		switch (Lane.Direction)
		{
		case EFlexLaneDirection::Forward:
			bOutIncoming = bNodeIsSegmentEnd;
			bOutOutgoing = !bNodeIsSegmentEnd;
			break;
		case EFlexLaneDirection::Backward:
			bOutIncoming = !bNodeIsSegmentEnd;
			bOutOutgoing = bNodeIsSegmentEnd;
			break;
		case EFlexLaneDirection::Bidirectional:
			bOutIncoming = true;
			bOutOutgoing = true;
			break;
		default:
			break;
		}
	}
}

bool FFlexIntersectionBuilder::NeedsJunction(const TArray<FFlexJunctionApproachInput>& Approaches, float StraightThroughAngleToleranceDegrees, float WidthMismatchTolerance)
{
	if (Approaches.Num() >= 3)
	{
		return true;
	}
	if (Approaches.Num() < 2)
	{
		return false;
	}

	const FVector DirA = GetOutwardTangent(Approaches[0]);
	const FVector DirB = GetOutwardTangent(Approaches[1]);
	const float CosAngle = FMath::Clamp(FVector::DotProduct(DirA, DirB), -1.f, 1.f);
	const float AngleDeg = FMath::RadiansToDegrees(FMath::Acos(CosAngle));
	// A smooth pass-through bend has the two approaches pointing roughly opposite (~180 degrees apart).
	const float DeviationFromStraight = FMath::Abs(180.f - AngleDeg);
	if (DeviationFromStraight > StraightThroughAngleToleranceDegrees)
	{
		return true;
	}

	const float WidthA = Approaches[0].Profile ? Approaches[0].Profile->GetRoadwayHalfWidth() : 0.f;
	const float WidthB = Approaches[1].Profile ? Approaches[1].Profile->GetRoadwayHalfWidth() : 0.f;
	if (FMath::Abs(WidthA - WidthB) > WidthMismatchTolerance)
	{
		return true;
	}

	return false;
}

FFlexJunctionData FFlexIntersectionBuilder::BuildJunction(const FVector& NodePosition, const FVector& NodeUp, const TArray<FFlexJunctionApproachInput>& Approaches, float DefaultFilletRadius, float CrosswalkWidth, float CrosswalkMinClearance, int32 FilletArcSegments)
{
	FFlexJunctionData Result;
	const int32 N = Approaches.Num();
	if (N < 2)
	{
		return Result;
	}

	const FLocalBasis Basis = MakeLocalBasis(NodeUp);

	TArray<FSortedApproach> Sorted;
	Sorted.Reserve(N);
	for (int32 i = 0; i < N; ++i)
	{
		FSortedApproach Entry;
		Entry.ApproachIndex = i;
		Entry.Direction2D = Basis.To2D(GetOutwardTangent(Approaches[i])).GetSafeNormal();
		Entry.Angle = FMath::Atan2(Entry.Direction2D.Y, Entry.Direction2D.X);
		Entry.OuterExtent = Approaches[i].Profile ? Approaches[i].Profile->GetOuterExtent() : 0.f;
		Sorted.Add(Entry);
	}
	Algo::SortBy(Sorted, [](const FSortedApproach& A) { return A.Angle; });

	TArray<FVector2D> Polygon2D;
	// Filled in parallel with Polygon2D: which approach's forward-edge each boundary point "belongs to" isn't tracked per point -- corners are computed per approach-pair below and trims derived directly from those, so we don't need a point->approach map here.
	TMap<int32, float> TrimDistance2DByApproachIndex; // ApproachIndex -> max forward distance among its two flanking corners

	auto UpdateTrim = [&TrimDistance2DByApproachIndex](int32 ApproachIndex, const FVector2D& CornerPoint, const FVector2D& Origin, const FVector2D& Dir)
	{
		const float Forward = FVector2D::DotProduct(CornerPoint - Origin, Dir);
		float& Existing = TrimDistance2DByApproachIndex.FindOrAdd(ApproachIndex, 0.f);
		Existing = FMath::Max(Existing, Forward);
	};

	for (int32 i = 0; i < N; ++i)
	{
		const FSortedApproach& A = Sorted[i];
		const FSortedApproach& B = Sorted[(i + 1) % N];

		// A's boundary edge on the side facing B ("right" of A's outward direction) and B's
		// boundary edge on the side facing A ("left" of B's outward direction).
		const FVector2D RightPerpA(A.Direction2D.Y, -A.Direction2D.X);
		const FVector2D LeftPerpB(-B.Direction2D.Y, B.Direction2D.X);

		const FVector2D OriginA = RightPerpA * A.OuterExtent;
		const FVector2D OriginB = LeftPerpB * B.OuterExtent;

		FVector2D Corner;
		bool bUsedSharpCorner = false;
		if (FlexGeometry2D::LineLineIntersection(OriginA, A.Direction2D, OriginB, B.Direction2D, Corner))
		{
			const float ForwardA = FVector2D::DotProduct(Corner - OriginA, A.Direction2D);
			const float ForwardB = FVector2D::DotProduct(Corner - OriginB, B.Direction2D);
			// A clean corner must lie "ahead of" the node along both boundary rays; if it lies
			// behind either one (very acute angle between the two approaches) the sharp
			// intersection isn't usable and we fillet instead.
			if (ForwardA >= 0.f && ForwardB >= 0.f)
			{
				bUsedSharpCorner = true;
			}
		}

		if (bUsedSharpCorner)
		{
			Polygon2D.Add(Corner);
			UpdateTrim(A.ApproachIndex, Corner, FVector2D::ZeroVector, A.Direction2D);
			UpdateTrim(B.ApproachIndex, Corner, FVector2D::ZeroVector, B.Direction2D);
		}
		else
		{
			const float Radius = DefaultFilletRadius;
			TArray<FVector2D> Arc;
			if (FlexGeometry2D::ComputeFilletArc(FVector2D::ZeroVector, A.Direction2D, B.Direction2D, Radius, Arc, FilletArcSegments))
			{
				Polygon2D.Append(Arc);
				for (const FVector2D& P : Arc)
				{
					UpdateTrim(A.ApproachIndex, P, FVector2D::ZeroVector, A.Direction2D);
					UpdateTrim(B.ApproachIndex, P, FVector2D::ZeroVector, B.Direction2D);
				}
			}
			else
			{
				// Directions are (nearly) collinear -- nothing sensible to round; fall back to the
				// simple offset corner so the polygon still closes.
				const FVector2D Fallback = (OriginA + OriginB) * 0.5f;
				Polygon2D.Add(Fallback);
				UpdateTrim(A.ApproachIndex, Fallback, FVector2D::ZeroVector, A.Direction2D);
				UpdateTrim(B.ApproachIndex, Fallback, FVector2D::ZeroVector, B.Direction2D);
			}
		}
	}

	if (Polygon2D.Num() < 3)
	{
		return Result;
	}

	// Convert to world space and triangulate.
	Result.PolygonBoundary.Reserve(Polygon2D.Num());
	for (const FVector2D& P : Polygon2D)
	{
		Result.PolygonBoundary.Add(Basis.To3D(NodePosition, P));
	}
	FlexTriangulation::EarClipTriangulate(Polygon2D, Result.PolygonTriangleIndices);

	// Resolve per-approach trim arc-lengths (in the approach's own segment parameterization) and
	// place crosswalks/curb-cuts.
	for (const FSortedApproach& Entry : Sorted)
	{
		const FFlexJunctionApproachInput& Approach = Approaches[Entry.ApproachIndex];
		const float SegmentLength = Approach.ArcLengthTable.GetTotalLength();
		const float* FoundTrim2D = TrimDistance2DByApproachIndex.Find(Entry.ApproachIndex);
		float TrimDistance2D = FoundTrim2D ? *FoundTrim2D : Entry.OuterExtent;
		TrimDistance2D = FMath::Max(TrimDistance2D, Entry.OuterExtent * 0.5f);
		TrimDistance2D = FMath::Clamp(TrimDistance2D, 0.f, SegmentLength * 0.45f);

		const float TrimArcLength = Approach.bNodeIsSegmentEnd ? (SegmentLength - TrimDistance2D) : TrimDistance2D;
		Result.TrimArcLengthBySegment.Add(Approach.SegmentId, TrimArcLength);

		if (Approach.Profile && Approach.Profile->SidewalkWidth > KINDA_SMALL_NUMBER)
		{
			const float ClearedDistance2D = FMath::Max(TrimDistance2D, CrosswalkMinClearance);
			const FFlexCurveFrame Frame = FFlexRoadMeshBuilder::SampleFrameAtArcLength(Approach.Curve, Approach.ArcLengthTable, Approach.bNodeIsSegmentEnd ? SegmentLength - ClearedDistance2D : ClearedDistance2D, NodeUp);

			FFlexCrosswalkPlacement Crosswalk;
			Crosswalk.Center = Frame.Position;
			Crosswalk.CrossingDirection = Frame.Right;
			Crosswalk.Width = CrosswalkWidth;
			Crosswalk.Length = Approach.Profile->GetRoadwayHalfWidth() * 2.f;
			Result.Crosswalks.Add(Crosswalk);
		}
	}

	// Lane connectors: every legal incoming-lane -> outgoing-lane pair across distinct approaches.
	struct FResolvedLane
	{
		int32 ApproachIndex;
		int32 LaneIndex;
		FVector Position;
		FVector Tangent; // direction of travel at this connector endpoint
		float SpeedLimit;
	};
	TArray<FResolvedLane> IncomingLanes;
	TArray<FResolvedLane> OutgoingLanes;

	for (const FSortedApproach& Entry : Sorted)
	{
		const FFlexJunctionApproachInput& Approach = Approaches[Entry.ApproachIndex];
		if (!Approach.Profile)
		{
			continue;
		}
		const float TrimArcLength = Result.TrimArcLengthBySegment.FindChecked(Approach.SegmentId);
		const FFlexCurveFrame Frame = FFlexRoadMeshBuilder::SampleFrameAtArcLength(Approach.Curve, Approach.ArcLengthTable, TrimArcLength, NodeUp);

		const FVector InwardDir = Approach.bNodeIsSegmentEnd ? Frame.Tangent : -Frame.Tangent;
		const FVector OutwardDir = -InwardDir;

		const TArray<FRoadLaneDescriptor>& Lanes = Approach.Profile->Lanes;
		for (int32 LaneIndex = 0; LaneIndex < Lanes.Num(); ++LaneIndex)
		{
			bool bIncoming = false, bOutgoing = false;
			GetLaneRoles(Lanes[LaneIndex], Approach.bNodeIsSegmentEnd, bIncoming, bOutgoing);
			if (!bIncoming && !bOutgoing)
			{
				continue;
			}

			const FVector LanePos = Frame.Position + Frame.Right * Lanes[LaneIndex].LateralOffset;

			if (bIncoming)
			{
				IncomingLanes.Add(FResolvedLane{ Entry.ApproachIndex, LaneIndex, LanePos, InwardDir, Lanes[LaneIndex].SpeedLimit });
			}
			if (bOutgoing)
			{
				OutgoingLanes.Add(FResolvedLane{ Entry.ApproachIndex, LaneIndex, LanePos, OutwardDir, Lanes[LaneIndex].SpeedLimit });
			}
		}
	}

	for (const FResolvedLane& In : IncomingLanes)
	{
		for (const FResolvedLane& Out : OutgoingLanes)
		{
			if (In.ApproachIndex == Out.ApproachIndex)
			{
				// No U-turn back onto the same approach; a real through-connection between two
				// different approaches covers the ordinary 2-approach sharp-junction case.
				continue;
			}

			const float Dist = FVector::Dist(In.Position, Out.Position);
			const float HandleLength = FMath::Max(Dist / 3.f, 1.f);

			FFlexLaneConnector Connector;
			Connector.FromSegment = Approaches[In.ApproachIndex].SegmentId;
			Connector.FromLaneIndex = In.LaneIndex;
			Connector.ToSegment = Approaches[Out.ApproachIndex].SegmentId;
			Connector.ToLaneIndex = Out.LaneIndex;
			Connector.ConnectorCurve.P0 = In.Position;
			Connector.ConnectorCurve.P1 = In.Position + In.Tangent * HandleLength;
			Connector.ConnectorCurve.P2 = Out.Position - Out.Tangent * HandleLength;
			Connector.ConnectorCurve.P3 = Out.Position;
			Connector.SpeedLimit = FMath::Min(In.SpeedLimit, Out.SpeedLimit);
			Result.LaneConnectors.Add(Connector);
		}
	}

	return Result;
}

FFlexJunctionMeshResult FFlexIntersectionBuilder::BuildJunctionMesh(const FVector& NodeUp, const FFlexJunctionData& JunctionData, UMaterialInterface* SurfaceMaterial, UMaterialInterface* CrosswalkMaterial)
{
	FFlexJunctionMeshResult Result;
	if (JunctionData.IsEmpty())
	{
		return Result;
	}

	const FVector Up = NodeUp.GetSafeNormal(UE_SMALL_NUMBER, FVector::UpVector);
	const FLocalBasis Basis = MakeLocalBasis(Up);

	Result.Surface.Material = SurfaceMaterial;
	Result.Surface.Vertices = JunctionData.PolygonBoundary;
	Result.Surface.Triangles = JunctionData.PolygonTriangleIndices;
	Result.Surface.Normals.Init(Up, JunctionData.PolygonBoundary.Num());
	Result.Surface.VertexColors.Init(FColor::White, JunctionData.PolygonBoundary.Num());
	const FVector JunctionCentroid = JunctionData.PolygonBoundary.Num() > 0
		? Algo::Accumulate(JunctionData.PolygonBoundary, FVector::ZeroVector) / JunctionData.PolygonBoundary.Num()
		: FVector::ZeroVector;
	for (const FVector& Vertex : JunctionData.PolygonBoundary)
	{
		const FVector2D Local2D = Basis.To2D(Vertex - JunctionCentroid) / 100.f; // meters
		Result.Surface.UV0.Add(Local2D);
		Result.Surface.Tangents.Add(FProcMeshTangent(Basis.AxisX, false));
	}

	Result.Crosswalks.Material = CrosswalkMaterial;
	for (const FFlexCrosswalkPlacement& Crosswalk : JunctionData.Crosswalks)
	{
		const FVector Along = Crosswalk.CrossingDirection.GetSafeNormal(UE_SMALL_NUMBER, Basis.AxisX);
		const FVector Across = FVector::CrossProduct(Up, Along).GetSafeNormal(UE_SMALL_NUMBER, Basis.AxisY);
		const FVector HalfAlong = Along * (Crosswalk.Length * 0.5f);
		const FVector HalfAcross = Across * (Crosswalk.Width * 0.5f);
		const FVector ZOffset = Up * 0.5f; // avoid z-fighting with the surface section

		const FVector A = Crosswalk.Center - HalfAlong - HalfAcross + ZOffset;
		const FVector B = Crosswalk.Center + HalfAlong - HalfAcross + ZOffset;
		const FVector C = Crosswalk.Center + HalfAlong + HalfAcross + ZOffset;
		const FVector D = Crosswalk.Center - HalfAlong + HalfAcross + ZOffset;

		Result.Crosswalks.AppendQuad(A, B, C, D, Up, Along, FVector2D(0.f, 0.f), FVector2D(1.f, 0.f), FVector2D(1.f, 1.f), FVector2D(0.f, 1.f));
	}

	return Result;
}
