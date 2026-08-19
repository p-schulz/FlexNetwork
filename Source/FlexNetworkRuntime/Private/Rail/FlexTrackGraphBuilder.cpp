#include "Rail/FlexTrackGraphBuilder.h"

#include "FlexNetworkSubsystem.h"
#include "FlexRoadNode.h"
#include "FlexRoadSegment.h"
#include "RoadTypeProfile.h"
#include "Math/FlexBezierMath.h"

namespace
{
	struct FRailEndpoint
	{
		FFlexSegmentId SegmentId;
		bool bAtSegmentEnd = false;
	};

	FVector OutgoingDirection(const UFlexNetworkSubsystem& Network, const FRailEndpoint& Endpoint)
	{
		const FFlexRoadSegment* Segment = Network.GetSegment(Endpoint.SegmentId);
		if (!Segment)
		{
			return FVector::ForwardVector;
		}
		return Endpoint.bAtSegmentEnd
			? -FFlexBezierMath::EvaluateDerivative(Segment->Curve, 1.f).GetSafeNormal()
			: FFlexBezierMath::EvaluateDerivative(Segment->Curve, 0.f).GetSafeNormal();
	}
}

FFlexTrackGraph FFlexTrackGraphBuilder::Build(const UFlexNetworkSubsystem& Network, const URoadTypeProfile* ProfileFilter, float JunctionAngleToleranceDegrees)
{
	FFlexTrackGraph Graph;

	TMap<FFlexNodeId, TArray<FRailEndpoint>> EndpointsByNode;

	for (const TPair<FFlexSegmentId, FFlexRoadSegment>& Pair : Network.GetAllSegments())
	{
		const FFlexRoadSegment& Segment = Pair.Value;
		if (!Segment.Profile || !Segment.Profile->bIsRailProfile)
		{
			continue;
		}
		if (ProfileFilter && Segment.Profile != ProfileFilter)
		{
			continue;
		}

		FFlexTrackSegmentRef Track;
		Track.SegmentId = Pair.Key;
		Track.Gauge = Segment.Profile->RailGauge;
		Track.RailWidth = Segment.Profile->RailWidth;
		Track.MinTurnRadius = Segment.Profile->MinTurnRadius;
		Graph.Tracks.Add(Track);

		EndpointsByNode.FindOrAdd(Segment.StartNodeId).Add(FRailEndpoint{ Pair.Key, false });
		EndpointsByNode.FindOrAdd(Segment.EndNodeId).Add(FRailEndpoint{ Pair.Key, true });
	}

	for (const TPair<FFlexNodeId, TArray<FRailEndpoint>>& Pair : EndpointsByNode)
	{
		const FFlexNodeId NodeId = Pair.Key;
		const TArray<FRailEndpoint>& Endpoints = Pair.Value;

		if (Endpoints.Num() >= 3)
		{
			Graph.JunctionNodeIds.Add(NodeId);
			continue;
		}
		if (Endpoints.Num() != 2)
		{
			continue; // Dead end -- no junction needed.
		}

		// Exactly two rail segments meet here: only a real junction if they don't continue in
		// roughly the same direction. Both OutgoingDirection() values point away from the node, so
		// a smooth pass-through bend has them nearly opposite (their angle close to 180 degrees).
		const FVector DirA = OutgoingDirection(Network, Endpoints[0]);
		const FVector DirB = OutgoingDirection(Network, Endpoints[1]);
		const float CosAngle = FVector::DotProduct(DirA, DirB);
		const float AngleDegrees = FMath::RadiansToDegrees(FMath::Acos(FMath::Clamp(CosAngle, -1.f, 1.f)));
		if (FMath::Abs(180.f - AngleDegrees) > JunctionAngleToleranceDegrees)
		{
			Graph.JunctionNodeIds.Add(NodeId);
		}
	}

	return Graph;
}
