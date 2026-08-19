#include "Rail/FlexTrackJunctionSolver.h"

#include "FlexNetworkSubsystem.h"
#include "FlexRoadNode.h"
#include "FlexRoadSegment.h"
#include "RoadTypeProfile.h"
#include "Math/FlexBezierMath.h"

FFlexTrackJunction FFlexTrackJunctionSolver::Solve(FFlexNodeId NodeId, const UFlexNetworkSubsystem& Network,
	const URoadTypeProfile* ProfileFilter, float InitialTrimDistance, float MaxTrimDistance, float TrimDistanceStep, int32 MaxIterations)
{
	FFlexTrackJunction Junction;
	Junction.NodeId = NodeId;

	float TrimDistance = FMath::Min(InitialTrimDistance, MaxTrimDistance);
	int32 PreviousSolvedCount = -1;

	for (int32 Iteration = 0; Iteration < FMath::Max(MaxIterations, 1); ++Iteration)
	{
		Junction.Ports = BuildPorts(NodeId, Network, ProfileFilter, TrimDistance);
		if (Junction.Ports.Num() < 2)
		{
			Junction.Movements.Reset();
			break;
		}

		TArray<FFlexTrackMovement> Movements;
		for (int32 IndexA = 0; IndexA < Junction.Ports.Num(); ++IndexA)
		{
			for (int32 IndexB = IndexA + 1; IndexB < Junction.Ports.Num(); ++IndexB)
			{
				FFlexTrackMovement Movement;
				if (SolvePairMovement(Junction.Ports[IndexA], Junction.Ports[IndexB], IndexA, IndexB, Movement))
				{
					Movements.Add(MoveTemp(Movement));
				}
			}
		}
		Junction.Movements = Movements;

		const int32 MaxPossiblePairs = (Junction.Ports.Num() * (Junction.Ports.Num() - 1)) / 2;
		const bool bResolvedEveryPair = Movements.Num() == MaxPossiblePairs;
		if (bResolvedEveryPair || Movements.Num() <= PreviousSolvedCount || TrimDistance >= MaxTrimDistance)
		{
			break;
		}
		PreviousSolvedCount = Movements.Num();
		TrimDistance = FMath::Min(TrimDistance + TrimDistanceStep, MaxTrimDistance);
	}

	return Junction;
}

TArray<FFlexTrackPort> FFlexTrackJunctionSolver::BuildPorts(FFlexNodeId NodeId, const UFlexNetworkSubsystem& Network, const URoadTypeProfile* ProfileFilter, float TrimDistance)
{
	TArray<FFlexTrackPort> Ports;
	const FFlexRoadNode* Node = Network.GetNode(NodeId);

	for (const TPair<FFlexSegmentId, FFlexRoadSegment>& Pair : Network.GetAllSegments())
	{
		const FFlexRoadSegment& Segment = Pair.Value;
		if (!Segment.Profile || !Segment.Profile->bIsRailProfile || !Segment.ArcLengthTable.IsValid())
		{
			continue;
		}
		if (ProfileFilter && Segment.Profile != ProfileFilter)
		{
			continue;
		}

		const bool bAtStart = Segment.StartNodeId == NodeId;
		const bool bAtEnd = Segment.EndNodeId == NodeId;
		if (!bAtStart && !bAtEnd)
		{
			continue;
		}

		const float SegmentLength = Segment.GetLength();
		const float ClampedTrim = FMath::Clamp(TrimDistance, 0.f, SegmentLength * 0.5f);

		FFlexTrackPort Port;
		Port.SegmentId = Pair.Key;
		Port.bAtSegmentEnd = bAtEnd;
		Port.Gauge = Segment.Profile->RailGauge;
		Port.RailWidth = Segment.Profile->RailWidth;
		Port.MinTurnRadius = Segment.Profile->MinTurnRadius;
		Port.Up = Node ? Node->UpVector : FVector::UpVector;

		if (bAtStart)
		{
			Port.TrimArcLength = ClampedTrim;
			const float T = FFlexBezierMath::ArcLengthToT(Segment.ArcLengthTable, Port.TrimArcLength);
			Port.Position = FFlexBezierMath::Evaluate(Segment.Curve, T);
			Port.Direction = FFlexBezierMath::EvaluateDerivative(Segment.Curve, T).GetSafeNormal();
		}
		else
		{
			Port.TrimArcLength = SegmentLength - ClampedTrim;
			const float T = FFlexBezierMath::ArcLengthToT(Segment.ArcLengthTable, Port.TrimArcLength);
			Port.Position = FFlexBezierMath::Evaluate(Segment.Curve, T);
			Port.Direction = -FFlexBezierMath::EvaluateDerivative(Segment.Curve, T).GetSafeNormal();
		}

		Ports.Add(Port);
	}

	return Ports;
}

bool FFlexTrackJunctionSolver::SolvePairMovement(const FFlexTrackPort& PortA, const FFlexTrackPort& PortB,
	int32 IndexA, int32 IndexB, FFlexTrackMovement& OutMovement)
{
	const float MinimumRadius = FMath::Max(PortA.MinTurnRadius, PortB.MinTurnRadius);
	const float HandleLength = FMath::Max(FVector::Dist(PortA.Position, PortB.Position) / 3.f, 10.f);

	FFlexBezierCurve Curve;
	Curve.P0 = PortA.Position;
	Curve.P3 = PortB.Position;
	// A port's Direction points away from the junction (outward), so the movement curve enters
	// travelling the opposite way at PortA and exits travelling with PortB's own Direction.
	Curve.P1 = PortA.Position - PortA.Direction.GetSafeNormal() * HandleLength;
	Curve.P2 = PortB.Position - PortB.Direction.GetSafeNormal() * HandleLength;

	if (!ValidateMinimumRadius(Curve, MinimumRadius))
	{
		return false;
	}

	OutMovement.FromPortIndex = IndexA;
	OutMovement.ToPortIndex = IndexB;
	OutMovement.Centerline = Curve;
	OutMovement.MinimumRadius = MinimumRadius;

	const float CosAngle = FVector::DotProduct(PortA.Direction.GetSafeNormal(), PortB.Direction.GetSafeNormal());
	const float AngleDegrees = FMath::RadiansToDegrees(FMath::Acos(FMath::Clamp(CosAngle, -1.f, 1.f)));
	OutMovement.bIsStraight = FMath::Abs(180.f - AngleDegrees) < 5.f;
	OutMovement.bIsTurnout = !OutMovement.bIsStraight;

	return true;
}

bool FFlexTrackJunctionSolver::ValidateMinimumRadius(const FFlexBezierCurve& Curve, float MinimumRadius)
{
	if (MinimumRadius <= KINDA_SMALL_NUMBER)
	{
		return true;
	}
	const float MaxCurvature = 1.f / MinimumRadius;
	for (int32 Step = 1; Step < 10; ++Step)
	{
		const float T = static_cast<float>(Step) / 10.f;
		if (FFlexBezierMath::EstimateCurvature(Curve, T) > MaxCurvature)
		{
			return false;
		}
	}
	return true;
}
