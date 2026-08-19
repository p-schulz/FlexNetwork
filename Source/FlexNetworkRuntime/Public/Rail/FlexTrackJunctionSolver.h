#pragma once

#include "CoreMinimal.h"
#include "Rail/FlexTrackJunction.h"

class UFlexNetworkSubsystem;
class URoadTypeProfile;

/**
 * Solves one FFlexTrackJunction at a rail junction node: trims each connected rail segment back to
 * a FFlexTrackPort at a configurable distance, then solves a curvature-controlled Bezier movement
 * for every pair of ports, retrying with a larger trim distance (up to MaxTrimDistance) when doing
 * so lets more pairs clear their minimum-radius check.
 *
 * A port pair that still cannot clear minimum radius at the maximum trim distance is left
 * unconnected rather than forced -- e.g. two rails that simply cross (a diamond crossing) rather
 * than merge, where a "movement" between them would require an implausibly tight turn. This is a
 * deliberate simplification: OSM rail topology rarely encodes explicit turn restrictions, so every
 * port pair is a *candidate* movement and only the ones that are geometrically plausible at tram
 * curvature survive. A genuine switch/turnout's ports almost always solve; two merely-crossing
 * tracks almost always don't, at any reasonable trim distance -- so this test doubles as the
 * doc's "large turning angle -> not a real connection" rule without needing tagged data.
 *
 * Entirely independent of FFlexIntersectionBuilder: it only reads rail-profile segments from the
 * shared graph and never touches road junction polygons, curbs, lane connectors, or ZoneGraph.
 */
class FLEXNETWORKRUNTIME_API FFlexTrackJunctionSolver
{
public:
	static FFlexTrackJunction Solve(
		FFlexNodeId NodeId,
		const UFlexNetworkSubsystem& Network,
		const URoadTypeProfile* ProfileFilter,
		float InitialTrimDistance,
		float MaxTrimDistance,
		float TrimDistanceStep,
		int32 MaxIterations = 4);

private:
	static TArray<FFlexTrackPort> BuildPorts(FFlexNodeId NodeId, const UFlexNetworkSubsystem& Network, const URoadTypeProfile* ProfileFilter, float TrimDistance);
	static bool SolvePairMovement(const FFlexTrackPort& PortA, const FFlexTrackPort& PortB, int32 IndexA, int32 IndexB, FFlexTrackMovement& OutMovement);
	static bool ValidateMinimumRadius(const FFlexBezierCurve& Curve, float MinimumRadius);
};
