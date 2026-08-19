#pragma once

#include "CoreMinimal.h"
#include "Rail/FlexTrackGraph.h"
#include "Rail/FlexTrackJunction.h"
#include "Rail/FlexRailGraph.h"

class UFlexNetworkSubsystem;

/**
 * Builds the design doc's FRailGraph ("which physical pieces of steel exist") from a solved
 * FTrackGraph: offsets ordinary track (trimmed at any junction boundaries) and every solved
 * junction movement into left/right rail polylines, then:
 *
 *  - MERGE: where exactly two rail polylines on the same side (left-left or right-right) share a
 *    junction port, folds however much of their leading span stays within MergeToleranceCm/
 *    AngleToleranceDegrees of each other into one Shared edge -- the common leg of a turnout.
 *    More than two polylines sharing one port (rare multi-way slip switches) are left unmerged as
 *    a graceful fallback; each keeps its own edge.
 *
 *  - CROSSING: where two polylines at the same junction that do NOT share a port genuinely
 *    intersect (e.g. the two straight-through pairs of a diamond crossing), splits both at that
 *    point into independent edges with NO shared node -- see the doc's "Distinguish Merges from
 *    Crossings". FFlexRailMeshBuilder trims a small visual gap at a Crossing edge.
 *
 * See the .cpp for the exact heuristics and their scope limits.
 */
class FLEXNETWORKRUNTIME_API FFlexRailGraphBuilder
{
public:
	static FFlexRailGraph Build(
		const FFlexTrackGraph& TrackGraph,
		TArrayView<const FFlexTrackJunction> Junctions,
		const UFlexNetworkSubsystem& Network,
		float SampleStep,
		float MergeToleranceCm,
		float AngleToleranceDegrees,
		float CrossingAngleToleranceDegrees);
};
