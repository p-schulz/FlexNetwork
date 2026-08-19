#pragma once

#include "CoreMinimal.h"
#include "FlexNetworkTypes.h"

/**
 * One rail-profile road-graph segment as seen by the rail pipeline, with its rail parameters
 * resolved from the segment's URoadTypeProfile at build time. Deliberately does not duplicate the
 * segment's curve/arc-length table -- those are read from UFlexNetworkSubsystem by SegmentId
 * whenever needed, so this stays a lightweight, always-rebuildable view rather than a second copy
 * of graph data that could drift out of sync.
 */
struct FLEXNETWORKRUNTIME_API FFlexTrackSegmentRef
{
	FFlexSegmentId SegmentId;

	float Gauge = 143.5f;
	float RailWidth = 15.6f;
	float MinTurnRadius = 1800.f;
};

/**
 * The design doc's FTrackGraph ("where can a tram travel"): a read-only view over
 * UFlexNetworkSubsystem's node/segment graph, narrowed to rail-profile segments only. Built fresh
 * by FFlexTrackGraphBuilder every time the rail pipeline runs; never mutated in place and never
 * written back to the subsystem, so building it can never affect road generation or the shared
 * node/segment graph roads also use.
 */
struct FLEXNETWORKRUNTIME_API FFlexTrackGraph
{
	TArray<FFlexTrackSegmentRef> Tracks;

	/** Nodes where 3+ rail segments meet, or exactly two meet at a sharp angle -- candidates FFlexTrackJunctionSolver resolves into ports and movements. */
	TArray<FFlexNodeId> JunctionNodeIds;

	const FFlexTrackSegmentRef* FindTrack(FFlexSegmentId SegmentId) const
	{
		return Tracks.FindByPredicate([SegmentId](const FFlexTrackSegmentRef& Track) { return Track.SegmentId == SegmentId; });
	}
};
