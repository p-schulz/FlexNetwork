#pragma once

#include "CoreMinimal.h"
#include "Rail/FlexTrackGraph.h"

class UFlexNetworkSubsystem;
class URoadTypeProfile;

/**
 * Extracts a read-only FFlexTrackGraph from UFlexNetworkSubsystem's shared node/segment graph:
 * every bIsRailProfile segment (optionally restricted to one ProfileFilter) becomes a track, and
 * every node where rail segments meet in a way that needs explicit junction handling (3+ rail
 * segments, or exactly two at a sharp angle) is recorded as a junction node candidate for
 * FFlexTrackJunctionSolver.
 *
 * Purely a query over the subsystem -- never mutates it, never calls FFlexIntersectionBuilder, and
 * has no effect on road generation. A node touched only by two rail segments continuing smoothly
 * in roughly the same direction (an ordinary bend) is deliberately NOT a junction node here, so
 * ordinary straight/curving track is left as one continuous rail rather than being cut and rejoined
 * at every bend.
 *
 * ProfileFilter restricts both the track list and junction-node detection to segments using that
 * exact profile -- the caller (UFlexNetworkSubsystem::BuildRailMeshResults) builds one TrackGraph
 * per distinct rail profile in the graph, since two differently-gauged rail lines meeting at a
 * node can't physically share a switch anyway; treating them as independent TrackGraphs is more
 * correct than trying to solve one shared junction across profiles.
 */
class FLEXNETWORKRUNTIME_API FFlexTrackGraphBuilder
{
public:
	static FFlexTrackGraph Build(const UFlexNetworkSubsystem& Network, const URoadTypeProfile* ProfileFilter = nullptr, float JunctionAngleToleranceDegrees = 15.f);
};
