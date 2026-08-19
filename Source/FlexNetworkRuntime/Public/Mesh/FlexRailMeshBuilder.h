#pragma once

#include "CoreMinimal.h"
#include "Rail/FlexRailGraph.h"
#include "Mesh/FlexMeshSectionData.h"

class URoadTypeProfile;

/**
 * Sweeps an already topology-resolved FFlexRailGraph into rail meshes: one closed rail-head
 * profile extruded independently along each edge's frames, plus a local groove-cutter boolean per
 * edge when the profile uses a grooved tram rail. Because FFlexRailGraphBuilder has already
 * deduplicated shared rail portions and split crossings, edges never overlap in space, so unlike
 * the previous box-sweep implementation this never needs a global union boolean across the whole
 * graph -- each edge is appended as its own disjoint solid.
 *
 * A Crossing edge is trimmed back by RailCrossingGapCm at both ends before sweeping, producing a
 * simple visual gap where two rails cross without connecting (the design doc's "TopologyCorrect"
 * quality tier). SwitchBlade/Frog edges are swept identically to Crossing for now -- ERailEdgeType
 * simply isn't produced for those two by the current FFlexRailGraphBuilder; real switch-blade/frog
 * geometry is the doc's "Detailed" tier, an explicit follow-up.
 */
class FLEXNETWORKRUNTIME_API FFlexRailMeshBuilder
{
public:
	static bool BuildRailMesh(
		const FFlexRailGraph& RailGraph,
		const URoadTypeProfile* Profile,
		float RailCrossingGapCm,
		FFlexMeshSectionData& OutSection);
};
