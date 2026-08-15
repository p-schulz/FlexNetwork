#pragma once

#include "CoreMinimal.h"
#include "FlexNetworkTypes.h"

class UFlexNetworkSubsystem;

/**
 * Extension point for exporting the FlexNetwork graph into another engine navigation/lane-graph
 * system (most notably UE5's built-in ZoneGraph plugin, which Epic's Mass traffic sample builds
 * on). Deliberately left unimplemented for now: the direct query API on UFlexNetworkSubsystem
 * (GetLaneAtArcLength, GetLaneConnectorsAtNode, etc.) is the primary integration path for this
 * project's own traffic simulation, and a project with its own graph representation is generally
 * better served by consuming that directly rather than round-tripping through ZoneGraph. If a
 * ZoneGraph export is wanted later, implement it as a class deriving from this interface (e.g.
 * FFlexZoneGraphExporter in a new FlexNetworkZoneGraph module, mirroring FlexNetworkEditor's
 * relationship to FlexNetworkRuntime) rather than adding a hard ZoneGraph dependency here.
 */
class FLEXNETWORKRUNTIME_API IFlexNetworkExporter
{
public:
	virtual ~IFlexNetworkExporter() = default;

	/** Full export of the current graph state. */
	virtual void ExportFullNetwork(const UFlexNetworkSubsystem& Subsystem) = 0;

	/** Incremental export in response to FOnRoadNetworkChanged, for exporters that can update in place rather than rebuilding everything. */
	virtual void ExportChangedRegion(const UFlexNetworkSubsystem& Subsystem, const TArray<FFlexNodeId>& ChangedNodes, const TArray<FFlexSegmentId>& ChangedSegments) = 0;
};
