#pragma once

#include "CoreMinimal.h"
#include "FlexNetworkTypes.h"

class UFlexNetworkSubsystem;

/**
 * Extension point for exporting the FlexNetwork graph into another engine navigation/lane-graph
 * system (most notably UE5's built-in ZoneGraph plugin, which Epic's Mass traffic sample builds
 * on). The optional FlexNetworkZoneGraph module now provides the full one-shot ZoneShape/bake
 * pipeline used by the editor mode. It intentionally remains outside FlexNetworkRuntime so
 * projects consuming the direct query API do not inherit ZoneGraph/MassTraffic dependencies.
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
