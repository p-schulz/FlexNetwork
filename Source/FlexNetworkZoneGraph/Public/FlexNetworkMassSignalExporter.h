#pragma once

#include "CoreMinimal.h"

class UFlexNetworkSubsystem;
class UMassTrafficLightInstancesDataAsset;
class UMassTrafficLightTypesDataAsset;

struct FLEXNETWORKZONEGRAPH_API FFlexMassSignalExportOptions
{
	FString AssetName = TEXT("DA_FlexNetworkTrafficLights");
	UMassTrafficLightTypesDataAsset* TrafficLightTypes = nullptr;
};

struct FLEXNETWORKZONEGRAPH_API FFlexMassSignalExportResult
{
	UMassTrafficLightTypesDataAsset* Types = nullptr;
	UMassTrafficLightInstancesDataAsset* Instances = nullptr;
	int32 NumTrafficLights = 0;
};

/** Projects FlexNetwork's authoritative controls into the data format consumed by MassTraffic. */
class FLEXNETWORKZONEGRAPH_API FFlexNetworkMassSignalExporter
{
public:
	/** Idempotently creates or updates /Game/FlexNetwork/Generated/<AssetName>. */
	static FFlexMassSignalExportResult GenerateOrUpdate(
		const UFlexNetworkSubsystem& Network,
		const FFlexMassSignalExportOptions& Options = FFlexMassSignalExportOptions());
};
