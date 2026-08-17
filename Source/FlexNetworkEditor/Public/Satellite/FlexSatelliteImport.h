#pragma once

#include "CoreMinimal.h"

class UWorld;
class UOsmDataAsset;
struct FFlexOsmImportSettings;
class UFlexSatelliteImagerySettings;
class AFlexSatelliteTileActor;

namespace FlexNetwork::Satellite
{
	struct FLEXNETWORKEDITOR_API FImportResult
	{
		int32 NumTilesRequested = 0;
		int32 NumTilesSpawned = 0;
		TArray<FString> Warnings;
	};

	/**
	 * Fetches LGL-BW aerial/land-use imagery covering OsmAsset's own extent and spawns one
	 * AFlexSatelliteTileActor per tile in World, laid out on a grid snapped to the SAME projection
	 * origin FFlexOsmGraphBuilder::BuildFromOsm would resolve for this exact OsmAsset/OsmSettings
	 * pair (via FFlexOsmGraphBuilder::ResolveOrigin) -- i.e. call this with the same OsmAsset and
	 * OsmImportSettings you last called (or are about to call) BuildFromOsm with, and the imagery
	 * lines up with the resulting road network exactly. Every spawned actor is appended to
	 * OutSpawnedTiles (for "Bake Imagery To Content" to read back afterward). Runs asynchronously
	 * (WMS tile fetches are independent, fire-and-forget HTTP requests); OnComplete is invoked
	 * exactly once, after every tile request has resolved (successfully or not).
	 */
	FLEXNETWORKEDITOR_API void ImportSatelliteImagery(
		UWorld* World,
		const UOsmDataAsset& OsmAsset,
		const FFlexOsmImportSettings& OsmSettings,
		const UFlexSatelliteImagerySettings& ImagerySettings,
		TArray<TWeakObjectPtr<AFlexSatelliteTileActor>>& OutSpawnedTiles,
		TFunction<void(FImportResult)> OnComplete);
}
