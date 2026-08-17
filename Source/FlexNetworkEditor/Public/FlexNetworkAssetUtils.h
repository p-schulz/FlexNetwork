#pragma once

#include "CoreMinimal.h"

class URoadTypeProfile;
class UMaterialInterface;

/** Small shared helper for the two places that need to create+save real URoadTypeProfile assets from code: the FlexNetwork.CreateDefaultProfiles console command and the OSM importer's profile resolver. */
namespace FlexNetworkAssetUtils
{
	/**
	 * Creates AssetName under PackagePath (e.g. "/FlexNetwork/Profiles/OSM"), lets Configure fill
	 * in its properties, then saves it to disk. Always creates fresh (overwriting any existing
	 * asset of the same name) rather than loading-and-reusing an existing one -- callers that need
	 * dedup across multiple assets in one run (e.g. per OSM lane-signature) are expected to keep
	 * their own cache of already-created profiles for the duration of that run.
	 */
	FLEXNETWORKEDITOR_API URoadTypeProfile* CreateRoadTypeProfileAsset(const FString& PackagePath, const FString& AssetName, TFunctionRef<void(URoadTypeProfile&)> Configure);

	/**
	 * Finds every URoadTypeProfile asset in the project (via the asset registry, not just ones the
	 * current session created) and overwrites whichever of its four material slots has a non-null
	 * override supplied here -- a null argument leaves that slot untouched on every profile, so a
	 * caller can update e.g. just RoadMaterial across the board without disturbing per-profile
	 * SidewalkMaterial/etc. choices. Existing-scale escape hatch for a large OSM import, which can
	 * generate dozens of auto-named profiles that would otherwise all need their materials set by
	 * hand one at a time. Saves each modified profile to disk. Returns how many were changed.
	 */
	FLEXNETWORKEDITOR_API int32 ApplyMaterialsToAllProfiles(UMaterialInterface* RoadMaterial, UMaterialInterface* SidewalkMaterial, UMaterialInterface* JunctionMaterial, UMaterialInterface* MedianMaterial);
}
