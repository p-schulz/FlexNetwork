#pragma once

#include "CoreMinimal.h"
#include "Osm/FlexOsmImportSettings.h"

class UOsmDataAsset;

/** One projected hole ring returned without exposing FlexNetwork's raw FOsm* model. */
struct FLEXNETWORKRUNTIME_API FFlexOsmBuildingRing
{
	TArray<FVector2D> PointsMeters;
};

/**
 * Dependency-safe building-footprint transfer type. Coordinates use FlexNetwork's X=east,
 * Y=north convention but are expressed in meters for geometry consumers such as BuildingGrammar.
 */
struct FLEXNETWORKRUNTIME_API FFlexOsmBuildingFootprint
{
	int64 OsmId = 0;
	FString SourceType;
	TArray<FVector2D> OuterRingMeters;
	TArray<FFlexOsmBuildingRing> Holes;
	TMap<FString, FString> Tags;
	bool bIsBuildingPart = false;
};

namespace FFlexOsmBuildingFootprints
{
	/** Extracts closed building ways and building multipolygons using the same origin as road import. */
	FLEXNETWORKRUNTIME_API bool ExtractProjected(
		const UOsmDataAsset* OsmAsset,
		const FFlexOsmImportSettings& ImportSettings,
		TArray<FFlexOsmBuildingFootprint>& OutFootprints,
		FString& OutError);
}
