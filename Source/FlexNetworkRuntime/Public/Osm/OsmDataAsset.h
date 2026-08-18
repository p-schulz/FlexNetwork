#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
// Keep this relative to this header's own directory. Other plugins may also have a public
// "Osm/OsmTypes.h" (BuildingGrammar does), and dependency include-path order must not decide which
// unrelated FOsmNode model backs this reflected asset.
#include "OsmTypes.h"
#include "OsmDataAsset.generated.h"

/**
 * Parsed OpenStreetMap data (nodes, ways, relations, and their tags), imported from a .osm XML
 * file via UOsmDataAssetFactory (FlexNetworkEditor). This is a generic OSM data container, not
 * road-import-specific -- FFlexOsmGraphBuilder (also in this module) is what turns it into a
 * FlexNetwork graph, but nothing about this asset assumes that's the only consumer; any other
 * system that wants raw OSM node/way/relation/tag data can read it directly.
 */
UCLASS(BlueprintType)
class FLEXNETWORKRUNTIME_API UOsmDataAsset : public UDataAsset
{
	GENERATED_BODY()

public:
	/** The file's <bounds> element, if it had one -- parsed first since it's always the first child of <osm> in a standard export. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "OSM")
	FOsmBounds Bounds;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "OSM")
	TMap<int64, FOsmNode> Nodes;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "OSM")
	TMap<int64, FOsmWay> Ways;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "OSM")
	TMap<int64, FOsmRelation> Relations;

	/** Path of the .osm file this was imported from, for re-import/reference. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "OSM")
	FString SourceFilePath;
};
