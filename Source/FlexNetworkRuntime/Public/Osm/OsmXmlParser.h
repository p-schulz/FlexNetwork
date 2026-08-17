#pragma once

#include "CoreMinimal.h"

class UOsmDataAsset;
class FXmlNode;

/**
 * Parses standard OSM XML (as exported by e.g. the Overpass API or JOSM: <osm> containing
 * <node>/<way>/<relation> elements, each optionally with <tag k=".." v="..">  children; <way>
 * additionally has <nd ref=".."> children and <relation> has <member type=".." ref=".." role="..">
 * children) into a UOsmDataAsset. Lives in the runtime module (on top of the always-available
 * "XmlParser" engine module) so parsing isn't an editor-only capability -- FlexNetworkEditor's
 * UOsmDataAssetFactory is a thin wrapper around this for the Content Browser import flow, but
 * nothing stops a runtime caller from re-parsing a downloaded/streamed .osm payload the same way.
 */
namespace FOsmXmlParser
{
	/** Loads and parses Path (a .osm file) into OutAsset, replacing its existing contents. Returns false (with a message appended to OutWarnings, if given) if the file couldn't be loaded or isn't valid XML. */
	FLEXNETWORKRUNTIME_API bool ParseFile(const FString& Path, UOsmDataAsset& OutAsset, TArray<FString>* OutWarnings = nullptr);

	/** Parses XmlText (the raw contents of a .osm file, already in memory) into OutAsset, replacing its existing contents. */
	FLEXNETWORKRUNTIME_API bool ParseText(const FString& XmlText, UOsmDataAsset& OutAsset, TArray<FString>* OutWarnings = nullptr);
}
