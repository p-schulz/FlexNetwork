#pragma once

#include "CoreMinimal.h"
#include "Osm/FlexOsmImportSettings.h"

class UOsmDataAsset;
class UFlexNetworkSubsystem;
class URoadTypeProfile;

/**
 * Turns a UOsmDataAsset's ways (filtered by highway tag) into FlexNetwork graph nodes/segments on
 * a UFlexNetworkSubsystem: projects lat/lon to local world-space, merges OSM nodes that sit within
 * FFlexOsmImportSettings::JunctionMergeRadius of each other into single junction nodes, fits
 * smooth (Catmull-Rom-derived) Bezier tangents through each way's shape points, and resolves a
 * URoadTypeProfile per distinct lane configuration found in the data via the caller-supplied
 * ResolveProfile callback.
 *
 * Deliberately takes profile *resolution* as an injected callback rather than creating profiles
 * itself: creating+saving a persistent .uasset is an editor-only operation (UPackage::SavePackage,
 * asset registry notification), so this stays callable from a plain Runtime context (e.g. a
 * shipped game streaming in OSM data at runtime, using in-memory-only profiles) while
 * FlexNetworkEditor's caller supplies a resolver that creates real saved assets.
 */
namespace FFlexOsmGraphBuilder
{
	/** The lane configuration derived from one OSM way's tags -- also the dedup key FlexNetworkEditor's default resolver uses to decide whether an existing generated profile can be reused. */
	struct FLEXNETWORKRUNTIME_API FLaneSignature
	{
		FString HighwayTag;
		int32 ForwardLanes = 1;
		int32 BackwardLanes = 1;
		float LaneWidth = 350.f;
		float SpeedLimitKmh = 50.f;
		float SidewalkWidth = 200.f;

		/** Stable string key identifying this exact lane configuration, for profile dedup/lookup and for naming generated assets. */
		FString ToKey() const;
	};

	struct FLEXNETWORKRUNTIME_API FImportResult
	{
		int32 NumWaysImported = 0;
		int32 NumSegmentsCreated = 0;
		int32 NumNodesCreated = 0;
		int32 NumJunctionsMerged = 0; // Clusters that combined 2+ distinct OSM nodes into one FlexNetwork node.
		int32 NumDistinctLaneSignatures = 0;
		TArray<FString> Warnings;

		bool WasSuccessful() const { return NumWaysImported > 0; }
	};

	/** Fills in a profile's Lanes/SidewalkWidth/etc. from a lane signature -- shared by any ResolveProfile implementation so the OSM-tag-to-lane-layout mapping lives in one place. */
	FLEXNETWORKRUNTIME_API void ConfigureProfileFromLaneSignature(URoadTypeProfile& Profile, const FLaneSignature& Signature);

	/**
	 * Local tangent-plane (equirectangular) projection from WGS84 lat/lon to world-space
	 * centimeters relative to (OriginLatDeg, OriginLonDeg) -- X = east, Y = north. The exact
	 * formula BuildFromOsm itself uses to place every node, exposed here so any other importer
	 * that needs to line up with FlexNetwork-imported OSM roads (e.g. satellite/land-use imagery)
	 * can reuse the identical math instead of risking a subtly different approximation or axis
	 * convention.
	 */
	FLEXNETWORKRUNTIME_API FVector2D ProjectLatLonToLocalCm(double Lat, double Lon, double OriginLatDeg, double OriginLonDeg);

	/** Exact algebraic inverse of ProjectLatLonToLocalCm -- local world-space centimeters (X = east, Y = north) back to WGS84 lat/lon, relative to the same origin. */
	FLEXNETWORKRUNTIME_API void UnprojectLocalCmToLatLon(double LocalX, double LocalY, double OriginLatDeg, double OriginLonDeg, double& OutLat, double& OutLon);

	/**
	 * Resolves the same projection origin BuildFromOsm would use for this OsmAsset/Settings pair,
	 * without actually building anything -- an explicit OriginLatLon override if
	 * Settings.bUseOriginOverride is set, else OsmAsset.Bounds' midpoint if the file had a <bounds>
	 * element, else the first node found in OsmAsset.Nodes as a last resort (an approximation of
	 * BuildFromOsm's own "first node referenced by a matching way" fallback, which needs the
	 * highway-tag-filtered way list to reproduce exactly; the two only differ when the file has no
	 * <bounds> element AND the first node in map order isn't reachable from any matching way, an
	 * edge case not worth the extra coupling here). Returns false (leaving Out* untouched) if no
	 * origin could be resolved at all (an empty asset).
	 */
	FLEXNETWORKRUNTIME_API bool ResolveOrigin(const UOsmDataAsset& OsmAsset, const FFlexOsmImportSettings& Settings, double& OutOriginLat, double& OutOriginLon);

	/**
	 * Resolves the same origin ResolveOrigin would (with one refinement -- see below) and computes
	 * the local-space (centimeter, X=east/Y=north) bounding box of every node referenced by a way
	 * whose highway tag matches Settings.HighwayTags -- i.e. exactly the set of nodes BuildFromOsm
	 * would actually place into the graph, NOT the file's own (often much larger -- a raw Overpass
	 * download bbox is routinely bigger than the specific roads a HighwayTags filter ends up
	 * keeping) <bounds> declaration. A caller that needs a bounding box around the roads
	 * BuildFromOsm will actually create for this exact OsmAsset/Settings pair -- e.g. laying out a
	 * satellite-imagery tile grid that should track where the roads really are, not the file's
	 * nominal extent -- should use this instead of OsmAsset.Bounds directly.
	 *
	 * The refinement: unlike ResolveOrigin's own third fallback (first node in OsmAsset.Nodes'
	 * iteration order, which needn't be reachable from any matching way at all), this one matches
	 * BuildFromOsm's actual third fallback exactly -- the first node referenced by a *matching* way
	 * -- so the resolved origin is guaranteed identical to what BuildFromOsm itself would use even
	 * for a file with no <bounds> element.
	 *
	 * Returns false (leaving every Out* untouched) if no way matched the filter, or none of their
	 * referenced nodes were found in OsmAsset.Nodes.
	 */
	FLEXNETWORKRUNTIME_API bool ComputeMatchingRoadExtent(const UOsmDataAsset& OsmAsset, const FFlexOsmImportSettings& Settings, double& OutOriginLat, double& OutOriginLon, FVector2D& OutMinLocalCm, FVector2D& OutMaxLocalCm);

	/**
	 * Builds the graph. ResolveProfile is called once per distinct FLaneSignature encountered
	 * (not once per way) and must return a valid, already-configured-or-to-be-configured profile;
	 * a typical implementation checks a cache by Signature.ToKey(), and on a miss creates a new
	 * URoadTypeProfile, calls ConfigureProfileFromLaneSignature on it, persists it however the
	 * caller sees fit, and caches it for subsequent calls with the same key.
	 */
	FLEXNETWORKRUNTIME_API FImportResult BuildFromOsm(UFlexNetworkSubsystem& Subsystem, const UOsmDataAsset& OsmAsset, const FFlexOsmImportSettings& Settings, TFunctionRef<URoadTypeProfile*(const FLaneSignature&)> ResolveProfile);
}
