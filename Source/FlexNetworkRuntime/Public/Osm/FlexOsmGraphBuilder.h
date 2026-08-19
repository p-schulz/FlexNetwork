#pragma once

#include "CoreMinimal.h"
#include "Osm/FlexOsmImportSettings.h"

class UOsmDataAsset;
class UFlexNetworkSubsystem;
class URoadTypeProfile;

/**
 * Turns a UOsmDataAsset's highway and railway ways into FlexNetwork graph nodes/segments on
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
		FString RailwayTag;
		bool bIsRailway = false;
		int32 ForwardLanes = 1;
		int32 BackwardLanes = 1;
		float LaneWidth = 350.f;
		float SpeedLimitKmh = 50.f;
		float SidewalkWidth = 200.f;
		/** Whole-profile offset from the digitized OSM way, derived from placement=* (cm, +right). */
		float LateralOffset = 0.f;
		int32 RailTrackCount = 1;
		float RailGauge = 143.5f;
		float RailWidth = 15.6f;
		float RailTopWidth = 11.5f;
		float RailHeight = 7.2f;
		bool bUseGroovedRailProfile = false;
		float RailGrooveWidth = 4.f;
		float RailGrooveDepth = 4.5f;
		float RailGrooveInwardOffset = 1.5f;
		float RailBooleanOverlap = 0.5f;
		float RailTrackSpacing = 400.f;
		bool bRailOneWay = false;
		bool bRailReverse = false;

		/** Stable string key identifying this exact lane configuration, for profile dedup/lookup and for naming generated assets. */
		FString ToKey() const;
	};

	struct FLEXNETWORKRUNTIME_API FImportResult
	{
		int32 NumWaysImported = 0;
		int32 NumRailwayWaysImported = 0;
		int32 NumSegmentsCreated = 0;
		int32 NumNodesCreated = 0;
		int32 NumJunctionsMerged = 0; // Clusters that combined 2+ distinct OSM nodes into one FlexNetwork node.
		/** Number of compact multi-node intersection interiors grouped into shared surface regions. */
		int32 NumComplexIntersectionsCollapsed = 0;
		int32 NumDistinctLaneSignatures = 0;
		int32 NumTrafficControlsImported = 0;
		TArray<FString> Warnings;

		bool WasSuccessful() const { return NumWaysImported > 0; }
	};

	/** Fills in a profile's Lanes/SidewalkWidth/etc. from a lane signature -- shared by any ResolveProfile implementation so the OSM-tag-to-lane-layout mapping lives in one place. */
	FLEXNETWORKRUNTIME_API void ConfigureProfileFromLaneSignature(URoadTypeProfile& Profile, const FLaneSignature& Signature);

	/**
	 * Local tangent-plane (equirectangular) projection from WGS84 lat/lon to world-space
	 * centimeters relative to (OriginLatDeg, OriginLonDeg) -- X = north, Y = east, matching
	 * Unreal's X-forward/Y-right world convention and BuildingGrammar/ProceduralRoads. The exact
	 * formula BuildFromOsm itself uses to place every node, exposed here so any other importer
	 * that needs to line up with FlexNetwork-imported OSM roads (e.g. satellite/land-use imagery)
	 * can reuse the identical math instead of risking a subtly different approximation or axis
	 * convention.
	 */
	FLEXNETWORKRUNTIME_API FVector2D ProjectLatLonToLocalCm(double Lat, double Lon, double OriginLatDeg, double OriginLonDeg);

	/** Exact algebraic inverse of ProjectLatLonToLocalCm -- local world-space centimeters (X = north, Y = east) back to WGS84 lat/lon, relative to the same origin. */
	FLEXNETWORKRUNTIME_API void UnprojectLocalCmToLatLon(double LocalX, double LocalY, double OriginLatDeg, double OriginLonDeg, double& OutLat, double& OutLon);

	/**
	 * Resolves the same projection origin BuildFromOsm would use for this OsmAsset/Settings pair,
	 * without actually building anything -- an explicit OriginLatLon override if
	 * Settings.bUseOriginOverride is set, else OsmAsset.Bounds' midpoint if the file had a <bounds>
	 * element, else the first referenced node of the lowest-ID matching highway/railway way. If an extract
	 * has no matching transport way (for example a building-only asset), its lowest-ID node is used as a
	 * final deterministic fallback. This is the exact resolver used by roads, building footprints,
	 * and imagery. Returns false (leaving Out* untouched) only for an empty asset.
	 */
	FLEXNETWORKRUNTIME_API bool ResolveOrigin(const UOsmDataAsset& OsmAsset, const FFlexOsmImportSettings& Settings, double& OutOriginLat, double& OutOriginLon);

	/**
	 * Resolves the same origin ResolveOrigin would and computes
	 * the local-space (centimeter, X=north/Y=east) bounding box of every node referenced by a way
	 * whose highway or railway tag matches the configured filters -- i.e. exactly the set of nodes BuildFromOsm
	 * would actually place into the graph, NOT the file's own (often much larger -- a raw Overpass
	 * download bbox is routinely bigger than the specific transport ways the filters end up
	 * keeping) <bounds> declaration. A caller that needs a bounding box around the generated network
	 * BuildFromOsm will actually create for this exact OsmAsset/Settings pair -- e.g. laying out a
	 * satellite-imagery tile grid that should track where the network really is, not the file's
	 * nominal extent -- should use this instead of OsmAsset.Bounds directly.
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
	 *
	 * All graph mutations are wrapped in one nestable subsystem batch. A direct runtime call
	 * validates/rebuilds once after the last segment; a caller may add an outer batch to combine
	 * adjacent state changes (the editor does this for its visualization-mode selection).
	 */
	FLEXNETWORKRUNTIME_API FImportResult BuildFromOsm(UFlexNetworkSubsystem& Subsystem, const UOsmDataAsset& OsmAsset, const FFlexOsmImportSettings& Settings, TFunctionRef<URoadTypeProfile*(const FLaneSignature&)> ResolveProfile);
}
