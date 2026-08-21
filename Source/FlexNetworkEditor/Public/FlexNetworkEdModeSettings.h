#pragma once

#include "CoreMinimal.h"
#include "UObject/Object.h"
#include "FlexNetworkTypes.h"
#include "Osm/FlexOsmImportSettings.h"
#include "FlexNetworkEdModeSettings.generated.h"

class URoadTypeProfile;
class UFlexSyntheticNetworkConfig;
class UOsmDataAsset;
class UMaterialInterface;
class UStaticMesh;
class AFlexSatelliteTileActor;
class UMassTrafficLightInstancesDataAsset;
class UMassTrafficLightTypesDataAsset;

/** Active viewport manipulation tool when Draw Mode is disabled. */
UENUM()
enum class EFlexNetworkNodeEditTool : uint8
{
	Move UMETA(DisplayName = "Move Node"),
	Rotate UMETA(DisplayName = "Rotate Node + Connected Tangents"),
	Tangent UMETA(DisplayName = "Adjust Tangent Handles")
};

/**
 * Transient per-editor-session settings for the FlexNetwork drawing tool, shown in its toolkit
 * via a plain IDetailsView -- this is what "select the road type to draw" (and "import from OSM")
 * looks like in this minimal editor UI.
 */
UCLASS(Transient)
class UFlexNetworkEdModeSettings : public UObject
{
	GENERATED_BODY()

public:
	/** Selects the transient output maintained for both hand-drawn and OSM-imported segments. */
	UPROPERTY(EditAnywhere, Category = "FlexNetwork", meta = (DisplayPriority = "0"))
	EFlexNetworkVisualizationMode VisualizationMode = EFlexNetworkVisualizationMode::GeneratedGeometry;

#if WITH_EDITOR
	virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
#endif
	/**
	 * When on: click once to place/continue a road's start point, move the mouse to preview,
	 * click again to commit the segment (like the Landscape Splines "Add Control Point" tool).
	 * When off ("Select" mode): click an existing node to select it, then drag the viewport
	 * gizmo to move it.
	 */
	UPROPERTY(EditAnywhere, Category = "FlexNetwork", meta = (DisplayName = "Draw Mode (off = Node Edit)"))
	bool bDrawModeActive = true;

	/** Selects how the viewport gizmo edits the currently-selected node when Draw Mode is off. */
	UPROPERTY(EditAnywhere, Category = "FlexNetwork", meta = (EditCondition = "!bDrawModeActive"))
	EFlexNetworkNodeEditTool NodeEditTool = EFlexNetworkNodeEditTool::Move;

	/** Road type profile new segments are drawn with. */
	UPROPERTY(EditAnywhere, Category = "FlexNetwork")
	TObjectPtr<URoadTypeProfile> ActiveProfile;

	/** Elevation type applied to newly-created nodes/segments. */
	UPROPERTY(EditAnywhere, Category = "FlexNetwork")
	EFlexRoadElevationType ActiveElevationType = EFlexRoadElevationType::Ground;

	/** Hold to disable 15-degree angle snapping while dragging. */
	UPROPERTY(EditAnywhere, Category = "FlexNetwork")
	bool bAngleSnapEnabled = true;

	/** Parsed OSM data (import a .osm file via Content Browser > Import first) to generate roads and railways from. */
	UPROPERTY(EditAnywhere, Category = "OSM Import")
	TObjectPtr<UOsmDataAsset> OsmAsset;

	UPROPERTY(EditAnywhere, Category = "OSM Import", meta = (ShowOnlyInnerProperties))
	FFlexOsmImportSettings OsmImportSettings;

	/** Publishes this asset/settings pair to the level so BuildingGrammar uses the identical source and projection. */
	UPROPERTY(EditAnywhere, Category = "OSM Import|Shared Context")
	bool bPublishSharedOsmContext = true;

	UFUNCTION(CallInEditor, Category = "OSM Import|Shared Context", meta = (DisplayName = "Publish OSM Context To Level"))
	void PublishOsmContextToLevel();

	UFUNCTION(CallInEditor, Category = "OSM Import|Shared Context", meta = (DisplayName = "Load OSM Context From Level"))
	void LoadOsmContextFromLevel();

	UPROPERTY(VisibleAnywhere, Category = "OSM Import|Shared Context", meta = (MultiLine = true))
	FString OsmContextStatus = TEXT("No shared OSM context has been published in this editor session.");

	/** Imports only configured highway ways, projects them to world space, and builds matching FlexNetwork road segments and generated profiles. */
	UFUNCTION(CallInEditor, Category = "OSM Import", meta = (DisplayName = "Generate Roads From OSM"))
	void GenerateRoadsFromOsm();

	/** Imports only configured railway ways and builds matching train/tram FlexNetwork segments and generated rail profiles. */
	UFUNCTION(CallInEditor, Category = "OSM Import|Railways", meta = (DisplayName = "Generate Rails From OSM"))
	void GenerateRailsFromOsm();

	/** Road type used for major (arterial) generated streamlines. Required to generate a synthetic network. */
	UPROPERTY(EditAnywhere, Category = "Synthetic Network")
	TObjectPtr<URoadTypeProfile> SyntheticArterialProfile;

	/** Road type used for minor (local) generated streamlines. Required to generate a synthetic network. */
	UPROPERTY(EditAnywhere, Category = "Synthetic Network")
	TObjectPtr<URoadTypeProfile> SyntheticLocalProfile;

	/** Author-able tensor-field regions (grid districts, radial centers, blended together) -- see UFlexSyntheticNetworkConfig. Leave unset to use the built-in default two-region field. */
	UPROPERTY(EditAnywhere, Category = "Synthetic Network")
	TObjectPtr<UFlexSyntheticNetworkConfig> SyntheticNetworkConfig;

	/** Half-width/half-height (cm) of the square domain generated around the world origin. */
	UPROPERTY(EditAnywhere, Category = "Synthetic Network", meta = (ClampMin = "1000.0", Units = "cm"))
	float SyntheticDomainHalfSize = 10000.f;

	/** Higher values seed more major streamlines and space minor (cross-street) streamlines closer together, producing smaller blocks. */
	UPROPERTY(EditAnywhere, Category = "Synthetic Network", meta = (ClampMin = "1", ClampMax = "10"))
	int32 SyntheticBlockDensity = 4;

	/** A streamline stops early if it gets this close (cm) to a different, already-traced streamline -- raise this if generated roads still read as overlapping/crowded (e.g. near a radial field's center), lower it to allow denser networks. */
	UPROPERTY(EditAnywhere, Category = "Synthetic Network", meta = (ClampMin = "0.0", Units = "cm"))
	float SyntheticMinStreamlineSeparation = 600.f;

	/**
	 * Generates a synthetic road network (see the plugin's synthetic-generation plan document) from
	 * either the assigned Synthetic Network Config's authored field regions, or (if unset) the same
	 * built-in default two-region field Phase 1 used, and adds it directly to the current FlexNetwork
	 * graph via the same AddNode/AddSegment API a hand-drawn or OSM-imported network uses.
	 */
	UFUNCTION(CallInEditor, Category = "Synthetic Network", meta = (DisplayName = "Generate Synthetic Network"))
	void GenerateSyntheticNetwork();

	UPROPERTY(VisibleAnywhere, Category = "Synthetic Network", meta = (MultiLine = true))
	FString SyntheticNetworkStatus = TEXT("No synthetic network has been generated in this editor session.");

	/** Rebuilds every road, sidewalk, junction and segment visualization from authoritative graph data. */
	UFUNCTION(CallInEditor, Category = "FlexNetwork", meta = (DisplayName = "Rebuild All Network Geometry"))
	void RebuildAllNetworkGeometry();

	/** Stores the current authored graph in a persistent level actor so PIE/runtime worlds can reconstruct all transient representations and query data. Save the level after baking. */
	UFUNCTION(CallInEditor, Category = "FlexNetwork|Bake", meta = (DisplayName = "Bake Network To Level"))
	void BakeNetworkToLevel();

	/** Replaces the current transient graph from the level's baked snapshot. */
	UFUNCTION(CallInEditor, Category = "FlexNetwork|Bake", meta = (DisplayName = "Restore Baked Network"))
	void RestoreBakedNetwork();

	/** Removes baked snapshot actors without clearing the currently loaded transient network. */
	UFUNCTION(CallInEditor, Category = "FlexNetwork|Bake", meta = (DisplayName = "Remove Network Bake"))
	void RemoveNetworkBake();

	UPROPERTY(VisibleAnywhere, Category = "FlexNetwork|Bake", meta = (MultiLine = true))
	FString BakeStatus = TEXT("No network has been baked in this editor session.");

	/** Distance between ZoneShape control points sampled along FlexNetwork Bezier segments and junction connectors. */
	UPROPERTY(EditAnywhere, Category = "Mass AI|ZoneGraph", meta = (ClampMin = "10.0", Units = "cm"))
	float ZoneGraphSampleSpacing = 200.f;

	/** Exports generated roadside sidewalks, junction corner walks, and crosswalks for MassCrowd. */
	UPROPERTY(EditAnywhere, Category = "Mass AI|ZoneGraph")
	bool bZoneGraphIncludePedestrians = true;

	/** Removes only the ZoneShape/ZoneGraphData actors generated by an earlier FlexNetwork export. */
	UPROPERTY(EditAnywhere, Category = "Mass AI|ZoneGraph")
	bool bZoneGraphReplaceExisting = true;

	/** Registers the shared tags and updates MassTraffic/MassCrowd filters and speed-limit buckets. */
	UPROPERTY(EditAnywhere, Category = "Mass AI|ZoneGraph")
	bool bZoneGraphConfigureMassAI = true;

	/** Generate the MassTraffic light-instances asset from FlexNetwork's authoritative controls. */
	UPROPERTY(EditAnywhere, Category = "Mass AI|Traffic Signals")
	bool bGenerateMassTrafficSignalsWithZoneGraph = true;

	/** Optional visual types; City Sample's types or a generated placeholder are used when unset. */
	UPROPERTY(EditAnywhere, Category = "Mass AI|Traffic Signals")
	TObjectPtr<UMassTrafficLightTypesDataAsset> MassTrafficLightTypes;

	UPROPERTY(EditAnywhere, Category = "Mass AI|Traffic Signals")
	FString MassTrafficLightInstancesAssetName = TEXT("DA_FlexNetworkTrafficLights");

	/** Regenerates only the MassTraffic signal instances asset; no ZoneGraph geometry is touched. */
	UFUNCTION(CallInEditor, Category = "Mass AI|Traffic Signals", meta = (DisplayName = "Generate MassTraffic Signals"))
	void GenerateMassTrafficSignals();

	UPROPERTY(VisibleAnywhere, Category = "Mass AI|Traffic Signals")
	TObjectPtr<UMassTrafficLightTypesDataAsset> GeneratedMassTrafficLightTypes;

	UPROPERTY(VisibleAnywhere, Category = "Mass AI|Traffic Signals")
	TObjectPtr<UMassTrafficLightInstancesDataAsset> GeneratedMassTrafficLightInstances;

	UPROPERTY(VisibleAnywhere, Category = "Mass AI|Traffic Signals", meta = (MultiLine = true))
	FString MassTrafficSignalStatus = TEXT("No MassTraffic signal asset has been generated in this editor session.");

	/** Converts current FlexNetwork road lanes, junction lane connectors, sidewalks, and crosswalks into ZoneGraph and bakes it into this level. */
	UFUNCTION(CallInEditor, Category = "Mass AI|ZoneGraph", meta = (DisplayName = "Generate Mass AI ZoneGraph"))
	void GenerateMassAIZoneGraph();

	/** Removes only ZoneGraph actors previously generated by FlexNetwork. */
	UFUNCTION(CallInEditor, Category = "Mass AI|ZoneGraph", meta = (DisplayName = "Remove FlexNetwork ZoneGraph"))
	void RemoveMassAIZoneGraph();

	UPROPERTY(VisibleAnywhere, Category = "Mass AI|ZoneGraph", meta = (MultiLine = true))
	FString ZoneGraphStatus = TEXT("No ZoneGraph has been generated by FlexNetwork in this editor session.");

	/** Flattens/blends the landscape under every Ground road to match the road's height (Bridge/Elevated/Tunnel/Ramp roads are left alone). Forces a re-apply even for roads that weren't just edited -- e.g. after hand-sculpting the landscape since the last change. */
	UFUNCTION(CallInEditor, Category = "Terrain", meta = (DisplayName = "Conform Terrain To Roads"))
	void ConformTerrainToRoads();

	/** Samples the landscape's existing surface under every Ground node and drapes the network onto it (Bridge/Elevated/Tunnel/Ramp nodes are left alone, since their elevation is deliberately offset from ground). The reverse of "Conform Terrain To Roads" -- use this to make roads follow the terrain instead of the terrain following the roads. */
	UFUNCTION(CallInEditor, Category = "Terrain", meta = (DisplayName = "Fit Roads To Terrain"))
	void FitRoadsToTerrain();

	/** Applied to every URoadTypeProfile asset in the project by "Apply Materials To All Profiles" below; leave any of these unset to leave that material slot untouched on every profile. */
	UPROPERTY(EditAnywhere, Category = "Materials")
	TObjectPtr<UMaterialInterface> StandardRoadMaterial;

	UPROPERTY(EditAnywhere, Category = "Materials")
	TObjectPtr<UMaterialInterface> StandardSidewalkMaterial;

	UPROPERTY(EditAnywhere, Category = "Materials")
	TObjectPtr<UMaterialInterface> StandardCrosswalkMaterial;

	UPROPERTY(EditAnywhere, Category = "Materials")
	TObjectPtr<UMaterialInterface> StandardJunctionMaterial;

	UPROPERTY(EditAnywhere, Category = "Materials")
	TObjectPtr<UMaterialInterface> StandardMedianMaterial;

	/** Material for solid road-marking lines (e.g. between opposite-direction lanes). */
	UPROPERTY(EditAnywhere, Category = "Materials")
	TObjectPtr<UMaterialInterface> StandardSolidMarkingMaterial;

	/** Material for dashed lane-boundary markings. */
	UPROPERTY(EditAnywhere, Category = "Materials")
	TObjectPtr<UMaterialInterface> StandardLaneDashMarkingMaterial;

	/** Material for dashed guide lines through intersections. */
	UPROPERTY(EditAnywhere, Category = "Materials")
	TObjectPtr<UMaterialInterface> StandardIntersectionDashMarkingMaterial;

	/** Material for dashed crosswalk border markings. */
	UPROPERTY(EditAnywhere, Category = "Materials")
	TObjectPtr<UMaterialInterface> StandardCrosswalkDashMarkingMaterial;

	/** Material for the bike-lane overlay strip drawn over every Bike-type lane. */
	UPROPERTY(EditAnywhere, Category = "Materials")
	TObjectPtr<UMaterialInterface> StandardBikeLaneMaterial;

	/** Material for the divider line generated between adjacent parking bays on every Parking-type lane. */
	UPROPERTY(EditAnywhere, Category = "Materials")
	TObjectPtr<UMaterialInterface> StandardParkingMarkingMaterial;

	/** Material for the road-surface overlay drawn over every Parking-type lane. */
	UPROPERTY(EditAnywhere, Category = "Materials")
	TObjectPtr<UMaterialInterface> StandardParkingLaneMaterial;

	/** Overwrites the material slots above (whichever ones aren't left unset) on every URoadTypeProfile asset in the project and saves them -- a one-click alternative to hand-editing dozens of auto-generated OSM-import profiles individually. */
	UFUNCTION(CallInEditor, Category = "Materials", meta = (DisplayName = "Apply Materials To All Profiles"))
	void ApplyMaterialsToAllProfiles();

	/** Static mesh spline-fit along every curb line (both edges of each Ground road with a sidewalk, plus each junction's own curb-return boundary) by "Generate Curbstones" below -- authored with its long axis as local X, since that's the axis the spline is fit along. */
	UPROPERTY(EditAnywhere, Category = "Curbstones")
	TObjectPtr<UStaticMesh> CurbstoneMesh;

	/** Generates the configured curbstones once when a hand-drawn road chain is completed with right-click/Escape. Intermediate placement clicks remain lightweight. */
	UPROPERTY(EditAnywhere, Category = "Curbstones")
	bool bGenerateCurbstonesOnPlacementComplete = true;

	/** Replaces every previously-generated curbstone with a fresh set spline-fit along the current road/junction curb lines using CurbstoneMesh. */
	UFUNCTION(CallInEditor, Category = "Curbstones", meta = (DisplayName = "Generate Curbstones"))
	void GenerateCurbstones();

	/** Destroys every actor a previous call spawned, then re-spawns one instance of each FRoadLaneDescriptor::LaneActors entry along every matching lane, graph-wide. See UFlexNetworkSubsystem::GenerateLaneActors. */
	UFUNCTION(CallInEditor, Category = "Lane Actors", meta = (DisplayName = "Generate Lane Actors"))
	void GenerateLaneActors();

	UPROPERTY(VisibleAnywhere, Category = "Lane Actors", meta = (MultiLine = true))
	FString LaneActorGenerationStatus = TEXT("No lane actors have been generated in this editor session.");

	/**
	 * Fetches LGL-BW aerial (and, if UFlexSatelliteImagerySettings::bFetchLandUseOverlay is on,
	 * land-use) imagery covering OsmAsset's own extent and spawns one flat preview tile per fetched
	 * square (see AFlexSatelliteTileActor) -- reuses OsmAsset/OsmImportSettings above as-is, so the
	 * imagery lands on the exact same projection origin the road and rail OSM commands use for
	 * the same asset/settings, and lines up with generated transport geometry. Project-wide service URLs,
	 * layer names, and tiling parameters live in Project Settings under "Flex Network - Satellite
	 * Imagery" (UFlexSatelliteImagerySettings). Replaces any previously-spawned satellite tiles.
	 */
	UFUNCTION(CallInEditor, Category = "Satellite Imagery", meta = (DisplayName = "Import Satellite/Landuse Imagery"))
	void ImportSatelliteImagery();

	/**
	 * Saves every tile spawned by "Import Satellite/Landuse Imagery" above as permanent,
	 * landscape-ready UTexture2D assets under UFlexSatelliteImagerySettings::BakePackagePath (in a
	 * subfolder named after OsmAsset), and -- if UFlexSatelliteImagerySettings::BaseLandscapeMaterial
	 * is set -- a UMaterialInstanceConstant per tile with those textures wired up, ready to assign
	 * directly to a Landscape's Material slot.
	 */
	UFUNCTION(CallInEditor, Category = "Satellite Imagery", meta = (DisplayName = "Bake Imagery To Content"))
	void BakeSatelliteImageryToContent();

	/** Tiles spawned by the most recent "Import Satellite/Landuse Imagery" run -- what "Bake Imagery To Content" reads back. Not exposed to the details panel; the level's own outliner is the place to inspect/delete them. */
	TArray<TWeakObjectPtr<AFlexSatelliteTileActor>> SpawnedSatelliteTiles;

	/** Set by FFlexNetworkEdMode when it creates/fetches this settings object, so the CallInEditor buttons above have a world to act on. */
	TWeakObjectPtr<UWorld> TargetWorld;

private:
	/** Shared implementation for the separate road and railway toolkit commands. */
	void GenerateTransportFromOsm(bool bIncludeRoads, bool bIncludeRailways);
};
