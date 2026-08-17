#pragma once

#include "CoreMinimal.h"
#include "UObject/Object.h"
#include "FlexNetworkTypes.h"
#include "Osm/FlexOsmImportSettings.h"
#include "FlexNetworkEdModeSettings.generated.h"

class URoadTypeProfile;
class UOsmDataAsset;
class UMaterialInterface;
class UStaticMesh;
class AFlexSatelliteTileActor;

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
	/**
	 * When on: click once to place/continue a road's start point, move the mouse to preview,
	 * click again to commit the segment (like the Landscape Splines "Add Control Point" tool).
	 * When off ("Select" mode): click an existing node to select it, then drag the viewport
	 * gizmo to move it.
	 */
	UPROPERTY(EditAnywhere, Category = "FlexNetwork", meta = (DisplayName = "Draw Mode (off = Select/Move)"))
	bool bDrawModeActive = true;

	/** Road type profile new segments are drawn with. */
	UPROPERTY(EditAnywhere, Category = "FlexNetwork")
	TObjectPtr<URoadTypeProfile> ActiveProfile;

	/** Elevation type applied to newly-created nodes/segments. */
	UPROPERTY(EditAnywhere, Category = "FlexNetwork")
	EFlexRoadElevationType ActiveElevationType = EFlexRoadElevationType::Ground;

	/** Hold to disable 15-degree angle snapping while dragging. */
	UPROPERTY(EditAnywhere, Category = "FlexNetwork")
	bool bAngleSnapEnabled = true;

	/** Parsed OSM data (import a .osm file via Content Browser > Import first) to generate roads from. */
	UPROPERTY(EditAnywhere, Category = "OSM Import")
	TObjectPtr<UOsmDataAsset> OsmAsset;

	UPROPERTY(EditAnywhere, Category = "OSM Import", meta = (ShowOnlyInnerProperties))
	FFlexOsmImportSettings OsmImportSettings;

	/** Filters OsmAsset's ways by highway tag, projects to world space, merges nearby junction nodes, and builds matching FlexNetwork roads + auto-generated lane profiles. See FFlexOsmGraphBuilder. */
	UFUNCTION(CallInEditor, Category = "OSM Import", meta = (DisplayName = "Generate Roads From OSM"))
	void GenerateRoadsFromOsm();

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
	TObjectPtr<UMaterialInterface> StandardJunctionMaterial;

	UPROPERTY(EditAnywhere, Category = "Materials")
	TObjectPtr<UMaterialInterface> StandardMedianMaterial;

	/** Overwrites the material slots above (whichever ones aren't left unset) on every URoadTypeProfile asset in the project and saves them -- a one-click alternative to hand-editing dozens of auto-generated OSM-import profiles individually. */
	UFUNCTION(CallInEditor, Category = "Materials", meta = (DisplayName = "Apply Materials To All Profiles"))
	void ApplyMaterialsToAllProfiles();

	/** Static mesh spline-fit along every curb line (both edges of each Ground road with a sidewalk, plus each junction's own curb-return boundary) by "Generate Curbstones" below -- authored with its long axis as local X, since that's the axis the spline is fit along. */
	UPROPERTY(EditAnywhere, Category = "Curbstones")
	TObjectPtr<UStaticMesh> CurbstoneMesh;

	/** Replaces every previously-generated curbstone with a fresh set spline-fit along the current road/junction curb lines using CurbstoneMesh. A manual step (not run automatically on every edit) since it can place a lot of components for a large network. */
	UFUNCTION(CallInEditor, Category = "Curbstones", meta = (DisplayName = "Generate Curbstones"))
	void GenerateCurbstones();

	/**
	 * Fetches LGL-BW aerial (and, if UFlexSatelliteImagerySettings::bFetchLandUseOverlay is on,
	 * land-use) imagery covering OsmAsset's own extent and spawns one flat preview tile per fetched
	 * square (see AFlexSatelliteTileActor) -- reuses OsmAsset/OsmImportSettings above as-is, so the
	 * imagery lands on the exact same projection origin "Generate Roads From OSM" would use for the
	 * same asset/settings, and lines up with roads generated from it. Project-wide service URLs,
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
};
