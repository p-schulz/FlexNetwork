#pragma once

#include "CoreMinimal.h"
#include "Engine/DeveloperSettings.h"
#include "Engine/EngineTypes.h"
#include "UObject/SoftObjectPath.h"
#include "FlexSatelliteImagerySettings.generated.h"

class UMaterialInterface;

/**
 * One WMS GetMap layer to fetch as a per-tile texture -- LGL-BW (Landesamt fuer Geoinformation und
 * Landentwicklung Baden-Wuerttemberg) publishes both an aerial-photo layer and an official
 * land-use-polygon layer through the same WMS shape, so both fit this one preset struct.
 * LayerName is inserted into the request URL as-is (not URL-encoded) -- for a layer name
 * containing characters like ':' (e.g. GeoServer's "nora:Landnutzung"), pre-encode it yourself
 * ("nora%3ALandnutzung") rather than relying on this code to encode it for you.
 */
USTRUCT()
struct FFlexLGLImageryLayerPreset
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Category = "LGL-BW Service")
	FString ServiceUrl;

	UPROPERTY(EditAnywhere, Category = "LGL-BW Service")
	FString LayerName;

	// WMS STYLES parameter -- most layers (e.g. the aerial photo) use the server's default (leave
	// empty); a thematic layer like land-use polygons typically needs a specific fill style to
	// render as anything other than blank.
	UPROPERTY(EditAnywhere, Category = "LGL-BW Service")
	FString Style;

	// Requests TRANSPARENT=true -- needed for a layer (like land-use polygons) that doesn't cover
	// every pixel, so the gaps come back alpha=0 instead of an opaque background fill.
	UPROPERTY(EditAnywhere, Category = "LGL-BW Service")
	bool bTransparent = false;

	// Parameter name this layer's downloaded texture is assigned to on a spawned tile's dynamic
	// material instance (see FlexSpawnSatelliteTileActor) and, when baked, on the generated
	// landscape-ready material instance (see FlexNetwork::Satellite::BakeTileToContent).
	UPROPERTY(EditAnywhere, Category = "Materials")
	FName TextureParameterName;
};

/**
 * Project-wide configuration for LGL Baden-Wuerttemberg satellite/land-use imagery fetching (see
 * Satellite/FlexSatelliteImport.h). LGL-BW's WMS (https://owsproxy.lgl-bw.de) is Open Data
 * (Datenlizenz Deutschland - Namensnennung 2.0 -- attribution required: "(c) LGL
 * Baden-Wuerttemberg (www.lgl-bw.de), Datenlizenz Deutschland - Namensnennung 2.0"), and only
 * covers Baden-Wuerttemberg (roughly lon 7.2-10.7, lat 47.4-50.0) -- requests for an area outside
 * that will fail per-tile, not silently return blank imagery.
 *
 * The service is a plain WMS with no fixed tile grid of its own, so "one tile" here is this
 * plugin's own choice of a (2*TileRadiusM) meter square per WMS GetMap request. Both AerialLayer
 * and LandUseLayer (when enabled) are fetched for the SAME bbox/pixel size per tile, so their
 * texture pixels line up exactly for blending. The service's GetMap only returns imagery for
 * CRS=EPSG:3857 (Web Mercator) -- EPSG:4326 requests come back blank -- so every fetch goes
 * through Web Mercator regardless of what other CRS the capabilities document advertises; this is
 * purely a wire-format detail, converted back to lat/lon and then to the same local
 * FFlexOsmGraphBuilder::ProjectLatLonToLocalCm projection every OSM-imported road uses before it
 * ever reaches Unreal world space, so imagery tiles and OSM roads sharing the same origin line up
 * exactly.
 */
UCLASS(config = Game, defaultconfig, meta = (DisplayName = "Flex Network - Satellite Imagery"))
class FLEXNETWORKEDITOR_API UFlexSatelliteImagerySettings : public UDeveloperSettings
{
	GENERATED_BODY()

public:
	UFlexSatelliteImagerySettings()
	{
		AerialLayer.ServiceUrl = TEXT("https://owsproxy.lgl-bw.de/owsproxy/ows/WMS_LGL-BW_ATKIS_DOP_20_C");
		AerialLayer.LayerName = TEXT("IMAGES_DOP_20_RGB");
		// Parameter names match ProceduralRoads' own M_Satellite exactly -- see PreviewTileMaterial's
		// default below -- not a FlexNetwork-authored naming choice.
		AerialLayer.TextureParameterName = TEXT("TileTexture");

		// ATKIS Basis-DLM land use (residential/industrial/agriculture/forestry/road-traffic/etc.),
		// rendered with its own fill-color style -- an independent classification layer, not another
		// aerial photo, meant to be alpha-blended on top of AerialLayer (see LandUseOpacity).
		LandUseLayer.ServiceUrl = TEXT("https://owsproxy.lgl-bw.de/owsproxy/ows/WMS_LGL-BW_Landnutzung");
		LandUseLayer.LayerName = TEXT("nora%3ALandnutzung");
		LandUseLayer.Style = TEXT("ln_landnutzng_f");
		LandUseLayer.bTransparent = true;
		LandUseLayer.TextureParameterName = TEXT("LanduseTexture");

		// Defaults straight to ProceduralRoads' own already-authored satellite material rather than a
		// FlexNetwork-owned duplicate -- same asset, zero extra setup, one less thing to keep in
		// sync. Only resolves if the ProceduralRoads plugin/content is present; otherwise this
		// TSoftObjectPtr just stays unresolved (nullptr), same as any default pointing at a path that
		// doesn't exist -- reassign PreviewTileMaterial here in Project Settings to something else if
		// ProceduralRoads isn't enabled in a given project.
		PreviewTileMaterial = TSoftObjectPtr<UMaterialInterface>(FSoftObjectPath(TEXT("/ProceduralRoads/Material/M_Satellite.M_Satellite")));
	}

	UPROPERTY(EditAnywhere, config, Category = "Layers")
	FFlexLGLImageryLayerPreset AerialLayer;

	// Fetched and blended in ADDITION to AerialLayer when bFetchLandUseOverlay is true -- one extra
	// WMS request per tile.
	UPROPERTY(EditAnywhere, config, Category = "Layers")
	FFlexLGLImageryLayerPreset LandUseLayer;

	UPROPERTY(EditAnywhere, config, Category = "Layers")
	bool bFetchLandUseOverlay = false;

	// Scalar material parameter controlling how strongly LandUseLayer's own alpha blends over
	// AerialLayer (TileMaterial/BakedLandscapeMaterial are expected to multiply LandUseTexture's
	// alpha by this before using it as a Lerp factor).
	UPROPERTY(EditAnywhere, config, Category = "Layers", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	double LandUseOpacity = 160.0 / 255.0;

	UPROPERTY(EditAnywhere, config, Category = "Materials")
	FName LandUseOpacityParameterName = TEXT("LandUseOpacity");

	// Half-side length of one fetched tile, in meters.
	UPROPERTY(EditAnywhere, config, Category = "Tiling", meta = (ClampMin = "10.0"))
	double TileRadiusM = 150.0;

	// Target ground resolution, meters/pixel -- DOP20 imagery is native 20cm; requesting finer than
	// that just upsamples, it doesn't add real detail.
	UPROPERTY(EditAnywhere, config, Category = "Tiling", meta = (ClampMin = "0.01"))
	double ResolutionM = 0.2;

	// Upper bound on one WMS request's image side, in pixels -- keeps individual requests well under
	// typical WMS server request-size limits. A TileRadiusM/ResolutionM combination that would exceed
	// this is silently capped (the fetched tile is still the requested ground size, just coarser).
	UPROPERTY(EditAnywhere, config, Category = "Tiling", meta = (ClampMin = "64", ClampMax = "4096"))
	int32 MaxPixelsPerSide = 2500;

	// Applied to every spawned tile plane in the level (preview only -- see FlexSatelliteTileActor).
	// Must expose AerialLayer.TextureParameterName (RGB), and -- if bFetchLandUseOverlay is ever
	// enabled -- LandUseLayer.TextureParameterName (RGBA) and LandUseOpacityParameterName (scalar),
	// blended together as roughly
	// Lerp(AerialTexture.RGB, LandUseTexture.RGB, LandUseTexture.A * LandUseOpacity). Left unset
	// (None), tiles spawn with the engine default material.
	UPROPERTY(EditAnywhere, config, Category = "Materials")
	TSoftObjectPtr<UMaterialInterface> PreviewTileMaterial;

	// Base material "Bake Imagery To Content" instances per baked tile (creating a
	// UMaterialInstanceConstant with the baked textures wired to the same parameter names as
	// PreviewTileMaterial above), ready to assign directly as a Landscape's material. Should be
	// authored to sample its texture parameters using the Landscape's own world-aligned/Landscape
	// Coords UV node rather than a mesh UV channel, since a baked instance is meant to be dropped
	// straight onto ALandscape::LandscapeMaterial. Left unset, baking only saves the textures
	// themselves (still directly usable as texture parameters in a hand-authored landscape
	// material).
	UPROPERTY(EditAnywhere, config, Category = "Materials")
	TSoftObjectPtr<UMaterialInterface> BaseLandscapeMaterial;

	// Content-browser path (e.g. "/Game/FlexNetwork/Satellite") new textures/material instances are
	// saved under by "Bake Imagery To Content" -- one subfolder per import, named from the source
	// .osm asset, is created under this.
	UPROPERTY(EditAnywhere, config, Category = "Save")
	FDirectoryPath BakePackagePath = { TEXT("/Game/FlexNetwork/Satellite") };

	// sRGB is correct for a normal photographic aerial layer (AerialLayer) but usually wrong for a
	// flat-fill classification layer like LandUseLayer, whose "colors" are really discrete category
	// IDs a material should read back linearly -- kept as a separate toggle per layer rather than
	// one shared flag.
	UPROPERTY(EditAnywhere, config, Category = "Save")
	bool bAerialTextureSRGB = true;

	UPROPERTY(EditAnywhere, config, Category = "Save")
	bool bLandUseTextureSRGB = false;

	// The WMS-fetched native resolution (2*TileRadiusM/ResolutionM, clamped by MaxPixelsPerSide) is
	// essentially never a power of two, which full mip-chain generation and most GPU compression
	// formats want. When on, "Bake Imagery To Content" resizes (bilinear) each baked texture up or
	// down to a square BakeTextureSize before saving.
	UPROPERTY(EditAnywhere, config, Category = "Save")
	bool bResizeToPowerOfTwo = false;

	// Rounded up to the nearest power of two at bake time regardless of what's typed here, so an
	// odd value like 2000 still produces a valid 2048 texture rather than silently misbehaving.
	UPROPERTY(EditAnywhere, config, Category = "Save", meta = (EditCondition = "bResizeToPowerOfTwo", ClampMin = "4", ClampMax = "8192"))
	int32 BakeTextureSize = 2048;
};
