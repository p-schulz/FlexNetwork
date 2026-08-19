#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "FlexSatelliteTileActor.generated.h"

class UProceduralMeshComponent;
class UTexture2D;
class UMaterialInterface;
struct FFlexLGLImageryLayerPreset;

/**
 * A single flat imagery tile: a ground-plane quad (Z=0, matching FlexNetwork's own Ground-road
 * datum) spanning world X=[SouthCm,NorthCm], Y=[WestCm,EastCm] in centimeters, textured with
 * one or two fetched LGL-BW layers via a dynamic material instance. Preview-only (placed in the
 * level so satellite/land-use imagery can be seen against the imported road network in-editor) --
 * "Bake Imagery To Content" (FlexSatelliteImageBaker.h) is what turns the underlying textures into
 * permanent, landscape-usable assets; this actor and its transient textures are not meant to be
 * shipped or saved with the level as-is.
 */
UCLASS(NotBlueprintable)
class FLEXNETWORKEDITOR_API AFlexSatelliteTileActor : public AActor
{
	GENERATED_BODY()

public:
	AFlexSatelliteTileActor();

	/**
	 * Builds the quad and material for this tile. AerialTexture/AerialLayer are required;
	 * LandUseTexture/LandUseLayer may be null/default if land-use overlay wasn't fetched.
	 * TileMaterial is instanced (MID) with AerialLayer.TextureParameterName/LandUseLayer's set to
	 * the textures and LandUseOpacityParameterName set to LandUseOpacity -- left null, the quad
	 * gets the engine default material instead.
	 */
	void Initialize(
		double SouthCm, double NorthCm, double WestCm, double EastCm,
		UTexture2D* AerialTexture, const FFlexLGLImageryLayerPreset& AerialLayer,
		UTexture2D* LandUseTexture, const FFlexLGLImageryLayerPreset& LandUseLayer,
		double LandUseOpacity, FName LandUseOpacityParameterName,
		UMaterialInterface* TileMaterial);

	/** The aerial texture this tile was built from -- read back by "Bake Imagery To Content". */
	UPROPERTY(Transient)
	TObjectPtr<UTexture2D> AerialTexture;

	/** The land-use texture this tile was built from, if any -- read back by "Bake Imagery To Content". */
	UPROPERTY(Transient)
	TObjectPtr<UTexture2D> LandUseTexture;

	/**
	 * The material instance dynamic actually applied to the quad's mesh section -- created from
	 * TileMaterial by Initialize (or the engine default surface material, if TileMaterial was null
	 * or failed to load). Exposed (VisibleAnywhere) so a spawned tile's material can be inspected
	 * directly on the actor in the Details panel, without having to dig into MeshComponent.
	 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "FlexNetwork", Transient)
	TObjectPtr<UMaterialInterface> AppliedMaterial;

	/** The component the tile's quad and material live on. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "FlexNetwork")
	TObjectPtr<UProceduralMeshComponent> MeshComponent;
};
