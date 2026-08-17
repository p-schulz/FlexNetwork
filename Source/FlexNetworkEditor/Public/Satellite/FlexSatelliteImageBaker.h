#pragma once

#include "CoreMinimal.h"

class UTexture2D;
class UTexture;
class UMaterialInterface;
class UMaterialInstanceConstant;

namespace FlexNetwork::Satellite
{
	/**
	 * Saves a transient texture (e.g. AFlexSatelliteTileActor::AerialTexture/LandUseTexture, as
	 * produced by FetchLGLImageryTileAsync) as a permanent .uasset UTexture2D under
	 * PackagePath/AssetName, with settings suited to sampling it in a landscape material
	 * (TC_Default compression, TEXTUREGROUP_World, mips regenerated from the full-res source). Only
	 * SourceTexture's pixel data is read (via its platform data's mip 0) -- the object itself stays
	 * transient/unsaved. Returns nullptr if SourceTexture has no readable pixel data, or if the
	 * package couldn't be created/saved.
	 *
	 * ResizeToSquareSize, if > 0, bilinearly resizes the pixel data to a ResizeToSquareSize x
	 * ResizeToSquareSize square before saving -- the WMS-fetched native resolution is essentially
	 * never a power of two, which full mip-chain generation and most GPU compression formats want.
	 * Rounded up to the nearest power of two regardless of the value passed in, so an odd size like
	 * 2000 still produces a valid 2048 texture.
	 */
	FLEXNETWORKEDITOR_API UTexture2D* BakeTextureToContent(UTexture2D* SourceTexture, const FString& PackagePath, const FString& AssetName, bool bSRGB, int32 ResizeToSquareSize = 0);

	/**
	 * Creates (overwriting if one already exists at that path) a UMaterialInstanceConstant parented
	 * to BaseMaterial under PackagePath/AssetName, with TextureParameters/ScalarParameters set --
	 * ready to assign directly to a Landscape's Material slot (BaseMaterial should be authored to
	 * read its texture parameters via a Landscape-Coords/world-aligned UV node, not a mesh UV
	 * channel, since a Landscape has no UV0 of its own to sample with). Returns nullptr if
	 * BaseMaterial is null or the package couldn't be created/saved.
	 */
	FLEXNETWORKEDITOR_API UMaterialInstanceConstant* CreateLandscapeMaterialInstance(
		UMaterialInterface* BaseMaterial, const FString& PackagePath, const FString& AssetName,
		const TMap<FName, UTexture*>& TextureParameters, const TMap<FName, float>& ScalarParameters);
}
