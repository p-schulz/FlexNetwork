#pragma once

#include "CoreMinimal.h"

struct FFlexLGLImageryLayerPreset;
class UTexture2D;

namespace FlexNetwork::Satellite
{
	/**
	 * Fetches one square LGL-BW WMS tile (Layer) centered on (CenterLat, CenterLon) -- a
	 * (2*TileRadiusM) meter square, per ResolutionM/MaxPixelsPerSide -- decodes it into a transient
	 * UTexture2D, and invokes OnComplete (on the game thread, exactly once) with the texture and
	 * the tile's LOCAL world-space bounds in centimeters, projected with the exact same
	 * FFlexOsmGraphBuilder::ProjectLatLonToLocalCm formula (and OriginLat/OriginLon) used to place
	 * FlexNetwork's own OSM-imported roads, so a tile lines up with any road network imported from
	 * the same .osm file/origin. Call this once per layer you want for the same tile (e.g. once for
	 * UFlexSatelliteImagerySettings::AerialLayer, again for LandUseLayer) -- passing the SAME
	 * TileRadiusM/ResolutionM/MaxPixelsPerSide/CenterLat/CenterLon/Origin each time guarantees both
	 * layers' textures cover the exact same bbox at the same pixel size, so they line up for
	 * blending. On failure (network error, non-image response, decode failure, or the point being
	 * outside LGL-BW's Baden-Wuerttemberg coverage), Texture is nullptr and Error is non-empty.
	 */
	FLEXNETWORKEDITOR_API void FetchLGLImageryTileAsync(
		const FFlexLGLImageryLayerPreset& Layer,
		double TileRadiusM, double ResolutionM, int32 MaxPixelsPerSide,
		double CenterLat, double CenterLon,
		double OriginLat, double OriginLon,
		TFunction<void(UTexture2D* Texture, double SouthCm, double NorthCm, double WestCm, double EastCm, const FString& Error)> OnComplete);
}
