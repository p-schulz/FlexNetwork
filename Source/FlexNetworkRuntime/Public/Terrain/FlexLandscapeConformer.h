#pragma once

#include "CoreMinimal.h"
#include "Terrain/IFlexTerrainConformer.h"

class ALandscape;

/**
 * Default IFlexTerrainConformer: flattens/blends an ALandscape's heightmap under a road strip.
 *
 * Implementation note / deviation from the spec's literal ask: the spec calls for painting into
 * a dedicated non-destructive Landscape Edit Layer (so flattening is reversible/layered rather
 * than a permanent heightmap edit). UE5.8's edit-layer identity API is mid-refactor in this
 * engine build -- FLandscapeLayer's Guid/Name fields are marked _DEPRECATED in
 * Landscape.h in favor of a newer UObject-based ULandscapeEditLayerBase system, and guessing the
 * replacement create/target call sequence without being able to compile-test against a live
 * editor session risks silently-wrong behavior. Given that, this class instead edits the
 * landscape's active heightmap directly via FLandscapeEditDataInterface, but records each
 * segment's pre-edit heights before touching them so RemoveSegmentConforming can restore the
 * original terrain exactly -- giving the same practical reversibility the edit-layer approach
 * would, without depending on the part of the API that's in flux. Swapping in true edit-layer
 * targeting (FScopedSetLandscapeEditingLayer + a dedicated layer) later is a localized change
 * confined to this one class.
 */
class FLEXNETWORKRUNTIME_API FFlexLandscapeConformer : public IFlexTerrainConformer
{
public:
	virtual void ConformSegment(UWorld* World, FFlexSegmentId SegmentId, const TArray<FFlexCurveFrame>& Frames, float RoadHalfWidth, float Margin, float FalloffDistance) override;
	virtual void RemoveSegmentConforming(UWorld* World, FFlexSegmentId SegmentId) override;

private:
	struct FSavedHeightRegion
	{
		int32 MinX = 0;
		int32 MinY = 0;
		int32 MaxX = 0;
		int32 MaxY = 0;
		TArray<uint16> OriginalHeights;
	};

	TMap<FFlexSegmentId, FSavedHeightRegion> SavedRegionsBySegment;

	static ALandscape* FindLandscape(UWorld* World);

	/** Writes SavedRegionsBySegment's stored heights for SegmentId back to the landscape, then forgets the saved region. No-op if none saved. */
	void RestoreSavedRegion(UWorld* World, FFlexSegmentId SegmentId);
};
