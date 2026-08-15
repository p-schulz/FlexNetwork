#pragma once

#include "CoreMinimal.h"
#include "FlexNetworkTypes.h"
#include "Math/FlexRotationMinimizingFrame.h"

/**
 * Pluggable terrain integration: given a segment's sampled path frames, flatten/blend the
 * underlying terrain to the road's height with a falloff at the lateral edges. A plain
 * (non-UObject) interface rather than a UInterface -- there's exactly one call site (the
 * subsystem) and no need for Blueprint-implementable terrain conformers, so the extra UObject
 * ceremony wouldn't earn its keep. Kept separate from URoadTypeProfile/UFlexNetworkSubsystem
 * specifically so a project without Landscape (e.g. a custom terrain system) can supply its own
 * implementation instead of FFlexLandscapeConformer.
 */
class FLEXNETWORKRUNTIME_API IFlexTerrainConformer
{
public:
	virtual ~IFlexTerrainConformer() = default;

	/**
	 * Flattens terrain under one segment to the road's height, blending back to the original
	 * terrain over FalloffDistance beyond RoadHalfWidth + Margin on each side. Frames are the
	 * same rotation-minimizing frames used to build that segment's visible mesh, so the
	 * conformed strip exactly follows the road surface.
	 */
	virtual void ConformSegment(UWorld* World, FFlexSegmentId SegmentId, const TArray<FFlexCurveFrame>& Frames, float RoadHalfWidth, float Margin, float FalloffDistance) = 0;

	/** Restores terrain under a segment to what it was before ConformSegment was last called for it (segment deleted, moved, or no longer needs conforming). */
	virtual void RemoveSegmentConforming(UWorld* World, FFlexSegmentId SegmentId) = 0;
};
