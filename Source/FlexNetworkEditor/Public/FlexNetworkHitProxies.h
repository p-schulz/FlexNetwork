#pragma once

#include "CoreMinimal.h"
#include "HitProxies.h"
#include "FlexNetworkTypes.h"

/** Lets FFlexNetworkEdMode::HandleClick identify exactly which graph node a viewport click landed on, for selection + gizmo-drag movement. */
struct HFlexNodeHitProxy : public HHitProxy
{
	DECLARE_HIT_PROXY()

	FFlexNodeId NodeId;

	explicit HFlexNodeHitProxy(FFlexNodeId InNodeId)
		: HHitProxy(HPP_UI)
		, NodeId(InNodeId)
	{
	}
};

/** Lets FFlexNetworkEdMode::HandleClick identify exactly which graph segment a viewport click landed on, for selection (and delete) -- no gizmo movement, a segment has no single position to drag. */
struct HFlexSegmentHitProxy : public HHitProxy
{
	DECLARE_HIT_PROXY()

	FFlexSegmentId SegmentId;

	explicit HFlexSegmentHitProxy(FFlexSegmentId InSegmentId)
		: HHitProxy(HPP_UI)
		, SegmentId(InSegmentId)
	{
	}
};
