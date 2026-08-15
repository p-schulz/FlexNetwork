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
