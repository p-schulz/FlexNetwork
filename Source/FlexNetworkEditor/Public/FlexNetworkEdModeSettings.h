#pragma once

#include "CoreMinimal.h"
#include "UObject/Object.h"
#include "FlexNetworkTypes.h"
#include "FlexNetworkEdModeSettings.generated.h"

class URoadTypeProfile;

/**
 * Transient per-editor-session settings for the FlexNetwork drawing tool, shown in its toolkit
 * via a plain IDetailsView -- this is what "select the road type to draw" looks like in this
 * minimal editor UI.
 */
UCLASS(Transient)
class UFlexNetworkEdModeSettings : public UObject
{
	GENERATED_BODY()

public:
	/** Road type profile new segments are drawn with. */
	UPROPERTY(EditAnywhere, Category = "FlexNetwork")
	TObjectPtr<URoadTypeProfile> ActiveProfile;

	/** Elevation type applied to newly-created nodes/segments. */
	UPROPERTY(EditAnywhere, Category = "FlexNetwork")
	EFlexRoadElevationType ActiveElevationType = EFlexRoadElevationType::Ground;

	/** Hold to disable 15-degree angle snapping while dragging. */
	UPROPERTY(EditAnywhere, Category = "FlexNetwork")
	bool bAngleSnapEnabled = true;
};
