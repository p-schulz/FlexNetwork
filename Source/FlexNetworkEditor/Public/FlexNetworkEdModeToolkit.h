#pragma once

#include "CoreMinimal.h"
#include "Toolkits/BaseToolkit.h"

/**
 * Minimal toolkit for FFlexNetworkEdMode: a single details panel over UFlexNetworkEdModeSettings
 * (active road profile, elevation type, angle-snap toggle) plus a short instructions block. The
 * spec explicitly allows this UI to be functional rather than polished -- it exists to make the
 * runtime pipeline exercisable interactively, not as a finished authoring product.
 */
class FFlexNetworkEdModeToolkit : public FModeToolkit
{
public:
	FFlexNetworkEdModeToolkit();

	virtual void Init(const TSharedPtr<IToolkitHost>& InitToolkitHost) override;
	virtual FName GetToolkitFName() const override { return FName("FlexNetworkEdMode"); }
	virtual FText GetBaseToolkitName() const override;
	virtual class FEdMode* GetEditorMode() const override;
	virtual TSharedPtr<SWidget> GetInlineContent() const override { return ToolkitWidget; }

private:
	TSharedPtr<SWidget> ToolkitWidget;
};
