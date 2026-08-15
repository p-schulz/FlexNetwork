#pragma once

#include "CoreMinimal.h"
#include "EdMode.h"
#include "FlexNetworkTypes.h"
#include "FlexCurveTypes.h"

class UFlexNetworkSubsystem;
class UFlexNetworkEdModeSettings;

extern const FEditorModeID FlexNetworkEdModeId;

/**
 * Interactive click-drag road drawing tool. Legacy FEdMode rather than the modern UEdMode/
 * Interactive-Tools-Framework stack -- see FlexNetworkEditor.Build.cs for why. Holds no
 * algorithmic logic of its own: every drag ends by calling into UFlexNetworkSubsystem, which is
 * the only place curve math/snapping-resolution/graph mutation actually happens.
 */
class FFlexNetworkEdMode : public FEdMode
{
public:
	FFlexNetworkEdMode();
	virtual ~FFlexNetworkEdMode() override;

	// FEdMode
	virtual void Enter() override;
	virtual void Exit() override;
	virtual bool MouseMove(FEditorViewportClient* ViewportClient, FViewport* Viewport, int32 X, int32 Y) override;
	virtual bool InputKey(FEditorViewportClient* ViewportClient, FViewport* Viewport, FKey Key, EInputEvent Event) override;
	virtual void Render(const FSceneView* View, FViewport* Viewport, FPrimitiveDrawInterface* PDI) override;
	virtual bool UsesToolkits() const override { return true; }
	virtual void AddReferencedObjects(FReferenceCollector& Collector) override;
	virtual FString GetReferencerName() const override { return TEXT("FFlexNetworkEdMode"); }

	/** Public so the toolkit can bind a details view to it. */
	UFlexNetworkEdModeSettings* GetOrCreateModeSettings() const;

private:
	enum class EDragState : uint8
	{
		Idle,
		Dragging
	};

	EDragState DragState = EDragState::Idle;

	FVector HoverWorldPoint = FVector::ZeroVector;
	bool bHoverValid = false;
	FFlexNodeId HoverNodeId;			// Snap candidate under the cursor right now (Idle) or drag start (Dragging).
	FFlexSegmentId HoverSegmentId;		// Segment-midspan snap candidate under the cursor, if HoverNodeId is invalid.
	float HoverSegmentArcLength = 0.f;

	FVector DragStartPoint = FVector::ZeroVector;
	FFlexNodeId DragStartNodeId;
	FFlexSegmentId DragStartSegmentId;	// Valid if the drag started by snapping onto a segment's midspan.
	float DragStartSegmentArcLength = 0.f;

	FFlexBezierCurve PreviewCurve;
	bool bPreviewValid = false;
	FText PreviewInvalidReason;

	FFlexNodeId SelectedNodeId; // For the "click an existing node to edit its elevation" flow.

	mutable TObjectPtr<UFlexNetworkEdModeSettings> ModeSettings = nullptr;

	UFlexNetworkSubsystem* GetSubsystem() const;

	bool ComputeGroundPlanePoint(FEditorViewportClient* ViewportClient, FViewport* Viewport, int32 X, int32 Y, FVector& OutPoint) const;
	void UpdateHover(const FVector& WorldPoint);
	FVector ApplyAngleSnap(const FVector& From, const FVector& To) const;
	void UpdatePreviewCurve();
	void BeginDrag();
	void CommitDrag();
	void CancelDrag();

	/** Resolves a world-space endpoint to a graph node: snaps to an existing node, splits an existing segment's midspan and uses the new node, or creates a fresh node -- in that priority order. */
	FFlexNodeId ResolveEndpointNode(const FVector& WorldPoint, FFlexNodeId SnapNodeId, FFlexSegmentId SnapSegmentId, float SnapSegmentArcLength);

	/** Shifts Curve's start (bStart) or end endpoint+handle so it matches NodeId's actual position exactly, preserving the handle's offset. */
	void ReconcileCurveEndpoint(FFlexBezierCurve& Curve, bool bStart, FFlexNodeId NodeId) const;
};
