#pragma once

#include "CoreMinimal.h"
#include "EdMode.h"
#include "FlexNetworkTypes.h"
#include "FlexCurveTypes.h"

class UFlexNetworkSubsystem;
class UFlexNetworkEdModeSettings;
class FScopedTransaction;
enum class EFlexNetworkNodeEditTool : uint8;

extern const FEditorModeID FlexNetworkEdModeId;

/**
 * Interactive road authoring tool: a click-click drawing gesture (not click-drag -- see
 * UFlexNetworkEdModeSettings::bDrawModeActive) plus classic-gizmo node move/rotation and
 * per-segment endpoint-tangent editing, all
 * built on a real world raycast (landscape/mesh collision, with an ortho-camera-aware ray and a
 * flat-plane fallback for an empty level) rather than a flat-plane assumption. Legacy FEdMode
 * rather than the modern UEdMode/Interactive-Tools-Framework stack -- see
 * FlexNetworkEditor.Build.cs for why; the classic transform-widget hooks used here for node
 * dragging (ShouldDrawWidget/GetWidgetLocation/InputDelta/HandleClick) are exactly what every
 * other legacy mode (Landscape, BSP, ...) uses for the same purpose. Holds no algorithmic logic
 * of its own: every commit ends by calling into UFlexNetworkSubsystem, which is the only place
 * curve math/snapping-resolution/graph mutation actually happens.
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

	// Node-selection gizmos (FLegacyEdModeWidgetHelper hooks -- the same mechanism Landscape/BSP/
	// etc. use for classic translate/rotate widgets).
	virtual bool ShouldDrawWidget() const override;
	virtual FVector GetWidgetLocation() const override;
	virtual bool AllowWidgetMove() override;
	virtual bool UsesTransformWidget() const override;
	virtual bool UsesTransformWidget(UE::Widget::EWidgetMode CheckMode) const override;
	virtual bool HandleClick(FEditorViewportClient* InViewportClient, HHitProxy* HitProxy, const FViewportClick& Click) override;
	virtual bool StartTracking(FEditorViewportClient* InViewportClient, FViewport* InViewport) override;
	virtual bool EndTracking(FEditorViewportClient* InViewportClient, FViewport* InViewport) override;
	virtual bool InputDelta(FEditorViewportClient* InViewportClient, FViewport* InViewport, FVector& InDrag, FRotator& InRot, FVector& InScale) override;

	/** Public so the toolkit can bind a details view to it. */
	UFlexNetworkEdModeSettings* GetOrCreateModeSettings() const;

private:
	enum class EDrawState : uint8
	{
		Idle,
		Placing	// First point committed (click 1); previewing/waiting for the second click.
	};

	EDrawState DrawState = EDrawState::Idle;

	FVector HoverWorldPoint = FVector::ZeroVector;
	bool bHoverValid = false;
	FFlexNodeId HoverNodeId;			// Snap candidate under the cursor right now (Idle) or the draw's start point (Placing).
	FFlexSegmentId HoverSegmentId;		// Segment-midspan snap candidate under the cursor, if HoverNodeId is invalid.
	float HoverSegmentArcLength = 0.f;

	FVector DrawStartPoint = FVector::ZeroVector;
	FFlexNodeId DrawStartNodeId;
	FFlexSegmentId DrawStartSegmentId;	// Valid if the draw started by snapping onto a segment's midspan.
	float DrawStartSegmentArcLength = 0.f;

	FFlexBezierCurve PreviewCurve;
	bool bPreviewValid = false;
	bool bPlacementCommittedSegment = false;
	FText PreviewInvalidReason;

	FFlexNodeId SelectedNodeId; // Drives the transform-widget hooks for move/rotate/tangent editing.
	FFlexSegmentId SelectedSegmentId; // Click-select only -- a segment has no single position, so no gizmo movement for it.
	FFlexSegmentId SelectedTangentSegmentId;
	bool bSelectedTangentIsStart = false;
	TUniquePtr<FScopedTransaction> ActiveNodeEditTransaction;

	/** Deletes whichever of SelectedNodeId/SelectedSegmentId is currently valid (node takes priority, matching HandleClick's mutual exclusivity), wrapped in one undo transaction. */
	void DeleteSelection();

	mutable TObjectPtr<UFlexNetworkEdModeSettings> ModeSettings = nullptr;

	UFlexNetworkSubsystem* GetSubsystem() const;
	bool IsDrawModeActive() const;
	EFlexNetworkNodeEditTool GetNodeEditTool() const;
	/** Resolves the explicitly selected tangent, or the first connected endpoint handle as a fallback. */
	bool ResolveActiveTangentHandle(FFlexSegmentId& OutSegmentId, bool& bOutStartHandle, FVector& OutHandlePosition) const;

	/**
	 * Casts a real ray into the world (landscape/static-mesh collision on the editor visibility
	 * channel, ortho-camera-aware) to find the point under the cursor; falls back to a flat plane
	 * (through the draw's start point while placing, else world Z=0) if nothing is hit, so the
	 * tool still works in a completely empty level.
	 */
	bool TraceCursorToWorld(FEditorViewportClient* ViewportClient, FViewport* Viewport, int32 X, int32 Y, FVector& OutPoint) const;

	void UpdateHover(const FVector& WorldPoint);
	FVector ApplyAngleSnap(const FVector& From, const FVector& To) const;
	void UpdatePreviewCurve();
	void BeginPlacement();
	void CommitPlacement();
	void CancelPlacement();

	/** Resolves a world-space endpoint to a graph node: snaps to an existing node, splits an existing segment's midspan and uses the new node, or creates a fresh node -- in that priority order. */
	FFlexNodeId ResolveEndpointNode(const FVector& WorldPoint, FFlexNodeId SnapNodeId, FFlexSegmentId SnapSegmentId, float SnapSegmentArcLength);

	/** Shifts Curve's start (bStart) or end endpoint+handle so it matches NodeId's actual position exactly, preserving the handle's offset. */
	void ReconcileCurveEndpoint(FFlexBezierCurve& Curve, bool bStart, FFlexNodeId NodeId) const;
};
