#include "FlexNetworkEdMode.h"
#include "FlexNetworkEdModeSettings.h"
#include "FlexNetworkEdModeToolkit.h"
#include "FlexNetworkHitProxies.h"
#include "FlexNetworkSubsystem.h"
#include "FlexNetworkSettings.h"
#include "RoadTypeProfile.h"
#include "Math/FlexBezierMath.h"
#include "EditorViewportClient.h"
#include "SceneView.h"
#include "SceneManagement.h"
#include "ScopedTransaction.h"
#include "Misc/ScopeExit.h"
#include "Engine/World.h"
#include "Engine/HitResult.h"
#include "Engine/EngineTypes.h"
#include "CollisionQueryParams.h"
#include "EngineDefines.h"
#include "Toolkits/ToolkitManager.h"
#include "EditorModeManager.h"

const FEditorModeID FlexNetworkEdModeId = TEXT("FlexNetworkEdMode");

FFlexNetworkEdMode::FFlexNetworkEdMode()
{
}

FFlexNetworkEdMode::~FFlexNetworkEdMode()
{
}

void FFlexNetworkEdMode::Enter()
{
	FEdMode::Enter();
	DrawState = EDrawState::Idle;

	if (!Toolkit.IsValid() && UsesToolkits())
	{
		Toolkit = MakeShareable(new FFlexNetworkEdModeToolkit);
		Toolkit->Init(Owner->GetToolkitHost());
	}
	if (Owner)
	{
		Owner->SetWidgetMode(GetNodeEditTool() == EFlexNetworkNodeEditTool::Rotate
			? UE::Widget::WM_Rotate
			: UE::Widget::WM_Translate);
	}
}

void FFlexNetworkEdMode::Exit()
{
	CancelPlacement();
	ActiveNodeEditTransaction.Reset();

	if (Toolkit.IsValid())
	{
		FToolkitManager::Get().CloseToolkit(Toolkit.ToSharedRef());
		Toolkit.Reset();
	}

	FEdMode::Exit();
}

void FFlexNetworkEdMode::AddReferencedObjects(FReferenceCollector& Collector)
{
	FEdMode::AddReferencedObjects(Collector);
	Collector.AddReferencedObject(ModeSettings);
}

UFlexNetworkSubsystem* FFlexNetworkEdMode::GetSubsystem() const
{
	if (UWorld* World = GetWorld())
	{
		return World->GetSubsystem<UFlexNetworkSubsystem>();
	}
	return nullptr;
}

UFlexNetworkEdModeSettings* FFlexNetworkEdMode::GetOrCreateModeSettings() const
{
	const UWorld* PreviousWorld = ModeSettings ? ModeSettings->TargetWorld.Get() : nullptr;
	if (!ModeSettings)
	{
		ModeSettings = NewObject<UFlexNetworkEdModeSettings>(GetTransientPackage(), NAME_None, RF_Transactional);
	}
	// Refreshed on every call, not just at creation: ModeSettings itself persists for the whole
	// editor session (it's a member of this FFlexNetworkEdMode, which outlives any one level), so
	// caching TargetWorld only once would go stale the moment the user changes/reloads levels
	// while still in this mode -- silently pointing the OSM road/rail commands at a world that's no
	// longer the one being viewed. Every other operation (draw, select/move) already avoids this
	// by calling GetWorld() fresh each time via GetSubsystem(); this keeps the OSM path consistent
	// with that instead of being the one place with a caching bug.
	ModeSettings->TargetWorld = GetWorld();
	if (ModeSettings->TargetWorld.Get() != PreviousWorld)
	{
		// A saved level context is the initial value for this transient mode instance. This only runs
		// on a world change, so it never overwrites edits the user is currently making in the panel.
		ModeSettings->LoadOsmContextFromLevel();
	}
	if (UFlexNetworkSubsystem* Subsystem = GetSubsystem())
	{
		Subsystem->SetVisualizationMode(ModeSettings->VisualizationMode);
	}
	return ModeSettings;
}

bool FFlexNetworkEdMode::IsDrawModeActive() const
{
	const UFlexNetworkEdModeSettings* Settings = GetOrCreateModeSettings();
	return Settings && Settings->bDrawModeActive;
}

EFlexNetworkNodeEditTool FFlexNetworkEdMode::GetNodeEditTool() const
{
	const UFlexNetworkEdModeSettings* Settings = GetOrCreateModeSettings();
	return Settings ? Settings->NodeEditTool : EFlexNetworkNodeEditTool::Move;
}

bool FFlexNetworkEdMode::ResolveActiveTangentHandle(FFlexSegmentId& OutSegmentId,
	bool& bOutStartHandle, FVector& OutHandlePosition) const
{
	OutSegmentId = FFlexSegmentId::Invalid();
	bOutStartHandle = false;
	OutHandlePosition = FVector::ZeroVector;

	const UFlexNetworkSubsystem* Subsystem = GetSubsystem();
	const FFlexRoadNode* Node = Subsystem ? Subsystem->GetNode(SelectedNodeId) : nullptr;
	if (!Node)
	{
		return false;
	}

	auto ResolveSegment = [Subsystem, this, &OutSegmentId, &bOutStartHandle, &OutHandlePosition](
		const FFlexSegmentId SegmentId, const bool bPreferSelectedEndpoint) -> bool
	{
		const FFlexRoadSegment* Segment = Subsystem->GetSegment(SegmentId);
		if (!Segment)
		{
			return false;
		}
		if (Segment->StartNodeId == SelectedNodeId
			&& (!bPreferSelectedEndpoint || bSelectedTangentIsStart))
		{
			OutSegmentId = SegmentId;
			bOutStartHandle = true;
			OutHandlePosition = Segment->Curve.P1;
			return true;
		}
		if (Segment->EndNodeId == SelectedNodeId
			&& (!bPreferSelectedEndpoint || !bSelectedTangentIsStart))
		{
			OutSegmentId = SegmentId;
			bOutStartHandle = false;
			OutHandlePosition = Segment->Curve.P2;
			return true;
		}
		return false;
	};

	if (SelectedTangentSegmentId.IsValid()
		&& ResolveSegment(SelectedTangentSegmentId, true))
	{
		return true;
	}
	for (const FFlexSegmentId SegmentId : Node->ConnectedSegments)
	{
		if (ResolveSegment(SegmentId, false))
		{
			return true;
		}
	}
	return false;
}

bool FFlexNetworkEdMode::TraceCursorToWorld(FEditorViewportClient* ViewportClient, FViewport* Viewport, int32 X, int32 Y, FVector& OutPoint) const
{
	if (!ViewportClient || !Viewport)
	{
		return false;
	}

	FSceneViewFamilyContext ViewFamily(FSceneViewFamily::ConstructionValues(Viewport, ViewportClient->GetScene(), ViewportClient->EngineShowFlags).SetRealtimeUpdate(ViewportClient->IsRealtime()));
	FSceneView* View = ViewportClient->CalcSceneView(&ViewFamily);
	if (!View)
	{
		return false;
	}

	const FViewportCursorLocation CursorLocation(View, ViewportClient, X, Y);
	const FVector Direction = CursorLocation.GetDirection();
	FVector Start = CursorLocation.GetOrigin();

	// In orthographic views the "camera" sits arbitrarily far back along the view direction from
	// whatever's under the cursor, rather than at a single perspective-projection eye point -- so
	// the ray origin itself needs to be pulled back first, exactly like Landscape's own cursor
	// trace (FEdModeLandscape::LandscapeMouseTrace) does, or top-down/ortho clicks miss entirely.
	if (ViewportClient->IsOrtho())
	{
		Start -= WORLD_MAX * Direction;
	}
	const FVector End = Start + WORLD_MAX * Direction;

	if (UWorld* World = GetWorld())
	{
		FHitResult Hit;
		FCollisionQueryParams Params(SCENE_QUERY_STAT(FlexNetworkCursorTrace), /*bTraceComplex=*/ true);
		if (World->LineTraceSingleByChannel(Hit, Start, End, ECC_Visibility, Params))
		{
			OutPoint = Hit.Location;
			return true;
		}
	}

	// Nothing under the cursor (e.g. a level with no landscape/floor yet) -- fall back to a flat
	// plane so the tool still works, through the draw's start height while placing, else world Z=0.
	const float PlaneZ = (DrawState == EDrawState::Placing) ? DrawStartPoint.Z : 0.f;
	if (FMath::Abs(Direction.Z) <= KINDA_SMALL_NUMBER)
	{
		return false;
	}
	const float T = (PlaneZ - Start.Z) / Direction.Z;
	if (T < 0.f)
	{
		return false;
	}
	OutPoint = Start + Direction * T;
	return true;
}

void FFlexNetworkEdMode::UpdateHover(const FVector& WorldPoint)
{
	HoverWorldPoint = WorldPoint;
	HoverNodeId = FFlexNodeId::Invalid();
	HoverSegmentId = FFlexSegmentId::Invalid();
	HoverSegmentArcLength = 0.f;
	bHoverValid = true;

	UFlexNetworkSubsystem* Subsystem = GetSubsystem();
	if (!Subsystem)
	{
		return;
	}
	const UFlexNetworkSettings* Settings = GetDefault<UFlexNetworkSettings>();

	FFlexNodeId FoundNode;
	if (Subsystem->FindNearestNode(WorldPoint, Settings->NodeSnapRadius, FoundNode))
	{
		HoverNodeId = FoundNode;
		return;
	}

	FFlexSegmentId FoundSegment;
	float ArcLength = 0.f;
	FVector PointOnCurve;
	if (Subsystem->FindNearestSegmentPoint(WorldPoint, Settings->SegmentSnapRadius, FoundSegment, ArcLength, PointOnCurve))
	{
		HoverSegmentId = FoundSegment;
		HoverSegmentArcLength = ArcLength;
	}
}

FVector FFlexNetworkEdMode::ApplyAngleSnap(const FVector& From, const FVector& To) const
{
	const UFlexNetworkEdModeSettings* ModeSettingsPtr = GetOrCreateModeSettings();
	if (!ModeSettingsPtr || !ModeSettingsPtr->bAngleSnapEnabled)
	{
		return To;
	}

	const FVector Delta = To - From;
	const float HorizontalLength = Delta.Size2D();
	if (HorizontalLength <= KINDA_SMALL_NUMBER)
	{
		return To;
	}

	const float AngleIncrement = FMath::DegreesToRadians(GetDefault<UFlexNetworkSettings>()->AngleSnapIncrementDegrees);
	const float Angle = FMath::Atan2(Delta.Y, Delta.X);
	const float SnappedAngle = FMath::RoundToFloat(Angle / AngleIncrement) * AngleIncrement;

	return From + FVector(FMath::Cos(SnappedAngle) * HorizontalLength, FMath::Sin(SnappedAngle) * HorizontalLength, Delta.Z);
}

void FFlexNetworkEdMode::UpdatePreviewCurve()
{
	UFlexNetworkSubsystem* Subsystem = GetSubsystem();
	if (!Subsystem)
	{
		return;
	}

	FVector EndPoint = bHoverValid ? HoverWorldPoint : DrawStartPoint;
	if (HoverNodeId.IsValid())
	{
		if (const FFlexRoadNode* Node = Subsystem->GetNode(HoverNodeId))
		{
			EndPoint = Node->Position;
		}
	}
	else
	{
		EndPoint = ApplyAngleSnap(DrawStartPoint, EndPoint);
	}

	const float HandleLength = FMath::Max(FVector::Dist(DrawStartPoint, EndPoint) / 3.f, 1.f);

	FVector StartTangentDir;
	if (DrawStartNodeId.IsValid())
	{
		StartTangentDir = Subsystem->SuggestOutgoingTangentDirection(DrawStartNodeId);
	}
	else
	{
		StartTangentDir = (EndPoint - DrawStartPoint).GetSafeNormal(UE_SMALL_NUMBER, FVector::ForwardVector);
	}

	PreviewCurve.P0 = DrawStartPoint;
	PreviewCurve.P1 = DrawStartPoint + StartTangentDir * HandleLength;
	PreviewCurve.P3 = EndPoint;
	const FVector ApproachDir = (EndPoint - DrawStartPoint).GetSafeNormal(UE_SMALL_NUMBER, StartTangentDir);
	PreviewCurve.P2 = EndPoint - ApproachDir * HandleLength;

	const UFlexNetworkEdModeSettings* ModeSettingsPtr = GetOrCreateModeSettings();
	bPreviewValid = Subsystem->ValidateProposedSegment(PreviewCurve, ModeSettingsPtr ? ModeSettingsPtr->ActiveProfile.Get() : nullptr, PreviewInvalidReason);
}

void FFlexNetworkEdMode::BeginPlacement()
{
	DrawState = EDrawState::Placing;
	bPlacementCommittedSegment = false;
	DrawStartPoint = HoverWorldPoint;
	DrawStartNodeId = HoverNodeId;
	DrawStartSegmentId = HoverSegmentId;
	DrawStartSegmentArcLength = HoverSegmentArcLength;

	if (DrawStartNodeId.IsValid())
	{
		if (UFlexNetworkSubsystem* Subsystem = GetSubsystem())
		{
			if (const FFlexRoadNode* Node = Subsystem->GetNode(DrawStartNodeId))
			{
				DrawStartPoint = Node->Position;
			}
		}
	}

	UpdatePreviewCurve();
}

void FFlexNetworkEdMode::CancelPlacement()
{
	const bool bGenerateCompletedCurbstones = bPlacementCommittedSegment;
	DrawState = EDrawState::Idle;
	DrawStartNodeId = FFlexNodeId::Invalid();
	DrawStartSegmentId = FFlexSegmentId::Invalid();
	bPreviewValid = false;
	bPlacementCommittedSegment = false;

	// Curbstone spline meshes can be numerous, so defer their generation until the user ends the
	// complete click-click road chain instead of rebuilding them after every committed segment.
	if (bGenerateCompletedCurbstones)
	{
		if (UFlexNetworkEdModeSettings* Settings = GetOrCreateModeSettings();
			Settings && Settings->bGenerateCurbstonesOnPlacementComplete && Settings->CurbstoneMesh)
		{
			Settings->GenerateCurbstones();
		}
	}
}

void FFlexNetworkEdMode::ReconcileCurveEndpoint(FFlexBezierCurve& Curve, bool bStart, FFlexNodeId NodeId) const
{
	UFlexNetworkSubsystem* Subsystem = GetSubsystem();
	if (!Subsystem)
	{
		return;
	}
	const FFlexRoadNode* Node = Subsystem->GetNode(NodeId);
	if (!Node)
	{
		return;
	}

	if (bStart)
	{
		const FVector Delta = Node->Position - Curve.P0;
		Curve.P0 += Delta;
		Curve.P1 += Delta;
	}
	else
	{
		const FVector Delta = Node->Position - Curve.P3;
		Curve.P3 += Delta;
		Curve.P2 += Delta;
	}
}

FFlexNodeId FFlexNetworkEdMode::ResolveEndpointNode(const FVector& WorldPoint, FFlexNodeId SnapNodeId, FFlexSegmentId SnapSegmentId, float SnapSegmentArcLength)
{
	UFlexNetworkSubsystem* Subsystem = GetSubsystem();
	if (!Subsystem)
	{
		return FFlexNodeId::Invalid();
	}

	if (SnapNodeId.IsValid())
	{
		return SnapNodeId;
	}
	if (SnapSegmentId.IsValid())
	{
		return Subsystem->SplitSegment(SnapSegmentId, SnapSegmentArcLength);
	}

	const UFlexNetworkEdModeSettings* Settings = GetOrCreateModeSettings();
	return Subsystem->AddNode(WorldPoint, Settings ? Settings->ActiveElevationType : EFlexRoadElevationType::Ground);
}

void FFlexNetworkEdMode::CommitPlacement()
{
	UFlexNetworkSubsystem* Subsystem = GetSubsystem();
	UFlexNetworkEdModeSettings* Settings = GetOrCreateModeSettings();

	if (!Subsystem || !Settings || !Settings->ActiveProfile || !bPreviewValid)
	{
		CancelPlacement();
		return;
	}

	FScopedTransaction Transaction(NSLOCTEXT("FlexNetwork", "DrawRoad", "Draw Flex Road"));
	bool bCommittedThisClick = false;

	// Resolve endpoints against the graph as it stood *before* this commit, so the crossing
	// search below only ever considers pre-existing roads, never the one being added.
	const FFlexNodeId StartNodeId = ResolveEndpointNode(PreviewCurve.P0, DrawStartNodeId, DrawStartSegmentId, DrawStartSegmentArcLength);
	const FFlexNodeId EndNodeId = ResolveEndpointNode(PreviewCurve.P3, HoverNodeId, HoverSegmentId, HoverSegmentArcLength);

	if (!StartNodeId.IsValid() || !EndNodeId.IsValid() || StartNodeId == EndNodeId)
	{
		CancelPlacement();
		return;
	}

	// A placement crossing N existing roads calls SplitSegment/AddSegment 2N+1 times below; batching
	// collapses all of their independent RebuildDirty() passes into exactly one at scope exit,
	// instead of one full rebuild per crossing.
	Subsystem->BeginBatchUpdate();
	ON_SCOPE_EXIT { Subsystem->EndBatchUpdate(); };

	FFlexBezierCurve FullCurve = PreviewCurve;
	ReconcileCurveEndpoint(FullCurve, true, StartNodeId);
	ReconcileCurveEndpoint(FullCurve, false, EndNodeId);

	// Auto-split-on-crossing (spec 1.7): every existing road this new curve crosses gets split at
	// the crossing point, and the new road is built as a chain of segments through each resulting
	// junction node instead of one long segment that would just visually overlap the crossed roads.
	TArray<FFlexSegmentCrossing> Crossings = Subsystem->FindCrossings(FullCurve);
	Crossings.Sort([](const FFlexSegmentCrossing& A, const FFlexSegmentCrossing& B) { return A.ArcLengthOnProposedCurve < B.ArcLengthOnProposedCurve; });

	FFlexBezierCurve RemainingCurve = FullCurve;
	float ConsumedArcLength = 0.f;
	FFlexNodeId PreviousNodeId = StartNodeId;

	for (const FFlexSegmentCrossing& Crossing : Crossings)
	{
		if (Crossing.ArcLengthOnProposedCurve <= ConsumedArcLength + KINDA_SMALL_NUMBER)
		{
			continue; // A crossing right at an already-resolved endpoint isn't a genuine mid-span crossing.
		}

		const FFlexArcLengthTable RemainingTable = FFlexBezierMath::BuildArcLengthTable(RemainingCurve);
		const float LocalArcLength = Crossing.ArcLengthOnProposedCurve - ConsumedArcLength;
		if (LocalArcLength >= RemainingTable.GetTotalLength() - KINDA_SMALL_NUMBER)
		{
			continue;
		}
		const float LocalT = FFlexBezierMath::ArcLengthToT(RemainingTable, LocalArcLength);

		FFlexBezierCurve LeftPiece, RightPiece;
		FFlexBezierMath::Subdivide(RemainingCurve, LocalT, LeftPiece, RightPiece);

		const FFlexNodeId JunctionNodeId = Subsystem->SplitSegment(Crossing.ExistingSegmentId, Crossing.ArcLengthOnExistingSegment);
		if (!JunctionNodeId.IsValid())
		{
			continue;
		}

		ReconcileCurveEndpoint(LeftPiece, false, JunctionNodeId);
		bCommittedThisClick |= Subsystem->AddSegment(PreviousNodeId, JunctionNodeId, LeftPiece.P1, LeftPiece.P2,
			Settings->ActiveProfile, Settings->ActiveElevationType).IsValid();

		PreviousNodeId = JunctionNodeId;
		RemainingCurve = RightPiece;
		ReconcileCurveEndpoint(RemainingCurve, true, JunctionNodeId);
		ConsumedArcLength = Crossing.ArcLengthOnProposedCurve;
	}

	bCommittedThisClick |= Subsystem->AddSegment(PreviousNodeId, EndNodeId, RemainingCurve.P1, RemainingCurve.P2,
		Settings->ActiveProfile, Settings->ActiveElevationType).IsValid();
	bPlacementCommittedSegment |= bCommittedThisClick;

	// The new road's endpoint becomes the natural start of the next one -- continuing a chain of
	// segments (e.g. drawing a winding street) doesn't need to re-click the same spot.
	DrawState = EDrawState::Placing;
	DrawStartPoint = PreviewCurve.P3;
	DrawStartNodeId = EndNodeId;
	DrawStartSegmentId = FFlexSegmentId::Invalid();
	DrawStartSegmentArcLength = 0.f;
	UpdatePreviewCurve();
}

// ---------------------------------------------------------------- Node selection / gizmo movement

bool FFlexNetworkEdMode::ShouldDrawWidget() const
{
	if (!SelectedNodeId.IsValid() || IsDrawModeActive())
	{
		return false;
	}
	const UFlexNetworkSubsystem* Subsystem = GetSubsystem();
	if (!Subsystem || !Subsystem->GetNode(SelectedNodeId))
	{
		return false;
	}
	if (GetNodeEditTool() == EFlexNetworkNodeEditTool::Tangent)
	{
		FFlexSegmentId SegmentId;
		bool bStartHandle = false;
		FVector HandlePosition;
		return ResolveActiveTangentHandle(SegmentId, bStartHandle, HandlePosition);
	}
	return true;
}

FVector FFlexNetworkEdMode::GetWidgetLocation() const
{
	if (GetNodeEditTool() == EFlexNetworkNodeEditTool::Tangent)
	{
		FFlexSegmentId SegmentId;
		bool bStartHandle = false;
		FVector HandlePosition;
		if (ResolveActiveTangentHandle(SegmentId, bStartHandle, HandlePosition))
		{
			return HandlePosition;
		}
	}
	if (UFlexNetworkSubsystem* Subsystem = GetSubsystem())
	{
		if (const FFlexRoadNode* Node = Subsystem->GetNode(SelectedNodeId))
		{
			return Node->Position;
		}
	}
	return FVector::ZeroVector;
}

bool FFlexNetworkEdMode::AllowWidgetMove()
{
	return ShouldDrawWidget();
}

bool FFlexNetworkEdMode::UsesTransformWidget() const
{
	return ShouldDrawWidget();
}

bool FFlexNetworkEdMode::UsesTransformWidget(UE::Widget::EWidgetMode CheckMode) const
{
	if (!ShouldDrawWidget())
	{
		return false;
	}
	return GetNodeEditTool() == EFlexNetworkNodeEditTool::Rotate
		? CheckMode == UE::Widget::WM_Rotate
		: CheckMode == UE::Widget::WM_Translate;
}

bool FFlexNetworkEdMode::HandleClick(FEditorViewportClient* InViewportClient, HHitProxy* HitProxy, const FViewportClick& Click)
{
	if (IsDrawModeActive())
	{
		// Placement clicks are handled in InputKey instead, so drawing and node-selection clicks
		// (which use two different gestures) never fight over the same mouse-down event.
		return false;
	}

	if (HitProxy && HitProxy->IsA(HFlexTangentHitProxy::StaticGetType()))
	{
		const HFlexTangentHitProxy* TangentProxy = static_cast<HFlexTangentHitProxy*>(HitProxy);
		if (const UFlexNetworkSubsystem* Subsystem = GetSubsystem())
		{
			if (const FFlexRoadSegment* Segment = Subsystem->GetSegment(TangentProxy->SegmentId))
			{
				SelectedNodeId = TangentProxy->bStartHandle ? Segment->StartNodeId : Segment->EndNodeId;
				SelectedSegmentId = FFlexSegmentId::Invalid();
				SelectedTangentSegmentId = TangentProxy->SegmentId;
				bSelectedTangentIsStart = TangentProxy->bStartHandle;
				if (InViewportClient)
				{
					InViewportClient->SetWidgetMode(UE::Widget::WM_Translate);
				}
				return true;
			}
		}
	}

	if (HitProxy && HitProxy->IsA(HFlexNodeHitProxy::StaticGetType()))
	{
		SelectedNodeId = static_cast<HFlexNodeHitProxy*>(HitProxy)->NodeId;
		SelectedSegmentId = FFlexSegmentId::Invalid();
		SelectedTangentSegmentId = FFlexSegmentId::Invalid();
		if (InViewportClient)
		{
			InViewportClient->SetWidgetMode(GetNodeEditTool() == EFlexNetworkNodeEditTool::Rotate
				? UE::Widget::WM_Rotate
				: UE::Widget::WM_Translate);
		}
		return true;
	}

	if (HitProxy && HitProxy->IsA(HFlexSegmentHitProxy::StaticGetType()))
	{
		SelectedSegmentId = static_cast<HFlexSegmentHitProxy*>(HitProxy)->SegmentId;
		SelectedNodeId = FFlexNodeId::Invalid();
		SelectedTangentSegmentId = FFlexSegmentId::Invalid();
		return true;
	}

	SelectedNodeId = FFlexNodeId::Invalid();
	SelectedSegmentId = FFlexSegmentId::Invalid();
	SelectedTangentSegmentId = FFlexSegmentId::Invalid();
	return FEdMode::HandleClick(InViewportClient, HitProxy, Click);
}

bool FFlexNetworkEdMode::StartTracking(FEditorViewportClient* InViewportClient, FViewport* InViewport)
{
	if (ShouldDrawWidget())
	{
		const EFlexNetworkNodeEditTool Tool = GetNodeEditTool();
		const FText TransactionText = Tool == EFlexNetworkNodeEditTool::Rotate
			? NSLOCTEXT("FlexNetwork", "RotateNode", "Rotate Flex Road Node")
			: Tool == EFlexNetworkNodeEditTool::Tangent
				? NSLOCTEXT("FlexNetwork", "AdjustTangent", "Adjust Flex Road Tangent")
				: NSLOCTEXT("FlexNetwork", "MoveNode", "Move Flex Road Node");
		ActiveNodeEditTransaction = MakeUnique<FScopedTransaction>(TransactionText);
		// A drag gesture calls InputDelta (and therefore SetNodePosition/RotateNode/SetSegmentCurve)
		// once per viewport tick while the mouse moves; batching collapses every one of those
		// independent RebuildDirty() passes into exactly one, fired from EndTracking below.
		if (UFlexNetworkSubsystem* Subsystem = GetSubsystem())
		{
			Subsystem->BeginBatchUpdate();
			bBatchedNodeEditUpdate = true;
		}
		return true;
	}
	return FEdMode::StartTracking(InViewportClient, InViewport);
}

bool FFlexNetworkEdMode::EndTracking(FEditorViewportClient* InViewportClient, FViewport* InViewport)
{
	if (ActiveNodeEditTransaction.IsValid())
	{
		ActiveNodeEditTransaction.Reset();
		if (bBatchedNodeEditUpdate)
		{
			bBatchedNodeEditUpdate = false;
			if (UFlexNetworkSubsystem* Subsystem = GetSubsystem())
			{
				Subsystem->EndBatchUpdate();
			}
		}
		return true;
	}
	return FEdMode::EndTracking(InViewportClient, InViewport);
}

bool FFlexNetworkEdMode::InputDelta(FEditorViewportClient* InViewportClient, FViewport* InViewport, FVector& InDrag, FRotator& InRot, FVector& InScale)
{
	if (!SelectedNodeId.IsValid())
	{
		return FEdMode::InputDelta(InViewportClient, InViewport, InDrag, InRot, InScale);
	}

	UFlexNetworkSubsystem* Subsystem = GetSubsystem();
	if (!Subsystem)
	{
		return false;
	}

	switch (GetNodeEditTool())
	{
	case EFlexNetworkNodeEditTool::Move:
		if (!InDrag.IsNearlyZero())
		{
			if (const FFlexRoadNode* Node = Subsystem->GetNode(SelectedNodeId))
			{
				Subsystem->SetNodePosition(SelectedNodeId, Node->Position + InDrag);
			}
			return true;
		}
		break;

	case EFlexNetworkNodeEditTool::Rotate:
		if (!InRot.IsNearlyZero())
		{
			Subsystem->RotateNode(SelectedNodeId, InRot.Quaternion());
			return true;
		}
		break;

	case EFlexNetworkNodeEditTool::Tangent:
		if (!InDrag.IsNearlyZero())
		{
			FFlexSegmentId SegmentId;
			bool bStartHandle = false;
			FVector HandlePosition;
			if (ResolveActiveTangentHandle(SegmentId, bStartHandle, HandlePosition))
			{
				if (const FFlexRoadSegment* Segment = Subsystem->GetSegment(SegmentId))
				{
					FVector StartHandle = Segment->Curve.P1;
					FVector EndHandle = Segment->Curve.P2;
					if (bStartHandle)
					{
						StartHandle += InDrag;
					}
					else
					{
						EndHandle += InDrag;
					}
					Subsystem->SetSegmentCurve(SegmentId, StartHandle, EndHandle);
					return true;
				}
			}
		}
		break;
	}
	return FEdMode::InputDelta(InViewportClient, InViewport, InDrag, InRot, InScale);
}

void FFlexNetworkEdMode::DeleteSelection()
{
	UFlexNetworkSubsystem* Subsystem = GetSubsystem();
	if (!Subsystem)
	{
		return;
	}

	if (SelectedNodeId.IsValid())
	{
		FScopedTransaction Transaction(NSLOCTEXT("FlexNetwork", "DeleteNode", "Delete Flex Road Node"));
		Subsystem->RemoveNode(SelectedNodeId); // Cascades to remove every segment still connected to it.
		SelectedNodeId = FFlexNodeId::Invalid();
		SelectedTangentSegmentId = FFlexSegmentId::Invalid();
	}
	else if (SelectedSegmentId.IsValid())
	{
		FScopedTransaction Transaction(NSLOCTEXT("FlexNetwork", "DeleteSegment", "Delete Flex Road Segment"));
		Subsystem->RemoveSegment(SelectedSegmentId); // Leaves both endpoint nodes in place, even if orphaned.
		SelectedSegmentId = FFlexSegmentId::Invalid();
	}
}

// ---------------------------------------------------------------- Input / render

bool FFlexNetworkEdMode::MouseMove(FEditorViewportClient* ViewportClient, FViewport* Viewport, int32 X, int32 Y)
{
	FVector WorldPoint;
	if (TraceCursorToWorld(ViewportClient, Viewport, X, Y, WorldPoint))
	{
		UpdateHover(WorldPoint);
		if (DrawState == EDrawState::Placing)
		{
			UpdatePreviewCurve();
		}
	}
	return true;
}

bool FFlexNetworkEdMode::InputKey(FEditorViewportClient* ViewportClient, FViewport* Viewport, FKey Key, EInputEvent Event)
{
	if (IsDrawModeActive() && Key == EKeys::LeftMouseButton)
	{
		if (Event == IE_Pressed)
		{
			if (DrawState == EDrawState::Idle)
			{
				BeginPlacement();
			}
			else
			{
				CommitPlacement();
			}
		}
		return true; // Own the whole click stream in draw mode -- no drag-tracking/base click handling to fall through to.
	}

	if (Key == EKeys::RightMouseButton || Key == EKeys::Escape)
	{
		if (Event == IE_Pressed && DrawState == EDrawState::Placing)
		{
			CancelPlacement();
			return true;
		}
	}

	if (Key == EKeys::Delete || Key == EKeys::Platform_Delete)
	{
		if (Event == IE_Pressed && (SelectedNodeId.IsValid() || SelectedSegmentId.IsValid()))
		{
			DeleteSelection();
			return true;
		}
	}

	return FEdMode::InputKey(ViewportClient, Viewport, Key, Event);
}

void FFlexNetworkEdMode::Render(const FSceneView* View, FViewport* Viewport, FPrimitiveDrawInterface* PDI)
{
	FEdMode::Render(View, Viewport, PDI);

	UFlexNetworkSubsystem* Subsystem = GetSubsystem();
	if (!Subsystem || !PDI)
	{
		return;
	}

	for (const TPair<FFlexNodeId, FFlexRoadNode>& Pair : Subsystem->GetAllNodes())
	{
		const bool bSelected = Pair.Key == SelectedNodeId;
		const bool bHovered = Pair.Key == HoverNodeId;
		const FColor Color = bSelected ? FColor::Yellow : (bHovered ? FColor::Cyan : FColor::White);

		// Hit-proxied so HandleClick can tell exactly which node was clicked (Node Edit mode);
		// harmless to leave the proxy active in Draw mode too, since HandleClick there just
		// returns false and defers to the placement click handled in InputKey instead.
		PDI->SetHitProxy(new HFlexNodeHitProxy(Pair.Key));
		PDI->DrawPoint(Pair.Value.Position, Color, (bHovered || bSelected) ? 16.f : 10.f, SDPG_Foreground);
		PDI->SetHitProxy(nullptr);
	}

	// A thin hit-proxied line along each segment's curve so it can be click-selected (node edit
	// mode) even though the segment's own generated mesh has no hit proxy of its own -- the actual
	// road mesh already shows the segment visually, so this stays unobtrusive except when selected.
	for (const TPair<FFlexSegmentId, FFlexRoadSegment>& Pair : Subsystem->GetAllSegments())
	{
		const bool bSelected = Pair.Key == SelectedSegmentId;
		const FColor Color = bSelected ? FColor::Yellow : FColor(255, 255, 255, 64);
		const float Thickness = bSelected ? 6.f : 1.f;

		PDI->SetHitProxy(new HFlexSegmentHitProxy(Pair.Key));
		constexpr int32 NumSamples = 16;
		FVector Prev = FFlexBezierMath::Evaluate(Pair.Value.Curve, 0.f);
		for (int32 i = 1; i <= NumSamples; ++i)
		{
			const float T = static_cast<float>(i) / static_cast<float>(NumSamples);
			const FVector Next = FFlexBezierMath::Evaluate(Pair.Value.Curve, T);
			PDI->DrawLine(Prev, Next, Color, SDPG_Foreground, Thickness);
			Prev = Next;
		}
		PDI->SetHitProxy(nullptr);
	}

	if (!IsDrawModeActive() && SelectedNodeId.IsValid())
	{
		const FFlexRoadNode* SelectedNode = Subsystem->GetNode(SelectedNodeId);
		if (SelectedNode && GetNodeEditTool() == EFlexNetworkNodeEditTool::Rotate)
		{
			// The node orientation consists of its local up vector plus the incident endpoint
			// tangents. Showing both makes rotation edits legible even for a flat road where yaw
			// leaves the up-vector indicator unchanged.
			PDI->DrawLine(SelectedNode->Position,
				SelectedNode->Position + SelectedNode->UpVector * 150.f,
				FColor::Blue, SDPG_Foreground, 4.f);
			for (const FFlexSegmentId SegmentId : SelectedNode->ConnectedSegments)
			{
				if (const FFlexRoadSegment* Segment = Subsystem->GetSegment(SegmentId))
				{
					if (Segment->StartNodeId == SelectedNodeId)
					{
						PDI->DrawLine(SelectedNode->Position, Segment->Curve.P1,
							FColor::Orange, SDPG_Foreground, 3.f);
					}
					if (Segment->EndNodeId == SelectedNodeId)
					{
						PDI->DrawLine(SelectedNode->Position, Segment->Curve.P2,
							FColor::Orange, SDPG_Foreground, 3.f);
					}
				}
			}
		}
		else if (SelectedNode && GetNodeEditTool() == EFlexNetworkNodeEditTool::Tangent)
		{
			FFlexSegmentId ActiveSegmentId;
			bool bActiveStartHandle = false;
			FVector ActiveHandlePosition;
			ResolveActiveTangentHandle(ActiveSegmentId, bActiveStartHandle, ActiveHandlePosition);

			auto DrawHandle = [PDI, SelectedNode, &ActiveSegmentId, bActiveStartHandle](
				const FFlexSegmentId SegmentId, const bool bStartHandle, const FVector& HandlePosition)
			{
				const bool bActive = SegmentId == ActiveSegmentId && bStartHandle == bActiveStartHandle;
				const FColor Color = bActive ? FColor::Yellow : FColor::Magenta;
				PDI->DrawLine(SelectedNode->Position, HandlePosition, Color,
					SDPG_Foreground, bActive ? 4.f : 2.f);
				PDI->SetHitProxy(new HFlexTangentHitProxy(SegmentId, bStartHandle));
				PDI->DrawPoint(HandlePosition, Color, bActive ? 18.f : 14.f, SDPG_Foreground);
				PDI->SetHitProxy(nullptr);
			};

			for (const FFlexSegmentId SegmentId : SelectedNode->ConnectedSegments)
			{
				if (const FFlexRoadSegment* Segment = Subsystem->GetSegment(SegmentId))
				{
					if (Segment->StartNodeId == SelectedNodeId)
					{
						DrawHandle(SegmentId, true, Segment->Curve.P1);
					}
					if (Segment->EndNodeId == SelectedNodeId)
					{
						DrawHandle(SegmentId, false, Segment->Curve.P2);
					}
				}
			}
		}
	}

	if (DrawState == EDrawState::Placing)
	{
		const FColor Color = bPreviewValid ? FColor::Green : FColor::Red;
		constexpr int32 NumSamples = 24;
		FVector Prev = FFlexBezierMath::Evaluate(PreviewCurve, 0.f);
		for (int32 i = 1; i <= NumSamples; ++i)
		{
			const float T = static_cast<float>(i) / static_cast<float>(NumSamples);
			const FVector Next = FFlexBezierMath::Evaluate(PreviewCurve, T);
			PDI->DrawLine(Prev, Next, Color, SDPG_Foreground, 4.f);
			Prev = Next;
		}
	}
}
