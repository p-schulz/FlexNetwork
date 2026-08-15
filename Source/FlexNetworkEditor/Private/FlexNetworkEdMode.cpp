#include "FlexNetworkEdMode.h"
#include "FlexNetworkEdModeSettings.h"
#include "FlexNetworkEdModeToolkit.h"
#include "FlexNetworkSubsystem.h"
#include "FlexNetworkSettings.h"
#include "RoadTypeProfile.h"
#include "Math/FlexBezierMath.h"
#include "EditorViewportClient.h"
#include "SceneView.h"
#include "SceneManagement.h"
#include "ScopedTransaction.h"
#include "Engine/World.h"
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
	DragState = EDragState::Idle;

	if (!Toolkit.IsValid() && UsesToolkits())
	{
		Toolkit = MakeShareable(new FFlexNetworkEdModeToolkit);
		Toolkit->Init(Owner->GetToolkitHost());
	}
}

void FFlexNetworkEdMode::Exit()
{
	CancelDrag();

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
	if (!ModeSettings)
	{
		ModeSettings = NewObject<UFlexNetworkEdModeSettings>(GetTransientPackage(), NAME_None, RF_Transactional);
	}
	return ModeSettings;
}

bool FFlexNetworkEdMode::ComputeGroundPlanePoint(FEditorViewportClient* ViewportClient, FViewport* Viewport, int32 X, int32 Y, FVector& OutPoint) const
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
	const FVector Origin = CursorLocation.GetOrigin();
	const FVector Direction = CursorLocation.GetDirection();

	// Ground plane: while dragging, use the height of the drag's start point (so a drag begun at
	// an elevated node stays on a plane through that node instead of snapping back to world
	// Z=0); otherwise world Z=0. A dedicated terrain-following raycast would be a nicer UX but
	// isn't required for exercising the graph->geometry pipeline this tool exists to test.
	const float PlaneZ = (DragState == EDragState::Dragging) ? DragStartPoint.Z : 0.f;
	if (FMath::Abs(Direction.Z) <= KINDA_SMALL_NUMBER)
	{
		return false;
	}
	const float T = (PlaneZ - Origin.Z) / Direction.Z;
	if (T < 0.f)
	{
		return false;
	}
	OutPoint = Origin + Direction * T;
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

	FVector EndPoint = bHoverValid ? HoverWorldPoint : DragStartPoint;
	if (HoverNodeId.IsValid())
	{
		if (const FFlexRoadNode* Node = Subsystem->GetNode(HoverNodeId))
		{
			EndPoint = Node->Position;
		}
	}
	else
	{
		EndPoint = ApplyAngleSnap(DragStartPoint, EndPoint);
	}

	const float HandleLength = FMath::Max(FVector::Dist(DragStartPoint, EndPoint) / 3.f, 1.f);

	FVector StartTangentDir;
	if (DragStartNodeId.IsValid())
	{
		StartTangentDir = Subsystem->SuggestOutgoingTangentDirection(DragStartNodeId);
	}
	else
	{
		StartTangentDir = (EndPoint - DragStartPoint).GetSafeNormal(UE_SMALL_NUMBER, FVector::ForwardVector);
	}

	PreviewCurve.P0 = DragStartPoint;
	PreviewCurve.P1 = DragStartPoint + StartTangentDir * HandleLength;
	PreviewCurve.P3 = EndPoint;
	const FVector ApproachDir = (EndPoint - DragStartPoint).GetSafeNormal(UE_SMALL_NUMBER, StartTangentDir);
	PreviewCurve.P2 = EndPoint - ApproachDir * HandleLength;

	const UFlexNetworkEdModeSettings* ModeSettingsPtr = GetOrCreateModeSettings();
	bPreviewValid = Subsystem->ValidateProposedSegment(PreviewCurve, ModeSettingsPtr ? ModeSettingsPtr->ActiveProfile.Get() : nullptr, PreviewInvalidReason);
}

void FFlexNetworkEdMode::BeginDrag()
{
	DragState = EDragState::Dragging;
	DragStartPoint = HoverWorldPoint;
	DragStartNodeId = HoverNodeId;
	DragStartSegmentId = HoverSegmentId;
	DragStartSegmentArcLength = HoverSegmentArcLength;

	if (DragStartNodeId.IsValid())
	{
		if (UFlexNetworkSubsystem* Subsystem = GetSubsystem())
		{
			if (const FFlexRoadNode* Node = Subsystem->GetNode(DragStartNodeId))
			{
				DragStartPoint = Node->Position;
			}
		}
	}

	UpdatePreviewCurve();
}

void FFlexNetworkEdMode::CancelDrag()
{
	DragState = EDragState::Idle;
	DragStartNodeId = FFlexNodeId::Invalid();
	DragStartSegmentId = FFlexSegmentId::Invalid();
	bPreviewValid = false;
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

void FFlexNetworkEdMode::CommitDrag()
{
	UFlexNetworkSubsystem* Subsystem = GetSubsystem();
	UFlexNetworkEdModeSettings* Settings = GetOrCreateModeSettings();

	if (!Subsystem || !Settings || !Settings->ActiveProfile || !bPreviewValid)
	{
		CancelDrag();
		return;
	}

	FScopedTransaction Transaction(NSLOCTEXT("FlexNetwork", "DrawRoad", "Draw Flex Road"));

	// Resolve endpoints against the graph as it stood *before* this commit, so the crossing
	// search below only ever considers pre-existing roads, never the one being added.
	const FFlexNodeId StartNodeId = ResolveEndpointNode(PreviewCurve.P0, DragStartNodeId, DragStartSegmentId, DragStartSegmentArcLength);
	const FFlexNodeId EndNodeId = ResolveEndpointNode(PreviewCurve.P3, HoverNodeId, HoverSegmentId, HoverSegmentArcLength);

	if (!StartNodeId.IsValid() || !EndNodeId.IsValid() || StartNodeId == EndNodeId)
	{
		CancelDrag();
		return;
	}

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
		Subsystem->AddSegment(PreviousNodeId, JunctionNodeId, LeftPiece.P1, LeftPiece.P2, Settings->ActiveProfile, Settings->ActiveElevationType);

		PreviousNodeId = JunctionNodeId;
		RemainingCurve = RightPiece;
		ReconcileCurveEndpoint(RemainingCurve, true, JunctionNodeId);
		ConsumedArcLength = Crossing.ArcLengthOnProposedCurve;
	}

	Subsystem->AddSegment(PreviousNodeId, EndNodeId, RemainingCurve.P1, RemainingCurve.P2, Settings->ActiveProfile, Settings->ActiveElevationType);

	DragState = EDragState::Idle;
	DragStartNodeId = FFlexNodeId::Invalid();
	DragStartSegmentId = FFlexSegmentId::Invalid();
	bPreviewValid = false;
}

bool FFlexNetworkEdMode::MouseMove(FEditorViewportClient* ViewportClient, FViewport* Viewport, int32 X, int32 Y)
{
	FVector WorldPoint;
	if (ComputeGroundPlanePoint(ViewportClient, Viewport, X, Y, WorldPoint))
	{
		UpdateHover(WorldPoint);
		if (DragState == EDragState::Dragging)
		{
			UpdatePreviewCurve();
		}
	}
	return true;
}

bool FFlexNetworkEdMode::InputKey(FEditorViewportClient* ViewportClient, FViewport* Viewport, FKey Key, EInputEvent Event)
{
	if (Key == EKeys::LeftMouseButton)
	{
		if (Event == IE_Pressed)
		{
			if (DragState == EDragState::Idle)
			{
				if (HoverNodeId.IsValid() && !HoverSegmentId.IsValid())
				{
					SelectedNodeId = HoverNodeId;
				}
				BeginDrag();
			}
			return true;
		}
		if (Event == IE_Released)
		{
			if (DragState == EDragState::Dragging)
			{
				CommitDrag();
			}
			return true;
		}
	}
	else if (Key == EKeys::RightMouseButton || Key == EKeys::Escape)
	{
		if (Event == IE_Pressed && DragState == EDragState::Dragging)
		{
			CancelDrag();
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
		PDI->DrawPoint(Pair.Value.Position, Color, (bHovered || bSelected) ? 14.f : 8.f, SDPG_Foreground);
	}

	if (DragState == EDragState::Dragging)
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
