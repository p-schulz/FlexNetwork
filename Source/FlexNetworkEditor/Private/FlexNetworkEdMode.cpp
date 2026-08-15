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
}

void FFlexNetworkEdMode::Exit()
{
	CancelPlacement();
	ActiveNodeMoveTransaction.Reset();

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

bool FFlexNetworkEdMode::IsDrawModeActive() const
{
	const UFlexNetworkEdModeSettings* Settings = GetOrCreateModeSettings();
	return Settings && Settings->bDrawModeActive;
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
	DrawState = EDrawState::Idle;
	DrawStartNodeId = FFlexNodeId::Invalid();
	DrawStartSegmentId = FFlexSegmentId::Invalid();
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

	// Resolve endpoints against the graph as it stood *before* this commit, so the crossing
	// search below only ever considers pre-existing roads, never the one being added.
	const FFlexNodeId StartNodeId = ResolveEndpointNode(PreviewCurve.P0, DrawStartNodeId, DrawStartSegmentId, DrawStartSegmentArcLength);
	const FFlexNodeId EndNodeId = ResolveEndpointNode(PreviewCurve.P3, HoverNodeId, HoverSegmentId, HoverSegmentArcLength);

	if (!StartNodeId.IsValid() || !EndNodeId.IsValid() || StartNodeId == EndNodeId)
	{
		CancelPlacement();
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
	return SelectedNodeId.IsValid();
}

FVector FFlexNetworkEdMode::GetWidgetLocation() const
{
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
	return SelectedNodeId.IsValid();
}

bool FFlexNetworkEdMode::UsesTransformWidget() const
{
	return SelectedNodeId.IsValid();
}

bool FFlexNetworkEdMode::UsesTransformWidget(UE::Widget::EWidgetMode CheckMode) const
{
	// Nodes only have a position, no rotation/scale -- only the translate widget makes sense.
	return SelectedNodeId.IsValid() && CheckMode == UE::Widget::WM_Translate;
}

bool FFlexNetworkEdMode::HandleClick(FEditorViewportClient* InViewportClient, HHitPro