#include "FlexNetworkSubsystem.h"
#include "FlexNetworkSettings.h"
#include "FlexNetworkMeshActor.h"
#include "FlexNetworkSegmentActor.h"
#include "FlexNetworkBakeActor.h"
#include "FlexNetworkBakeTypes.h"
#include "PCGComponent.h"
#include "Math/FlexBezierMath.h"
#include "Math/FlexGeometry2D.h"
#include "Mesh/FlexRoadMeshBuilder.h"
#include "Mesh/FlexRailMeshBuilder.h"
#include "Mesh/FlexUnifiedRoadMeshBuilder.h"
#include "Intersection/FlexIntersectionBuilder.h"
#include "Terrain/FlexLandscapeConformer.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "Math/RotationMatrix.h"

namespace
{
	// Chooses tangent-handle Z so the curve's own Z-component naturally forms the desired ease
	// shape (see FlexNetworkSubsystem.h's class comment for why this needs no separate sampling
	// pass): a cubic Bezier's derivative at t=0 is proportional to (P1-P0), and at t=1 to
	// (P3-P2), so levelling a handle with its endpoint gives that end a zero vertical slope --
	// exactly a smoothstep-style ease -- for free.
	void ApplyElevationEase(FFlexRoadSegment& Segment)
	{
		const double Z0 = Segment.Curve.P0.Z;
		const double Z3 = Segment.Curve.P3.Z;

		switch (Segment.ElevationProfile.Ease)
		{
		case EFlexElevationEase::Linear:
			Segment.Curve.P1.Z = FMath::Lerp(Z0, Z3, 1.0 / 3.0);
			Segment.Curve.P2.Z = FMath::Lerp(Z0, Z3, 2.0 / 3.0);
			break;
		case EFlexElevationEase::EaseIn:
			Segment.Curve.P1.Z = Z0;
			Segment.Curve.P2.Z = FMath::Lerp(Z0, Z3, 2.0 / 3.0);
			break;
		case EFlexElevationEase::EaseOut:
			Segment.Curve.P1.Z = FMath::Lerp(Z0, Z3, 1.0 / 3.0);
			Segment.Curve.P2.Z = Z3;
			break;
		case EFlexElevationEase::EaseInOut:
		default:
			Segment.Curve.P1.Z = Z0;
			Segment.Curve.P2.Z = Z3;
			break;
		}
	}

	TArray<FVector> BuildRoadFootprint(const TArray<FFlexCurveFrame>& Frames, float MinOffset, float MaxOffset, float VerticalOffset = 0.f)
	{
		TArray<FVector> Boundary;
		if (Frames.Num() < 2 || MaxOffset - MinOffset <= KINDA_SMALL_NUMBER)
		{
			return Boundary;
		}
		Boundary.Reserve(Frames.Num() * 2);
		for (const FFlexCurveFrame& Frame : Frames)
		{
			Boundary.Add(Frame.Position + Frame.Right * MinOffset + Frame.Up * VerticalOffset);
		}
		for (int32 Index = Frames.Num() - 1; Index >= 0; --Index)
		{
			Boundary.Add(Frames[Index].Position + Frames[Index].Right * MaxOffset + Frames[Index].Up * VerticalOffset);
		}
		return Boundary;
	}

	TArray<FVector> BuildConvexHullXY(TConstArrayView<FVector> SourcePoints)
	{
		TArray<FVector> Points;
		Points.Reserve(SourcePoints.Num());
		double ZSum = 0.0;
		for (const FVector& Point : SourcePoints)
		{
			Points.Add(Point);
			ZSum += Point.Z;
		}
		Points.Sort([](const FVector& A, const FVector& B)
		{
			return A.X < B.X || (FMath::IsNearlyEqual(A.X, B.X) && A.Y < B.Y);
		});
		for (int32 Index = Points.Num() - 1; Index > 0; --Index)
		{
			if (FVector2D(Points[Index].X, Points[Index].Y).Equals(
				FVector2D(Points[Index - 1].X, Points[Index - 1].Y), 0.1))
			{
				Points.RemoveAt(Index);
			}
		}
		if (Points.Num() < 3)
		{
			return {};
		}

		auto Cross = [](const FVector& Origin, const FVector& A, const FVector& B)
		{
			return (A.X - Origin.X) * (B.Y - Origin.Y) - (A.Y - Origin.Y) * (B.X - Origin.X);
		};
		TArray<FVector> Hull;
		for (const FVector& Point : Points)
		{
			while (Hull.Num() >= 2 && Cross(Hull[Hull.Num() - 2], Hull.Last(), Point) <= UE_DOUBLE_SMALL_NUMBER)
			{
				Hull.Pop();
			}
			Hull.Add(Point);
		}
		const int32 LowerCount = Hull.Num();
		for (int32 Index = Points.Num() - 2; Index >= 0; --Index)
		{
			while (Hull.Num() > LowerCount && Cross(Hull[Hull.Num() - 2], Hull.Last(), Points[Index]) <= UE_DOUBLE_SMALL_NUMBER)
			{
				Hull.Pop();
			}
			Hull.Add(Points[Index]);
		}
		Hull.Pop();
		const double AverageZ = ZSum / static_cast<double>(SourcePoints.Num());
		for (FVector& Point : Hull)
		{
			Point.Z = AverageZ;
		}
		return Hull;
	}

	void GetLaneRolesForNode(const FRoadLaneDescriptor& Lane, bool bNodeIsSegmentEnd, bool& bOutIncoming, bool& bOutOutgoing)
	{
		bOutIncoming = false;
		bOutOutgoing = false;
		if (!Lane.IsDrivable())
		{
			return;
		}
		switch (Lane.Direction)
		{
		case EFlexLaneDirection::Forward:
			bOutIncoming = bNodeIsSegmentEnd;
			bOutOutgoing = !bNodeIsSegmentEnd;
			break;
		case EFlexLaneDirection::Backward:
			bOutIncoming = !bNodeIsSegmentEnd;
			bOutOutgoing = bNodeIsSegmentEnd;
			break;
		case EFlexLaneDirection::Bidirectional:
			bOutIncoming = true;
			bOutOutgoing = true;
			break;
		default:
			break;
		}
	}

	void ValidateLaneConnectivity(FFlexNodeId NodeId, TConstArrayView<FFlexJunctionApproachInput> Approaches, const FFlexJunctionData& Junction)
	{
		for (const FFlexJunctionApproachInput& From : Approaches)
		{
			if (!From.Profile)
			{
				continue;
			}
			for (int32 FromLaneIndex = 0; FromLaneIndex < From.Profile->Lanes.Num(); ++FromLaneIndex)
			{
				const FRoadLaneDescriptor& FromLane = From.Profile->Lanes[FromLaneIndex];
				bool bIncoming = false, bOutgoing = false;
				GetLaneRolesForNode(FromLane, From.bNodeIsSegmentEnd, bIncoming, bOutgoing);
				if (!bIncoming)
				{
					continue;
				}

				for (const FFlexJunctionApproachInput& To : Approaches)
				{
					if (To.SegmentId == From.SegmentId || !To.Profile)
					{
						continue;
					}
					const bool bHasOutgoing = To.Profile->Lanes.ContainsByPredicate([&](const FRoadLaneDescriptor& Lane)
					{
						bool bToIncoming = false, bToOutgoing = false;
						GetLaneRolesForNode(Lane, To.bNodeIsSegmentEnd, bToIncoming, bToOutgoing);
						return bToOutgoing && Lane.IsRail() == FromLane.IsRail();
					});
					if (!bHasOutgoing)
					{
						continue;
					}

					const bool bConnected = Junction.LaneConnectors.ContainsByPredicate([&](const FFlexLaneConnector& Connector)
					{
						return Connector.FromSegment == From.SegmentId
							&& Connector.FromLaneIndex == FromLaneIndex
							&& Connector.ToSegment == To.SegmentId;
					});
					if (!bConnected)
					{
						UE_LOG(LogTemp, Warning, TEXT("FlexNetwork: junction %u:%u has no legal connector from segment %u:%u lane %d to neighbouring segment %u:%u."),
							NodeId.Index, NodeId.Generation,
							From.SegmentId.Index, From.SegmentId.Generation, FromLaneIndex,
							To.SegmentId.Index, To.SegmentId.Generation);
					}
				}
			}
		}
	}
}

void UFlexNetworkSubsystem::Initialize(FSubsystemCollectionBase& Collection)
{
	Super::Initialize(Collection);
	TerrainConformer = MakeUnique<FFlexLandscapeConformer>();
}

void UFlexNetworkSubsystem::Deinitialize()
{
	ClearNetwork();
	TerrainConformer.Reset();
	Exporters.Reset();
	Super::Deinitialize();
}

void UFlexNetworkSubsystem::OnWorldBeginPlay(UWorld& InWorld)
{
	Super::OnWorldBeginPlay(InWorld);
	for (TActorIterator<AFlexNetworkBakeActor> It(&InWorld); It; ++It)
	{
		if (It->bRestoreAutomatically)
		{
			It->RestoreToSubsystem(*this);
			break; // One authoritative baked graph per world.
		}
	}
}

void UFlexNetworkSubsystem::ClearNetwork()
{
	const bool bHadTrafficSignals = !TrafficSignals.IsEmpty();
	for (TPair<FFlexSegmentId, TObjectPtr<AFlexNetworkSegmentActor>>& Pair : SegmentActors)
	{
		if (Pair.Value)
		{
			if (Pair.Value->PCGComponent) Pair.Value->PCGComponent->CleanupLocalImmediate(true, true);
			Pair.Value->Destroy();
		}
	}
	SegmentActors.Reset();
	RailPCGOwner = FFlexSegmentId::Invalid();
	if (MeshActor)
	{
		MeshActor->Destroy();
		MeshActor = nullptr;
	}

	if (TerrainConformer)
	{
		for (const TPair<FFlexSegmentId, FFlexRoadSegment>& Pair : Segments)
		{
			TerrainConformer->RemoveSegmentConforming(GetWorld(), Pair.Key);
		}
	}

	Nodes.Reset();
	Segments.Reset();
	JunctionDataByNode.Reset();
	TrafficSignals.Reset();
	SpatialGrid.Clear();
	NodeIdAllocator.Reset();
	SegmentIdAllocator.Reset();
	DirtyNodes.Reset();
	DirtySegments.Reset();
	BatchDepth = 0;
	bTrafficSignalsChangedDuringBatch = false;
	NextComplexIntersectionRegionIndex = 0;
	if (bHadTrafficSignals)
	{
		BroadcastTrafficSignalsChanged();
	}
}

AFlexNetworkSegmentActor* UFlexNetworkSubsystem::GetSegmentActor(FFlexSegmentId SegmentId) const
{
	const TObjectPtr<AFlexNetworkSegmentActor>* Found = SegmentActors.Find(SegmentId);
	return Found ? Found->Get() : nullptr;
}

AFlexNetworkSegmentActor* UFlexNetworkSubsystem::GetOrCreateSegmentActor(FFlexSegmentId SegmentId)
{
	if (TObjectPtr<AFlexNetworkSegmentActor>* Found = SegmentActors.Find(SegmentId)) return *Found;
	UWorld* World = GetWorld();
	if (!World) return nullptr;
	TSubclassOf<AFlexNetworkSegmentActor> ActorClass = GetSettings()->SegmentActorClass;
	if (!ActorClass)
	{
		ActorClass = AFlexNetworkSegmentActor::StaticClass();
	}
	// Set the stable ID before components register/PCG can auto-generate from a configured subclass.
	AFlexNetworkSegmentActor* Actor = World->SpawnActorDeferred<AFlexNetworkSegmentActor>(
		ActorClass, FTransform::Identity, nullptr, nullptr, ESpawnActorCollisionHandlingMethod::AlwaysSpawn);
	if (Actor)
	{
		Actor->SegmentId = SegmentId;
		Actor->SetFlags(RF_Transient);
		Actor->FinishSpawning(FTransform::Identity);
	}
	if (Actor) SegmentActors.Add(SegmentId, Actor);
	return Actor;
}

void UFlexNetworkSubsystem::SetVisualizationMode(EFlexNetworkVisualizationMode NewMode)
{
	if (VisualizationMode == NewMode) return;
	VisualizationMode = NewMode;

	const bool bWantGeometry = NewMode != EFlexNetworkVisualizationMode::SegmentActors;
	const bool bWantActors = NewMode != EFlexNetworkVisualizationMode::GeneratedGeometry;
	if (!bWantGeometry && MeshActor)
	{
		MeshActor->Destroy();
		MeshActor = nullptr;
	}
	if (!bWantActors)
	{
		for (TPair<FFlexSegmentId, TObjectPtr<AFlexNetworkSegmentActor>>& Pair : SegmentActors)
		{
			if (Pair.Value)
			{
				if (Pair.Value->PCGComponent) Pair.Value->PCGComponent->CleanupLocalImmediate(true, true);
				Pair.Value->Destroy();
			}
		}
		SegmentActors.Reset();
	}

	// Use the normal incremental pipeline with every graph item marked dirty. This also covers
	// segments created by a previous OSM batch without introducing a second import-only path.
	for (const TPair<FFlexSegmentId, FFlexRoadSegment>& Pair : Segments) DirtySegments.Add(Pair.Key);
	for (const TPair<FFlexNodeId, FFlexRoadNode>& Pair : Nodes) DirtyNodes.Add(Pair.Key);
	RebuildDirty();
}

void UFlexNetworkSubsystem::RebuildAllNetworkGeometry()
{
	for (const TPair<FFlexSegmentId, FFlexRoadSegment>& Pair : Segments) DirtySegments.Add(Pair.Key);
	for (const TPair<FFlexNodeId, FFlexRoadNode>& Pair : Nodes) DirtyNodes.Add(Pair.Key);
	RebuildDirty();
}

const UFlexNetworkSettings* UFlexNetworkSubsystem::GetSettings() const
{
	return GetDefault<UFlexNetworkSettings>();
}

AFlexNetworkMeshActor* UFlexNetworkSubsystem::GetOrCreateMeshActor()
{
	if (!MeshActor)
	{
		if (UWorld* World = GetWorld())
		{
			FActorSpawnParameters Params;
			Params.ObjectFlags |= RF_Transient;
			MeshActor = World->SpawnActor<AFlexNetworkMeshActor>(Params);
		}
	}
	return MeshActor;
}

FVector2D UFlexNetworkSubsystem::NodeSegmentBoundsMin(const FFlexRoadSegment& Segment) const
{
	double MinX = Segment.Curve.P0.X, MinY = Segment.Curve.P0.Y;
	for (const FVector& P : { Segment.Curve.P1, Segment.Curve.P2, Segment.Curve.P3 })
	{
		MinX = FMath::Min(MinX, static_cast<double>(P.X));
		MinY = FMath::Min(MinY, static_cast<double>(P.Y));
	}
	return FVector2D(MinX, MinY);
}

FVector2D UFlexNetworkSubsystem::NodeSegmentBoundsMax(const FFlexRoadSegment& Segment) const
{
	double MaxX = Segment.Curve.P0.X, MaxY = Segment.Curve.P0.Y;
	for (const FVector& P : { Segment.Curve.P1, Segment.Curve.P2, Segment.Curve.P3 })
	{
		MaxX = FMath::Max(MaxX, static_cast<double>(P.X));
		MaxY = FMath::Max(MaxY, static_cast<double>(P.Y));
	}
	return FVector2D(MaxX, MaxY);
}

void UFlexNetworkSubsystem::AddSegmentToSpatialGrid(FFlexSegmentId Id, const FFlexRoadSegment& Segment)
{
	SpatialGrid.AddSegment(Id, NodeSegmentBoundsMin(Segment), NodeSegmentBoundsMax(Segment));
}

void UFlexNetworkSubsystem::RemoveSegmentFromSpatialGrid(FFlexSegmentId Id, const FFlexRoadSegment& Segment)
{
	SpatialGrid.RemoveSegment(Id, NodeSegmentBoundsMin(Segment), NodeSegmentBoundsMax(Segment));
}

void UFlexNetworkSubsystem::DetachSegmentFromNode(FFlexNodeId NodeId, FFlexSegmentId SegmentId)
{
	if (FFlexRoadNode* Node = Nodes.Find(NodeId))
	{
		Node->ConnectedSegments.RemoveSingleSwap(SegmentId);
	}
}

// ---------------------------------------------------------------- Mutation API

FFlexNodeId UFlexNetworkSubsystem::AddNode(const FVector& Position, EFlexRoadElevationType ElevationType, const FVector& UpVector)
{
	const FFlexNodeId Id = NodeIdAllocator.Allocate();

	FFlexRoadNode Node;
	Node.Position = Position;
	Node.UpVector = UpVector.GetSafeNormal(UE_SMALL_NUMBER, FVector::UpVector);
	Node.ElevationType = ElevationType;
	Node.RoleFlags = static_cast<uint8>(EFlexNodeRole::Endpoint);
	Nodes.Add(Id, Node);

	SpatialGrid.AddNode(Id, FVector2D(Position.X, Position.Y));
	return Id;
}

FFlexSegmentId UFlexNetworkSubsystem::AddSegment(FFlexNodeId StartNodeId, FFlexNodeId EndNodeId, const FVector& StartTangentHandle, const FVector& EndTangentHandle, URoadTypeProfile* Profile, EFlexRoadElevationType ElevationType, const FFlexElevationProfile& ElevationProfile)
{
	FFlexRoadNode* StartNode = Nodes.Find(StartNodeId);
	FFlexRoadNode* EndNode = Nodes.Find(EndNodeId);
	if (!StartNode || !EndNode || !Profile)
	{
		return FFlexSegmentId::Invalid();
	}

	FFlexRoadSegment Segment;
	Segment.StartNodeId = StartNodeId;
	Segment.EndNodeId = EndNodeId;
	Segment.Curve.P0 = StartNode->Position;
	Segment.Curve.P1 = StartTangentHandle;
	Segment.Curve.P2 = EndTangentHandle;
	Segment.Curve.P3 = EndNode->Position;
	Segment.Profile = Profile;
	Segment.ElevationType = ElevationType;
	Segment.ElevationProfile = ElevationProfile;
	Segment.bDirty = true;
	ApplyElevationEase(Segment);

	const FFlexSegmentId Id = SegmentIdAllocator.Allocate();
	AddSegmentToSpatialGrid(Id, Segment);
	Segments.Add(Id, Segment);

	StartNode->ConnectedSegments.Add(Id);
	EndNode->ConnectedSegments.Add(Id);

	DirtySegments.Add(Id);
	DirtyNodes.Add(StartNodeId);
	DirtyNodes.Add(EndNodeId);
	RebuildDirty();
	return Id;
}

int32 UFlexNetworkSubsystem::LoadBakedNetwork(
	const TArray<FFlexBakedNode>& BakedNodes,
	const TArray<FFlexBakedSegment>& BakedSegments,
	const TArray<FFlexBakedTrafficSignal>& BakedSignals,
	EFlexNetworkVisualizationMode BakedVisualizationMode)
{
	ClearNetwork();
	VisualizationMode = BakedVisualizationMode;

	TMap<FFlexNodeId, FFlexNodeId> RestoredNodeIds;
	RestoredNodeIds.Reserve(BakedNodes.Num());
	BeginBatchUpdate();

	for (const FFlexBakedNode& Record : BakedNodes)
	{
		const FFlexNodeId NewId = AddNode(Record.Position, Record.ElevationType, Record.UpVector);
		if (FFlexRoadNode* Node = Nodes.Find(NewId))
		{
			Node->FilletRadiusOverride = Record.FilletRadiusOverride;
			Node->ComplexIntersectionRegionIndex = Record.ComplexIntersectionRegionIndex;
			if (Record.ComplexIntersectionRegionIndex != INDEX_NONE)
			{
				NextComplexIntersectionRegionIndex = FMath::Max(
					NextComplexIntersectionRegionIndex, Record.ComplexIntersectionRegionIndex + 1);
			}
			RestoredNodeIds.Add(Record.SourceId, NewId);
		}
	}

	int32 RestoredSegments = 0;
	TMap<FFlexSegmentId, FFlexSegmentId> RestoredSegmentIds;
	RestoredSegmentIds.Reserve(BakedSegments.Num());
	for (const FFlexBakedSegment& Record : BakedSegments)
	{
		const FFlexNodeId* StartId = RestoredNodeIds.Find(Record.StartSourceNodeId);
		const FFlexNodeId* EndId = RestoredNodeIds.Find(Record.EndSourceNodeId);
		if (!StartId || !EndId || !Record.Profile)
		{
			UE_LOG(LogTemp, Warning, TEXT("FlexNetwork bake: skipped segment %s because an endpoint or road profile is unavailable."), *Record.SourceId.ToString());
			continue;
		}
		const FFlexSegmentId NewSegmentId = AddSegment(*StartId, *EndId, Record.StartTangentHandle, Record.EndTangentHandle,
			Record.Profile, Record.ElevationType, Record.ElevationProfile);
		if (NewSegmentId.IsValid())
		{
			RestoredSegmentIds.Add(Record.SourceId, NewSegmentId);
			++RestoredSegments;
		}
	}

	int32 RestoredSignals = 0;
	for (const FFlexBakedTrafficSignal& Record : BakedSignals)
	{
		FFlexTrafficSignal Signal = Record.Signal;
		const FFlexSegmentId* AnchorId = RestoredSegmentIds.Find(Signal.AnchorSegmentId);
		const FFlexSegmentId* ApproachId = RestoredSegmentIds.Find(Signal.ControlledApproachSegmentId);
		const FFlexNodeId* JunctionId = RestoredNodeIds.Find(Signal.ControlledJunctionNodeId);
		if (!AnchorId || !ApproachId || !JunctionId)
		{
			UE_LOG(LogTemp, Warning, TEXT("FlexNetwork bake: skipped traffic control %s because a graph attachment is unavailable."), *Signal.Id.ToString());
			continue;
		}
		Signal.AnchorSegmentId = *AnchorId;
		Signal.ControlledApproachSegmentId = *ApproachId;
		Signal.ControlledJunctionNodeId = *JunctionId;
		if (AddTrafficSignal(Signal).IsValid())
		{
			++RestoredSignals;
		}
	}

	EndBatchUpdate();
	UE_LOG(LogTemp, Display, TEXT("FlexNetwork: restored baked graph with %d node(s), %d segment(s), and %d traffic control(s) in world '%s'."),
		RestoredNodeIds.Num(), RestoredSegments, RestoredSignals, GetWorld() ? *GetWorld()->GetName() : TEXT("<none>"));
	return RestoredSegments;
}

int32 UFlexNetworkSubsystem::LoadBakedNetwork(
	const TArray<FFlexBakedNode>& BakedNodes,
	const TArray<FFlexBakedSegment>& BakedSegments,
	EFlexNetworkVisualizationMode BakedVisualizationMode)
{
	return LoadBakedNetwork(BakedNodes, BakedSegments, TArray<FFlexBakedTrafficSignal>(), BakedVisualizationMode);
}

bool UFlexNetworkSubsystem::ValidateTrafficSignalAttachments(const FFlexTrafficSignal& Signal) const
{
	const FFlexRoadSegment* Anchor = Segments.Find(Signal.AnchorSegmentId);
	const FFlexRoadSegment* Approach = Segments.Find(Signal.ControlledApproachSegmentId);
	return Anchor && Approach && Nodes.Contains(Signal.ControlledJunctionNodeId)
		&& (Approach->StartNodeId == Signal.ControlledJunctionNodeId
			|| Approach->EndNodeId == Signal.ControlledJunctionNodeId);
}

void UFlexNetworkSubsystem::BroadcastTrafficSignalsChanged()
{
	if (BatchDepth > 0)
	{
		bTrafficSignalsChangedDuringBatch = true;
		return;
	}
	OnTrafficSignalsChanged.Broadcast();
	OnTrafficSignalsChangedBP.Broadcast();
}

FGuid UFlexNetworkSubsystem::AddTrafficSignal(const FFlexTrafficSignal& InSignal)
{
	if (!ValidateTrafficSignalAttachments(InSignal))
	{
		return FGuid();
	}
	FFlexTrafficSignal Signal = InSignal;
	Signal.AnchorFraction = FMath::Clamp(Signal.AnchorFraction, 0.f, 1.f);
	if (!Signal.SourceId.IsEmpty())
	{
		for (TPair<FGuid, FFlexTrafficSignal>& Pair : TrafficSignals)
		{
			if (Pair.Value.SourceId == Signal.SourceId)
			{
				Signal.Id = Pair.Key;
				Pair.Value = MoveTemp(Signal);
				BroadcastTrafficSignalsChanged();
				return Pair.Key;
			}
		}
	}
	if (!Signal.Id.IsValid())
	{
		Signal.Id = FGuid::NewGuid();
	}
	const FGuid NewId = Signal.Id;
	if (TrafficSignals.Contains(Signal.Id))
	{
		return FGuid();
	}
	TrafficSignals.Add(Signal.Id, MoveTemp(Signal));
	BroadcastTrafficSignalsChanged();
	return NewId;
}

bool UFlexNetworkSubsystem::UpdateTrafficSignal(const FFlexTrafficSignal& Signal)
{
	if (!Signal.Id.IsValid() || !TrafficSignals.Contains(Signal.Id)
		|| !ValidateTrafficSignalAttachments(Signal))
	{
		return false;
	}
	FFlexTrafficSignal Updated = Signal;
	Updated.AnchorFraction = FMath::Clamp(Updated.AnchorFraction, 0.f, 1.f);
	TrafficSignals.FindChecked(Signal.Id) = MoveTemp(Updated);
	BroadcastTrafficSignalsChanged();
	return true;
}

bool UFlexNetworkSubsystem::RemoveTrafficSignal(const FGuid& SignalId)
{
	if (TrafficSignals.Remove(SignalId) == 0)
	{
		return false;
	}
	BroadcastTrafficSignalsChanged();
	return true;
}

void UFlexNetworkSubsystem::ClearTrafficSignals()
{
	if (TrafficSignals.IsEmpty())
	{
		return;
	}
	TrafficSignals.Reset();
	BroadcastTrafficSignalsChanged();
}

bool UFlexNetworkSubsystem::ResolveTrafficSignal(const FFlexTrafficSignal& Signal, FFlexResolvedTrafficSignal& OutResolved) const
{
	if (!Signal.bEnabled || !ValidateTrafficSignalAttachments(Signal))
	{
		return false;
	}
	const FFlexRoadSegment& Anchor = Segments.FindChecked(Signal.AnchorSegmentId);
	const FFlexRoadSegment& Approach = Segments.FindChecked(Signal.ControlledApproachSegmentId);
	const float AnchorArc = FMath::Clamp(Signal.AnchorFraction, 0.f, 1.f) * Anchor.GetLength();
	const FFlexCurveFrame AnchorFrame = SampleSegmentAtArcLength(Signal.AnchorSegmentId, AnchorArc);
	const bool bJunctionAtStart = Approach.StartNodeId == Signal.ControlledJunctionNodeId;
	const float ApproachJunctionArc = bJunctionAtStart ? 0.f : Approach.GetLength();
	const FFlexCurveFrame ApproachFrame = SampleSegmentAtArcLength(Signal.ControlledApproachSegmentId, ApproachJunctionArc);
	const FVector Inbound = (bJunctionAtStart ? -ApproachFrame.Tangent : ApproachFrame.Tangent)
		.GetSafeNormal(UE_SMALL_NUMBER, FVector::ForwardVector);
	const FVector Up = AnchorFrame.Up.GetSafeNormal(UE_SMALL_NUMBER, FVector::UpVector);
	FVector AnchorInbound = AnchorFrame.Tangent.GetSafeNormal(UE_SMALL_NUMBER, Inbound);
	if (FVector::DotProduct(AnchorInbound, Inbound) < 0.f)
	{
		AnchorInbound *= -1.f;
	}
	const FVector AnchorInboundRight = FVector::CrossProduct(Up, AnchorInbound)
		.GetSafeNormal(UE_SMALL_NUMBER, AnchorFrame.Right);
	const FVector Position = AnchorFrame.Position + AnchorInboundRight * Signal.LateralOffset + Up * Signal.HeightOffset;
	const FVector ApproachInboundRight = FVector::CrossProduct(ApproachFrame.Up, Inbound)
		.GetSafeNormal(UE_SMALL_NUMBER, ApproachFrame.Right);

	float TrimStart = 0.f;
	float TrimEnd = Approach.GetLength();
	GetSegmentTrimRange(Signal.ControlledApproachSegmentId, TrimStart, TrimEnd);
	const float SideArc = bJunctionAtStart ? TrimStart : TrimEnd;
	FFlexCurveFrame SideFrame = SampleSegmentAtArcLength(Signal.ControlledApproachSegmentId, SideArc);
	if (Approach.Profile)
	{
		SideFrame.Position += SideFrame.Right * Approach.Profile->GetRoadwayCenterOffset();
	}
	FVector ControlledSidePoint = SideFrame.Position;
	if (const FFlexJunctionData* Junction = JunctionDataByNode.Find(Signal.ControlledJunctionNodeId))
	{
		// MassTraffic searches from the left-most inbound intersection lane. Use the matching
		// connector start exactly when available, rather than relying on a road-center midpoint
		// whose lateral error can exceed the default 400 cm search radius on a wide arterial.
		bool bFoundInboundLane = false;
		float LeftMostOffset = TNumericLimits<float>::Max();
		for (const FFlexLaneConnector& Connector : Junction->LaneConnectors)
		{
			if (Connector.FromSegment != Signal.ControlledApproachSegmentId)
			{
				continue;
			}
			const float Offset = FVector::DotProduct(Connector.ConnectorCurve.P0 - SideFrame.Position, ApproachInboundRight);
			if (!bFoundInboundLane || Offset < LeftMostOffset)
			{
				bFoundInboundLane = true;
				LeftMostOffset = Offset;
				ControlledSidePoint = Connector.ConnectorCurve.P0;
			}
		}
	}

	OutResolved.InboundDirection = Inbound;
	OutResolved.ControlledIntersectionSideMidpoint = ControlledSidePoint;
	OutResolved.Transform = FTransform(FRotationMatrix::MakeFromXZ(AnchorInbound, Up).ToQuat(), Position);
	return true;
}

int32 UFlexNetworkSubsystem::RegisterComplexIntersectionRegion(TConstArrayView<FFlexNodeId> MemberNodeIds)
{
	TArray<FFlexNodeId> ValidMembers;
	for (const FFlexNodeId NodeId : MemberNodeIds)
	{
		if (Nodes.Contains(NodeId))
		{
			ValidMembers.AddUnique(NodeId);
		}
	}
	if (ValidMembers.Num() < 2)
	{
		return INDEX_NONE;
	}

	const int32 RegionIndex = NextComplexIntersectionRegionIndex++;
	for (const FFlexNodeId NodeId : ValidMembers)
	{
		FFlexRoadNode& Node = Nodes.FindChecked(NodeId);
		Node.ComplexIntersectionRegionIndex = RegionIndex;
		DirtyNodes.Add(NodeId);
	}
	RebuildDirty();
	return RegionIndex;
}

bool UFlexNetworkSubsystem::RemoveSegment(FFlexSegmentId SegmentId)
{
	FFlexRoadSegment* Segment = Segments.Find(SegmentId);
	if (!Segment)
	{
		return false;
	}

	const bool bRemovedRail = Segment->Profile && Segment->Profile->bIsRailProfile;
	TArray<FGuid> AttachedSignalIds;
	for (const TPair<FGuid, FFlexTrafficSignal>& Pair : TrafficSignals)
	{
		if (Pair.Value.AnchorSegmentId == SegmentId || Pair.Value.ControlledApproachSegmentId == SegmentId)
		{
			AttachedSignalIds.Add(Pair.Key);
		}
	}
	for (const FGuid& SignalId : AttachedSignalIds)
	{
		TrafficSignals.Remove(SignalId);
	}
	RemoveSegmentFromSpatialGrid(SegmentId, *Segment);
	if (TerrainConformer)
	{
		TerrainConformer->RemoveSegmentConforming(GetWorld(), SegmentId);
	}
	if (MeshActor)
	{
		MeshActor->RemoveSegmentMesh(SegmentId);
	}
	if (TObjectPtr<AFlexNetworkSegmentActor> SegmentActor; SegmentActors.RemoveAndCopyValue(SegmentId, SegmentActor) && SegmentActor)
	{
		if (SegmentActor->PCGComponent)
		{
			SegmentActor->PCGComponent->CleanupLocalImmediate(true, true);
		}
		SegmentActor->Destroy();
	}

	const FFlexNodeId StartId = Segment->StartNodeId;
	const FFlexNodeId EndId = Segment->EndNodeId;

	DetachSegmentFromNode(StartId, SegmentId);
	DetachSegmentFromNode(EndId, SegmentId);

	Segments.Remove(SegmentId);
	SegmentIdAllocator.Free(SegmentId);
	DirtySegments.Remove(SegmentId);
	if (bRemovedRail)
	{
		// Actor-driven PCG assigns the profile-wide rail boolean to the lowest live rail ID.
		// If that owner was removed (even as an isolated segment), refresh its successor.
		FFlexSegmentId NewRailOwner = FFlexSegmentId::Invalid();
		for (const TPair<FFlexSegmentId, FFlexRoadSegment>& Pair : Segments)
		{
			if (Pair.Value.Profile && Pair.Value.Profile->bIsRailProfile
				&& (!NewRailOwner.IsValid() || Pair.Key.Index < NewRailOwner.Index
					|| (Pair.Key.Index == NewRailOwner.Index && Pair.Key.Generation < NewRailOwner.Generation)))
			{
				NewRailOwner = Pair.Key;
			}
		}
		if (NewRailOwner.IsValid())
		{
			DirtySegments.Add(NewRailOwner);
		}
	}

	DirtyNodes.Add(StartId);
	DirtyNodes.Add(EndId);
	RebuildDirty();
	if (!AttachedSignalIds.IsEmpty())
	{
		BroadcastTrafficSignalsChanged();
	}
	return true;
}

bool UFlexNetworkSubsystem::RemoveNode(FFlexNodeId NodeId)
{
	FFlexRoadNode* Node = Nodes.Find(NodeId);
	if (!Node)
	{
		return false;
	}

	// Copy first: RemoveSegment mutates this node's ConnectedSegments array as it detaches each one.
	const TArray<FFlexSegmentId> Connected = Node->ConnectedSegments;
	for (FFlexSegmentId SegId : Connected)
	{
		RemoveSegment(SegId);
	}

	// Node may have been freed by a reentrant call if this ever becomes recursive; re-find defensively.
	if (FFlexRoadNode* StillThere = Nodes.Find(NodeId))
	{
		SpatialGrid.RemoveNode(NodeId, FVector2D(StillThere->Position.X, StillThere->Position.Y));
	}

	if (JunctionDataByNode.Contains(NodeId))
	{
		if (MeshActor)
		{
			MeshActor->RemoveJunctionMesh(NodeId);
		}
		JunctionDataByNode.Remove(NodeId);
	}

	Nodes.Remove(NodeId);
	NodeIdAllocator.Free(NodeId);
	DirtyNodes.Remove(NodeId);
	return true;
}

bool UFlexNetworkSubsystem::SetNodePosition(FFlexNodeId NodeId, const FVector& NewPosition)
{
	FFlexRoadNode* Node = Nodes.Find(NodeId);
	if (!Node)
	{
		return false;
	}

	const FVector2D OldPos2D(Node->Position.X, Node->Position.Y);
	Node->Position = NewPosition;
	SpatialGrid.UpdateNode(NodeId, OldPos2D, FVector2D(NewPosition.X, NewPosition.Y));

	for (FFlexSegmentId SegId : Node->ConnectedSegments)
	{
		FFlexRoadSegment* Segment = Segments.Find(SegId);
		if (!Segment)
		{
			continue;
		}

		RemoveSegmentFromSpatialGrid(SegId, *Segment);

		// Keep the tangent handle's planform (X/Y) offset from its endpoint so the curve's shape
		// moves rigidly with the node; Z is re-derived by ApplyElevationEase below rather than
		// preserved, since it depends on both endpoints' (possibly just-changed) heights.
		if (Segment->StartNodeId == NodeId)
		{
			const FVector Offset = Segment->Curve.P1 - Segment->Curve.P0;
			Segment->Curve.P0 = NewPosition;
			Segment->Curve.P1 = FVector(NewPosition.X + Offset.X, NewPosition.Y + Offset.Y, Segment->Curve.P1.Z);
		}
		if (Segment->EndNodeId == NodeId)
		{
			const FVector Offset = Segment->Curve.P2 - Segment->Curve.P3;
			Segment->Curve.P3 = NewPosition;
			Segment->Curve.P2 = FVector(NewPosition.X + Offset.X, NewPosition.Y + Offset.Y, Segment->Curve.P2.Z);
		}
		ApplyElevationEase(*Segment);

		AddSegmentToSpatialGrid(SegId, *Segment);
		DirtySegments.Add(SegId);
	}

	DirtyNodes.Add(NodeId);
	RebuildDirty();
	return true;
}

bool UFlexNetworkSubsystem::RotateNode(FFlexNodeId NodeId, const FQuat& DeltaRotation)
{
	FFlexRoadNode* Node = Nodes.Find(NodeId);
	if (!Node || DeltaRotation.ContainsNaN())
	{
		return false;
	}

	const FQuat Rotation = DeltaRotation.GetNormalized();
	if (Rotation.IsIdentity())
	{
		return true;
	}

	Node->UpVector = Rotation.RotateVector(Node->UpVector)
		.GetSafeNormal(UE_SMALL_NUMBER, FVector::UpVector);

	for (FFlexSegmentId SegId : Node->ConnectedSegments)
	{
		FFlexRoadSegment* Segment = Segments.Find(SegId);
		if (!Segment)
		{
			continue;
		}

		RemoveSegmentFromSpatialGrid(SegId, *Segment);
		if (Segment->StartNodeId == NodeId)
		{
			Segment->Curve.P1 = Segment->Curve.P0
				+ Rotation.RotateVector(Segment->Curve.P1 - Segment->Curve.P0);
		}
		if (Segment->EndNodeId == NodeId)
		{
			Segment->Curve.P2 = Segment->Curve.P3
				+ Rotation.RotateVector(Segment->Curve.P2 - Segment->Curve.P3);
		}
		ApplyElevationEase(*Segment);
		AddSegmentToSpatialGrid(SegId, *Segment);
		DirtySegments.Add(SegId);
	}

	DirtyNodes.Add(NodeId);
	RebuildDirty();
	return true;
}

bool UFlexNetworkSubsystem::SetNodeElevationType(FFlexNodeId NodeId, EFlexRoadElevationType NewType)
{
	FFlexRoadNode* Node = Nodes.Find(NodeId);
	if (!Node)
	{
		return false;
	}
	Node->ElevationType = NewType;
	DirtyNodes.Add(NodeId);
	RebuildDirty();
	return true;
}

bool UFlexNetworkSubsystem::SetSegmentProfile(FFlexSegmentId SegmentId, URoadTypeProfile* NewProfile)
{
	FFlexRoadSegment* Segment = Segments.Find(SegmentId);
	if (!Segment || !NewProfile)
	{
		return false;
	}
	Segment->Profile = NewProfile;
	DirtySegments.Add(SegmentId);
	DirtyNodes.Add(Segment->StartNodeId);
	DirtyNodes.Add(Segment->EndNodeId);
	RebuildDirty();
	return true;
}

bool UFlexNetworkSubsystem::SetSegmentCurve(FFlexSegmentId SegmentId, const FVector& StartTangentHandle, const FVector& EndTangentHandle)
{
	FFlexRoadSegment* Segment = Segments.Find(SegmentId);
	if (!Segment)
	{
		return false;
	}

	RemoveSegmentFromSpatialGrid(SegmentId, *Segment);
	Segment->Curve.P1 = StartTangentHandle;
	Segment->Curve.P2 = EndTangentHandle;
	ApplyElevationEase(*Segment);
	AddSegmentToSpatialGrid(SegmentId, *Segment);

	DirtySegments.Add(SegmentId);
	DirtyNodes.Add(Segment->StartNodeId);
	DirtyNodes.Add(Segment->EndNodeId);
	RebuildDirty();
	return true;
}

FFlexNodeId UFlexNetworkSubsystem::SplitSegment(FFlexSegmentId SegmentId, float ArcLength)
{
	FFlexRoadSegment* Segment = Segments.Find(SegmentId);
	if (!Segment || !Segment->ArcLengthTable.IsValid())
	{
		return FFlexNodeId::Invalid();
	}

	const float ClampedArcLength = FMath::Clamp(ArcLength, 0.f, Segment->GetLength());
	const float OriginalLength = Segment->GetLength();
	const float T = FFlexBezierMath::ArcLengthToT(Segment->ArcLengthTable, ClampedArcLength);

	FFlexBezierCurve Left, Right;
	FFlexBezierMath::Subdivide(Segment->Curve, T, Left, Right);

	const FFlexNodeId OldStartId = Segment->StartNodeId;
	const FFlexNodeId OldEndId = Segment->EndNodeId;
	const FFlexRoadNode* OldStartNode = Nodes.Find(OldStartId);
	const FFlexRoadNode* OldEndNode = Nodes.Find(OldEndId);
	const int32 InheritedComplexRegion = OldStartNode && OldEndNode
		&& OldStartNode->ComplexIntersectionRegionIndex != INDEX_NONE
		&& OldStartNode->ComplexIntersectionRegionIndex == OldEndNode->ComplexIntersectionRegionIndex
		? OldStartNode->ComplexIntersectionRegionIndex : INDEX_NONE;
	URoadTypeProfile* Profile = Segment->Profile;
	const EFlexRoadElevationType ElevationType = Segment->ElevationType;
	const FFlexElevationProfile ElevationProfile = Segment->ElevationProfile;
	const FVector ReferenceUp = Nodes.Contains(OldStartId) ? Nodes.FindChecked(OldStartId).UpVector : FVector::UpVector;
	const FVector SplitUp = FFlexRoadMeshBuilder::SampleFrameAtArcLength(
		Segment->Curve, Segment->ArcLengthTable, ClampedArcLength, ReferenceUp).Up;

	const FFlexNodeId NewNodeId = AddNode(Left.P3, ElevationType, SplitUp);
	if (FFlexRoadNode* NewNode = Nodes.Find(NewNodeId))
	{
		NewNode->ComplexIntersectionRegionIndex = InheritedComplexRegion;
	}

	DetachSegmentFromNode(OldEndId, SegmentId);
	RemoveSegmentFromSpatialGrid(SegmentId, *Segment);

	Segment->EndNodeId = NewNodeId;
	Segment->Curve = Left;
	Segment->bDirty = true;
	AddSegmentToSpatialGrid(SegmentId, *Segment);
	if (FFlexRoadNode* NewNode = Nodes.Find(NewNodeId))
	{
		NewNode->ConnectedSegments.Add(SegmentId);
	}

	FFlexRoadSegment NewSegment;
	NewSegment.StartNodeId = NewNodeId;
	NewSegment.EndNodeId = OldEndId;
	NewSegment.Curve = Right;
	NewSegment.Profile = Profile;
	NewSegment.ElevationType = ElevationType;
	NewSegment.ElevationProfile = ElevationProfile;
	NewSegment.bDirty = true;

	const FFlexSegmentId NewSegmentId = SegmentIdAllocator.Allocate();
	AddSegmentToSpatialGrid(NewSegmentId, NewSegment);
	Segments.Add(NewSegmentId, NewSegment);

	if (FFlexRoadNode* NewNode = Nodes.Find(NewNodeId))
	{
		NewNode->ConnectedSegments.Add(NewSegmentId);
	}
	if (FFlexRoadNode* EndNode = Nodes.Find(OldEndId))
	{
		EndNode->ConnectedSegments.Add(NewSegmentId);
	}

	// Preserve authoritative signal attachments across the topology edit. Physical anchors after
	// the cut move to the new right-hand segment; a control aimed at the old end junction must use
	// that same new segment as its inbound approach.
	bool bSignalAttachmentChanged = false;
	for (TPair<FGuid, FFlexTrafficSignal>& Pair : TrafficSignals)
	{
		FFlexTrafficSignal& Signal = Pair.Value;
		if (Signal.AnchorSegmentId == SegmentId)
		{
			const float OldAnchorArc = FMath::Clamp(Signal.AnchorFraction, 0.f, 1.f) * OriginalLength;
			if (OldAnchorArc > ClampedArcLength && OriginalLength - ClampedArcLength > KINDA_SMALL_NUMBER)
			{
				Signal.AnchorSegmentId = NewSegmentId;
				Signal.AnchorFraction = (OldAnchorArc - ClampedArcLength) / (OriginalLength - ClampedArcLength);
			}
			else
			{
				Signal.AnchorFraction = ClampedArcLength > KINDA_SMALL_NUMBER
					? OldAnchorArc / ClampedArcLength : 0.f;
			}
			bSignalAttachmentChanged = true;
		}
		if (Signal.ControlledApproachSegmentId == SegmentId
			&& Signal.ControlledJunctionNodeId == OldEndId)
		{
			Signal.ControlledApproachSegmentId = NewSegmentId;
			bSignalAttachmentChanged = true;
		}
	}

	DirtySegments.Add(SegmentId);
	DirtySegments.Add(NewSegmentId);
	DirtyNodes.Add(OldStartId);
	DirtyNodes.Add(NewNodeId);
	DirtyNodes.Add(OldEndId);
	RebuildDirty();
	if (bSignalAttachmentChanged)
	{
		BroadcastTrafficSignalsChanged();
	}

	return NewNodeId;
}

// ---------------------------------------------------------------- Terrain (bulk editor commands)

void UFlexNetworkSubsystem::ConformAllTerrainToRoads()
{
	if (!TerrainConformer)
	{
		return;
	}
	// Reuses RebuildDirty()'s existing per-segment conforming (and its Ground-only guard) rather
	// than duplicating the frame-building/conforming call here -- marking everything dirty and
	// letting the normal rebuild path run is simpler and can't drift out of sync with it.
	for (const TPair<FFlexSegmentId, FFlexRoadSegment>& Pair : Segments)
	{
		DirtySegments.Add(Pair.Key);
	}
	RebuildDirty();
}

void UFlexNetworkSubsystem::FitNodesToTerrain()
{
	if (!TerrainConformer)
	{
		return;
	}

	UWorld* World = GetWorld();
	BeginBatchUpdate();
	for (TPair<FFlexNodeId, FFlexRoadNode>& Pair : Nodes)
	{
		if (Pair.Value.ElevationType != EFlexRoadElevationType::Ground)
		{
			continue;
		}
		float SampledHeight = 0.f;
		if (TerrainConformer->SampleHeight(World, FVector2D(Pair.Value.Position.X, Pair.Value.Position.Y), SampledHeight))
		{
			SetNodePosition(Pair.Key, FVector(Pair.Value.Position.X, Pair.Value.Position.Y, SampledHeight));
		}
	}
	EndBatchUpdate();
}

// ---------------------------------------------------------------- Snap / crossing queries

bool UFlexNetworkSubsystem::FindNearestNode(const FVector& WorldPosition, float Radius, FFlexNodeId& OutNodeId) const
{
	const FVector2D Pos2D(WorldPosition.X, WorldPosition.Y);
	const TArray<FFlexNodeId> Candidates = SpatialGrid.QueryNodesNear(Pos2D, Radius);

	float BestDistSq = FMath::Square(Radius);
	bool bFound = false;
	for (FFlexNodeId Id : Candidates)
	{
		const FFlexRoadNode* Node = Nodes.Find(Id);
		if (!Node)
		{
			continue;
		}
		const float DistSq = FVector2D::DistSquared(Pos2D, FVector2D(Node->Position.X, Node->Position.Y));
		if (DistSq <= BestDistSq)
		{
			BestDistSq = DistSq;
			OutNodeId = Id;
			bFound = true;
		}
	}
	return bFound;
}

bool UFlexNetworkSubsystem::FindNearestSegmentPoint(const FVector& WorldPosition, float Radius, FFlexSegmentId& OutSegmentId, float& OutArcLength, FVector& OutPointOnCurve) const
{
	const FVector2D Pos2D(WorldPosition.X, WorldPosition.Y);
	const TArray<FFlexSegmentId> Candidates = SpatialGrid.QuerySegmentsNear(Pos2D, Radius);

	float BestDistSq = FMath::Square(Radius);
	bool bFound = false;

	for (FFlexSegmentId Id : Candidates)
	{
		const FFlexRoadSegment* Segment = Segments.Find(Id);
		if (!Segment || !Segment->ArcLengthTable.IsValid())
		{
			continue;
		}

		for (const FFlexArcLengthSample& Sample : Segment->ArcLengthTable.Samples)
		{
			const FVector P = FFlexBezierMath::Evaluate(Segment->Curve, Sample.T);
			const float DistSq = FVector2D::DistSquared(Pos2D, FVector2D(P.X, P.Y));
			if (DistSq <= BestDistSq)
			{
				BestDistSq = DistSq;
				OutSegmentId = Id;
				OutArcLength = Sample.ArcLength;
				OutPointOnCurve = P;
				bFound = true;
			}
		}
	}
	return bFound;
}

TArray<FFlexSegmentCrossing> UFlexNetworkSubsystem::FindCrossings(const FFlexBezierCurve& ProposedCurve) const
{
	TArray<FFlexSegmentCrossing> Result;

	FVector BoundsMin = ProposedCurve.P0;
	FVector BoundsMax = ProposedCurve.P0;
	for (const FVector& P : { ProposedCurve.P1, ProposedCurve.P2, ProposedCurve.P3 })
	{
		BoundsMin = BoundsMin.ComponentMin(P);
		BoundsMax = BoundsMax.ComponentMax(P);
	}
	const FVector2D Center2D((BoundsMin.X + BoundsMax.X) * 0.5, (BoundsMin.Y + BoundsMax.Y) * 0.5);
	const float SearchRadius = static_cast<float>(FVector2D::Distance(FVector2D(BoundsMin.X, BoundsMin.Y), FVector2D(BoundsMax.X, BoundsMax.Y)) * 0.5 + 1.0);

	const TArray<FFlexSegmentId> Candidates = SpatialGrid.QuerySegmentsNear(Center2D, SearchRadius);

	const FFlexArcLengthTable ProposedTable = FFlexBezierMath::BuildArcLengthTable(ProposedCurve);
	TArray<FVector> ProposedPolyline;
	ProposedPolyline.Reserve(ProposedTable.Samples.Num());
	for (const FFlexArcLengthSample& Sample : ProposedTable.Samples)
	{
		ProposedPolyline.Add(FFlexBezierMath::Evaluate(ProposedCurve, Sample.T));
	}

	for (FFlexSegmentId Id : Candidates)
	{
		const FFlexRoadSegment* Segment = Segments.Find(Id);
		if (!Segment || !Segment->ArcLengthTable.IsValid())
		{
			continue;
		}

		for (int32 i = 0; i + 1 < ProposedPolyline.Num(); ++i)
		{
			for (int32 j = 0; j + 1 < Segment->ArcLengthTable.Samples.Num(); ++j)
			{
				const FVector ExistingA = FFlexBezierMath::Evaluate(Segment->Curve, Segment->ArcLengthTable.Samples[j].T);
				const FVector ExistingB = FFlexBezierMath::Evaluate(Segment->Curve, Segment->ArcLengthTable.Samples[j + 1].T);

				FVector2D CrossPoint;
				float AlphaProposed = 0.f, AlphaExisting = 0.f;
				if (FlexGeometry2D::SegmentSegmentIntersection(
					FVector2D(ProposedPolyline[i].X, ProposedPolyline[i].Y), FVector2D(ProposedPolyline[i + 1].X, ProposedPolyline[i + 1].Y),
					FVector2D(ExistingA.X, ExistingA.Y), FVector2D(ExistingB.X, ExistingB.Y),
					CrossPoint, AlphaProposed, AlphaExisting))
				{
					FFlexSegmentCrossing Crossing;
					Crossing.ExistingSegmentId = Id;
					Crossing.ArcLengthOnExistingSegment = FMath::Lerp(Segment->ArcLengthTable.Samples[j].ArcLength, Segment->ArcLengthTable.Samples[j + 1].ArcLength, AlphaExisting);
					Crossing.ArcLengthOnProposedCurve = FMath::Lerp(ProposedTable.Samples[i].ArcLength, ProposedTable.Samples[i + 1].ArcLength, AlphaProposed);
					Crossing.WorldPoint = FVector(CrossPoint.X, CrossPoint.Y, FMath::Lerp(ExistingA.Z, ExistingB.Z, AlphaExisting));
					Result.Add(Crossing);
				}
			}
		}
	}

	// Two polyline sub-segments that share a vertex very close to the true crossing point can
	// each independently register a hit there (once per curve, so up to 4 near-identical entries
	// for what is geometrically one crossing) -- collapse those down to one per existing segment.
	TArray<FFlexSegmentCrossing> Deduped;
	Deduped.Reserve(Result.Num());
	for (const FFlexSegmentCrossing& Candidate : Result)
	{
		const bool bAlreadyHave = Deduped.ContainsByPredicate([&Candidate](const FFlexSegmentCrossing& Existing)
		{
			return Existing.ExistingSegmentId == Candidate.ExistingSegmentId
				&& FVector::DistSquared(Existing.WorldPoint, Candidate.WorldPoint) < FMath::Square(10.f);
		});
		if (!bAlreadyHave)
		{
			Deduped.Add(Candidate);
		}
	}

	return Deduped;
}

FVector UFlexNetworkSubsystem::SuggestOutgoingTangentDirection(FFlexNodeId NodeId) const
{
	const FFlexRoadNode* Node = Nodes.Find(NodeId);
	if (!Node || Node->ConnectedSegments.Num() == 0)
	{
		return FVector::ForwardVector;
	}

	FVector Accum = FVector::ZeroVector;
	for (FFlexSegmentId Id : Node->ConnectedSegments)
	{
		const FFlexRoadSegment* Segment = Segments.Find(Id);
		if (!Segment)
		{
			continue;
		}
		const bool bIsEnd = Segment->EndNodeId == NodeId;
		// Direction of travel arriving at this node, whichever end of the segment it is.
		const FVector InwardTangent = bIsEnd
			? FFlexBezierMath::EvaluateDerivative(Segment->Curve, 1.f)
			: -FFlexBezierMath::EvaluateDerivative(Segment->Curve, 0.f);
		Accum += InwardTangent.GetSafeNormal();
	}
	return Accum.GetSafeNormal(UE_SMALL_NUMBER, FVector::ForwardVector);
}

bool UFlexNetworkSubsystem::ValidateProposedSegment(const FFlexBezierCurve& Curve, const URoadTypeProfile* Profile, FText& OutReason) const
{
	const UFlexNetworkSettings* Settings = GetSettings();
	const float MinLength = (Profile && Profile->MinSegmentLengthOverride > 0.f) ? Profile->MinSegmentLengthOverride : Settings->MinSegmentLength;
	const float MinRadius = Profile ? Profile->MinTurnRadius : 0.f;

	const FFlexArcLengthTable Table = FFlexBezierMath::BuildArcLengthTable(Curve, Settings->ArcLengthChordTolerance, Settings->MaxArcLengthSubdivisionDepth);
	const float Length = Table.GetTotalLength();

	if (Length < MinLength)
	{
		OutReason = FText::Format(NSLOCTEXT("FlexNetwork", "TooShort", "Segment is too short ({0} cm, minimum {1} cm)."), FText::AsNumber(Length), FText::AsNumber(MinLength));
		return false;
	}

	if (MinRadius > 0.f)
	{
		for (const FFlexArcLengthSample& Sample : Table.Samples)
		{
			const float Curvature = FFlexBezierMath::EstimateCurvature(Curve, Sample.T);
			if (Curvature > KINDA_SMALL_NUMBER && (1.f / Curvature) < MinRadius)
			{
				OutReason = NSLOCTEXT("FlexNetwork", "TooSharp", "Curve is tighter than this road type's minimum turn radius.");
				return false;
			}
		}
	}

	TArray<FVector> Polyline;
	Polyline.Reserve(Table.Samples.Num());
	for (const FFlexArcLengthSample& Sample : Table.Samples)
	{
		Polyline.Add(FFlexBezierMath::Evaluate(Curve, Sample.T));
	}

	for (int32 i = 0; i + 1 < Polyline.Num(); ++i)
	{
		for (int32 j = i + 2; j + 1 < Polyline.Num(); ++j)
		{
			FVector2D CrossPoint;
			float AlphaA = 0.f, AlphaB = 0.f;
			if (FlexGeometry2D::SegmentSegmentIntersection(
				FVector2D(Polyline[i].X, Polyline[i].Y), FVector2D(Polyline[i + 1].X, Polyline[i + 1].Y),
				FVector2D(Polyline[j].X, Polyline[j].Y), FVector2D(Polyline[j + 1].X, Polyline[j + 1].Y),
				CrossPoint, AlphaA, AlphaB))
			{
				OutReason = NSLOCTEXT("FlexNetwork", "SelfIntersecting", "Curve self-intersects.");
				return false;
			}
		}
	}

	OutReason = FText::GetEmpty();
	return true;
}

// ---------------------------------------------------------------- Query API

TArray<FFlexSegmentId> UFlexNetworkSubsystem::GetConnectedSegments(FFlexNodeId NodeId) const
{
	if (const FFlexRoadNode* Node = Nodes.Find(NodeId))
	{
		return Node->ConnectedSegments;
	}
	return {};
}

TArray<FFlexLaneConnector> UFlexNetworkSubsystem::GetLaneConnectorsAtNode(FFlexNodeId NodeId) const
{
	if (const FFlexJunctionData* Data = JunctionDataByNode.Find(NodeId))
	{
		return Data->LaneConnectors;
	}
	return {};
}

bool UFlexNetworkSubsystem::IsInternalComplexIntersectionSegment(const FFlexRoadSegment& Segment) const
{
	const FFlexRoadNode* StartNode = Nodes.Find(Segment.StartNodeId);
	const FFlexRoadNode* EndNode = Nodes.Find(Segment.EndNodeId);
	return StartNode && EndNode
		&& StartNode->ComplexIntersectionRegionIndex != INDEX_NONE
		&& StartNode->ComplexIntersectionRegionIndex == EndNode->ComplexIntersectionRegionIndex;
}

bool UFlexNetworkSubsystem::IsComplexIntersectionRegionOwner(FFlexNodeId NodeId) const
{
	const FFlexRoadNode* Node = Nodes.Find(NodeId);
	if (!Node || Node->ComplexIntersectionRegionIndex == INDEX_NONE)
	{
		return false;
	}
	for (const TPair<FFlexNodeId, FFlexRoadNode>& Pair : Nodes)
	{
		if (Pair.Value.ComplexIntersectionRegionIndex == Node->ComplexIntersectionRegionIndex
			&& JunctionDataByNode.Contains(Pair.Key)
			&& (Pair.Key.Index < NodeId.Index
				|| (Pair.Key.Index == NodeId.Index && Pair.Key.Generation < NodeId.Generation)))
		{
			return false;
		}
	}
	return true;
}

bool UFlexNetworkSubsystem::BuildComplexIntersectionRegionSurface(int32 RegionIndex,
	FFlexUnifiedRoadPolygonInput& OutSurface) const
{
	OutSurface = FFlexUnifiedRoadPolygonInput();
	if (RegionIndex == INDEX_NONE)
	{
		return false;
	}

	TSet<FFlexNodeId> MemberNodes;
	for (const TPair<FFlexNodeId, FFlexRoadNode>& Pair : Nodes)
	{
		if (Pair.Value.ComplexIntersectionRegionIndex == RegionIndex)
		{
			MemberNodes.Add(Pair.Key);
		}
	}
	if (MemberNodes.Num() < 2)
	{
		return false;
	}

	TArray<FVector> MouthCorners;
	const FFlexRoadSegment* MaterialSegment = nullptr;
	float WidestRoad = 0.f;
	for (const TPair<FFlexSegmentId, FFlexRoadSegment>& Pair : Segments)
	{
		const FFlexRoadSegment& Segment = Pair.Value;
		if (!Segment.Profile || Segment.Profile->bIsRailProfile || !Segment.ArcLengthTable.IsValid())
		{
			continue;
		}
		const bool bStartInside = MemberNodes.Contains(Segment.StartNodeId);
		const bool bEndInside = MemberNodes.Contains(Segment.EndNodeId);
		if (!bStartInside && !bEndInside)
		{
			continue;
		}

		const float RoadWidth = Segment.Profile->GetRoadwayWidth();
		if (!MaterialSegment || RoadWidth > WidestRoad)
		{
			MaterialSegment = &Segment;
			WidestRoad = RoadWidth;
		}
		if (bStartInside == bEndInside)
		{
			continue; // Internal routing link: consumed by the region, not an outer mouth.
		}

		const FFlexRoadNode* StartNode = Nodes.Find(Segment.StartNodeId);
		const FVector ReferenceUp = StartNode ? StartNode->UpVector : FVector::UpVector;
		float MouthArcLength = bStartInside ? 0.f : Segment.GetLength();
		const FFlexNodeId PortalNodeId = bStartInside ? Segment.StartNodeId : Segment.EndNodeId;
		if (const FFlexJunctionData* PortalJunction = JunctionDataByNode.Find(PortalNodeId))
		{
			if (const float* TrimArcLength = PortalJunction->TrimArcLengthBySegment.Find(Pair.Key))
			{
				MouthArcLength = *TrimArcLength;
			}
		}
		const FFlexCurveFrame Frame = FFlexRoadMeshBuilder::SampleFrameAtArcLength(
			Segment.Curve, Segment.ArcLengthTable, MouthArcLength, ReferenceUp);
		MouthCorners.Add(Frame.Position + Frame.Right * Segment.Profile->GetRoadwayMinOffset());
		MouthCorners.Add(Frame.Position + Frame.Right * Segment.Profile->GetRoadwayMaxOffset());
	}

	if (!MaterialSegment || MouthCorners.Num() < 3)
	{
		return false;
	}
	OutSurface.Boundary = BuildConvexHullXY(MouthCorners);
	if (OutSurface.Boundary.Num() < 3)
	{
		return false;
	}

	const URoadTypeProfile* Profile = MaterialSegment->Profile;
	OutSurface.ElevationLayer = static_cast<int32>(MaterialSegment->ElevationType);
	OutSurface.SidewalkWidth = Profile->SidewalkWidth;
	OutSurface.CurbHeight = Profile->CurbHeight;
	OutSurface.RoadMaterial = Profile->JunctionMaterial ? Profile->JunctionMaterial.Get() : Profile->RoadMaterial.Get();
	OutSurface.SidewalkMaterial = Profile->SidewalkMaterial;
	OutSurface.CurbMaterial = Profile->CurbMaterial ? Profile->CurbMaterial.Get() : Profile->SidewalkMaterial.Get();
	return true;
}

bool UFlexNetworkSubsystem::BuildSegmentMeshResult(FFlexSegmentId SegmentId, FFlexSegmentMeshResult& OutResult) const
{
	const FFlexRoadSegment* Segment = Segments.Find(SegmentId);
	if (!Segment || !Segment->Profile || !Segment->ArcLengthTable.IsValid())
	{
		return false;
	}
	if (IsInternalComplexIntersectionSegment(*Segment))
	{
		// This edge remains in the lane graph, but its physical footprint is represented by the
		// shared multi-port region. Emitting it separately recreates interior curbs/sidewalks.
		OutResult = FFlexSegmentMeshResult();
		return false;
	}

	float TrimStart = 0.f, TrimEnd = 0.f;
	if (Segment->Profile->bIsRailProfile)
	{
		// Rail junctions use overlapping continuous strips rather than a filled road polygon, so
		// there is no junction surface for a trim to meet.
		TrimEnd = Segment->GetLength();
	}
	else if (!GetSegmentTrimRange(SegmentId, TrimStart, TrimEnd))
	{
		return false;
	}

	const FFlexRoadNode* StartNode = Nodes.Find(Segment->StartNodeId);
	OutResult = FFlexRoadMeshBuilder::BuildSegmentMesh(Segment->Curve, Segment->ArcLengthTable,
		Segment->Profile, StartNode ? StartNode->UpVector : FVector::UpVector,
		GetSettings()->ArcLengthSampleStep, TrimStart, TrimEnd);
	return !OutResult.Roadway.IsEmpty() || !OutResult.Sidewalks.IsEmpty();
}

bool UFlexNetworkSubsystem::GetSegmentTrimRange(FFlexSegmentId SegmentId, float& OutTrimStart, float& OutTrimEnd) const
{
	const FFlexRoadSegment* Segment = Segments.Find(SegmentId);
	if (!Segment || !Segment->ArcLengthTable.IsValid()) return false;
	OutTrimStart = 0.f;
	OutTrimEnd = Segment->GetLength();
	if (const FFlexJunctionData* StartJunction = JunctionDataByNode.Find(Segment->StartNodeId))
	{
		if (const float* Trim = StartJunction->TrimArcLengthBySegment.Find(SegmentId))
		{
			OutTrimStart = *Trim;
		}
	}
	if (const FFlexJunctionData* EndJunction = JunctionDataByNode.Find(Segment->EndNodeId))
	{
		if (const float* Trim = EndJunction->TrimArcLengthBySegment.Find(SegmentId))
		{
			OutTrimEnd = *Trim;
		}
	}
	OutTrimStart = FMath::Clamp(OutTrimStart, 0.f, Segment->GetLength());
	OutTrimEnd = FMath::Clamp(OutTrimEnd, OutTrimStart, Segment->GetLength());
	return true;
}

bool UFlexNetworkSubsystem::BuildJunctionMeshResult(FFlexNodeId NodeId, FFlexJunctionMeshResult& OutResult) const
{
	const FFlexRoadNode* Node = Nodes.Find(NodeId);
	const FFlexJunctionData* Junction = JunctionDataByNode.Find(NodeId);
	if (!Node || !Junction || Junction->IsEmpty())
	{
		return false;
	}
	bool bAllApproachesAreRail = !Node->ConnectedSegments.IsEmpty();
	for (const FFlexSegmentId SegmentId : Node->ConnectedSegments)
	{
		const FFlexRoadSegment* Segment = Segments.Find(SegmentId);
		bAllApproachesAreRail &= Segment && Segment->Profile && Segment->Profile->bIsRailProfile;
	}
	if (bAllApproachesAreRail)
	{
		return false;
	}

	UMaterialInterface* JunctionMaterial = nullptr;
	UMaterialInterface* CrosswalkMaterial = nullptr;
	UMaterialInterface* SidewalkMaterial = nullptr;
	UMaterialInterface* MedianMaterial = nullptr;
	for (FFlexSegmentId SegmentId : Node->ConnectedSegments)
	{
		if (const FFlexRoadSegment* Segment = Segments.Find(SegmentId); Segment && Segment->Profile)
		{
			if (Segment->Profile->bIsRailProfile)
			{
				continue;
			}
			JunctionMaterial = JunctionMaterial ? JunctionMaterial : Segment->Profile->JunctionMaterial.Get();
			CrosswalkMaterial = CrosswalkMaterial ? CrosswalkMaterial : Segment->Profile->CrosswalkMaterial.Get();
			SidewalkMaterial = SidewalkMaterial ? SidewalkMaterial : Segment->Profile->SidewalkMaterial.Get();
			MedianMaterial = MedianMaterial ? MedianMaterial : Segment->Profile->MedianMaterial.Get();
		}
	}
	FFlexJunctionData MeshJunction = *Junction;
	if (Node->ComplexIntersectionRegionIndex != INDEX_NONE)
	{
		MeshJunction.Crosswalks.RemoveAll([this](const FFlexCrosswalkPlacement& Crosswalk)
		{
			const FFlexRoadSegment* Segment = Segments.Find(Crosswalk.SegmentId);
			return Segment && IsInternalComplexIntersectionSegment(*Segment);
		});
	}
	OutResult = FFlexIntersectionBuilder::BuildJunctionMesh(Node->UpVector, MeshJunction,
		JunctionMaterial, CrosswalkMaterial ? CrosswalkMaterial : SidewalkMaterial, SidewalkMaterial, MedianMaterial);
	if (Node->ComplexIntersectionRegionIndex != INDEX_NONE)
	{
		// The individual portal junctions still own their crosswalks and lane connectors, but
		// their overlapping local surfaces/islands are replaced by one region-wide surface.
		OutResult.Surface = FFlexMeshSectionData();
		OutResult.SidewalkCorners = FFlexMeshSectionData();
		OutResult.CornerIslands = FFlexMeshSectionData();
		if (IsComplexIntersectionRegionOwner(NodeId))
		{
			FFlexUnifiedRoadPolygonInput RegionSurface;
			if (BuildComplexIntersectionRegionSurface(Node->ComplexIntersectionRegionIndex, RegionSurface))
			{
				OutResult.Surface.Material = RegionSurface.RoadMaterial;
				OutResult.Surface.bEnableCollision = true;
				const FVector Up = Node->UpVector.GetSafeNormal(UE_SMALL_NUMBER, FVector::UpVector);
				const FVector Tangent = FVector::VectorPlaneProject(FVector::ForwardVector, Up)
					.GetSafeNormal(UE_SMALL_NUMBER, FVector::ForwardVector);
				for (int32 Index = 1; Index + 1 < RegionSurface.Boundary.Num(); ++Index)
				{
					OutResult.Surface.AppendTriangle(RegionSurface.Boundary[0], RegionSurface.Boundary[Index],
						RegionSurface.Boundary[Index + 1], Up, Tangent,
						FVector2D(RegionSurface.Boundary[0].X, RegionSurface.Boundary[0].Y) * 0.01f,
						FVector2D(RegionSurface.Boundary[Index].X, RegionSurface.Boundary[Index].Y) * 0.01f,
						FVector2D(RegionSurface.Boundary[Index + 1].X, RegionSurface.Boundary[Index + 1].Y) * 0.01f);
				}
			}
		}
	}
	return !OutResult.Surface.IsEmpty() || !OutResult.Crosswalks.IsEmpty()
		|| !OutResult.SidewalkCorners.IsEmpty() || !OutResult.CornerIslands.IsEmpty();
}

void UFlexNetworkSubsystem::BuildRailMeshResults(TArray<FFlexMeshSectionData>& OutResults) const
{
	OutResults.Reset();
	TMap<const URoadTypeProfile*, TArray<FFlexRailSweepInput>> SweepsByProfile;
	const UFlexNetworkSettings* Settings = GetSettings();
	for (const TPair<FFlexSegmentId, FFlexRoadSegment>& Pair : Segments)
	{
		const FFlexRoadSegment& Segment = Pair.Value;
		if (!Segment.Profile || !Segment.Profile->bIsRailProfile || !Segment.ArcLengthTable.IsValid())
		{
			continue;
		}
		const FFlexRoadNode* StartNode = Nodes.Find(Segment.StartNodeId);
		FFlexRailSweepInput Sweep;
		Sweep.Frames = FFlexRoadMeshBuilder::BuildFramesForRange(Segment.Curve, Segment.ArcLengthTable,
			StartNode ? StartNode->UpVector : FVector::UpVector, Settings->ArcLengthSampleStep,
			0.f, Segment.GetLength());
		if (Sweep.Frames.Num() >= 2)
		{
			SweepsByProfile.FindOrAdd(Segment.Profile.Get()).Add(MoveTemp(Sweep));
		}
	}

	for (const TPair<const URoadTypeProfile*, TArray<FFlexRailSweepInput>>& Pair : SweepsByProfile)
	{
		FFlexMeshSectionData Section;
		if (FFlexRailMeshBuilder::BuildRailMesh(Pair.Value, Pair.Key, Section))
		{
			OutResults.Add(MoveTemp(Section));
		}
	}
}

FFlexUnifiedNetworkMeshResult UFlexNetworkSubsystem::BuildUnifiedClassicMeshResult() const
{
	TArray<FFlexUnifiedRoadPolygonInput> SurfaceInputs;
	TArray<FFlexUnifiedRoadSuppressionInput> SuppressionInputs;
	const UFlexNetworkSettings* Settings = GetSettings();

	// Start with the current angle-trimmed segment strips. A short segment whose two junction
	// trims leave too little usable roadside is deliberately bridged at full length; its expanded
	// footprint is also supplied as a sidewalk/curb suppression region below.
	for (const TPair<FFlexSegmentId, FFlexRoadSegment>& Pair : Segments)
	{
		const FFlexSegmentId SegmentId = Pair.Key;
		const FFlexRoadSegment& Segment = Pair.Value;
		if (!Segment.Profile || !Segment.ArcLengthTable.IsValid())
		{
			continue;
		}
		if (IsInternalComplexIntersectionSegment(Segment))
		{
			continue;
		}

		const float SegmentLength = Segment.GetLength();
		float RawTrimStart = 0.f;
		float RawTrimEnd = SegmentLength;
		const FFlexJunctionData* StartJunction = JunctionDataByNode.Find(Segment.StartNodeId);
		const FFlexJunctionData* EndJunction = JunctionDataByNode.Find(Segment.EndNodeId);
		if (StartJunction)
		{
			if (const float* Trim = StartJunction->TrimArcLengthBySegment.Find(SegmentId))
			{
				RawTrimStart = FMath::Clamp(*Trim, 0.f, SegmentLength);
			}
		}
		if (EndJunction)
		{
			if (const float* Trim = EndJunction->TrimArcLengthBySegment.Find(SegmentId))
			{
				RawTrimEnd = FMath::Clamp(*Trim, 0.f, SegmentLength);
			}
		}

		const bool bRailProfile = Segment.Profile->bIsRailProfile;
		const float AvailableRoadsideLength = FMath::Max(0.f, RawTrimEnd - RawTrimStart);
		// The offset-based sidewalk pass can safely build even a short exposed roadside. Suppress it
		// only when the two junction trims actually meet/overlap, plus any explicit user-requested
		// clearance. The previous implicit max(SidewalkWidth * 2, 600 cm) classified ordinary,
		// visibly separate intersections as one consumed region and removed the full intermediate
		// sidewalk strip.
		const float CloseJunctionTolerance = FMath::Max(Settings->CloseJunctionRoadsideClearance, KINDA_SMALL_NUMBER);
		const bool bCloseJunctionBridge = !bRailProfile && StartJunction && EndJunction
			&& AvailableRoadsideLength <= CloseJunctionTolerance;
		const float SurfaceStart = bRailProfile || bCloseJunctionBridge ? 0.f : RawTrimStart;
		const float SurfaceEnd = bRailProfile || bCloseJunctionBridge ? SegmentLength : FMath::Max(RawTrimStart, RawTrimEnd);
		const FVector ReferenceUp = Nodes.Contains(Segment.StartNodeId) ? Nodes.FindChecked(Segment.StartNodeId).UpVector : FVector::UpVector;
		const TArray<FFlexCurveFrame> Frames = FFlexRoadMeshBuilder::BuildFramesForRange(
			Segment.Curve, Segment.ArcLengthTable, ReferenceUp, Settings->ArcLengthSampleStep, SurfaceStart, SurfaceEnd);

		if (bRailProfile)
		{
			// Physical rails are built as closed solids in one profile-wide pass below. Keeping
			// them out of the 2D road union also preserves raised rails at level crossings.
			continue;
		}

		FFlexUnifiedRoadPolygonInput Surface;
		Surface.Boundary = BuildRoadFootprint(Frames, Segment.Profile->GetRoadwayMinOffset(), Segment.Profile->GetRoadwayMaxOffset());
		Surface.ElevationLayer = static_cast<int32>(Segment.ElevationType);
		Surface.SidewalkWidth = Segment.Profile->SidewalkWidth;
		Surface.CurbHeight = Segment.Profile->CurbHeight;
		Surface.RoadMaterial = Segment.Profile->RoadMaterial;
		Surface.SidewalkMaterial = Segment.Profile->SidewalkMaterial;
		Surface.CurbMaterial = Segment.Profile->CurbMaterial ? Segment.Profile->CurbMaterial.Get() : Segment.Profile->SidewalkMaterial.Get();
		if (Surface.Boundary.Num() >= 3)
		{
			SurfaceInputs.Add(MoveTemp(Surface));
		}

		if (bCloseJunctionBridge)
		{
			// Keep the curb-line strictly inside the suppression polygon even for profiles whose
			// sidewalk width is zero; containment on a coincident polygon edge is intentionally
			// undefined in the boolean library.
			const float SuppressionExpansion = FMath::Max(Segment.Profile->SidewalkWidth, 50.f);
			FFlexUnifiedRoadSuppressionInput Suppression;
			Suppression.Boundary = BuildRoadFootprint(Frames,
				Segment.Profile->GetRoadwayMinOffset() - SuppressionExpansion,
				Segment.Profile->GetRoadwayMaxOffset() + SuppressionExpansion);
			Suppression.ElevationLayer = static_cast<int32>(Segment.ElevationType);
			if (Suppression.Boundary.Num() >= 3)
			{
				SuppressionInputs.Add(MoveTemp(Suppression));
			}
		}
	}

	// A compact OSM intersection keeps its original routing portals and headings. Fill the
	// convex envelope of their physical road mouths once; the short internal routing links were
	// deliberately omitted above, so they cannot create a central hole or interior sidewalks.
	TSet<int32> AddedComplexRegions;
	for (const TPair<FFlexNodeId, FFlexRoadNode>& Pair : Nodes)
	{
		const int32 RegionIndex = Pair.Value.ComplexIntersectionRegionIndex;
		if (RegionIndex == INDEX_NONE || AddedComplexRegions.Contains(RegionIndex))
		{
			continue;
		}
		FFlexUnifiedRoadPolygonInput RegionSurface;
		if (BuildComplexIntersectionRegionSurface(RegionIndex, RegionSurface))
		{
			SurfaceInputs.Add(MoveTemp(RegionSurface));
			AddedComplexRegions.Add(RegionIndex);
		}
	}

	// Junction surfaces use the same boolean input as road strips. Their angle-dependent trim
	// polygons therefore replace, rather than overlap, the road ends and are automatically merged
	// with close neighbouring junctions through the bridge strip above.
	for (const TPair<FFlexNodeId, FFlexJunctionData>& Pair : JunctionDataByNode)
	{
		const FFlexRoadNode* Node = Nodes.Find(Pair.Key);
		if (!Node || Pair.Value.PolygonBoundary.Num() < 3)
		{
			continue;
		}

		const FFlexRoadSegment* MaterialSegment = nullptr;
		for (const FFlexSegmentId SegmentId : Node->ConnectedSegments)
		{
			if (const FFlexRoadSegment* Candidate = Segments.Find(SegmentId); Candidate && Candidate->Profile)
			{
				if (!MaterialSegment || !Candidate->Profile->bIsRailProfile)
				{
					MaterialSegment = Candidate;
				}
				if (!Candidate->Profile->bIsRailProfile)
				{
					break;
				}
			}
		}
		if (!MaterialSegment || !MaterialSegment->Profile)
		{
			continue;
		}
		if (MaterialSegment->Profile->bIsRailProfile)
		{
			continue;
		}

		FFlexUnifiedRoadPolygonInput Surface;
		Surface.Boundary = Pair.Value.PolygonBoundary;
		Surface.ElevationLayer = static_cast<int32>(MaterialSegment->ElevationType);
		Surface.SidewalkWidth = MaterialSegment->Profile->SidewalkWidth;
		Surface.CurbHeight = MaterialSegment->Profile->CurbHeight;
		Surface.RoadMaterial = MaterialSegment->Profile->JunctionMaterial
			? MaterialSegment->Profile->JunctionMaterial.Get()
			: MaterialSegment->Profile->RoadMaterial.Get();
		Surface.SidewalkMaterial = MaterialSegment->Profile->SidewalkMaterial;
		Surface.CurbMaterial = MaterialSegment->Profile->CurbMaterial
			? MaterialSegment->Profile->CurbMaterial.Get()
			: MaterialSegment->Profile->SidewalkMaterial.Get();
		SurfaceInputs.Add(MoveTemp(Surface));

		// A crosswalk is also a curb cut. Reserve a slightly expanded rectangle at each crossing so
		// neither the procedural curb face nor the optional spline curbstones bridge across it. This
		// suppression is curb-only: the sidewalk surface beyond the road edge remains continuous.
		constexpr float CrosswalkCurbDepthPadding = 50.f;
		// Half the maximum classic curb chord. This makes midpoint-based chord rejection
		// conservative: a chord touching the crosswalk cannot survive by only a few centimeters.
		constexpr float CrosswalkSidePadding = 50.f;
		const FVector Up = Node->UpVector.GetSafeNormal(UE_SMALL_NUMBER, FVector::UpVector);
		for (const FFlexCrosswalkPlacement& Crosswalk : Pair.Value.Crosswalks)
		{
			if (Node->ComplexIntersectionRegionIndex != INDEX_NONE)
			{
				const FFlexRoadSegment* CrossedSegment = Segments.Find(Crosswalk.SegmentId);
				if (CrossedSegment && IsInternalComplexIntersectionSegment(*CrossedSegment))
				{
					continue;
				}
			}
			if (Crosswalk.Width <= KINDA_SMALL_NUMBER || Crosswalk.Length <= KINDA_SMALL_NUMBER)
			{
				continue;
			}
			const FVector Along = FVector::VectorPlaneProject(Crosswalk.CrossingDirection, Up).GetSafeNormal();
			const FVector Across = FVector::CrossProduct(Up, Along).GetSafeNormal();
			if (Along.IsNearlyZero() || Across.IsNearlyZero())
			{
				continue;
			}
			const FVector HalfAlong = Along * (Crosswalk.Length * 0.5f + CrosswalkCurbDepthPadding);
			const FVector HalfAcross = Across * (Crosswalk.Width * 0.5f + CrosswalkSidePadding);
			FFlexUnifiedRoadSuppressionInput CrosswalkSuppression;
			CrosswalkSuppression.Boundary = {
				Crosswalk.Center - HalfAlong - HalfAcross,
				Crosswalk.Center + HalfAlong - HalfAcross,
				Crosswalk.Center + HalfAlong + HalfAcross,
				Crosswalk.Center - HalfAlong + HalfAcross
			};
			CrosswalkSuppression.ElevationLayer = static_cast<int32>(MaterialSegment->ElevationType);
			CrosswalkSuppression.bSuppressSidewalks = false;
			CrosswalkSuppression.bSuppressCurbs = true;
			SuppressionInputs.Add(MoveTemp(CrosswalkSuppression));
		}
	}

	FFlexUnifiedNetworkMeshResult Result = FFlexUnifiedRoadMeshBuilder::Build(
		SurfaceInputs, SuppressionInputs, Settings->MinimumGeneratedPolygonArea);
	TArray<FFlexMeshSectionData> RailSections;
	BuildRailMeshResults(RailSections);
	Result.Roadways.Append(MoveTemp(RailSections));
	return Result;
}

FFlexCurveFrame UFlexNetworkSubsystem::SampleSegmentAtArcLength(FFlexSegmentId SegmentId, float ArcLength) const
{
	const FFlexRoadSegment* Segment = Segments.Find(SegmentId);
	if (!Segment)
	{
		return FFlexCurveFrame();
	}
	const FVector RefUp = Nodes.Contains(Segment->StartNodeId) ? Nodes.FindChecked(Segment->StartNodeId).UpVector : FVector::UpVector;
	return FFlexRoadMeshBuilder::SampleFrameAtArcLength(Segment->Curve, Segment->ArcLengthTable, ArcLength, RefUp);
}

// ---------------------------------------------------------------- Extension points

void UFlexNetworkSubsystem::RegisterExporter(TSharedPtr<IFlexNetworkExporter> Exporter)
{
	if (Exporter)
	{
		Exporters.AddUnique(Exporter);
		Exporter->ExportFullNetwork(*this);
	}
}

void UFlexNetworkSubsystem::UnregisterExporter(const TSharedPtr<IFlexNetworkExporter>& Exporter)
{
	Exporters.Remove(Exporter);
}

// ---------------------------------------------------------------- Rebuild pipeline

TArray<FFlexJunctionApproachInput> UFlexNetworkSubsystem::BuildApproachInputs(FFlexNodeId NodeId) const
{
	TArray<FFlexJunctionApproachInput> Result;
	const FFlexRoadNode* Node = Nodes.Find(NodeId);
	if (!Node)
	{
		return Result;
	}

	for (FFlexSegmentId SegId : Node->ConnectedSegments)
	{
		const FFlexRoadSegment* Segment = Segments.Find(SegId);
		if (!Segment)
		{
			continue;
		}
		FFlexJunctionApproachInput Input;
		Input.SegmentId = SegId;
		Input.bNodeIsSegmentEnd = (Segment->EndNodeId == NodeId);
		Input.Curve = Segment->Curve;
		Input.ArcLengthTable = Segment->ArcLengthTable;
		Input.Profile = Segment->Profile;
		Result.Add(Input);
	}
	return Result;
}

void UFlexNetworkSubsystem::BeginBatchUpdate()
{
	++BatchDepth;
}

void UFlexNetworkSubsystem::EndBatchUpdate()
{
	if (!ensure(BatchDepth > 0))
	{
		return;
	}
	if (--BatchDepth == 0)
	{
		RebuildDirty();
		if (bTrafficSignalsChangedDuringBatch)
		{
			bTrafficSignalsChangedDuringBatch = false;
			BroadcastTrafficSignalsChanged();
		}
	}
}

void UFlexNetworkSubsystem::RebuildDirty()
{
	if (BatchDepth > 0)
	{
		return; // Accumulate only -- EndBatchUpdate will trigger the real (combined) rebuild.
	}

	if (DirtySegments.Num() == 0 && DirtyNodes.Num() == 0)
	{
		return;
	}

	const UFlexNetworkSettings* Settings = GetSettings();

	// 1. Affected nodes = explicitly dirty nodes + endpoints of every dirty segment.
	TSet<FFlexNodeId> AffectedNodes = DirtyNodes;
	for (FFlexSegmentId SegId : DirtySegments)
	{
		if (const FFlexRoadSegment* Segment = Segments.Find(SegId))
		{
			AffectedNodes.Add(Segment->StartNodeId);
			AffectedNodes.Add(Segment->EndNodeId);
		}
	}
	// 2. A junction rebuild at a shared node can change every connected segment's trim distance,
	// not just the one that was directly edited -- so the final rebuild set is every segment
	// touching an affected node, not just the explicitly dirty ones. This (rather than a full
	// network rebuild) is what makes the rebuild "incremental": only segments/junctions actually
	// adjacent to an edit are touched.
	TSet<FFlexSegmentId> FinalDirtySegments = DirtySegments;
	for (FFlexNodeId NodeId : AffectedNodes)
	{
		if (const FFlexRoadNode* Node = Nodes.Find(NodeId))
		{
			for (FFlexSegmentId SegId : Node->ConnectedSegments)
			{
				FinalDirtySegments.Add(SegId);
			}
		}
	}

	// 3. Arc-length tables first -- junction building below needs every approach's up-to-date table.
	for (FFlexSegmentId SegId : FinalDirtySegments)
	{
		if (FFlexRoadSegment* Segment = Segments.Find(SegId))
		{
			Segment->ArcLengthTable = FFlexBezierMath::BuildArcLengthTable(Segment->Curve, Settings->ArcLengthChordTolerance, Settings->MaxArcLengthSubdivisionDepth);
			Segment->bDirty = false;
		}
	}

	TMap<FFlexSegmentId, TPair<float, float>> TrimRangeBySegment;
	for (FFlexSegmentId SegId : FinalDirtySegments)
	{
		if (const FFlexRoadSegment* Segment = Segments.Find(SegId))
		{
			TrimRangeBySegment.Add(SegId, TPair<float, float>(0.f, Segment->GetLength()));
		}
	}

	// 4. Node roles + junction polygons/lane-connectors.
	for (FFlexNodeId NodeId : AffectedNodes)
	{
		FFlexRoadNode* Node = Nodes.Find(NodeId);
		if (!Node)
		{
			continue;
		}

		const TArray<FFlexJunctionApproachInput> Approaches = BuildApproachInputs(NodeId);

		uint8 NewRoleFlags = static_cast<uint8>(EFlexNodeRole::None);
		if (Approaches.Num() == 1)
		{
			NewRoleFlags |= static_cast<uint8>(EFlexNodeRole::Endpoint);
		}
		else if (Approaches.Num() >= 2)
		{
			NewRoleFlags |= static_cast<uint8>(EFlexNodeRole::Bend);
		}

		TSet<EFlexRoadElevationType> ElevationTypesAtNode;
		for (FFlexSegmentId SegId : Node->ConnectedSegments)
		{
			if (const FFlexRoadSegment* Seg = Segments.Find(SegId))
			{
				ElevationTypesAtNode.Add(Seg->ElevationType);
			}
		}
		if (ElevationTypesAtNode.Num() > 1)
		{
			NewRoleFlags |= static_cast<uint8>(EFlexNodeRole::ElevationTransition);
		}

		if (FFlexIntersectionBuilder::NeedsJunction(Approaches))
		{
			NewRoleFlags |= static_cast<uint8>(EFlexNodeRole::Junction);
			NewRoleFlags &= ~static_cast<uint8>(EFlexNodeRole::Bend);

			const float FilletRadius = Node->FilletRadiusOverride > 0.f ? Node->FilletRadiusOverride : Settings->DefaultFilletRadius;
			FFlexJunctionData JunctionData = FFlexIntersectionBuilder::BuildJunction(Node->Position, Node->UpVector, Approaches, FilletRadius, Settings->CrosswalkWidth, Settings->CrosswalkMinClearance, Settings->CurbReturnRadius, Settings->ParallelApproachAngleToleranceDegrees, 8, Settings->CurbReturnArcSegments);
			ValidateLaneConnectivity(NodeId, Approaches, JunctionData);

			for (const TPair<FFlexSegmentId, float>& TrimPair : JunctionData.TrimArcLengthBySegment)
			{
				if (const FFlexRoadSegment* Segment = Segments.Find(TrimPair.Key))
				{
					TPair<float, float>& Range = TrimRangeBySegment.FindOrAdd(TrimPair.Key, TPair<float, float>(0.f, Segment->GetLength()));
					if (Segment->StartNodeId == NodeId)
					{
						Range.Key = TrimPair.Value;
					}
					if (Segment->EndNodeId == NodeId)
					{
						Range.Value = TrimPair.Value;
					}
				}
			}

			JunctionDataByNode.Add(NodeId, MoveTemp(JunctionData));
		}
		else
		{
			JunctionDataByNode.Remove(NodeId);
			if (MeshActor)
			{
				MeshActor->RemoveJunctionMesh(NodeId);
			}
		}

		Node->RoleFlags = NewRoleFlags;
	}

	// A segment pulled into the rebuild because one endpoint changed can still terminate at an
	// unchanged junction. Re-resolve BOTH ends from the authoritative cache after affected
	// junctions have been updated; otherwise the unchanged end silently resets to 0/full length.
	for (FFlexSegmentId SegId : FinalDirtySegments)
	{
		float TrimStart = 0.f, TrimEnd = 0.f;
		if (GetSegmentTrimRange(SegId, TrimStart, TrimEnd))
		{
			TrimRangeBySegment.FindOrAdd(SegId) = TPair<float, float>(TrimStart, TrimEnd);
		}
	}

	// 5. Segment actors and terrain remain incremental. Classic geometry is rebuilt as one
	// topology-first surface after this loop because a changed road can alter exposed boundary
	// edges beyond the component that originally owned them.
	TArray<FFlexSegmentId> SegmentIdsToRebuild = FinalDirtySegments.Array();
	const bool bGenerateGeometry = VisualizationMode != EFlexNetworkVisualizationMode::SegmentActors;
	const bool bGenerateSegmentActors = VisualizationMode != EFlexNetworkVisualizationMode::GeneratedGeometry;
	if (bGenerateSegmentActors && SegmentIdsToRebuild.Num() > 0)
	{
		// Any graph edit can alter a crossing with the shared rail result. Refresh the single
		// deterministic PCG owner as part of the same rebuild so it cannot retain stale booleans.
		FFlexSegmentId RailOwner = FFlexSegmentId::Invalid();
		for (const TPair<FFlexSegmentId, FFlexRoadSegment>& Pair : Segments)
		{
			if (Pair.Value.Profile && Pair.Value.Profile->bIsRailProfile
				&& (!RailOwner.IsValid() || Pair.Key.Index < RailOwner.Index
					|| (Pair.Key.Index == RailOwner.Index && Pair.Key.Generation < RailOwner.Generation)))
			{
				RailOwner = Pair.Key;
			}
		}
		if (RailOwner.IsValid())
		{
			SegmentIdsToRebuild.AddUnique(RailOwner);
		}
		if (RailPCGOwner.IsValid() && RailPCGOwner != RailOwner && Segments.Contains(RailPCGOwner))
		{
			// Clear the output on the former owner when profile changes or ID reuse transfers
			// ownership to another actor.
			SegmentIdsToRebuild.AddUnique(RailPCGOwner);
		}
		RailPCGOwner = RailOwner;
	}
	else if (!bGenerateSegmentActors)
	{
		RailPCGOwner = FFlexSegmentId::Invalid();
	}

	AFlexNetworkMeshActor* Actor = bGenerateGeometry ? GetOrCreateMeshActor() : nullptr;
	UWorld* World = GetWorld();
	for (int32 Index = 0; Index < SegmentIdsToRebuild.Num(); ++Index)
	{
		const FFlexSegmentId SegId = SegmentIdsToRebuild[Index];
		const FFlexRoadSegment* Segment = Segments.Find(SegId);
		if (!Segment)
		{
			continue;
		}

		if (bGenerateSegmentActors)
		{
			if (AFlexNetworkSegmentActor* SegmentActor = GetOrCreateSegmentActor(SegId))
			{
				const FVector RefUp = Nodes.Contains(Segment->StartNodeId) ? Nodes.FindChecked(Segment->StartNodeId).UpVector : FVector::UpVector;
				const TPair<float, float>* Range = TrimRangeBySegment.Find(SegId);
				const float TrimStart = Range ? Range->Key : 0.f;
				const float TrimEnd = Range ? Range->Value : Segment->GetLength();
				SegmentActor->UpdateFromSegment(SegId, *Segment, RefUp, Settings->ArcLengthSampleStep, TrimStart, TrimEnd);
			}
		}

		// Only Ground segments get the terrain flattened to their height -- a Bridge/Elevated
		// segment sits above the terrain by design (flattening under it would visually merge the
		// deck into the ground, defeating the point), and Tunnel/Ramp don't have a sensible single
		// "flatten to this height" interpretation either.
		if (TerrainConformer && Segment->Profile && Segment->ElevationType == EFlexRoadElevationType::Ground)
		{
			const FVector RefUp = Nodes.Contains(Segment->StartNodeId) ? Nodes.FindChecked(Segment->StartNodeId).UpVector : FVector::UpVector;
			const TPair<float, float>* Range = TrimRangeBySegment.Find(SegId);
			const float TrimStart = Range ? Range->Key : 0.f;
			const float TrimEnd = Range ? Range->Value : Segment->GetLength();
			TArray<FFlexCurveFrame> Frames = FFlexRoadMeshBuilder::BuildFramesForRange(Segment->Curve, Segment->ArcLengthTable, RefUp, Settings->ArcLengthSampleStep, TrimStart, TrimEnd);
			const float RoadwayCenterOffset = Segment->Profile->GetRoadwayCenterOffset();
			if (!FMath::IsNearlyZero(RoadwayCenterOffset))
			{
				for (FFlexCurveFrame& Frame : Frames)
				{
					Frame.Position += Frame.Right * RoadwayCenterOffset;
				}
			}
			TerrainConformer->ConformSegment(World, SegId, Frames, Segment->Profile->GetRoadwayHalfWidth(), Settings->TerrainConformMargin, Settings->TerrainFalloffDistance);
		}
	}

	if (Actor)
	{
		Actor->ApplyUnifiedNetworkMesh(BuildUnifiedClassicMeshResult());
	}

	// 6. The unified result owns road, junction, sidewalk and curb surfaces. Junction components
	// now carry crosswalk overlays only; rebuild every cached overlay because applying the unified
	// surface intentionally clears all legacy independently-generated junction surface sections.
	if (bGenerateGeometry)
	{
		for (const TPair<FFlexNodeId, FFlexJunctionData>& Pair : JunctionDataByNode)
		{
			const FFlexNodeId NodeId = Pair.Key;
			const FFlexRoadNode* Node = Nodes.Find(NodeId);
			if (!Node)
			{
				continue;
			}
			UMaterialInterface* CrosswalkMaterial = nullptr;
			if (Node->ConnectedSegments.Num() > 0)
			{
				if (const FFlexRoadSegment* FirstSegment = Segments.Find(Node->ConnectedSegments[0]))
				{
					if (FirstSegment->Profile)
					{
						CrosswalkMaterial = FirstSegment->Profile->CrosswalkMaterial
							? FirstSegment->Profile->CrosswalkMaterial.Get()
							: FirstSegment->Profile->SidewalkMaterial.Get();
					}
				}
			}
			FFlexJunctionData OverlayJunction = Pair.Value;
			if (Node->ComplexIntersectionRegionIndex != INDEX_NONE)
			{
				OverlayJunction.Crosswalks.RemoveAll([this](const FFlexCrosswalkPlacement& Crosswalk)
				{
					const FFlexRoadSegment* Segment = Segments.Find(Crosswalk.SegmentId);
					return Segment && IsInternalComplexIntersectionSegment(*Segment);
				});
			}
			FFlexJunctionMeshResult JunctionMesh = FFlexIntersectionBuilder::BuildJunctionMesh(
				Node->UpVector, OverlayJunction, nullptr, CrosswalkMaterial, nullptr, nullptr);
			JunctionMesh.Surface = FFlexMeshSectionData();
			JunctionMesh.SidewalkCorners = FFlexMeshSectionData();
			JunctionMesh.CornerIslands = FFlexMeshSectionData();
			if (Actor)
			{
				Actor->ApplyJunctionMesh(NodeId, JunctionMesh);
			}
		}
	}

	// 7. Notify + clear dirty state.
	const TArray<FFlexNodeId> ChangedNodesArray = AffectedNodes.Array();
	const TArray<FFlexSegmentId> ChangedSegmentsArray = SegmentIdsToRebuild;

	DirtyNodes.Reset();
	DirtySegments.Reset();

	UE_LOG(LogTemp, Verbose, TEXT("FlexNetwork: incremental rebuild touched %d node(s), %d segment(s)."), ChangedNodesArray.Num(), ChangedSegmentsArray.Num());

	OnRoadNetworkChanged.Broadcast(ChangedNodesArray, ChangedSegmentsArray);
	OnRoadNetworkChangedBP.Broadcast(ChangedNodesArray, ChangedSegmentsArray);

	for (const TSharedPtr<IFlexNetworkExporter>& Exporter : Exporters)
	{
		if (Exporter)
		{
			Exporter->ExportChangedRegion(*this, ChangedNodesArray, ChangedSegmentsArray);
		}
	}
}
