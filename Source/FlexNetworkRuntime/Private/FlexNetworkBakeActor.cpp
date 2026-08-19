#include "FlexNetworkBakeActor.h"

#include "FlexNetworkSubsystem.h"
#include "FlexNetworkMeshActor.h"
#include "FlexRoadNode.h"
#include "FlexRoadSegment.h"
#include "Components/SceneComponent.h"
#include "Engine/World.h"

AFlexNetworkBakeActor::AFlexNetworkBakeActor()
{
	PrimaryActorTick.bCanEverTick = false;
	bReplicates = false;
	// The snapshot is authoritative for the whole road graph, not a local visual actor. It must be
	// present even when its origin's World Partition cell is not currently streamed.
	SetIsSpatiallyLoaded(false);
	SceneRoot = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	SetRootComponent(SceneRoot);
}

int32 AFlexNetworkBakeActor::CaptureFromSubsystem(const UFlexNetworkSubsystem& Subsystem)
{
	Modify();
	BakedNodes.Reset();
	BakedSegments.Reset();
	BakedTrafficSignals.Reset();
	BakeFormatVersion = 4;
	VisualizationMode = Subsystem.GetVisualizationMode();
	CurbstoneMesh = nullptr;
	bRestoreCurbstones = false;
	if (const AFlexNetworkMeshActor* MeshActor = Subsystem.GetMeshActor())
	{
		CurbstoneMesh = MeshActor->GetAppliedCurbstoneMesh();
		bRestoreCurbstones = CurbstoneMesh != nullptr;
	}

	BakedNodes.Reserve(Subsystem.GetAllNodes().Num());
	for (const TPair<FFlexNodeId, FFlexRoadNode>& Pair : Subsystem.GetAllNodes())
	{
		FFlexBakedNode Record;
		Record.SourceId = Pair.Key;
		Record.Position = Pair.Value.Position;
		Record.UpVector = Pair.Value.UpVector;
		Record.ElevationType = Pair.Value.ElevationType;
		Record.FilletRadiusOverride = Pair.Value.FilletRadiusOverride;
		Record.ComplexIntersectionRegionIndex = Pair.Value.ComplexIntersectionRegionIndex;
		BakedNodes.Add(MoveTemp(Record));
	}

	BakedSegments.Reserve(Subsystem.GetAllSegments().Num());
	for (const TPair<FFlexSegmentId, FFlexRoadSegment>& Pair : Subsystem.GetAllSegments())
	{
		FFlexBakedSegment Record;
		Record.SourceId = Pair.Key;
		Record.StartSourceNodeId = Pair.Value.StartNodeId;
		Record.EndSourceNodeId = Pair.Value.EndNodeId;
		Record.StartTangentHandle = Pair.Value.Curve.P1;
		Record.EndTangentHandle = Pair.Value.Curve.P2;
		Record.Profile = Pair.Value.Profile;
		Record.ElevationType = Pair.Value.ElevationType;
		Record.ElevationProfile = Pair.Value.ElevationProfile;
		BakedSegments.Add(MoveTemp(Record));
	}

	TArray<const FFlexTrafficSignal*> OrderedSignals;
	OrderedSignals.Reserve(Subsystem.GetAllTrafficSignals().Num());
	for (const TPair<FGuid, FFlexTrafficSignal>& Pair : Subsystem.GetAllTrafficSignals())
	{
		OrderedSignals.Add(&Pair.Value);
	}
	OrderedSignals.Sort([](const FFlexTrafficSignal& A, const FFlexTrafficSignal& B)
	{
		return A.Id < B.Id;
	});
	BakedTrafficSignals.Reserve(OrderedSignals.Num());
	for (const FFlexTrafficSignal* Signal : OrderedSignals)
	{
		FFlexBakedTrafficSignal Record;
		Record.Signal = *Signal;
		BakedTrafficSignals.Add(MoveTemp(Record));
	}

	++BakeRevision;
	bRestoredThisWorld = true;
	MarkPackageDirty();
	return BakedSegments.Num();
}

int32 AFlexNetworkBakeActor::RestoreToSubsystem(UFlexNetworkSubsystem& Subsystem)
{
	if (bRestoredThisWorld || !bRestoreAutomatically || BakedNodes.IsEmpty())
	{
		return 0;
	}
	bRestoredThisWorld = true;
	const int32 RestoredSegments = Subsystem.LoadBakedNetwork(BakedNodes, BakedSegments, BakedTrafficSignals, VisualizationMode);
	RestoreCurbstones(Subsystem);
	return RestoredSegments;
}

void AFlexNetworkBakeActor::RestoreCurbstones(UFlexNetworkSubsystem& Subsystem) const
{
	if (!bRestoreCurbstones || !CurbstoneMesh)
	{
		return;
	}
	if (AFlexNetworkMeshActor* MeshActor = Subsystem.GetMeshActor())
	{
		MeshActor->ApplyCurbstones(MeshActor->GetUnifiedCurbLines(), CurbstoneMesh);
	}
}

void AFlexNetworkBakeActor::PostRegisterAllComponents()
{
	Super::PostRegisterAllComponents();
	UWorld* World = GetWorld();
	if (!IsTemplate() && World && World->WorldType == EWorldType::Editor)
	{
		if (UFlexNetworkSubsystem* Subsystem = World->GetSubsystem<UFlexNetworkSubsystem>())
		{
			RestoreToSubsystem(*Subsystem);
		}
	}
}

void AFlexNetworkBakeActor::BeginPlay()
{
	Super::BeginPlay();
	if (UWorld* World = GetWorld())
	{
		if (UFlexNetworkSubsystem* Subsystem = World->GetSubsystem<UFlexNetworkSubsystem>())
		{
			RestoreToSubsystem(*Subsystem);
		}
	}
}
