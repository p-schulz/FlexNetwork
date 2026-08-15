#include "FlexNetworkMeshActor.h"
#include "ProceduralMeshComponent.h"

namespace
{
	constexpr int32 kRoadwaySection = 0;
	constexpr int32 kSidewalkSection = 1;
	constexpr int32 kJunctionSurfaceSection = 0;
	constexpr int32 kJunctionCrosswalkSection = 1;
}

AFlexNetworkMeshActor::AFlexNetworkMeshActor()
{
	PrimaryActorTick.bCanEverTick = false;
	bReplicates = false;

	RootSceneComponent = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	SetRootComponent(RootSceneComponent);
}

UProceduralMeshComponent* AFlexNetworkMeshActor::GetOrCreateComponent(TMap<FFlexSegmentId, TObjectPtr<UProceduralMeshComponent>>& Map, FFlexSegmentId Id, const TCHAR* NamePrefix)
{
	if (TObjectPtr<UProceduralMeshComponent>* Found = Map.Find(Id))
	{
		return *Found;
	}

	UProceduralMeshComponent* Comp = NewObject<UProceduralMeshComponent>(this, *FString::Printf(TEXT("%s_%u_%u"), NamePrefix, Id.Index, Id.Generation));
	Comp->SetupAttachment(RootSceneComponent);
	Comp->RegisterComponent();
	Comp->SetMobility(EComponentMobility::Movable);
	Map.Add(Id, Comp);
	return Comp;
}

UProceduralMeshComponent* AFlexNetworkMeshActor::GetOrCreateComponent(TMap<FFlexNodeId, TObjectPtr<UProceduralMeshComponent>>& Map, FFlexNodeId Id, const TCHAR* NamePrefix)
{
	if (TObjectPtr<UProceduralMeshComponent>* Found = Map.Find(Id))
	{
		return *Found;
	}

	UProceduralMeshComponent* Comp = NewObject<UProceduralMeshComponent>(this, *FString::Printf(TEXT("%s_%u_%u"), NamePrefix, Id.Index, Id.Generation));
	Comp->SetupAttachment(RootSceneComponent);
	Comp->RegisterComponent();
	Comp->SetMobility(EComponentMobility::Movable);
	Map.Add(Id, Comp);
	return Comp;
}

void AFlexNetworkMeshActor::ApplySectionData(UProceduralMeshComponent* Comp, int32 SectionIndex, const FFlexMeshSectionData& Data)
{
	if (!Comp)
	{
		return;
	}

	if (Data.IsEmpty())
	{
		Comp->ClearMeshSection(SectionIndex);
		return;
	}

	Comp->CreateMeshSection(
		SectionIndex,
		Data.Vertices,
		Data.Triangles,
		Data.Normals,
		Data.UV0,
		TArray<FVector2D>(),
		TArray<FVector2D>(),
		TArray<FVector2D>(),
		Data.VertexColors,
		Data.Tangents,
		Data.bEnableCollision);

	if (Data.Material)
	{
		Comp->SetMaterial(SectionIndex, Data.Material);
	}
}

void AFlexNetworkMeshActor::ApplySegmentMesh(FFlexSegmentId SegmentId, const FFlexSegmentMeshResult& MeshResult)
{
	UProceduralMeshComponent* Comp = GetOrCreateComponent(SegmentComponents, SegmentId, TEXT("Segment"));
	ApplySectionData(Comp, kRoadwaySection, MeshResult.Roadway);
	ApplySectionData(Comp, kSidewalkSection, MeshResult.Sidewalks);
}

void AFlexNetworkMeshActor::RemoveSegmentMesh(FFlexSegmentId SegmentId)
{
	if (TObjectPtr<UProceduralMeshComponent> Comp; SegmentComponents.RemoveAndCopyValue(SegmentId, Comp) && Comp)
	{
		Comp->DestroyComponent();
	}
}

void AFlexNetworkMeshActor::ApplyJunctionMesh(FFlexNodeId NodeId, const FFlexJunctionMeshResult& MeshResult)
{
	UProceduralMeshComponent* Comp = GetOrCreateComponent(JunctionComponents, NodeId, TEXT("Junction"));
	ApplySectionData(Comp, kJunctionSurfaceSection, MeshResult.Surface);
	ApplySectionData(Comp, kJunctionCrosswalkSection, MeshResult.Crosswalks);
}

void AFlexNetworkMeshActor::RemoveJunctionMesh(FFlexNodeId NodeId)
{
	if (TObjectPtr<UProceduralMeshComponent> Comp; JunctionComponents.RemoveAndCopyValue(NodeId, Comp) && Comp)
	{
		Comp->DestroyComponent();
	}
}

void AFlexNetworkMeshActor::ClearAll()
{
	for (const TPair<FFlexSegmentId, TObjectPtr<UProceduralMeshComponent>>& Pair : SegmentComponents)
	{
		if (Pair.Value)
		{
			Pair.Value->DestroyComponent();
		}
	}
	SegmentComponents.Reset();

	for (const TPair<FFlexNodeId, TObjectPtr<UProceduralMeshComponent>>& Pair : JunctionComponents)
	{
		if (Pair.Value)
		{
			Pair.Value->DestroyComponent();
		}
	}
	JunctionComponents.Reset();
}
