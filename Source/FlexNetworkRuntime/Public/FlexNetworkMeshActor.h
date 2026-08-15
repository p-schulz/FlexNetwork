#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "FlexNetworkTypes.h"
#include "Mesh/FlexMeshSectionData.h"
#include "FlexNetworkMeshActor.generated.h"

class UProceduralMeshComponent;

/**
 * Hosts the generated UProceduralMeshComponents for one FlexNetwork graph: one component per
 * segment (section 0 = roadway, section 1 = sidewalks) and one per junction (section 0 =
 * surface, section 1 = crosswalks). UWorldSubsystem can't own components directly, so
 * UFlexNetworkSubsystem lazily spawns and drives one of these per world instead.
 */
UCLASS(NotBlueprintable)
class FLEXNETWORKRUNTIME_API AFlexNetworkMeshActor : public AActor
{
	GENERATED_BODY()

public:
	AFlexNetworkMeshActor();

	void ApplySegmentMesh(FFlexSegmentId SegmentId, const FFlexSegmentMeshResult& MeshResult);
	void RemoveSegmentMesh(FFlexSegmentId SegmentId);

	void ApplyJunctionMesh(FFlexNodeId NodeId, const FFlexJunctionMeshResult& MeshResult);
	void RemoveJunctionMesh(FFlexNodeId NodeId);

	void ClearAll();

private:
	UPROPERTY()
	TObjectPtr<USceneComponent> RootSceneComponent;

	UPROPERTY()
	TMap<FFlexSegmentId, TObjectPtr<UProceduralMeshComponent>> SegmentComponents;

	UPROPERTY()
	TMap<FFlexNodeId, TObjectPtr<UProceduralMeshComponent>> JunctionComponents;

	UProceduralMeshComponent* GetOrCreateComponent(TMap<FFlexSegmentId, TObjectPtr<UProceduralMeshComponent>>& Map, FFlexSegmentId Id, const TCHAR* NamePrefix);
	UProceduralMeshComponent* GetOrCreateComponent(TMap<FFlexNodeId, TObjectPtr<UProceduralMeshComponent>>& Map, FFlexNodeId Id, const TCHAR* NamePrefix);

	// CreateMeshSection replaces a section's data (and topology) wholesale, which is what we
	// always need here -- vertex/triangle counts change whenever a curve's length, trim points,
	// or sampling change, so UProceduralMeshComponent::UpdateMeshSection's "same topology, new
	// attributes only" contract doesn't fit. The "incremental" part of incremental rebuild is at
	// the segment/junction-component level (only touched ones get a new CreateMeshSection call),
	// not at the per-vertex level.
	static void ApplySectionData(UProceduralMeshComponent* Comp, int32 SectionIndex, const FFlexMeshSectionData& Data);
};
