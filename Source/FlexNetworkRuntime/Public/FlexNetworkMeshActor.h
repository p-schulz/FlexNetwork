#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "FlexNetworkTypes.h"
#include "Mesh/FlexMeshSectionData.h"
#include "FlexNetworkMeshActor.generated.h"

class UProceduralMeshComponent;
class USplineMeshComponent;
class UStaticMesh;

/**
 * Hosts generated classic geometry for one FlexNetwork graph. The current renderer uses one
 * topology-first component with material-partitioned roadway, sidewalk and curb sections, plus
 * small per-junction components for crosswalk overlays. The per-segment API remains available for
 * callers that explicitly request a standalone mesh result, but normal subsystem rebuilds replace
 * those components with the unified representation.
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

	/** Replaces the classic topology-first roadway, sidewalk, and curb sections. */
	void ApplyUnifiedNetworkMesh(const FFlexUnifiedNetworkMeshResult& MeshResult);
	const TArray<TArray<FVector>>& GetUnifiedCurbLines() const { return UnifiedCurbLines; }

	/**
	 * Replaces every previously-generated curbstone with a fresh set: for each entry in CurbLines
	 * (a polyline of consecutive world-space points to run curbstones along), spawns one
	 * USplineMeshComponent per point-pair, spline-fit between them with Mesh's local X axis as the
	 * spline's forward direction. A no-op (just clears existing curbstones) if Mesh is null.
	 */
	void ApplyCurbstones(const TArray<TArray<FVector>>& CurbLines, UStaticMesh* Mesh);
	UStaticMesh* GetAppliedCurbstoneMesh() const { return AppliedCurbstoneMesh.Get(); }

	void ClearAll();

private:
	UPROPERTY()
	TObjectPtr<USceneComponent> RootSceneComponent;

	UPROPERTY()
	TMap<FFlexSegmentId, TObjectPtr<UProceduralMeshComponent>> SegmentComponents;

	UPROPERTY()
	TMap<FFlexNodeId, TObjectPtr<UProceduralMeshComponent>> JunctionComponents;

	UPROPERTY()
	TObjectPtr<UProceduralMeshComponent> UnifiedNetworkComponent;

	TArray<TArray<FVector>> UnifiedCurbLines;

	UPROPERTY()
	TArray<TObjectPtr<USplineMeshComponent>> CurbstoneComponents;

	/** Last explicitly generated curbstone source, retained so a network bake can reproduce it. */
	UPROPERTY()
	TObjectPtr<UStaticMesh> AppliedCurbstoneMesh;

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
