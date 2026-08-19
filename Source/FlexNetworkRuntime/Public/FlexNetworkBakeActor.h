#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "FlexNetworkBakeTypes.h"
#include "FlexNetworkBakeActor.generated.h"

class UFlexNetworkSubsystem;
class USceneComponent;
class UStaticMesh;

/**
 * Persistent level-owned copy of FlexNetwork's authored graph. The subsystem and generated road
 * actors remain transient per-world; this actor is duplicated into PIE and reconstructs them there.
 */
UCLASS(BlueprintType, NotPlaceable)
class FLEXNETWORKRUNTIME_API AFlexNetworkBakeActor : public AActor
{
	GENERATED_BODY()

public:
	AFlexNetworkBakeActor();

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "FlexNetwork|Bake")
	int32 BakeFormatVersion = 4;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "FlexNetwork|Bake")
	int32 BakeRevision = 0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FlexNetwork|Bake")
	bool bRestoreAutomatically = true;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "FlexNetwork|Bake")
	EFlexNetworkVisualizationMode VisualizationMode = EFlexNetworkVisualizationMode::GeneratedGeometry;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "FlexNetwork|Bake")
	TArray<FFlexBakedNode> BakedNodes;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "FlexNetwork|Bake")
	TArray<FFlexBakedSegment> BakedSegments;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "FlexNetwork|Bake")
	TArray<FFlexBakedTrafficSignal> BakedTrafficSignals;

	/** Recreates the optional classic spline-curbstone pass after the road graph and curb lines restore. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FlexNetwork|Bake|Curbs")
	bool bRestoreCurbstones = false;

	/** Static mesh whose local X follows the curb and whose local Y extends away from the roadway. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FlexNetwork|Bake|Curbs", meta = (EditCondition = "bRestoreCurbstones"))
	TObjectPtr<UStaticMesh> CurbstoneMesh;

	/** Replaces the stored snapshot with the subsystem's current authored graph. */
	int32 CaptureFromSubsystem(const UFlexNetworkSubsystem& Subsystem);

	/** Restores once per actor/world instance; safe when both subsystem and actor lifecycle hooks call it. */
	int32 RestoreToSubsystem(UFlexNetworkSubsystem& Subsystem);

	/** Reapplies only the baked optional curbstone mesh to the subsystem's reconstructed curb lines. */
	void RestoreCurbstones(UFlexNetworkSubsystem& Subsystem) const;

	virtual void PostRegisterAllComponents() override;
	virtual void BeginPlay() override;

private:
	UPROPERTY()
	TObjectPtr<USceneComponent> SceneRoot;

	// Deliberately not reflected: a PIE duplicate starts false even if the editor instance restored.
	bool bRestoredThisWorld = false;
};
