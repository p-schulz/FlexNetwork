#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "FlexNetworkTypes.h"
#include "FlexNetworkSegmentActor.generated.h"

class UPCGComponent;
class USceneComponent;
class USplineComponent;
struct FFlexRoadSegment;

/**
 * Geometry-free representation of one FlexNetwork segment. The five splines visualize the
 * authored centerline, roadway edges and sidewalk outer edges and also give a PCG graph a stable
 * per-segment execution source. Actual road/sidewalk/intersection meshes remain PCG outputs.
 */
UCLASS(BlueprintType)
class FLEXNETWORKRUNTIME_API AFlexNetworkSegmentActor : public AActor
{
	GENERATED_BODY()

public:
	AFlexNetworkSegmentActor();

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="FlexNetwork")
	FFlexSegmentId SegmentId;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="FlexNetwork")
	TObjectPtr<USplineComponent> Centerline;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="FlexNetwork")
	TObjectPtr<USplineComponent> RoadLeftEdge;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="FlexNetwork")
	TObjectPtr<USplineComponent> RoadRightEdge;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="FlexNetwork")
	TObjectPtr<USplineComponent> SidewalkLeftEdge;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="FlexNetwork")
	TObjectPtr<USplineComponent> SidewalkRightEdge;

	/** Assign a FlexNetwork PCG graph here (or on a derived actor CDO) for per-segment generation. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="PCG")
	TObjectPtr<UPCGComponent> PCGComponent;

	void UpdateFromSegment(FFlexSegmentId InId, const FFlexRoadSegment& Segment, const FVector& ReferenceUp, float SampleStep, float TrimStartArcLength, float TrimEndArcLength);

private:
	UPROPERTY()
	TObjectPtr<USceneComponent> SceneRoot;
};
