#pragma once

#include "CoreMinimal.h"
#include "FlexNetworkTypes.h"
#include "FlexCurveTypes.h"
#include "Traffic/FlexTrafficSignal.h"
#include "FlexNetworkBakeTypes.generated.h"

class URoadTypeProfile;

/** Serializable authored node data stored by AFlexNetworkBakeActor. Derived connectivity is rebuilt. */
USTRUCT(BlueprintType)
struct FLEXNETWORKRUNTIME_API FFlexBakedNode
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "FlexNetwork")
	FFlexNodeId SourceId;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FlexNetwork")
	FVector Position = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FlexNetwork")
	FVector UpVector = FVector::UpVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FlexNetwork")
	EFlexRoadElevationType ElevationType = EFlexRoadElevationType::Ground;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FlexNetwork")
	float FilletRadiusOverride = 0.f;

	/** Preserves multi-port complex-intersection grouping across bake restore/PIE. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "FlexNetwork")
	int32 ComplexIntersectionRegionIndex = INDEX_NONE;

};

/** Serializable authored segment data. Arc tables, junctions, meshes and lane connectors are derived on restore. */
USTRUCT(BlueprintType)
struct FLEXNETWORKRUNTIME_API FFlexBakedSegment
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "FlexNetwork")
	FFlexSegmentId SourceId;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "FlexNetwork")
	FFlexNodeId StartSourceNodeId;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "FlexNetwork")
	FFlexNodeId EndSourceNodeId;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FlexNetwork")
	FVector StartTangentHandle = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FlexNetwork")
	FVector EndTangentHandle = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FlexNetwork")
	TObjectPtr<URoadTypeProfile> Profile = nullptr;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FlexNetwork")
	EFlexRoadElevationType ElevationType = EFlexRoadElevationType::Ground;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FlexNetwork")
	FFlexElevationProfile ElevationProfile;
};

/** Serializable signal record. Segment/node handles are remapped when the baked graph restores. */
USTRUCT(BlueprintType)
struct FLEXNETWORKRUNTIME_API FFlexBakedTrafficSignal
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FlexNetwork")
	FFlexTrafficSignal Signal;
};
