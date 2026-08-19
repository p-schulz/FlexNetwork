#pragma once

#include "CoreMinimal.h"
#include "FlexNetworkTypes.h"
#include "FlexTrafficSignal.generated.h"

/** Traffic-control semantics owned by FlexNetwork. MassTraffic currently consumes TrafficLight. */
UENUM(BlueprintType)
enum class EFlexTrafficControlType : uint8
{
	TrafficLight,
	StopSign,
	YieldSign
};

/**
 * One directed control on one junction approach. A bidirectional OSM signal is represented by
 * two records, because MassTraffic associates one light instance with one inbound intersection
 * side. The physical anchor and controlled approach are deliberately separate: OSM commonly
 * places a traffic_signals node at the stop line, one short segment before the actual junction.
 */
USTRUCT(BlueprintType)
struct FLEXNETWORKRUNTIME_API FFlexTrafficSignal
{
	GENERATED_BODY()

	/** Persistent identity used by editor tools, bake restore, and external simulation code. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "FlexNetwork|Traffic Signal")
	FGuid Id;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FlexNetwork|Traffic Signal")
	EFlexTrafficControlType Type = EFlexTrafficControlType::TrafficLight;

	/** Segment carrying the physical pole/stop control. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FlexNetwork|Traffic Signal")
	FFlexSegmentId AnchorSegmentId;

	/** Normalized arc position on AnchorSegmentId. Stable when the segment is reshaped. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FlexNetwork|Traffic Signal", meta = (ClampMin = "0.0", ClampMax = "1.0"))
	float AnchorFraction = 0.f;

	/** Segment whose inbound ZoneGraph lanes this control governs. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FlexNetwork|Traffic Signal")
	FFlexSegmentId ControlledApproachSegmentId;

	/** Junction endpoint of ControlledApproachSegmentId. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FlexNetwork|Traffic Signal")
	FFlexNodeId ControlledJunctionNodeId;

	/** Positive values move to the right of inbound traffic. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FlexNetwork|Traffic Signal", meta = (Units = "cm"))
	float LateralOffset = 0.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FlexNetwork|Traffic Signal", meta = (Units = "cm"))
	float HeightOffset = 0.f;

	/** Index into the MassTraffic light-types asset. INDEX_NONE lets MassTraffic select by lane count. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FlexNetwork|Traffic Signal")
	int32 TrafficLightTypeIndex = 0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FlexNetwork|Traffic Signal")
	bool bEnabled = true;

	/** Stable external provenance, for example OSM/node/123/way/456/forward. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "FlexNetwork|Traffic Signal")
	FString SourceId;
};

/** Current world-space projection of an authoritative signal record. */
USTRUCT(BlueprintType)
struct FLEXNETWORKRUNTIME_API FFlexResolvedTrafficSignal
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "FlexNetwork|Traffic Signal")
	FTransform Transform = FTransform::Identity;

	/** Point MassTraffic uses to associate this instance with an inbound ZoneGraph side. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "FlexNetwork|Traffic Signal")
	FVector ControlledIntersectionSideMidpoint = FVector::ZeroVector;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "FlexNetwork|Traffic Signal")
	FVector InboundDirection = FVector::ForwardVector;
};
