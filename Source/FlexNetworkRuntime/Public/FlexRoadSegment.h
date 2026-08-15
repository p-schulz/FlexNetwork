#pragma once

#include "CoreMinimal.h"
#include "FlexNetworkTypes.h"
#include "FlexCurveTypes.h"
#include "RoadTypeProfile.h"
#include "FlexRoadSegment.generated.h"

/**
 * An edge in the planar road graph, connecting exactly two nodes. The Bezier curve plus the
 * RoadTypeProfile reference are the only authored data; the arc-length table and every mesh/
 * intersection artifact are derived and cached, rebuilt whenever the curve or profile changes.
 */
USTRUCT(BlueprintType)
struct FLEXNETWORKRUNTIME_API FFlexRoadSegment
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "FlexNetwork")
	FFlexNodeId StartNodeId;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "FlexNetwork")
	FFlexNodeId EndNodeId;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FlexNetwork")
	FFlexBezierCurve Curve;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FlexNetwork")
	TObjectPtr<URoadTypeProfile> Profile = nullptr;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FlexNetwork")
	EFlexRoadElevationType ElevationType = EFlexRoadElevationType::Ground;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FlexNetwork")
	FFlexElevationProfile ElevationProfile;

	/** Cached t->arc-length table for Curve. Rebuilt by the subsystem whenever Curve changes; do not edit directly. */
	UPROPERTY(Transient)
	FFlexArcLengthTable ArcLengthTable;

	/** True until the subsystem next rebuilds ArcLengthTable/derived mesh/junction data for this segment. */
	bool bDirty = true;

	float GetLength() const { return ArcLengthTable.GetTotalLength(); }
};
