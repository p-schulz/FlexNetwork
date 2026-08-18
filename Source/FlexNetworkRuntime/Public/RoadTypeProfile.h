#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "Materials/MaterialInterface.h"
#include "FlexNetworkTypes.h"
#include "RoadTypeProfile.generated.h"

/**
 * One lane (in the broad sense: vehicle lane, parking lane, bike lane, sidewalk, median strip)
 * within a road's cross-section. LateralOffset is measured from the segment centerline, positive
 * to the right of the direction of travel (start->end). This is the shared source of truth for
 * both the visible mesh extrusion and the lane-connector graph the traffic sim queries.
 */
USTRUCT(BlueprintType)
struct FLEXNETWORKRUNTIME_API FRoadLaneDescriptor
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lane")
	FName LaneName;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lane")
	float LateralOffset = 0.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lane", meta = (ClampMin = "1.0"))
	float Width = 350.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lane")
	EFlexLaneType Type = EFlexLaneType::Vehicle;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lane")
	EFlexLaneDirection Direction = EFlexLaneDirection::Forward;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lane", meta = (ClampMin = "0.0", Units = "cm/s"))
	float SpeedLimit = 1389.f; // ~50 km/h in cm/s

	/** Inner (LateralOffset - Width/2) edge of this lane. */
	float GetInnerEdge() const { return LateralOffset - Width * 0.5f; }
	/** Outer (LateralOffset + Width/2) edge of this lane. */
	float GetOuterEdge() const { return LateralOffset + Width * 0.5f; }

	bool IsDrivable() const { return Type == EFlexLaneType::Vehicle || Type == EFlexLaneType::Bike; }
};

/**
 * Data-driven cross-section "recipe" for a road type (highway, arterial, residential,
 * footpath, ...). The same extrusion/intersection algorithms consume this for every road type;
 * only the lane list and constraints differ. This is the single source of truth for both
 * visible geometry and the lane graph the traffic simulation queries.
 */
UCLASS(BlueprintType)
class FLEXNETWORKRUNTIME_API URoadTypeProfile : public UDataAsset
{
	GENERATED_BODY()

public:
	/** Ordered list of lanes making up this road's cross-section. Order does not need to match LateralOffset order. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Profile")
	TArray<FRoadLaneDescriptor> Lanes;

	/** Width (cm) of sidewalk generated as an offset curve beyond the outermost drivable lane, per side. 0 disables sidewalks for this profile. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Profile", meta = (ClampMin = "0.0", Units = "cm"))
	float SidewalkWidth = 200.f;

	/** Vertical height (cm) of the curb between road surface and sidewalk. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Profile", meta = (ClampMin = "0.0", Units = "cm"))
	float CurbHeight = 15.f;

	/** Maximum grade (rise/run, e.g. 0.08 = 8%) this road type permits; enforced live while drawing. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Constraints", meta = (ClampMin = "0.001", ClampMax = "1.0"))
	float MaxGrade = 0.08f;

	/** Minimum turn radius (cm) a segment of this type may curve to. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Constraints", meta = (ClampMin = "1.0", Units = "cm"))
	float MinTurnRadius = 1000.f;

	/** Minimum segment length (cm) for this road type; falls back to UFlexNetworkSettings::MinSegmentLength when <= 0. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Constraints", meta = (Units = "cm"))
	float MinSegmentLengthOverride = 0.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rendering")
	TObjectPtr<UMaterialInterface> RoadMaterial = nullptr;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rendering")
	TObjectPtr<UMaterialInterface> SidewalkMaterial = nullptr;

	/** Material for PCG/procedural curb prisms; falls back to SidewalkMaterial when unset. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rendering")
	TObjectPtr<UMaterialInterface> CurbMaterial = nullptr;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rendering")
	TObjectPtr<UMaterialInterface> JunctionMaterial = nullptr;

	/** Material for intersection crosswalk strips; falls back to SidewalkMaterial when unset. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rendering")
	TObjectPtr<UMaterialInterface> CrosswalkMaterial = nullptr;

	/** Fill for the rounded corner islands/refuges the sidewalk curves around at junctions (grass/landscaping is typical -- falls back to SidewalkMaterial if left unset). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rendering")
	TObjectPtr<UMaterialInterface> MedianMaterial = nullptr;

	/** Distance (cm) between the centerline and the outermost extent of this profile on one side (drivable lanes + sidewalk). Used by the intersection builder's outer-edge extrapolation. */
	float GetOuterExtent() const;

	/** Half-width (cm) of just the drivable/lane portion (no sidewalk), i.e. max |lane outer edge|. */
	float GetRoadwayHalfWidth() const;

	/** Returns lanes sorted by LateralOffset ascending (left to right). */
	TArray<FRoadLaneDescriptor> GetLanesSortedByOffset() const;
};
