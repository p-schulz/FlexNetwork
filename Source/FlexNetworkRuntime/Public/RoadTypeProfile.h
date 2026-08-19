#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "Materials/MaterialInterface.h"
#include "FlexNetworkTypes.h"
#include "RoadTypeProfile.generated.h"

/**
 * One lane (in the broad sense: vehicle lane, parking lane, bike lane, sidewalk, median strip)
 * within a road's cross-section. LateralOffset is measured from the profile origin, positive to
 * the right of the segment direction (start->end); URoadTypeProfile::LateralOffset then places the
 * whole cross-section relative to the segment spline. This is the shared source of truth for both
 * visible mesh extrusion and the lane-connector graph the traffic sim queries.
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

	bool IsDrivable() const { return Type == EFlexLaneType::Vehicle || Type == EFlexLaneType::Bike || Type == EFlexLaneType::Rail; }
	bool IsRail() const { return Type == EFlexLaneType::Rail; }
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

	/**
	 * Lateral offset (cm) applied to the complete profile relative to the segment spline, positive
	 * to the spline's right. OSM imports derive this from placement=*; for an untagged asymmetric
	 * lane pack it recenters the roadway on the digitized OSM line.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Profile", meta = (Units = "cm"))
	float LateralOffset = 0.f;

	/** Railway profiles render each Rail lane as a pair of raised physical rail solids instead of a roadway slab. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Profile|Rail")
	bool bIsRailProfile = false;

	/** Distance between the inner running faces of the two rails (cm); OSM gauge=* is imported from millimeters. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Profile|Rail", meta = (EditCondition = "bIsRailProfile", ClampMin = "1.0", Units = "cm"))
	float RailGauge = 143.5f;

	/** Width of each rail at its base (cm). A typical grooved tram rail is about 156 mm wide. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Profile|Rail", meta = (EditCondition = "bIsRailProfile", ClampMin = "1.0", Units = "cm"))
	float RailWidth = 15.6f;

	/** Width of the rail crown (cm), producing the tapered sides of the raised outer solid. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Profile|Rail", meta = (EditCondition = "bIsRailProfile", ClampMin = "1.0", Units = "cm"))
	float RailTopWidth = 11.5f;

	/** Physical height of the raised rail profile above the spline/terrain datum. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Profile|Rail", meta = (EditCondition = "bIsRailProfile", ClampMin = "0.0", Units = "cm"))
	float RailHeight = 7.2f;

	/** Enables the asymmetric elevated boolean cut used by grooved tram rails. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Profile|Rail", meta = (EditCondition = "bIsRailProfile"))
	bool bUseGroovedRailProfile = false;

	/** Width of the raised cutter subtracted from the rail crown. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Profile|Rail", meta = (EditCondition = "bIsRailProfile && bUseGroovedRailProfile", ClampMin = "0.5", Units = "cm"))
	float RailGrooveWidth = 4.f;

	/** Depth of the groove measured down from the crown. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Profile|Rail", meta = (EditCondition = "bIsRailProfile && bUseGroovedRailProfile", ClampMin = "0.1", Units = "cm"))
	float RailGrooveDepth = 4.5f;

	/** Cutter shift toward the track center; leaves the wider shoulder on each rail's outside. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Profile|Rail", meta = (EditCondition = "bIsRailProfile && bUseGroovedRailProfile", ClampMin = "0.0", Units = "cm"))
	float RailGrooveInwardOffset = 1.5f;

	/** Longitudinal overlap used before boolean union so connected rail caps cannot leave seams. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Profile|Rail", meta = (EditCondition = "bIsRailProfile", ClampMin = "0.0", Units = "cm"))
	float RailBooleanOverlap = 0.5f;

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

	/** Maximum distance (cm) from the segment spline to either outside sidewalk edge. */
	float GetOuterExtent() const;

	/** Half of the physical roadway width (cm), independent of its lateral placement. */
	float GetRoadwayHalfWidth() const;

	/** Effective left/right roadway boundaries as signed offsets from the segment spline. */
	float GetRoadwayMinOffset() const;
	float GetRoadwayMaxOffset() const;

	/** Center of the physical roadway relative to the segment spline. */
	float GetRoadwayCenterOffset() const;

	/** Full physical roadway width (cm). */
	float GetRoadwayWidth() const;

	/** Effective lateral offset for a lane, including the profile-wide placement offset. */
	float GetLaneLateralOffset(const FRoadLaneDescriptor& Lane) const { return LateralOffset + Lane.LateralOffset; }

	/** Returns lanes sorted by LateralOffset ascending (left to right). */
	TArray<FRoadLaneDescriptor> GetLanesSortedByOffset() const;
};
