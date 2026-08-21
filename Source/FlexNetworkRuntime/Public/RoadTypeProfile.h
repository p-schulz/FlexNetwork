#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "Materials/MaterialInterface.h"
#include "FlexNetworkTypes.h"
#include "RoadTypeProfile.generated.h"

class AActor;

/**
 * One kind of actor (a Blueprint prop -- bollard, tree, lamppost, parked car, ...) regularly
 * spawned along a lane by UFlexNetworkSubsystem::GenerateLaneActors, an on-demand generation pass
 * separate from the automatic mesh rebuild: spawning full actors is comparatively expensive and,
 * unlike geometry, has no natural notion of "only the touched part changed" to incrementally
 * update, so it's triggered manually (see UFlexNetworkEdModeSettings::GenerateLaneActors) rather
 * than folded into RebuildDirty().
 */
USTRUCT(BlueprintType)
struct FLEXNETWORKRUNTIME_API FFlexLaneActorSpawnEntry
{
	GENERATED_BODY()

	/** Actor class spawned at each position -- typically a Blueprint. Left unset, this entry generates nothing. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lane Actors")
	TSubclassOf<AActor> ActorClass;

	/**
	 * Lateral distance (cm) from the lane's own outer edge (LateralOffset + Width/2, i.e. its +Right
	 * boundary) at which actors are placed -- positive moves further outward past the edge, negative
	 * pulls back into the lane. If you need the other (inner) edge instead, offset by the negative of
	 * the lane's own width, or adjust LateralOffset/Width to relocate the edge you need.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lane Actors", meta = (Units = "cm"))
	float LaneEdgeOffset = 0.f;

	/** Spacing (cm) between consecutive spawned actors along the lane. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lane Actors", meta = (ClampMin = "1.0", Units = "cm"))
	float SpacingDistance = 1000.f;

	/** Additional yaw (deg) applied on top of the lane's own direction of travel -- 0 faces along the lane (with the lane's own Direction, reversed for Backward), 90 faces to its right, etc. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lane Actors", meta = (Units = "deg"))
	float HeadingOffsetDegrees = 0.f;

	/**
	 * When on, each spawn's LaneEdgeOffset and heading are randomly jittered by up to
	 * OffsetVarianceRange/HeadingVarianceRangeDegrees -- breaks up an otherwise perfectly regular row
	 * for a more natural look (trees, street furniture).
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lane Actors")
	bool bUseOffsetAndHeadingVariance = false;

	/** Maximum random +/- jitter (cm) applied to LaneEdgeOffset per spawn when bUseOffsetAndHeadingVariance is on. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lane Actors", meta = (EditCondition = "bUseOffsetAndHeadingVariance", ClampMin = "0.0", Units = "cm"))
	float OffsetVarianceRange = 0.f;

	/** Maximum random +/- jitter (deg) applied to the actor's heading per spawn when bUseOffsetAndHeadingVariance is on. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lane Actors", meta = (EditCondition = "bUseOffsetAndHeadingVariance", ClampMin = "0.0", Units = "deg"))
	float HeadingVarianceRangeDegrees = 0.f;
};

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

	/**
	 * Only meaningful for a Parking-type lane: orientation of the divider line generated between
	 * adjacent parking bays (see FFlexRoadMarkingBuilder::BuildParkingSpotMarkings), from 0 deg
	 * (parallel parking -- cars lie along the road, dividers run perpendicular to it) to 90 deg
	 * (perpendicular/orthogonal parking -- cars sit crosswise, dividers run along the road). Does
	 * not affect the spacing between dividers; see UFlexNetworkSettings::MarkingParkingSpotSpacing
	 * for that.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lane", meta = (ClampMin = "0.0", ClampMax = "90.0", Units = "deg"))
	float ParkingAngleDegrees = 0.f;

	/** Actors (Blueprints, props) regularly spawned along this lane. See UFlexNetworkSubsystem::GenerateLaneActors. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lane")
	TArray<FFlexLaneActorSpawnEntry> LaneActors;

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

	/**
	 * Vertical height (cm) of every Median-type lane's raised top above the ordinary roadway
	 * surface, with a curb wall of the same height along both its long edges (see
	 * FFlexRoadMeshBuilder::AppendMedianOverlay) -- the same raised-island technique junction corner
	 * refuges already use (see FFlexIntersectionBuilder's CornerIslands), applied along a segment's
	 * own Median-type lane runs instead of a junction's approach gaps. 0 renders a Median lane flush
	 * with the roadway (painted only, no physical curb).
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Profile", meta = (ClampMin = "0.0", Units = "cm"))
	float MedianHeight = 15.f;

	/**
	 * Vertical offset (cm) baked into a Parking-lane overlay's own vertices, above the ordinary
	 * roadway surface it sits on -- keeps a distinct ParkingLaneMaterial from z-fighting with the
	 * plain roadway surface beneath it, the same reasoning as MedianHeight/CurbHeight but per-profile
	 * instead of a project-wide default.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Profile", meta = (ClampMin = "0.0", Units = "cm"))
	float ParkingLaneHeight = 0.3f;

	/**
	 * When on, generates a curb wall (ParkingLaneCurbHeight tall, see
	 * FFlexRoadMeshBuilder::AppendParkingLaneCurbs) along both long edges of every contiguous run of
	 * Parking-type lanes -- lets a parking lane read as a physically separated bay instead of flush
	 * roadway. Off by default since most parking lanes are ordinary flush roadway with only a painted
	 * boundary.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Profile")
	bool bGenerateParkingLaneCurbs = false;

	/** Vertical height (cm) of the curb wall generated along each Parking-type lane run when bGenerateParkingLaneCurbs is on -- independent of CurbHeight (the road/sidewalk curb) so a parking bay curb can be a different height. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Profile", meta = (EditCondition = "bGenerateParkingLaneCurbs", ClampMin = "0.0", Units = "cm"))
	float ParkingLaneCurbHeight = 15.f;

	/**
	 * Regularly-spaced small raised, fully-curbed planting patches (tree pits) embedded within this
	 * profile's sidewalks -- MedianMaterial for the fill, CurbMaterial (falling back to
	 * SidewalkMaterial) for the surrounding curb rim on all four sides, producing the "dashed line of
	 * green patches with trees" look common along urban sidewalks (see
	 * FFlexRoadMeshBuilder::AppendSidewalkTreePatches).
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Profile|Sidewalk Tree Patches")
	bool bGenerateSidewalkTreePatches = false;

	/** Spacing (cm) between consecutive tree patches along the sidewalk. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Profile|Sidewalk Tree Patches", meta = (EditCondition = "bGenerateSidewalkTreePatches", ClampMin = "1.0", Units = "cm"))
	float TreePatchSpacing = 800.f;

	/** Length (cm) of each tree patch along the road. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Profile|Sidewalk Tree Patches", meta = (EditCondition = "bGenerateSidewalkTreePatches", ClampMin = "1.0", Units = "cm"))
	float TreePatchLength = 150.f;

	/** Width (cm) of each tree patch across the sidewalk; clamped to fit within SidewalkWidth minus TreePatchInsetFromRoad. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Profile|Sidewalk Tree Patches", meta = (EditCondition = "bGenerateSidewalkTreePatches", ClampMin = "1.0", Units = "cm"))
	float TreePatchWidth = 100.f;

	/** Distance (cm) from the sidewalk's roadway-side edge to the patch's near edge -- positions the patch nearer the curb (typical, default) or further toward the building line. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Profile|Sidewalk Tree Patches", meta = (EditCondition = "bGenerateSidewalkTreePatches", ClampMin = "0.0", Units = "cm"))
	float TreePatchInsetFromRoad = 30.f;

	/** Height (cm) of each tree patch's raised curb rim above the sidewalk surface it sits on. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Profile|Sidewalk Tree Patches", meta = (EditCondition = "bGenerateSidewalkTreePatches", ClampMin = "0.0", Units = "cm"))
	float TreePatchHeight = 10.f;

	/**
	 * Network hierarchy this road type belongs to (motorway down to service), mirroring OSM's own
	 * highway=* classification -- OSM-imported profiles derive this from their highway tag
	 * automatically (see FFlexOsmGraphBuilder::ConfigureProfileFromLaneSignature); hand-authored
	 * profiles should set it to whatever tier the road represents. Intended for a future
	 * intersection right-of-way pass (right-before-left between equal levels, yield/stop for the
	 * lower level at unequal crossings) to compare directly -- this field only carries the
	 * classification today, nothing yet reads it to generate signals or signage.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traffic Rules")
	EFlexRoadDominanceLevel RoadDominanceLevel = EFlexRoadDominanceLevel::Unclassified;

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

	/**
	 * Road-surface overlay drawn over every Bike-type lane (a thin strip sitting just above the
	 * ordinary roadway, the same way markings sit above it -- the bike lane is still physically part
	 * of the roadway, just visually distinguished). Leave unset to render Bike lanes as plain
	 * roadway with no distinct overlay.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rendering")
	TObjectPtr<UMaterialInterface> BikeLaneMaterial = nullptr;

	/**
	 * Road-surface overlay drawn over every Parking-type lane, the same way BikeLaneMaterial
	 * distinguishes Bike lanes -- a thin strip sitting just above the ordinary roadway; the parking
	 * lane is still physically part of the roadway, just visually distinguished. Leave unset to
	 * render Parking lanes as plain roadway with no distinct overlay.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rendering")
	TObjectPtr<UMaterialInterface> ParkingLaneMaterial = nullptr;

	/** Material for solid road-marking lines (e.g. between opposite-direction lanes). Leave unset to generate no solid markings for this profile. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rendering|Markings")
	TObjectPtr<UMaterialInterface> SolidMarkingMaterial = nullptr;

	/** Material for dashed lane-boundary markings (between same-direction lanes, or the sole boundary on a two-lane road). Leave unset to generate no lane dash markings for this profile. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rendering|Markings")
	TObjectPtr<UMaterialInterface> LaneDashMarkingMaterial = nullptr;

	/** Material for the dashed guide line generated on the left border of qualifying lane connectors through a junction (see UFlexNetworkSettings::bMarkingIntersectionLeftmostLaneOnly). Leave unset to generate no intersection dash markings for this profile. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rendering|Markings")
	TObjectPtr<UMaterialInterface> IntersectionDashMarkingMaterial = nullptr;

	/** Material for the short dashes placed orthogonal to the road along a crosswalk's two borders. Leave unset to generate no crosswalk dash markings for this profile. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rendering|Markings")
	TObjectPtr<UMaterialInterface> CrosswalkDashMarkingMaterial = nullptr;

	/** Material for the solid stop line generated in front of a crosswalk for its incoming lane(s). Leave unset to generate no stop lines for this profile. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rendering|Markings")
	TObjectPtr<UMaterialInterface> StopLineMarkingMaterial = nullptr;

	/** Material for the divider line generated between adjacent parking bays on every Parking-type lane. Leave unset to generate no parking-spot markings for this profile. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rendering|Markings")
	TObjectPtr<UMaterialInterface> ParkingMarkingMaterial = nullptr;

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
