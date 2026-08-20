#pragma once

#include "CoreMinimal.h"
#include "Engine/DeveloperSettings.h"
#include "FlexNetworkSettings.generated.h"

class AFlexNetworkSegmentActor;

/**
 * Project-wide tunables for the FlexNetwork authoring/generation pipeline. Values that vary
 * per road type (max grade, min turn radius) live on URoadTypeProfile instead -- these are the
 * ones that make sense as global defaults (editing/snapping feel, sampling density).
 */
UCLASS(Config = FlexNetwork, DefaultConfig, meta = (DisplayName = "Flex Network"))
class FLEXNETWORKRUNTIME_API UFlexNetworkSettings : public UDeveloperSettings
{
	GENERATED_BODY()

public:
	UFlexNetworkSettings();

	/** World-space radius (cm) within which a new endpoint snaps to an existing node. */
	UPROPERTY(EditAnywhere, Config, Category = "Snapping", meta = (ClampMin = "1.0", Units = "cm"))
	float NodeSnapRadius = 150.f;

	/** World-space radius (cm) within which a dragged endpoint snaps onto an existing segment's midspan (for splitting). */
	UPROPERTY(EditAnywhere, Config, Category = "Snapping", meta = (ClampMin = "1.0", Units = "cm"))
	float SegmentSnapRadius = 100.f;

	/** Angle increment (degrees) that the drag direction snaps to when angle-snap is active. */
	UPROPERTY(EditAnywhere, Config, Category = "Snapping", meta = (ClampMin = "1.0", ClampMax = "90.0", Units = "deg"))
	float AngleSnapIncrementDegrees = 15.f;

	/** Segments shorter than this (cm) are rejected by the drawing tool as degenerate. */
	UPROPERTY(EditAnywhere, Config, Category = "Validation", meta = (ClampMin = "1.0", Units = "cm"))
	float MinSegmentLength = 500.f;

	/** Default fillet radius (cm) used at a junction corner when two extrapolated outer edges do not intersect cleanly. */
	UPROPERTY(EditAnywhere, Config, Category = "Intersections", meta = (ClampMin = "1.0", Units = "cm"))
	float DefaultFilletRadius = 300.f;

	/** Minimum clearance (cm) crosswalks/curb-cuts keep from the junction polygon's centroid. */
	UPROPERTY(EditAnywhere, Config, Category = "Intersections", meta = (ClampMin = "1.0", Units = "cm"))
	float CrosswalkMinClearance = 400.f;

	/** Width (cm) reserved for a crosswalk placed at a junction edge. */
	UPROPERTY(EditAnywhere, Config, Category = "Intersections", meta = (ClampMin = "1.0", Units = "cm"))
	float CrosswalkWidth = 200.f;

	/**
	 * Radius (cm) of the rounded corner island/curb return the sidewalk sweeps around at each
	 * junction corner -- independent of DefaultFilletRadius (which is only the *drivable* polygon
	 * corner's fallback rounding): real curb returns are typically a good deal larger than a tight
	 * pavement-corner fillet, so this defaults bigger.
	 */
	UPROPERTY(EditAnywhere, Config, Category = "Intersections", meta = (ClampMin = "1.0", Units = "cm"))
	float CurbReturnRadius = 450.f;

	/** How many segments each corner island/sidewalk-band arc is sampled with; higher = smoother curve. */
	UPROPERTY(EditAnywhere, Config, Category = "Intersections", meta = (ClampMin = "2", ClampMax = "32"))
	int32 CurbReturnArcSegments = 12;

	/**
	 * Minimum angle (degrees) between two angularly-adjacent approaches at a junction for that
	 * pair to be treated as a genuine corner. Below this, the two roads are near-parallel (e.g.
	 * two carriageways of a divided road meeting at a shallow angle) and get no curb-return
	 * sidewalk band/island between them -- sidewalks only bridge real corners, never a gap
	 * between roads that are essentially continuing in the same direction.
	 */
	UPROPERTY(EditAnywhere, Config, Category = "Intersections", meta = (ClampMin = "0.0", ClampMax = "90.0", Units = "deg"))
	float ParallelApproachAngleToleranceDegrees = 30.f;

	/**
	 * Optional extra clearance (cm) used when deciding whether two junction trim boundaries consume
	 * the road section between them. At zero, sidewalk/curb suppression occurs only when the trims
	 * touch or overlap. Increase this to deliberately merge nearby junction roadside regions.
	 */
	UPROPERTY(EditAnywhere, Config, Category = "Intersections", meta = (ClampMin = "0.0", Units = "cm"))
	float CloseJunctionRoadsideClearance = 0.f;

	/**
	 * Vertical offset (cm) every generated road/junction mesh sits above its own logical height --
	 * terrain conforming flattens the landscape to that same logical height, so without this the
	 * generated meshes and the landscape underneath them are exactly coplanar and z-fight. Applied
	 * as a uniform component-level offset (see AFlexNetworkMeshActor), not baked into vertex data.
	 */
	UPROPERTY(EditAnywhere, Config, Category = "Mesh", meta = (ClampMin = "0.0", Units = "cm"))
	float MeshZFightOffset = 1.f;

	/** Target distance (cm) between arc-length mesh-extrusion samples along a segment. */
	UPROPERTY(EditAnywhere, Config, Category = "Mesh", meta = (ClampMin = "1.0", Units = "cm"))
	float ArcLengthSampleStep = 100.f;

	/** Maximum parametric subdivision depth used when building the adaptive t->arc-length lookup table. */
	UPROPERTY(EditAnywhere, Config, Category = "Mesh", meta = (ClampMin = "4", ClampMax = "20"))
	int32 MaxArcLengthSubdivisionDepth = 10;

	/** Chord-deviation tolerance (cm) that stops adaptive arc-length subdivision early. */
	UPROPERTY(EditAnywhere, Config, Category = "Mesh", meta = (ClampMin = "0.01", Units = "cm"))
	float ArcLengthChordTolerance = 2.f;

	/**
	 * Minimum area (cm^2) retained for boolean-generated road/sidewalk components and holes.
	 * Applied only after road unification, so it removes residual slivers without changing the
	 * source graph or junction merge behavior. Set to 0 to disable filtering.
	 */
	UPROPERTY(EditAnywhere, Config, Category = "Mesh", meta = (ClampMin = "0.0"))
	double MinimumGeneratedPolygonArea = 10000.0;

	/** Vertical offset (cm) baked into a Bike-lane overlay's own vertices, above the ordinary roadway surface it sits on -- keeps it from z-fighting with the road beneath, the same way MarkingVerticalOffset does for road markings. */
	UPROPERTY(EditAnywhere, Config, Category = "Mesh", meta = (ClampMin = "0.0", Units = "cm"))
	float BikeLaneVerticalOffset = 0.3f;

	/** Vertical offset (cm) baked into a Parking-lane overlay's own vertices, above the ordinary roadway surface it sits on -- same reasoning as BikeLaneVerticalOffset. */
	UPROPERTY(EditAnywhere, Config, Category = "Mesh", meta = (ClampMin = "0.0", Units = "cm"))
	float ParkingLaneVerticalOffset = 0.3f;

	/**
	 * Optional actor subclass spawned for every segment. A Blueprint subclass can preconfigure the
	 * built-in PCG component with a graph; null uses the native visualization actor.
	 */
	UPROPERTY(EditAnywhere, Config, Category = "PCG")
	TSubclassOf<AFlexNetworkSegmentActor> SegmentActorClass;

	/** Extra margin (cm), beyond each road's own half-width, that terrain conforming flattens/blends. */
	UPROPERTY(EditAnywhere, Config, Category = "Terrain", meta = (ClampMin = "0.0", Units = "cm"))
	float TerrainConformMargin = 200.f;

	/** Lateral distance (cm) over which the flattened terrain strip eases back to the original heightmap. */
	UPROPERTY(EditAnywhere, Config, Category = "Terrain", meta = (ClampMin = "1.0", Units = "cm"))
	float TerrainFalloffDistance = 400.f;

	/** Number of segments/junctions that must be dirty before a rebuild is dispatched via ParallelFor instead of running inline. */
	UPROPERTY(EditAnywhere, Config, Category = "Performance", meta = (ClampMin = "1"))
	int32 ParallelRebuildThreshold = 4;

	/** Master switch for road-marking generation (crosswalk dashes, lane boundary lines, intersection guide dashes). Materials are configured per road type on URoadTypeProfile; a profile with every marking material slot left unset still generates none of its own even with this on. */
	UPROPERTY(EditAnywhere, Config, Category = "Road Markings")
	bool bGenerateRoadMarkings = true;

	/** Width (cm) of a solid marking line, generated between two adjacent lanes travelling in opposite directions. */
	UPROPERTY(EditAnywhere, Config, Category = "Road Markings", meta = (ClampMin = "0.1", Units = "cm"))
	float MarkingSolidLineWidth = 12.f;

	/** Width (cm) of each dash in a lane-boundary dashed line (between same-direction lanes, or the sole boundary on a two-lane road). */
	UPROPERTY(EditAnywhere, Config, Category = "Road Markings", meta = (ClampMin = "0.1", Units = "cm"))
	float MarkingLaneDashWidth = 12.f;

	/** Length (cm) of each individual dash in a lane-boundary dashed line. */
	UPROPERTY(EditAnywhere, Config, Category = "Road Markings", meta = (ClampMin = "0.1", Units = "cm"))
	float MarkingLaneDashLength = 300.f;

	/** Gap (cm) between consecutive dashes in a lane-boundary dashed line. */
	UPROPERTY(EditAnywhere, Config, Category = "Road Markings", meta = (ClampMin = "0.0", Units = "cm"))
	float MarkingLaneDashGapLength = 500.f;

	/** Width (cm) of each dash in a guide line generated on a lane's left border through a junction (see bMarkingIntersectionLeftmostLaneOnly for which lanes/movements qualify). */
	UPROPERTY(EditAnywhere, Config, Category = "Road Markings", meta = (ClampMin = "0.1", Units = "cm"))
	float MarkingIntersectionDashWidth = 10.f;

	/** Length (cm) of each individual intersection guide dash. */
	UPROPERTY(EditAnywhere, Config, Category = "Road Markings", meta = (ClampMin = "0.1", Units = "cm"))
	float MarkingIntersectionDashLength = 100.f;

	/** Gap (cm) between consecutive intersection guide dashes. */
	UPROPERTY(EditAnywhere, Config, Category = "Road Markings", meta = (ClampMin = "0.0", Units = "cm"))
	float MarkingIntersectionDashGapLength = 100.f;

	/**
	 * Restricts intersection guide dashes to typical German urban practice: only the leftmost
	 * incoming Vehicle/Bike lane at each approach (the one adjacent to oncoming traffic, normally
	 * the combined left-turn/straight lane) gets a guide dash, and only for its left-turn or
	 * straight movements -- right turns are tight and self-evident and conventionally left
	 * unmarked, as are any non-leftmost through/right lanes. Turn this off to mark every lane's
	 * every movement instead.
	 */
	UPROPERTY(EditAnywhere, Config, Category = "Road Markings")
	bool bMarkingIntersectionLeftmostLaneOnly = true;

	/** Maximum turn angle (degrees) a movement can have and still count as "straight" for bMarkingIntersectionLeftmostLaneOnly's left/straight-only rule. */
	UPROPERTY(EditAnywhere, Config, Category = "Road Markings", meta = (EditCondition = "bMarkingIntersectionLeftmostLaneOnly", ClampMin = "0.0", ClampMax = "90.0", Units = "deg"))
	float MarkingIntersectionStraightAngleToleranceDegrees = 15.f;

	/** Width (cm) of each dash's own thickness (perpendicular to its length, i.e. parallel to the road) along a crosswalk's two long edges. */
	UPROPERTY(EditAnywhere, Config, Category = "Road Markings", meta = (ClampMin = "0.1", Units = "cm"))
	float MarkingCrosswalkDashWidth = 15.f;

	/** Length (cm) of each dash along a crosswalk's long edge -- the edge itself runs orthogonal to the road, so this is each dash's reach across the road. */
	UPROPERTY(EditAnywhere, Config, Category = "Road Markings", meta = (ClampMin = "0.1", Units = "cm"))
	float MarkingCrosswalkDashLength = 40.f;

	/** Gap (cm) between consecutive dashes along a crosswalk's long edge. */
	UPROPERTY(EditAnywhere, Config, Category = "Road Markings", meta = (ClampMin = "0.0", Units = "cm"))
	float MarkingCrosswalkDashGapLength = 40.f;

	/** Extra margin (cm) beyond a crosswalk's own footprint that lane-boundary markings (solid or dashed) are kept clear of -- absorbs FFlexBezierMath::FindNearestArcLength's approximation error as well as giving a clean visual gap. */
	UPROPERTY(EditAnywhere, Config, Category = "Road Markings", meta = (ClampMin = "0.0", Units = "cm"))
	float MarkingCrosswalkExclusionPadding = 50.f;

	/**
	 * Distance (cm) before each end of a segment that actually borders a junction over which a
	 * solid lane-boundary line (opposite-direction lanes) tapers into a dashed one instead --
	 * matches the common real-world practice of breaking a no-passing solid line into a dashed one
	 * on the final approach to an intersection so turning/merging traffic can still change lanes.
	 * The result is that solid lines never run all the way up to (let alone into) a junction.
	 */
	UPROPERTY(EditAnywhere, Config, Category = "Road Markings", meta = (ClampMin = "0.0", Units = "cm"))
	float MarkingSolidToDashedTransitionDistance = 1000.f;

	/** Depth (cm), along the direction of travel, of the solid stop line generated in front of a crosswalk for its incoming lane(s). */
	UPROPERTY(EditAnywhere, Config, Category = "Road Markings", meta = (ClampMin = "0.1", Units = "cm"))
	float MarkingStopLineThickness = 30.f;

	/** Gap (cm) left between a stop line and the crosswalk's near edge. */
	UPROPERTY(EditAnywhere, Config, Category = "Road Markings", meta = (ClampMin = "0.0", Units = "cm"))
	float MarkingStopLineSetback = 50.f;

	/** Vertical offset (cm) baked into every marking's own vertices, above the road surface it sits on -- independent of MeshZFightOffset (which offsets the whole generated-geometry component, not markings specifically). */
	UPROPERTY(EditAnywhere, Config, Category = "Road Markings", meta = (ClampMin = "0.0", Units = "cm"))
	float MarkingVerticalOffset = 0.5f;

	/** Thickness (cm) of each parking-spot divider line generated between adjacent bays on a Parking-type lane. */
	UPROPERTY(EditAnywhere, Config, Category = "Road Markings", meta = (ClampMin = "0.1", Units = "cm"))
	float MarkingParkingLineWidth = 10.f;

	/** Spacing (cm) between consecutive parking-spot divider lines along a Parking-type lane -- the pitch of one parking bay. Independent of FRoadLaneDescriptor::ParkingAngleDegrees, which only controls each divider's own orientation, not how far apart they are. */
	UPROPERTY(EditAnywhere, Config, Category = "Road Markings", meta = (ClampMin = "1.0", Units = "cm"))
	float MarkingParkingSpotSpacing = 550.f;

	/** Starting distance (cm) FFlexTrackJunctionSolver trims each rail segment back from a junction node before solving movement curves between the resulting ports. */
	UPROPERTY(EditAnywhere, Config, Category = "Rail", meta = (ClampMin = "1.0", Units = "cm"))
	float RailJunctionTrimDistance = 1000.f;

	/** Upper bound (cm) the rail junction solver's trim distance is allowed to grow to while retrying movements that fail their minimum-radius check. */
	UPROPERTY(EditAnywhere, Config, Category = "Rail", meta = (ClampMin = "1.0", Units = "cm"))
	float RailJunctionMaxTrimDistance = 2000.f;

	/** How far the rail junction solver expands the trim distance on each retry. */
	UPROPERTY(EditAnywhere, Config, Category = "Rail", meta = (ClampMin = "1.0", Units = "cm"))
	float RailJunctionTrimDistanceStep = 300.f;

	/** Maximum number of trim-distance retries the rail junction solver attempts per junction. */
	UPROPERTY(EditAnywhere, Config, Category = "Rail", meta = (ClampMin = "1", ClampMax = "16"))
	int32 RailJunctionMaxSolveIterations = 4;

	/** Maximum lateral distance (cm) between two rail polylines for FFlexRailGraphBuilder to fold their shared span into one merged rail edge (e.g. the common leg of a turnout). */
	UPROPERTY(EditAnywhere, Config, Category = "Rail", meta = (ClampMin = "0.1", Units = "cm"))
	float RailMergeToleranceCm = 3.f;

	/** Maximum tangent deviation (degrees) between two rail polylines for FFlexRailGraphBuilder to still treat them as coincident/mergeable rather than diverging. */
	UPROPERTY(EditAnywhere, Config, Category = "Rail", meta = (ClampMin = "0.0", ClampMax = "90.0", Units = "deg"))
	float RailMergeAngleToleranceDegrees = 10.f;

	/** Minimum tangent angle (degrees) between two rail polylines that don't share a junction port for FFlexRailGraphBuilder to classify their intersection as a genuine crossing rather than a near-tangential touch. */
	UPROPERTY(EditAnywhere, Config, Category = "Rail", meta = (ClampMin = "0.0", ClampMax = "90.0", Units = "deg"))
	float RailCrossingAngleToleranceDegrees = 15.f;

	/** Total visual gap (cm), split evenly between the two rails meeting at a crossing/switch/frog edge, that FFlexRailMeshBuilder trims instead of sweeping rail mesh all the way to the interaction point. */
	UPROPERTY(EditAnywhere, Config, Category = "Rail", meta = (ClampMin = "0.0", Units = "cm"))
	float RailCrossingGapCm = 4.f;

#if WITH_EDITOR
	virtual FText GetSectionText() const override;
#endif
};
