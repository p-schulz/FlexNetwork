#pragma once

#include "CoreMinimal.h"
#include "FlexCurveTypes.h"
#include "Math/FlexRotationMinimizingFrame.h"
#include "Mesh/FlexMeshSectionData.h"

class URoadTypeProfile;
struct FFlexCrosswalkPlacement;
struct FFlexLaneConnector;
struct FRoadLaneDescriptor;

/**
 * Width/length/gap tunables for every marking category, resolved once by the caller from
 * UFlexNetworkSettings and threaded through as plain data so the builder itself stays decoupled
 * from the UDeveloperSettings object (and independently testable without one).
 */
struct FLEXNETWORKRUNTIME_API FFlexRoadMarkingParams
{
	float SolidLineWidth = 12.f;
	/** Distance, at each end of a lane boundary that actually borders a junction, over which a solid boundary tapers into a dashed one instead -- see BuildSegmentLaneMarkings. */
	float SolidToDashedTransitionDistance = 1000.f;

	float LaneDashWidth = 12.f;
	float LaneDashLength = 300.f;
	float LaneDashGap = 500.f;

	float IntersectionDashWidth = 10.f;
	float IntersectionDashLength = 100.f;
	float IntersectionDashGap = 100.f;

	/** Thickness of each crosswalk edge dash as painted (perpendicular to the edge, i.e. parallel to the road). */
	float CrosswalkDashWidth = 15.f;
	/** Length of each crosswalk edge dash along the edge itself -- the edge runs orthogonal to the road, so this is also each dash's reach across the road. */
	float CrosswalkDashLength = 40.f;
	float CrosswalkDashGap = 40.f;

	/** Depth (along the direction of travel) of a stop line. */
	float StopLineThickness = 30.f;
	/** Gap left between a stop line and the crosswalk's near edge. */
	float StopLineSetback = 50.f;

	float VerticalOffset = 0.5f;

	/** Thickness (cm) of each parking-spot divider line. */
	float ParkingLineWidth = 10.f;
	/** Spacing (cm) between consecutive parking-spot divider lines along a Parking-type lane. */
	float ParkingSpotSpacing = 550.f;
};

/**
 * Generates road-marking geometry: simple flat dashed quads, and solid lines swept the same way
 * roadway/sidewalk strips are (FFlexRoadMeshBuilder::AppendExtrudedStrip). Every output pointer is
 * optional (nullptr skips that category, e.g. because the profile has no material configured for
 * it) so callers only pay for the categories they actually need.
 */
class FLEXNETWORKRUNTIME_API FFlexRoadMarkingBuilder
{
public:
	/**
	 * Cases 2/3: one line at every adjacent boundary between Vehicle/Bike lanes in Profile, across
	 * [TrimStartArcLength, TrimEndArcLength] of Curve. Exactly two counted lanes always get a dashed
	 * boundary; with more than two, adjacent lanes travelling the same EFlexLaneDirection get a
	 * dashed boundary and opposite directions get a solid one -- except within
	 * Params.SolidToDashedTransitionDistance of an end that bStartAtJunction/bEndAtJunction marks as
	 * actually bordering a junction, where it's dashed regardless (a solid line never reaches a
	 * junction). CrosswalkExclusionArcRanges (each an inclusive [start,end] pair, same arc-length
	 * space as TrimStartArcLength/TrimEndArcLength) are cut out of every resulting span before
	 * sweeping/dashing, so no lane-boundary marking is generated over a crosswalk's footprint.
	 */
	static void BuildSegmentLaneMarkings(
		const FFlexBezierCurve& Curve,
		const FFlexArcLengthTable& ArcLengthTable,
		const URoadTypeProfile* Profile,
		const FVector& ReferenceUp,
		float SampleStep,
		float TrimStartArcLength,
		float TrimEndArcLength,
		bool bStartAtJunction,
		bool bEndAtJunction,
		TConstArrayView<FVector2D> CrosswalkExclusionArcRanges,
		const FFlexRoadMarkingParams& Params,
		FFlexMeshSectionData* OutSolid,
		FFlexMeshSectionData* OutLaneDash);

	/** Case 1: a dashed line, each dash oriented orthogonal to the road, along both of Crosswalk's long edges (the two sides of the crossing corridor, not its short near/far curb-line edges). Nothing else is generated in the crosswalk's footprint. */
	static void BuildCrosswalkMarkings(
		const FFlexCrosswalkPlacement& Crosswalk,
		const FVector& ReferenceUp,
		const FFlexRoadMarkingParams& Params,
		FFlexMeshSectionData* OutCrosswalkDash);

	/** Case 4: one dashed line on Connector's own left border (offset by half LaneWidth), for its full length through the junction. */
	static void BuildIntersectionLaneMarking(
		const FFlexLaneConnector& Connector,
		float LaneWidth,
		const FVector& ReferenceUp,
		const FFlexRoadMarkingParams& Params,
		FFlexMeshSectionData* OutIntersectionDash);

	/**
	 * A single solid stop-line quad centered at Frame (positioned Params.StopLineSetback before a
	 * crosswalk's near edge by the caller), spanning [SpanMinOffset, SpanMaxOffset] laterally
	 * (Frame.Right) -- normally the combined width of every incoming lane at that approach -- and
	 * Params.StopLineThickness deep along Frame.Tangent.
	 */
	static void BuildStopLineMarking(
		const FFlexCurveFrame& Frame,
		float SpanMinOffset,
		float SpanMaxOffset,
		const FFlexRoadMarkingParams& Params,
		FFlexMeshSectionData* OutStopLine);

	/**
	 * One straight divider line at every Params.ParkingSpotSpacing interval strictly inside
	 * [TrimStartArcLength, TrimEndArcLength] -- the boundary between two adjacent parking bays, not
	 * a bay row's own open ends. Each line runs along Frame.Right (Lane.ParkingAngleDegrees = 0,
	 * parallel parking -- spots end to end, dividers perpendicular to the road) rotated toward
	 * Frame.Tangent as the angle approaches 90 (perpendicular/orthogonal parking -- spots
	 * side-by-side, dividers running along the road), and spans Lane.Width centered on
	 * LaneAbsoluteLateralOffset (i.e. Profile->GetLaneLateralOffset(Lane), resolved by the caller so
	 * this stays decoupled from URoadTypeProfile like the rest of this builder). A no-op unless Lane
	 * is a Parking-type lane.
	 */
	static void BuildParkingSpotMarkings(
		const FFlexBezierCurve& Curve,
		const FFlexArcLengthTable& ArcLengthTable,
		const FRoadLaneDescriptor& Lane,
		float LaneAbsoluteLateralOffset,
		const FVector& ReferenceUp,
		float TrimStartArcLength,
		float TrimEndArcLength,
		const FFlexRoadMarkingParams& Params,
		FFlexMeshSectionData* OutParkingMarking);

	/**
	 * Subtracts every Excluded range from [Start, End] and returns the remaining sub-ranges in
	 * ascending order (possibly empty, possibly the original range unchanged if nothing overlapped).
	 * Exposed for testing and for callers that need to split other spans around the same exclusions.
	 */
	static TArray<FVector2D> SubtractExcludedRanges(float Start, float End, TConstArrayView<FVector2D> ExcludedRanges);
};
