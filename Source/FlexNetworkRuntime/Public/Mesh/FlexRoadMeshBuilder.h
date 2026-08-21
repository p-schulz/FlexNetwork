#pragma once

#include "CoreMinimal.h"
#include "FlexCurveTypes.h"
#include "RoadTypeProfile.h"
#include "Mesh/FlexMeshSectionData.h"
#include "Math/FlexRotationMinimizingFrame.h"

/**
 * Builds a segment's visible mesh (roadway strip + sidewalk offset strips) by walking the
 * segment's arc-length table in even steps, computing a rotation-minimizing frame at each
 * sample, and extruding the profile's lateral extents through those frames. Sidewalks reuse the
 * exact same frames as the roadway (offset further out laterally) rather than an independent
 * curve-offset computation, per the spec: a Bezier offset curve is not itself a Bezier curve in
 * general, so re-sampling the same RMF chain is both simpler and exactly consistent with the
 * road edge it has to butt up against.
 */
class FLEXNETWORKRUNTIME_API FFlexRoadMeshBuilder
{
public:
	/**
	 * TrimStartArcLength/TrimEndArcLength restrict extrusion to a sub-range of the curve (used to
	 * cut a segment's mesh off at a junction polygon boundary instead of extruding all the way to
	 * the node center); pass 0 and ArcLengthTable.GetTotalLength() for an untrimmed segment.
	 * Sidewalks are trimmed at the same points as the roadway -- the junction side is responsible
	 * for extending its own curb-return sidewalk bands out to meet that same point on each edge
	 * (see FFlexIntersectionBuilder::BuildJunctionCornersForRadius), so this trim doesn't need its
	 * own independent value.
	 */
	static FFlexSegmentMeshResult BuildSegmentMesh(
		const FFlexBezierCurve& Curve,
		const FFlexArcLengthTable& ArcLengthTable,
		const URoadTypeProfile* Profile,
		const FVector& ReferenceUp,
		float SampleStep,
		float TrimStartArcLength,
		float TrimEndArcLength,
		float BikeLaneVerticalOffset = 0.3f);

	/** Frame at a single arc-length position, used both internally and by the subsystem's SampleSegmentAtArcLength query API. */
	static FFlexCurveFrame SampleFrameAtArcLength(const FFlexBezierCurve& Curve, const FFlexArcLengthTable& ArcLengthTable, float ArcLength, const FVector& ReferenceUp);

	/** Same even-step sampling BuildSegmentMesh uses internally, exposed so callers that need the raw frames (e.g. terrain conforming) stay exactly consistent with the mesh instead of re-deriving their own sampling. */
	static TArray<FFlexCurveFrame> BuildFramesForRange(const FFlexBezierCurve& Curve, const FFlexArcLengthTable& ArcLengthTable, const FVector& ReferenceUp, float SampleStep, float TrimStartArcLength, float TrimEndArcLength);

	static TArray<float> BuildSampleArcLengths(float TrimStart, float TrimEnd, float SampleStep);

	/** Appends one flat strip (a quad per consecutive frame pair) spanning [InnerOffset, OuterOffset] laterally and VerticalOffset up from each frame -- the shared strip-extrusion primitive roadway/sidewalk generation and road-marking generation (Mesh/FlexRoadMarkingBuilder.h) both build on. */
	static void AppendExtrudedStrip(FFlexMeshSectionData& Section, const TArray<FFlexCurveFrame>& Frames, float InnerOffset, float OuterOffset, float VerticalOffset);

	/**
	 * Appends one strip per contiguous run of Bike-type lanes in Profile (adjacent Bike lanes with
	 * no other lane between them share one strip; two runs separated by a non-Bike lane get two),
	 * each raised VerticalOffset above the ordinary roadway surface Frames already describes -- a
	 * thin overlay, not a hole cut in the roadway, so it needs no change to roadway/junction
	 * footprint generation. A no-op if Profile has no Bike-type lanes. Does not set Section.Material;
	 * callers set it (typically Profile->BikeLaneMaterial) since a caller with no material configured
	 * may prefer to skip calling this entirely.
	 */
	static void AppendBikeLaneOverlay(FFlexMeshSectionData& Section, const TArray<FFlexCurveFrame>& Frames, const URoadTypeProfile& Profile, float VerticalOffset);

	/** Same merging behavior as AppendBikeLaneOverlay, for Parking-type lanes instead -- raised a thin VerticalOffset above the roadway so a distinct ParkingLaneMaterial doesn't z-fight with the plain roadway surface beneath it. */
	static void AppendParkingLaneOverlay(FFlexMeshSectionData& Section, const TArray<FFlexCurveFrame>& Frames, const URoadTypeProfile& Profile, float VerticalOffset);

	/**
	 * Appends a single vertical quad strip along Frames at lateral offset LateralOffset, from
	 * BaseVerticalOffset up to BaseVerticalOffset + WallHeight -- the curb "face" on one long edge of
	 * a raised strip (e.g. one side of a median, or a tree patch sitting BaseVerticalOffset above the
	 * sidewalk it's embedded in). Winding is auto-detected against the intended outward normal
	 * (bOutwardIsPositiveLateral picks +Right or -Right) the same way
	 * FlexUnifiedRoadMeshBuilder::AppendCurbEdge resolves it for the boolean-union curb pass, rather
	 * than relying on a fixed hand-picked corner order. A no-op if WallHeight is ~0.
	 */
	static void AppendVerticalCurbWall(FFlexMeshSectionData& Section, const TArray<FFlexCurveFrame>& Frames, float LateralOffset, float BaseVerticalOffset, float WallHeight, bool bOutwardIsPositiveLateral);

	/**
	 * Appends a single vertical quad "end cap" at Frame's own arc-length position, spanning laterally
	 * [LateralMinOffset, LateralMaxOffset] and vertically [BaseVerticalOffset, BaseVerticalOffset +
	 * WallHeight] -- the short face closing off one end of an isolated patch (as opposed to
	 * AppendVerticalCurbWall's long edges, which run along Frames). Same auto-detected-winding
	 * technique as AppendVerticalCurbWall, oriented along Frame.Tangent instead of Frame.Right
	 * (bOutwardIsPositiveTangent picks +Tangent or -Tangent). A no-op if WallHeight is ~0.
	 */
	static void AppendVerticalEndCapWall(FFlexMeshSectionData& Section, const FFlexCurveFrame& Frame, float LateralMinOffset, float LateralMaxOffset, float BaseVerticalOffset, float WallHeight, bool bOutwardIsPositiveTangent);

	/**
	 * Appends one raised top strip (into OutTop) and two AppendVerticalCurbWall side walls (into
	 * OutWalls) per contiguous run of Median-type lanes in Profile -- the same contiguous-run
	 * merging AppendBikeLaneOverlay uses, but raised MedianHeight with real curb walls instead of a
	 * flat overlay, since a physical median needs a visible edge a vehicle can't drive over. A no-op
	 * if Profile has no Median-type lanes.
	 */
	static void AppendMedianOverlay(FFlexMeshSectionData& OutTop, FFlexMeshSectionData& OutWalls, const TArray<FFlexCurveFrame>& Frames, const URoadTypeProfile& Profile, float MedianHeight);

	/**
	 * Appends two AppendVerticalCurbWall walls (both long edges, roadway-surface base) per
	 * contiguous run of Parking-type lanes in Profile -- see URoadTypeProfile::bGenerateParkingLaneCurbs.
	 * No top surface: the parking lane's own drivable surface is already part of the ordinary
	 * roadway strip, only its physical boundary is new here. A no-op if Profile has no Parking-type
	 * lanes or WallHeight is ~0.
	 */
	static void AppendParkingLaneCurbs(FFlexMeshSectionData& OutWalls, const TArray<FFlexCurveFrame>& Frames, const URoadTypeProfile& Profile, float WallHeight);

	/**
	 * Appends one small, fully-curbed rectangular patch (top into OutTop, all four curb walls into
	 * OutWalls) at every PatchSpacing interval along [TrimStartArcLength, TrimEndArcLength], spanning
	 * laterally between PatchLateralOffsetA and PatchLateralOffsetB (either order) and vertically from
	 * BaseVerticalOffset (the sidewalk surface it's embedded in) up to BaseVerticalOffset +
	 * PatchHeight -- a tree pit / planting island. Each patch is built from exactly two frames the
	 * same way FlexRoadMarkingBuilder's dash primitive is (a SampleStep equal to PatchLength), since a
	 * patch this small doesn't need finer sampling even on a curved segment. A no-op unless
	 * PatchSpacing/PatchLength and the resolved patch width are all positive.
	 */
	static void AppendSidewalkTreePatches(
		FFlexMeshSectionData& OutTop,
		FFlexMeshSectionData& OutWalls,
		const FFlexBezierCurve& Curve,
		const FFlexArcLengthTable& ArcLengthTable,
		const FVector& ReferenceUp,
		float PatchLateralOffsetA,
		float PatchLateralOffsetB,
		float BaseVerticalOffset,
		float PatchHeight,
		float PatchLength,
		float PatchSpacing,
		float TrimStartArcLength,
		float TrimEndArcLength);
};
