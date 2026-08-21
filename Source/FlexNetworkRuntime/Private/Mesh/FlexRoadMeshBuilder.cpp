#include "Mesh/FlexRoadMeshBuilder.h"
#include "Math/FlexBezierMath.h"
#include "Mesh/FlexRailMeshBuilder.h"
#include "Rail/FlexRailGraph.h"

namespace
{
	// UE units are centimeters; dividing arc length by this before using it as a UV V coordinate
	// gives "meters" of texture repeat, a reasonable default tiling scale for road/sidewalk materials.
	constexpr float kUvMetersScale = 100.f;

	/**
	 * Invokes VisitRun(RunInner, RunOuter) once per contiguous run of LaneType lanes in Profile
	 * (adjacent same-type lanes with nothing else between them share one run; two runs separated by
	 * a different-type lane get two separate calls). Shared by every per-lane-type overlay/curb
	 * generator (bike/parking lane overlays, median top+walls, parking lane curbs) so the run-merging
	 * logic itself lives in exactly one place.
	 */
	void ForEachLaneTypeRun(const URoadTypeProfile& Profile, EFlexLaneType LaneType, TFunctionRef<void(float RunInner, float RunOuter)> VisitRun)
	{
		const TArray<FRoadLaneDescriptor> SortedLanes = Profile.GetLanesSortedByOffset();

		bool bInRun = false;
		float RunInner = 0.f;
		float RunOuter = 0.f;
		auto FlushRun = [&VisitRun, &bInRun, &RunInner, &RunOuter]()
		{
			if (bInRun && RunOuter - RunInner > KINDA_SMALL_NUMBER)
			{
				VisitRun(RunInner, RunOuter);
			}
			bInRun = false;
		};

		for (const FRoadLaneDescriptor& Lane : SortedLanes)
		{
			if (Lane.Type != LaneType)
			{
				FlushRun();
				continue;
			}

			const float AbsInner = Profile.LateralOffset + FMath::Min(Lane.GetInnerEdge(), Lane.GetOuterEdge());
			const float AbsOuter = Profile.LateralOffset + FMath::Max(Lane.GetInnerEdge(), Lane.GetOuterEdge());
			if (!bInRun)
			{
				bInRun = true;
				RunInner = AbsInner;
				RunOuter = AbsOuter;
			}
			else
			{
				RunInner = FMath::Min(RunInner, AbsInner);
				RunOuter = FMath::Max(RunOuter, AbsOuter);
			}
		}
		FlushRun();
	}

	/** Shared implementation behind AppendBikeLaneOverlay/AppendParkingLaneOverlay. */
	void AppendLaneTypeOverlay(FFlexMeshSectionData& Section, const TArray<FFlexCurveFrame>& Frames, const URoadTypeProfile& Profile, EFlexLaneType LaneType, float VerticalOffset)
	{
		ForEachLaneTypeRun(Profile, LaneType, [&Section, &Frames, VerticalOffset](float RunInner, float RunOuter)
		{
			FFlexRoadMeshBuilder::AppendExtrudedStrip(Section, Frames, RunInner, RunOuter, VerticalOffset);
		});
	}
}

void FFlexRoadMeshBuilder::AppendExtrudedStrip(FFlexMeshSectionData& Section, const TArray<FFlexCurveFrame>& Frames, float InnerOffset, float OuterOffset, float VerticalOffset)
{
	if (Frames.Num() < 2)
	{
		return;
	}

	for (int32 i = 0; i + 1 < Frames.Num(); ++i)
	{
		const FFlexCurveFrame& F0 = Frames[i];
		const FFlexCurveFrame& F1 = Frames[i + 1];

		const FVector InnerA = F0.Position + F0.Right * InnerOffset + F0.Up * VerticalOffset;
		const FVector OuterA = F0.Position + F0.Right * OuterOffset + F0.Up * VerticalOffset;
		const FVector InnerB = F1.Position + F1.Right * InnerOffset + F1.Up * VerticalOffset;
		const FVector OuterB = F1.Position + F1.Right * OuterOffset + F1.Up * VerticalOffset;

		const float V0 = F0.ArcLength / kUvMetersScale;
		const float V1 = F1.ArcLength / kUvMetersScale;

		// CCW winding as seen from +Up (Inner->Outer->Outer->Inner) so the extruded strip faces up.
		Section.AppendQuadSmooth(
			InnerA, OuterA, OuterB, InnerB,
			F0.Up, F0.Up, F1.Up, F1.Up,
			F0.Tangent,
			FVector2D(0.f, V0), FVector2D(1.f, V0), FVector2D(1.f, V1), FVector2D(0.f, V1));
	}
}

void FFlexRoadMeshBuilder::AppendBikeLaneOverlay(FFlexMeshSectionData& Section, const TArray<FFlexCurveFrame>& Frames, const URoadTypeProfile& Profile, float VerticalOffset)
{
	AppendLaneTypeOverlay(Section, Frames, Profile, EFlexLaneType::Bike, VerticalOffset);
}

void FFlexRoadMeshBuilder::AppendParkingLaneOverlay(FFlexMeshSectionData& Section, const TArray<FFlexCurveFrame>& Frames, const URoadTypeProfile& Profile, float VerticalOffset)
{
	AppendLaneTypeOverlay(Section, Frames, Profile, EFlexLaneType::Parking, VerticalOffset);
}

void FFlexRoadMeshBuilder::AppendVerticalCurbWall(FFlexMeshSectionData& Section, const TArray<FFlexCurveFrame>& Frames, float LateralOffset, float BaseVerticalOffset, float WallHeight, bool bOutwardIsPositiveLateral)
{
	if (Frames.Num() < 2 || WallHeight <= KINDA_SMALL_NUMBER)
	{
		return;
	}

	const float OutwardSign = bOutwardIsPositiveLateral ? 1.f : -1.f;
	for (int32 i = 0; i + 1 < Frames.Num(); ++i)
	{
		const FFlexCurveFrame& F0 = Frames[i];
		const FFlexCurveFrame& F1 = Frames[i + 1];

		const FVector BottomA = F0.Position + F0.Right * LateralOffset + F0.Up * BaseVerticalOffset;
		const FVector TopA = BottomA + F0.Up * WallHeight;
		const FVector BottomB = F1.Position + F1.Right * LateralOffset + F1.Up * BaseVerticalOffset;
		const FVector TopB = BottomB + F1.Up * WallHeight;
		const FVector NormalA = (F0.Right * OutwardSign).GetSafeNormal();
		const FVector NormalB = (F1.Right * OutwardSign).GetSafeNormal();
		const FVector Normal = (NormalA + NormalB).GetSafeNormal();

		const float V0 = F0.ArcLength / kUvMetersScale;
		const float V1 = F1.ArcLength / kUvMetersScale;

		// Auto-detect winding against the intended outward Normal -- same technique
		// FlexUnifiedRoadMeshBuilder::AppendCurbEdge uses -- instead of hand-picking corner order
		// for two different possible OutwardSign values.
		const FVector Cross = FVector::CrossProduct(BottomB - BottomA, TopA - BottomA);
		if (FVector::DotProduct(Cross, Normal) > 0.f)
		{
			Section.AppendQuadSmooth(BottomA, TopA, TopB, BottomB, NormalA, NormalA, NormalB, NormalB, F0.Tangent,
				FVector2D(0.f, V0), FVector2D(1.f, V0), FVector2D(1.f, V1), FVector2D(0.f, V1));
		}
		else
		{
			Section.AppendQuadSmooth(BottomA, BottomB, TopB, TopA, NormalA, NormalB, NormalB, NormalA, F0.Tangent,
				FVector2D(0.f, V0), FVector2D(0.f, V1), FVector2D(1.f, V1), FVector2D(1.f, V0));
		}
	}
}

void FFlexRoadMeshBuilder::AppendVerticalEndCapWall(FFlexMeshSectionData& Section, const FFlexCurveFrame& Frame, float LateralMinOffset, float LateralMaxOffset, float BaseVerticalOffset, float WallHeight, bool bOutwardIsPositiveTangent)
{
	if (WallHeight <= KINDA_SMALL_NUMBER || LateralMaxOffset - LateralMinOffset <= KINDA_SMALL_NUMBER)
	{
		return;
	}

	const FVector BottomMin = Frame.Position + Frame.Right * LateralMinOffset + Frame.Up * BaseVerticalOffset;
	const FVector BottomMax = Frame.Position + Frame.Right * LateralMaxOffset + Frame.Up * BaseVerticalOffset;
	const FVector TopMin = BottomMin + Frame.Up * WallHeight;
	const FVector TopMax = BottomMax + Frame.Up * WallHeight;
	const float OutwardSign = bOutwardIsPositiveTangent ? 1.f : -1.f;
	const FVector Normal = (Frame.Tangent * OutwardSign).GetSafeNormal();

	// Same auto-detected-winding technique as AppendVerticalCurbWall, just against a Tangent-facing
	// Normal instead of a Right-facing one.
	const FVector Cross = FVector::CrossProduct(BottomMax - BottomMin, TopMin - BottomMin);
	if (FVector::DotProduct(Cross, Normal) > 0.f)
	{
		Section.AppendQuad(BottomMin, TopMin, TopMax, BottomMax, Normal, Frame.Right,
			FVector2D(0.f, 0.f), FVector2D(0.f, 1.f), FVector2D(1.f, 1.f), FVector2D(1.f, 0.f));
	}
	else
	{
		Section.AppendQuad(BottomMin, BottomMax, TopMax, TopMin, Normal, Frame.Right,
			FVector2D(0.f, 0.f), FVector2D(1.f, 0.f), FVector2D(1.f, 1.f), FVector2D(0.f, 1.f));
	}
}

void FFlexRoadMeshBuilder::AppendMedianOverlay(FFlexMeshSectionData& OutTop, FFlexMeshSectionData& OutWalls, const TArray<FFlexCurveFrame>& Frames, const URoadTypeProfile& Profile, float MedianHeight)
{
	ForEachLaneTypeRun(Profile, EFlexLaneType::Median, [&OutTop, &OutWalls, &Frames, MedianHeight](float RunInner, float RunOuter)
	{
		AppendExtrudedStrip(OutTop, Frames, RunInner, RunOuter, MedianHeight);
		AppendVerticalCurbWall(OutWalls, Frames, RunInner, 0.f, MedianHeight, false);
		AppendVerticalCurbWall(OutWalls, Frames, RunOuter, 0.f, MedianHeight, true);
	});
}

void FFlexRoadMeshBuilder::AppendParkingLaneCurbs(FFlexMeshSectionData& OutWalls, const TArray<FFlexCurveFrame>& Frames, const URoadTypeProfile& Profile, float WallHeight)
{
	ForEachLaneTypeRun(Profile, EFlexLaneType::Parking, [&OutWalls, &Frames, WallHeight](float RunInner, float RunOuter)
	{
		AppendVerticalCurbWall(OutWalls, Frames, RunInner, 0.f, WallHeight, false);
		AppendVerticalCurbWall(OutWalls, Frames, RunOuter, 0.f, WallHeight, true);
	});
}

void FFlexRoadMeshBuilder::AppendSidewalkTreePatches(
	FFlexMeshSectionData& OutTop, FFlexMeshSectionData& OutWalls,
	const FFlexBezierCurve& Curve, const FFlexArcLengthTable& ArcLengthTable, const FVector& ReferenceUp,
	float PatchLateralOffsetA, float PatchLateralOffsetB, float BaseVerticalOffset, float PatchHeight,
	float PatchLength, float PatchSpacing, float TrimStartArcLength, float TrimEndArcLength)
{
	if (PatchSpacing <= KINDA_SMALL_NUMBER || PatchLength <= KINDA_SMALL_NUMBER
		|| TrimEndArcLength - TrimStartArcLength <= KINDA_SMALL_NUMBER || !ArcLengthTable.IsValid())
	{
		return;
	}

	const float InnerOffset = FMath::Min(PatchLateralOffsetA, PatchLateralOffsetB);
	const float OuterOffset = FMath::Max(PatchLateralOffsetA, PatchLateralOffsetB);
	if (OuterOffset - InnerOffset <= KINDA_SMALL_NUMBER)
	{
		return;
	}

	// One patch centered at every PatchSpacing interval, starting half a spacing in so a patch
	// doesn't sit flush against the span's own open end.
	for (float Center = TrimStartArcLength + PatchSpacing * 0.5f; Center < TrimEndArcLength; Center += PatchSpacing)
	{
		const float SpanStart = FMath::Max(TrimStartArcLength, Center - PatchLength * 0.5f);
		const float SpanEnd = FMath::Min(TrimEndArcLength, Center + PatchLength * 0.5f);
		if (SpanEnd - SpanStart <= KINDA_SMALL_NUMBER)
		{
			continue;
		}

		// Exactly two frames (start/end of this patch's short span) -- the same technique
		// FlexRoadMarkingBuilder's dash primitive uses, since a patch this small doesn't benefit
		// from finer sampling even on a curved segment.
		const TArray<FFlexCurveFrame> PatchFrames = BuildFramesForRange(Curve, ArcLengthTable, ReferenceUp, PatchLength, SpanStart, SpanEnd);
		if (PatchFrames.Num() < 2)
		{
			continue;
		}

		AppendExtrudedStrip(OutTop, PatchFrames, InnerOffset, OuterOffset, BaseVerticalOffset + PatchHeight);
		AppendVerticalCurbWall(OutWalls, PatchFrames, InnerOffset, BaseVerticalOffset, PatchHeight, false);
		AppendVerticalCurbWall(OutWalls, PatchFrames, OuterOffset, BaseVerticalOffset, PatchHeight, true);
		AppendVerticalEndCapWall(OutWalls, PatchFrames[0], InnerOffset, OuterOffset, BaseVerticalOffset, PatchHeight, false);
		AppendVerticalEndCapWall(OutWalls, PatchFrames.Last(), InnerOffset, OuterOffset, BaseVerticalOffset, PatchHeight, true);
	}
}

TArray<float> FFlexRoadMeshBuilder::BuildSampleArcLengths(float TrimStart, float TrimEnd, float SampleStep)
{
	TArray<float> ArcLengths;
	const float Span = TrimEnd - TrimStart;
	if (Span <= KINDA_SMALL_NUMBER)
	{
		ArcLengths.Add(TrimStart);
		return ArcLengths;
	}

	const int32 NumSteps = FMath::Max(1, FMath::CeilToInt(Span / FMath::Max(SampleStep, 1.f)));
	ArcLengths.Reserve(NumSteps + 1);
	for (int32 i = 0; i <= NumSteps; ++i)
	{
		ArcLengths.Add(TrimStart + Span * (static_cast<float>(i) / static_cast<float>(NumSteps)));
	}
	return ArcLengths;
}

TArray<FFlexCurveFrame> FFlexRoadMeshBuilder::BuildFramesForRange(const FFlexBezierCurve& Curve, const FFlexArcLengthTable& ArcLengthTable, const FVector& ReferenceUp, float SampleStep, float TrimStartArcLength, float TrimEndArcLength)
{
	const float TotalLength = ArcLengthTable.GetTotalLength();
	const float TrimStart = FMath::Clamp(TrimStartArcLength, 0.f, TotalLength);
	const float TrimEnd = FMath::Clamp(TrimEndArcLength, TrimStart, TotalLength);
	const TArray<float> ArcLengths = BuildSampleArcLengths(TrimStart, TrimEnd, SampleStep);
	return FFlexRotationMinimizingFrame::ComputeFramesAtArcLengths(Curve, ArcLengthTable, ArcLengths, ReferenceUp);
}

FFlexCurveFrame FFlexRoadMeshBuilder::SampleFrameAtArcLength(const FFlexBezierCurve& Curve, const FFlexArcLengthTable& ArcLengthTable, float ArcLength, const FVector& ReferenceUp)
{
	// A single-point query still needs to propagate the RMF from the start of the curve to stay
	// consistent with the frames the rest of the segment's mesh was built with, rather than
	// re-seeding fresh at ArcLength (which would only match by coincidence on a curved segment).
	TArray<float> ArcLengths = BuildSampleArcLengths(0.f, ArcLengthTable.GetTotalLength(), 100.f);
	ArcLengths.AddUnique(FMath::Clamp(ArcLength, 0.f, ArcLengthTable.GetTotalLength()));
	ArcLengths.Sort();

	const TArray<FFlexCurveFrame> Frames = FFlexRotationMinimizingFrame::ComputeFramesAtArcLengths(Curve, ArcLengthTable, ArcLengths, ReferenceUp);
	int32 BestIndex = 0;
	float BestDelta = MAX_flt;
	for (int32 i = 0; i < Frames.Num(); ++i)
	{
		const float Delta = FMath::Abs(Frames[i].ArcLength - ArcLength);
		if (Delta < BestDelta)
		{
			BestDelta = Delta;
			BestIndex = i;
		}
	}
	return Frames.IsValidIndex(BestIndex) ? Frames[BestIndex] : FFlexCurveFrame();
}

FFlexSegmentMeshResult FFlexRoadMeshBuilder::BuildSegmentMesh(const FFlexBezierCurve& Curve, const FFlexArcLengthTable& ArcLengthTable, const URoadTypeProfile* Profile, const FVector& ReferenceUp, float SampleStep, float TrimStartArcLength, float TrimEndArcLength, float BikeLaneVerticalOffset)
{
	FFlexSegmentMeshResult Result;

	if (!Profile || !ArcLengthTable.IsValid())
	{
		return Result;
	}

	const float TotalLength = ArcLengthTable.GetTotalLength();
	const float TrimStart = FMath::Clamp(TrimStartArcLength, 0.f, TotalLength);
	const float TrimEnd = FMath::Clamp(TrimEndArcLength, TrimStart, TotalLength);
	Result.TrimStartArcLength = TrimStart;
	Result.TrimEndArcLength = TrimEnd;

	if (TrimEnd - TrimStart <= KINDA_SMALL_NUMBER)
	{
		// Fully consumed by junction trimming at both ends (e.g. a very short segment between two
		// closely-spaced junctions) -- nothing left to extrude.
		return Result;
	}

	const TArray<FFlexCurveFrame> Frames = BuildFramesForRange(Curve, ArcLengthTable, ReferenceUp, SampleStep, TrimStart, TrimEnd);

	Result.Roadway.Material = Profile->RoadMaterial;
	if (Profile->bIsRailProfile)
	{
		// Single-segment preview (used by the per-segment visualization actor, independent of the
		// graph-wide topology-first pipeline UFlexNetworkSubsystem::BuildRailMeshResults runs): just
		// this one segment's two rails, with no junction/switch/crossing awareness.
		FFlexRailGraph RailGraph;
		const float RailCenterOffset = (Profile->RailGauge + Profile->RailWidth) * 0.5f;
		for (const float Side : { -1.f, 1.f })
		{
			FFlexRailEdge Edge;
			Edge.Type = ERailEdgeType::Normal;
			Edge.bLeftRail = Side < 0.f;
			Edge.Frames.Reserve(Frames.Num());
			for (const FFlexCurveFrame& Frame : Frames)
			{
				FFlexCurveFrame OffsetFrame = Frame;
				OffsetFrame.Position = Frame.Position + Frame.Right * (Side * RailCenterOffset);
				Edge.Frames.Add(OffsetFrame);
			}
			RailGraph.Edges.Add(MoveTemp(Edge));
		}
		FFlexRailMeshBuilder::BuildRailMesh(RailGraph, Profile, 0.f, Result.Roadway);
		return Result;
	}

	const float RoadwayMinOffset = Profile->GetRoadwayMinOffset();
	const float RoadwayMaxOffset = Profile->GetRoadwayMaxOffset();
	if (RoadwayMaxOffset - RoadwayMinOffset > KINDA_SMALL_NUMBER)
	{
		AppendExtrudedStrip(Result.Roadway, Frames, RoadwayMinOffset, RoadwayMaxOffset, 0.f);
	}

	if (Profile->SidewalkWidth > KINDA_SMALL_NUMBER)
	{
		Result.Sidewalks.Material = Profile->SidewalkMaterial;
		// Sidewalks sit CurbHeight above the roadway surface and immediately outside its edges.
		AppendExtrudedStrip(Result.Sidewalks, Frames, RoadwayMinOffset - Profile->SidewalkWidth, RoadwayMinOffset, Profile->CurbHeight);
		AppendExtrudedStrip(Result.Sidewalks, Frames, RoadwayMaxOffset, RoadwayMaxOffset + Profile->SidewalkWidth, Profile->CurbHeight);
	}

	if (Profile->BikeLaneMaterial)
	{
		Result.BikeLanes.Material = Profile->BikeLaneMaterial;
		AppendBikeLaneOverlay(Result.BikeLanes, Frames, *Profile, BikeLaneVerticalOffset);
	}

	if (Profile->MedianMaterial || Profile->CurbMaterial || Profile->SidewalkMaterial)
	{
		Result.Median.Material = Profile->MedianMaterial;
		Result.MedianCurb.Material = Profile->CurbMaterial ? Profile->CurbMaterial : Profile->SidewalkMaterial;
		AppendMedianOverlay(Result.Median, Result.MedianCurb, Frames, *Profile, Profile->MedianHeight);
	}

	if (Profile->ParkingLaneMaterial)
	{
		Result.ParkingLanes.Material = Profile->ParkingLaneMaterial;
		AppendParkingLaneOverlay(Result.ParkingLanes, Frames, *Profile, Profile->ParkingLaneHeight);
	}

	if (Profile->bGenerateParkingLaneCurbs)
	{
		Result.ParkingLaneCurbs.Material = Profile->CurbMaterial ? Profile->CurbMaterial : Profile->SidewalkMaterial;
		AppendParkingLaneCurbs(Result.ParkingLaneCurbs, Frames, *Profile, Profile->ParkingLaneCurbHeight);
	}

	if (Profile->bGenerateSidewalkTreePatches && Profile->SidewalkWidth > KINDA_SMALL_NUMBER
		&& Profile->TreePatchWidth > KINDA_SMALL_NUMBER && Profile->TreePatchLength > KINDA_SMALL_NUMBER
		&& Profile->TreePatchSpacing > KINDA_SMALL_NUMBER)
	{
		const float ActualPatchWidth = FMath::Min(Profile->TreePatchWidth, FMath::Max(0.f, Profile->SidewalkWidth - Profile->TreePatchInsetFromRoad));
		if (ActualPatchWidth > KINDA_SMALL_NUMBER)
		{
			Result.SidewalkTreePatches.Material = Profile->MedianMaterial;
			Result.SidewalkTreePatchCurbs.Material = Profile->CurbMaterial ? Profile->CurbMaterial : Profile->SidewalkMaterial;

			// Left/inner sidewalk sits at [RoadwayMinOffset - SidewalkWidth, RoadwayMinOffset], so its
			// near (road-side) edge is at RoadwayMinOffset and the patch sits further out (more negative).
			const float LeftNear = RoadwayMinOffset - Profile->TreePatchInsetFromRoad;
			const float LeftFar = LeftNear - ActualPatchWidth;
			AppendSidewalkTreePatches(Result.SidewalkTreePatches, Result.SidewalkTreePatchCurbs, Curve, ArcLengthTable, ReferenceUp,
				LeftNear, LeftFar, Profile->CurbHeight, Profile->TreePatchHeight, Profile->TreePatchLength, Profile->TreePatchSpacing, TrimStart, TrimEnd);

			// Right/outer sidewalk sits at [RoadwayMaxOffset, RoadwayMaxOffset + SidewalkWidth], mirrored.
			const float RightNear = RoadwayMaxOffset + Profile->TreePatchInsetFromRoad;
			const float RightFar = RightNear + ActualPatchWidth;
			AppendSidewalkTreePatches(Result.SidewalkTreePatches, Result.SidewalkTreePatchCurbs, Curve, ArcLengthTable, ReferenceUp,
				RightNear, RightFar, Profile->CurbHeight, Profile->TreePatchHeight, Profile->TreePatchLength, Profile->TreePatchSpacing, TrimStart, TrimEnd);
		}
	}

	return Result;
}
