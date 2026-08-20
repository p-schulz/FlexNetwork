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
	 * Shared implementation behind AppendBikeLaneOverlay/AppendParkingLaneOverlay: merges every
	 * contiguous run of LaneType lanes in Profile (adjacent same-type lanes with nothing else
	 * between them share one strip) into one raised strip each, VerticalOffset above the ordinary
	 * roadway surface Frames already describes.
	 */
	void AppendLaneTypeOverlay(FFlexMeshSectionData& Section, const TArray<FFlexCurveFrame>& Frames, const URoadTypeProfile& Profile, EFlexLaneType LaneType, float VerticalOffset)
	{
		const TArray<FRoadLaneDescriptor> SortedLanes = Profile.GetLanesSortedByOffset();

		bool bInRun = false;
		float RunInner = 0.f;
		float RunOuter = 0.f;
		auto FlushRun = [&Section, &Frames, VerticalOffset, &bInRun, &RunInner, &RunOuter]()
		{
			if (bInRun && RunOuter - RunInner > KINDA_SMALL_NUMBER)
			{
				FFlexRoadMeshBuilder::AppendExtrudedStrip(Section, Frames, RunInner, RunOuter, VerticalOffset);
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

void FFlexRoadMeshBuilder::AppendVerticalCurbWall(FFlexMeshSectionData& Section, const TArray<FFlexCurveFrame>& Frames, float LateralOffset, float WallHeight, bool bOutwardIsPositiveLateral)
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

		const FVector BottomA = F0.Position + F0.Right * LateralOffset;
		const FVector TopA = BottomA + F0.Up * WallHeight;
		const FVector BottomB = F1.Position + F1.Right * LateralOffset;
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

void FFlexRoadMeshBuilder::AppendMedianOverlay(FFlexMeshSectionData& OutTop, FFlexMeshSectionData& OutWalls, const TArray<FFlexCurveFrame>& Frames, const URoadTypeProfile& Profile, float MedianHeight)
{
	const TArray<FRoadLaneDescriptor> SortedLanes = Profile.GetLanesSortedByOffset();

	bool bInRun = false;
	float RunInner = 0.f;
	float RunOuter = 0.f;
	auto FlushRun = [&OutTop, &OutWalls, &Frames, MedianHeight, &bInRun, &RunInner, &RunOuter]()
	{
		if (bInRun && RunOuter - RunInner > KINDA_SMALL_NUMBER)
		{
			AppendExtrudedStrip(OutTop, Frames, RunInner, RunOuter, MedianHeight);
			AppendVerticalCurbWall(OutWalls, Frames, RunInner, MedianHeight, false);
			AppendVerticalCurbWall(OutWalls, Frames, RunOuter, MedianHeight, true);
		}
		bInRun = false;
	};

	for (const FRoadLaneDescriptor& Lane : SortedLanes)
	{
		if (Lane.Type != EFlexLaneType::Median)
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

FFlexSegmentMeshResult FFlexRoadMeshBuilder::BuildSegmentMesh(const FFlexBezierCurve& Curve, const FFlexArcLengthTable& ArcLengthTable, const URoadTypeProfile* Profile, const FVector& ReferenceUp, float SampleStep, float TrimStartArcLength, float TrimEndArcLength, float BikeLaneVerticalOffset, float ParkingLaneVerticalOffset)
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
		AppendParkingLaneOverlay(Result.ParkingLanes, Frames, *Profile, ParkingLaneVerticalOffset);
	}

	return Result;
}
