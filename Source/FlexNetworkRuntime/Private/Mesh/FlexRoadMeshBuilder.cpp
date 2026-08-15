#include "Mesh/FlexRoadMeshBuilder.h"
#include "Math/FlexBezierMath.h"

namespace
{
	// UE units are centimeters; dividing arc length by this before using it as a UV V coordinate
	// gives "meters" of texture repeat, a reasonable default tiling scale for road/sidewalk materials.
	constexpr float kUvMetersScale = 100.f;

	void AppendExtrudedStrip(FFlexMeshSectionData& Section, const TArray<FFlexCurveFrame>& Frames, float InnerOffset, float OuterOffset, float VerticalOffset)
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

FFlexSegmentMeshResult FFlexRoadMeshBuilder::BuildSegmentMesh(const FFlexBezierCurve& Curve, const FFlexArcLengthTable& ArcLengthTable, const URoadTypeProfile* Profile, const FVector& ReferenceUp, float SampleStep, float TrimStartArcLength, float TrimEndArcLength)
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

	const float RoadwayHalfWidth = Profile->GetRoadwayHalfWidth();
	Result.Roadway.Material = Profile->RoadMaterial;
	if (RoadwayHalfWidth > KINDA_SMALL_NUMBER)
	{
		AppendExtrudedStrip(Result.Roadway, Frames, -RoadwayHalfWidth, RoadwayHalfWidth, 0.f);
	}

	if (Profile->SidewalkWidth > KINDA_SMALL_NUMBER)
	{
		Result.Sidewalks.Material = Profile->SidewalkMaterial;
		// Sidewalks sit CurbHeight above the roadway surface and immediately outside its edges.
		AppendExtrudedStrip(Result.Sidewalks, Frames, -RoadwayHalfWidth - Profile->SidewalkWidth, -RoadwayHalfWidth, Profile->CurbHeight);
		AppendExtrudedStrip(Result.Sidewalks, Frames, RoadwayHalfWidth, RoadwayHalfWidth + Profile->SidewalkWidth, Profile->CurbHeight);
	}

	return Result;
}
