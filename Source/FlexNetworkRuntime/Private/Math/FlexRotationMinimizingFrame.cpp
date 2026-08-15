#include "Math/FlexRotationMinimizingFrame.h"
#include "Math/FlexBezierMath.h"

TArray<FFlexCurveFrame> FFlexRotationMinimizingFrame::ComputeFrames(const FFlexBezierCurve& Curve, const FFlexArcLengthTable& Table, float SampleStep, const FVector& ReferenceUp)
{
	const float TotalLength = Table.GetTotalLength();
	if (TotalLength <= KINDA_SMALL_NUMBER)
	{
		const TArray<float> SingleArcLength = { 0.f };
		return ComputeFramesAtArcLengths(Curve, Table, SingleArcLength, ReferenceUp);
	}

	const int32 NumSteps = FMath::Max(1, FMath::CeilToInt(TotalLength / FMath::Max(SampleStep, 1.f)));

	TArray<float> ArcLengths;
	ArcLengths.Reserve(NumSteps + 1);
	for (int32 i = 0; i <= NumSteps; ++i)
	{
		ArcLengths.Add((TotalLength * static_cast<float>(i)) / static_cast<float>(NumSteps));
	}

	return ComputeFramesAtArcLengths(Curve, Table, ArcLengths, ReferenceUp);
}

TArray<FFlexCurveFrame> FFlexRotationMinimizingFrame::ComputeFramesAtArcLengths(const FFlexBezierCurve& Curve, const FFlexArcLengthTable& Table, TConstArrayView<float> ArcLengths, const FVector& ReferenceUp)
{
	TArray<FFlexCurveFrame> Frames;
	Frames.Reserve(ArcLengths.Num());

	for (int32 i = 0; i < ArcLengths.Num(); ++i)
	{
		const float ArcLength = ArcLengths[i];
		const float T = FFlexBezierMath::ArcLengthToT(Table, ArcLength);

		FFlexCurveFrame Frame;
		Frame.Position = FFlexBezierMath::Evaluate(Curve, T);
		Frame.ArcLength = ArcLength;
		Frame.Tangent = FFlexBezierMath::EvaluateDerivative(Curve, T).GetSafeNormal(UE_SMALL_NUMBER, Frames.Num() > 0 ? Frames.Last().Tangent : FVector::ForwardVector);

		if (Frames.Num() == 0)
		{
			// Deterministic seed (not dependent on any propagation history) so that two segments
			// which both touch a shared bend node with aligned tangents and the same ReferenceUp
			// naturally agree here even though each segment's RMF is computed independently. Note
			// this only guarantees an exact match at the node where a segment *starts*; a segment's
			// *propagated* frame at the far end of a strongly curved segment can drift slightly from
			// this same formula (RMF minimizes twist along the way but isn't required to reconverge
			// on it) -- true continuity would need propagating one RMF across a whole chain of
			// connected non-junction segments, which is more machinery than this pass budgets for.
			FVector Right = FVector::CrossProduct(ReferenceUp, Frame.Tangent);
			if (Right.IsNearlyZero())
			{
				Right = FVector::CrossProduct(FVector::RightVector, Frame.Tangent);
			}
			Frame.Right = Right.GetSafeNormal(UE_SMALL_NUMBER, FVector::RightVector);
			Frame.Up = FVector::CrossProduct(Frame.Tangent, Frame.Right).GetSafeNormal(UE_SMALL_NUMBER, ReferenceUp);
		}
		else
		{
			// Double reflection method: reflect the previous frame's (Tangent, Right) pair through
			// the plane bisecting Prev.Position->Position, then through the plane bisecting
			// Prev.Tangent(reflected)->Tangent. Two reflections compose to a rotation, so the
			// result stays an orthonormal, minimally-twisted continuation of the previous frame --
			// no dependence on curvature sign, so no flip at inflection points (unlike a naive
			// Frenet frame, which reads Right/Up directly off the second derivative every sample).
			const FFlexCurveFrame& Prev = Frames.Last();

			const FVector V1 = Frame.Position - Prev.Position;
			const float C1 = FVector::DotProduct(V1, V1);

			FVector ReflectedRight = Prev.Right;
			FVector ReflectedTangent = Prev.Tangent;
			if (C1 > KINDA_SMALL_NUMBER)
			{
				ReflectedRight = Prev.Right - (2.f / C1) * FVector::DotProduct(V1, Prev.Right) * V1;
				ReflectedTangent = Prev.Tangent - (2.f / C1) * FVector::DotProduct(V1, Prev.Tangent) * V1;
			}

			const FVector V2 = Frame.Tangent - ReflectedTangent;
			const float C2 = FVector::DotProduct(V2, V2);

			FVector NewRight = ReflectedRight;
			if (C2 > KINDA_SMALL_NUMBER)
			{
				NewRight = ReflectedRight - (2.f / C2) * FVector::DotProduct(V2, ReflectedRight) * V2;
			}

			// Re-orthogonalize against the (already-normalized) tangent to cancel any drift from
			// floating point error accumulating over many propagation steps.
			NewRight = (NewRight - Frame.Tangent * FVector::DotProduct(NewRight, Frame.Tangent)).GetSafeNormal(UE_SMALL_NUMBER, Prev.Right);

			Frame.Right = NewRight;
			Frame.Up = FVector::CrossProduct(Frame.Tangent, Frame.Right).GetSafeNormal(UE_SMALL_NUMBER, Prev.Up);
		}

		Frames.Add(Frame);
	}

	return Frames;
}
