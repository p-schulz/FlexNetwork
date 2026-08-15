#pragma once

#include "CoreMinimal.h"
#include "FlexCurveTypes.generated.h"

/** Cubic Bezier curve: P0/P3 are the endpoint (node) positions, P1/P2 are absolute tangent-handle positions. */
USTRUCT(BlueprintType)
struct FLEXNETWORKRUNTIME_API FFlexBezierCurve
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curve")
	FVector P0 = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curve")
	FVector P1 = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curve")
	FVector P2 = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Curve")
	FVector P3 = FVector::ZeroVector;

	bool IsNearlyDegenerate(float Tolerance = KINDA_SMALL_NUMBER) const
	{
		return FVector::DistSquared(P0, P3) <= FMath::Square(Tolerance);
	}
};

/** One sample in a t->arc-length lookup table, built via adaptive subdivision. */
USTRUCT()
struct FLEXNETWORKRUNTIME_API FFlexArcLengthSample
{
	GENERATED_BODY()

	UPROPERTY()
	float T = 0.f;

	UPROPERTY()
	float ArcLength = 0.f;
};

/**
 * Monotonically-increasing t->arc-length lookup table for a single FFlexBezierCurve, built by
 * adaptive subdivision (see FFlexBezierMath::BuildArcLengthTable). Samples[0] is always
 * (T=0, ArcLength=0); the last sample's ArcLength is the curve's total length.
 */
USTRUCT()
struct FLEXNETWORKRUNTIME_API FFlexArcLengthTable
{
	GENERATED_BODY()

	UPROPERTY()
	TArray<FFlexArcLengthSample> Samples;

	float GetTotalLength() const { return Samples.Num() > 0 ? Samples.Last().ArcLength : 0.f; }
	bool IsValid() const { return Samples.Num() >= 2; }
};

/** How a segment's start/end elevation difference (if any) is blended along its length. */
UENUM(BlueprintType)
enum class EFlexElevationEase : uint8
{
	Linear,
	EaseInOut,
	EaseIn,
	EaseOut
};

USTRUCT(BlueprintType)
struct FLEXNETWORKRUNTIME_API FFlexElevationProfile
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Elevation")
	EFlexElevationEase Ease = EFlexElevationEase::EaseInOut;
};
