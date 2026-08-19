#pragma once

#include "CoreMinimal.h"
#include "Math/FlexRotationMinimizingFrame.h"
#include "Mesh/FlexMeshSectionData.h"

class URoadTypeProfile;

/** One sampled railway centerline which contributes both physical rails to a shared result. */
struct FLEXNETWORKRUNTIME_API FFlexRailSweepInput
{
	TArray<FFlexCurveFrame> Frames;
};

/**
 * Builds closed railway solids. Grooved tram profiles are made by first unifying all outer rail
 * solids, unifying their raised asymmetric groove cutters, and finally subtracting the cutter
 * union. Performing the boolean after collecting the complete profile group keeps switches and
 * crossings watertight instead of leaving one capped extrusion per source segment.
 */
class FLEXNETWORKRUNTIME_API FFlexRailMeshBuilder
{
public:
	static bool BuildRailMesh(
		TConstArrayView<FFlexRailSweepInput> Sweeps,
		const URoadTypeProfile* Profile,
		FFlexMeshSectionData& OutSection);
};
