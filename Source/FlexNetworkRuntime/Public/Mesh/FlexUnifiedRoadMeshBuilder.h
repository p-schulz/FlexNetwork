#pragma once

#include "CoreMinimal.h"
#include "Mesh/FlexMeshSectionData.h"

class UMaterialInterface;

/** One already-trimmed road or junction surface supplied to the classic topology pass. */
struct FLEXNETWORKRUNTIME_API FFlexUnifiedRoadPolygonInput
{
	TArray<FVector> Boundary;
	int32 ElevationLayer = 0;
	float SidewalkWidth = 0.f;
	float CurbHeight = 0.f;
	UMaterialInterface* RoadMaterial = nullptr;
	UMaterialInterface* SidewalkMaterial = nullptr;
	UMaterialInterface* CurbMaterial = nullptr;
};

/** Region in which close junctions consume the available roadside space. */
struct FLEXNETWORKRUNTIME_API FFlexUnifiedRoadSuppressionInput
{
	TArray<FVector> Boundary;
	int32 ElevationLayer = 0;
};

/**
 * Boolean-unifies classic road/junction footprints and generates roadside geometry from the
 * resulting exposed edges. It is intentionally independent from the PCG path.
 */
class FLEXNETWORKRUNTIME_API FFlexUnifiedRoadMeshBuilder
{
public:
	static FFlexUnifiedNetworkMeshResult Build(
		TConstArrayView<FFlexUnifiedRoadPolygonInput> SurfaceInputs,
		TConstArrayView<FFlexUnifiedRoadSuppressionInput> SuppressionInputs);
};
