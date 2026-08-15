#pragma once

#include "CoreMinimal.h"
#include "Engine/DeveloperSettings.h"
#include "FlexNetworkSettings.generated.h"

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

	/** Target distance (cm) between arc-length mesh-extrusion samples along a segment. */
	UPROPERTY(EditAnywhere, Config, Category = "Mesh", meta = (ClampMin = "1.0", Units = "cm"))
	float ArcLengthSampleStep = 100.f;

	/** Maximum parametric subdivision depth used when building the adaptive t->arc-length lookup table. */
	UPROPERTY(EditAnywhere, Config, Category = "Mesh", meta = (ClampMin = "4", ClampMax = "20"))
	int32 MaxArcLengthSubdivisionDepth = 10;

	/** Chord-deviation tolerance (cm) that stops adaptive arc-length subdivision early. */
	UPROPERTY(EditAnywhere, Config, Category = "Mesh", meta = (ClampMin = "0.01", Units = "cm"))
	float ArcLengthChordTolerance = 2.f;

	/** Extra margin (cm), beyond each road's own half-width, that terrain conforming flattens/blends. */
	UPROPERTY(EditAnywhere, Config, Category = "Terrain", meta = (ClampMin = "0.0", Units = "cm"))
	float TerrainConformMargin = 200.f;

	/** Lateral distance (cm) over which the flattened terrain strip eases back to the original heightmap. */
	UPROPERTY(EditAnywhere, Config, Category = "Terrain", meta = (ClampMin = "1.0", Units = "cm"))
	float TerrainFalloffDistance = 400.f;

	/** Number of segments/junctions that must be dirty before a rebuild is dispatched via ParallelFor instead of running inline. */
	UPROPERTY(EditAnywhere, Config, Category = "Performance", meta = (ClampMin = "1"))
	int32 ParallelRebuildThreshold = 4;

#if WITH_EDITOR
	virtual FText GetSectionText() const override;
#endif
};
