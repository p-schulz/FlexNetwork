#pragma once

#include "CoreMinimal.h"
#include "FlexOsmImportSettings.generated.h"

/** User-configurable knobs for FFlexOsmGraphBuilder::BuildFromOsm -- exposed as-is in the Flex Network edit mode's toolkit. */
USTRUCT(BlueprintType)
struct FLEXNETWORKRUNTIME_API FFlexOsmImportSettings
{
	GENERATED_BODY()

	/** OSM highway=<value> tags to import; ways tagged with anything else are skipped entirely. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OSM Import")
	TArray<FString> HighwayTags = { TEXT("primary"), TEXT("secondary"), TEXT("tertiary"), TEXT("residential") };

	/** World-space radius (cm) within which distinct OSM nodes are merged into a single FlexNetwork junction node. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OSM Import", meta = (ClampMin = "0.0", Units = "cm"))
	float JunctionMergeRadius = 500.f;

	/** Fallback per-lane width (cm), used when a way has no width tag to derive one from. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OSM Import", meta = (ClampMin = "1.0", Units = "cm"))
	float DefaultLaneWidth = 350.f;

	/** Fallback total lane count (both directions combined), used when a way has no lanes tag. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OSM Import", meta = (ClampMin = "1"))
	int32 DefaultLaneCount = 2;

	/** Fallback speed limit (km/h), used when a way has no maxspeed tag. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OSM Import", meta = (ClampMin = "1.0", Units = "km/h"))
	float DefaultSpeedLimitKmh = 50.f;

	/** Sidewalk width (cm) applied to every generated profile; 0 disables sidewalks on OSM-imported roads. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OSM Import", meta = (ClampMin = "0.0", Units = "cm"))
	float SidewalkWidth = 200.f;

	/** If set, this lat/lon is used as the local-projection origin (world position 0,0) instead of the first parsed OSM node -- useful for aligning repeated imports into the same level to the same spot. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OSM Import")
	bool bUseOriginOverride = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OSM Import", meta = (EditCondition = "bUseOriginOverride"))
	FVector2D OriginLatLon = FVector2D::ZeroVector;

	/** Vertical spacing (cm) per whole step of OSM's layer=<n> tag -- e.g. layer=2 sits at 2 * LayerHeightStep above ground datum. Mirrors real-world stacked-grade-separation spacing (a highway overpass above a local road, itself above a rail cutting, ...). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OSM Import|Elevation", meta = (ClampMin = "0.0", Units = "cm"))
	float LayerHeightStep = 500.f;

	/** Height (cm) a bridge=* way is lifted above ground datum when it has no explicit layer tag (or layer=0). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OSM Import|Elevation", meta = (ClampMin = "0.0", Units = "cm"))
	float DefaultBridgeHeight = 500.f;

	/** Depth (cm) a tunnel=* way is sunk below ground datum when it has no explicit layer tag (or layer=0). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OSM Import|Elevation", meta = (ClampMin = "0.0", Units = "cm"))
	float DefaultTunnelDepth = 500.f;

	/**
	 * Distance (cm) over which a node's height eases from the previous way's elevation to this
	 * way's own target elevation, when consecutive ways along the same physical road change
	 * layer/bridge/tunnel status -- creates a smooth ramp on/off a bridge or into/out of a tunnel
	 * instead of an abrupt vertical step at the shared node. Clamped to the way's own length, so a
	 * short way always reaches its target elevation by its last node.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OSM Import|Elevation", meta = (ClampMin = "0.0", Units = "cm"))
	float ElevationTransitionLength = 1500.f;
};
