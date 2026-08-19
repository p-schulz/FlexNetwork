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

	/** OSM railway=<value> tags imported into the same FlexNetwork graph as roads. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OSM Import|Railways")
	TArray<FString> RailwayTags = { TEXT("rail"), TEXT("light_rail"), TEXT("tram") };

	/** Fallback railway gauge in centimeters (standard gauge = 1435 mm). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OSM Import|Railways", meta = (ClampMin = "1.0", Units = "cm"))
	float DefaultRailGauge = 143.5f;

	/** Width of each generated rail at its base (typical grooved tram rail: 156 mm). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OSM Import|Railways", meta = (ClampMin = "1.0", Units = "cm"))
	float RailWidth = 15.6f;

	/** Width of the raised rail crown (typical grooved tram rail: 115 mm). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OSM Import|Railways", meta = (ClampMin = "1.0", Units = "cm"))
	float RailTopWidth = 11.5f;

	/** Physical rail height above its spline datum (typical grooved tram rail: 72 mm). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OSM Import|Railways", meta = (ClampMin = "0.0", Units = "cm"))
	float RailHeight = 7.2f;

	/** Width of the elevated cutter used for railway=tram grooves. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OSM Import|Railways|Tram", meta = (ClampMin = "0.5", Units = "cm"))
	float TramGrooveWidth = 4.f;

	/** Depth of the tram groove measured down from the crown. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OSM Import|Railways|Tram", meta = (ClampMin = "0.1", Units = "cm"))
	float TramGrooveDepth = 4.5f;

	/** Cutter shift toward the track center, leaving a wider outer shoulder on each rail. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OSM Import|Railways|Tram", meta = (ClampMin = "0.0", Units = "cm"))
	float TramGrooveInwardOffset = 1.5f;

	/** Extra longitudinal overlap used when unifying rails at switches/intersections. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OSM Import|Railways", meta = (ClampMin = "0.0", Units = "cm"))
	float RailBooleanOverlap = 0.5f;

	/** Center-to-center spacing used when an OSM railway way declares tracks=2 or more. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OSM Import|Railways", meta = (ClampMin = "1.0", Units = "cm"))
	float DefaultRailTrackSpacing = 400.f;

	/** Fallback train/light-rail speed when the railway way has no maxspeed tag. Trams use DefaultSpeedLimitKmh. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OSM Import|Railways", meta = (ClampMin = "1.0", Units = "km/h"))
	float DefaultRailSpeedLimitKmh = 80.f;

	/** World-space radius (cm) within which distinct OSM nodes are merged into a single FlexNetwork junction node. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OSM Import", meta = (ClampMin = "0.0", Units = "cm"))
	float JunctionMergeRadius = 500.f;

	/**
	 * Group a compact set of junction nodes connected by short at-grade road segments into one
	 * physical multi-port intersection region. Routing nodes and headings remain distinct, while
	 * internal link geometry is consumed by one filled surface with a single outside curb boundary.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OSM Import|Intersections", meta = (DisplayName = "Group Complex Intersection Interiors"))
	bool bCollapseComplexIntersectionInteriors = true;

	/** Maximum path length (cm), including degree-2 shape points, between portal junctions in one region. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OSM Import|Intersections", meta = (EditCondition = "bCollapseComplexIntersectionInteriors", ClampMin = "0.0", Units = "cm"))
	float ComplexIntersectionInternalEdgeLength = 2000.f;

	/** Maximum diameter (cm) of the complete junction-node component allowed in one shared region. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OSM Import|Intersections", meta = (EditCondition = "bCollapseComplexIntersectionInteriors", ClampMin = "0.0", Units = "cm"))
	float ComplexIntersectionMaxDiameter = 5000.f;

	/**
	 * Compatibility setting retained for existing assets. Multi-port complex regions now preserve
	 * every OSM portal position and heading instead of redirecting approaches through a centroid.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OSM Import|Intersections", meta = (EditCondition = "bCollapseComplexIntersectionInteriors"))
	bool bAlignCollapsedIntersectionThroughRoads = true;

	/** Compatibility value for legacy centroid-alignment data; ignored by multi-port regions. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OSM Import|Intersections", meta = (EditCondition = "bCollapseComplexIntersectionInteriors && bAlignCollapsedIntersectionThroughRoads", ClampMin = "90.0", ClampMax = "180.0", Units = "deg"))
	float ComplexIntersectionMinimumContinuationAngle = 120.f;

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

	/** Imports highway=traffic_signals/stop/give_way nodes as directed, graph-attached controls. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OSM Import|Traffic Signals")
	bool bImportTrafficControls = true;

	/** Places centerline-tagged OSM controls this far beyond the inbound roadway edge. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "OSM Import|Traffic Signals", meta = (EditCondition = "bImportTrafficControls", ClampMin = "0.0", Units = "cm"))
	float TrafficControlRoadEdgeClearance = 25.f;

	/** If set, this lat/lon is used as world XY origin instead of the asset bounds/matching-road fallback. Share the same setting for roads, buildings, and imagery. */
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
