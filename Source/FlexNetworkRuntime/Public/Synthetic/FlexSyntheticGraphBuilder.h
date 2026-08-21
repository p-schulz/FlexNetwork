#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "FlexSyntheticGraphBuilder.generated.h"

class UFlexNetworkSubsystem;
class URoadTypeProfile;

/**
 * Phase 1 of the synthetic road-network generation plan: traces hyperstreamlines through a
 * tensor field, planarizes them (via UE::Geometry::FArrangement2d), collapses the result into
 * real road chains, fits a cheap Bezier per chain, and lands everything in UFlexNetworkSubsystem
 * through the exact same AddNode/AddSegment API a human drawing by hand or
 * FlexOsmGraphBuilder::BuildFromOsm would use -- so every existing downstream system (mesh
 * generation, markings, medians, parking, rail, ZoneGraph, MassTraffic) works on a synthetic
 * network for free, with zero new integration code.
 *
 * Phase 2 adds author-able, savable field regions (UFlexSyntheticNetworkConfig below) in place of
 * Phase 1's single built-in two-region field -- this is the plan's actual differentiator: a
 * designer can blend a dense grid district into an organic radial one, or any other combination,
 * by placing FFlexSyntheticFieldRegion entries rather than editing C++.
 */

/** Which of the tensor field's two eigenvector directions a traced streamline followed -- maps directly to road hierarchy. */
enum class EFlexSyntheticRoadTier : uint8
{
	/** Major (field-aligned) streamlines -- the field's own primary direction. */
	Arterial,
	/** Minor (perpendicular) streamlines, seeded along arterials. */
	Local
};

/** Which shape of direction field one FFlexSyntheticFieldRegion contributes. */
UENUM(BlueprintType)
enum class EFlexSyntheticFieldKind : uint8
{
	/** Uniform direction (GridAngleDegrees) everywhere, weighted by distance from Center like every other region. */
	Grid,
	/** Direction radiates outward from Center. */
	Radial
};

/** One basis field contributing to the blended tensor field -- see FlexSyntheticGraphBuilder::SampleFieldDirection. */
USTRUCT(BlueprintType)
struct FLEXNETWORKRUNTIME_API FFlexSyntheticFieldRegion
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Field Region")
	EFlexSyntheticFieldKind Kind = EFlexSyntheticFieldKind::Grid;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Field Region", meta = (Units = "cm"))
	FVector2D Center = FVector2D::ZeroVector;

	/** Only used when Kind == Grid. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Field Region", meta = (EditCondition = "Kind == EFlexSyntheticFieldKind::Grid", Units = "deg"))
	double GridAngleDegrees = 0.0;

	/** Gaussian falloff radius (cm) controlling how far this region's influence reaches relative to others. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Field Region", meta = (ClampMin = "1.0", Units = "cm"))
	double DecayRadius = 5000.0;
};

/**
 * Author-able, savable set of field regions for FlexSyntheticGraphBuilder::GenerateSyntheticNetwork
 * -- mirrors how an OSM import is configured once (a data asset plus reusable import settings)
 * rather than every generation run being hand-built in code. Assign one to
 * UFlexNetworkEdModeSettings::SyntheticNetworkConfig in the toolkit; leave unset there to fall back
 * to the same built-in two-region field Phase 1 used.
 */
UCLASS(BlueprintType)
class FLEXNETWORKRUNTIME_API UFlexSyntheticNetworkConfig : public UDataAsset
{
	GENERATED_BODY()

public:
	/** Every basis field contributing to the blended tensor field. At least one entry is required for this config to be used. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Synthetic Network")
	TArray<FFlexSyntheticFieldRegion> FieldRegions;
};

/**
 * Everything FlexSyntheticGraphBuilder::GenerateSyntheticNetwork needs. FieldRegions is normally
 * populated from a UFlexSyntheticNetworkConfig asset (or a built-in default) by the caller, not
 * hand-built here -- this struct itself stays a plain (non-asset) settings bag, the same shape a
 * per-run OSM import settings struct takes.
 */
struct FLEXNETWORKRUNTIME_API FFlexSyntheticGenerationSettings
{
	FVector2d DomainMin = FVector2d(-10000.0, -10000.0);
	FVector2d DomainMax = FVector2d(10000.0, 10000.0);
	TArray<FFlexSyntheticFieldRegion> FieldRegions;

	/** Integration step length (cm) for streamline tracing. */
	double StepSize = 150.0;
	/** Per-direction step cap (so a streamline traces at most 2x this many points -- forward + backward from its seed). */
	int32 MaxStepsPerStreamline = 120;
	/** Major-direction (arterial) streamlines are seeded on a NumMajorSeeds x NumMajorSeeds grid across the domain. */
	int32 NumMajorSeeds = 4;
	/** Arc-length spacing (cm) along each major streamline at which a perpendicular minor (local) streamline is seeded. */
	double MinorSeedSpacing = 3000.0;
	/** Edges shorter than this (cm) after planarization are dropped as degenerate slivers. */
	double MinSegmentLength = 10.0;

	/**
	 * A streamline stops tracing early if it gets within this distance (cm) of a *different*,
	 * already-completed streamline -- a simplified version of the density-controlled (Jobard &
	 * Lefer-style) seeding this generator otherwise lacks. Without it, streamlines that converge
	 * toward a shared point (e.g. every major streamline's own path toward a radial field's center,
	 * or minor streamlines seeded along several nearby majors all tracing near-identical tangential
	 * arcs) keep going straight through each other, producing many overlapping roads exactly where
	 * the field has the tightest curvature. 0 disables the check entirely.
	 */
	double MinStreamlineSeparation = 600.0;

	/**
	 * Hard cap on how many raw segments get inserted into the FArrangement2d. Planarization cost is
	 * O(segment count squared) -- FArrangement2d::Insert() linearly scans every already-inserted
	 * edge to find crossings, with no spatial acceleration -- so an unbounded segment count can turn
	 * this into a many-minutes-or-worse synchronous stall. Once this cap is hit, remaining
	 * streamlines are skipped and a warning is logged; generation still completes (with a
	 * smaller/incomplete graph) rather than hanging. Phase 0's spike measured ~5000 segments in
	 * ~0.3s, so this can be raised well past the default once you've confirmed timing on your data.
	 */
	int32 MaxRawSegmentsToInsert = 5000;

	/** Flat Z (cm) applied to every generated node -- terrain-awareness is explicitly out of scope for this phase (see the plan document); run "Fit Roads To Terrain" afterward if needed. */
	double BaseElevation = 0.0;
};

struct FLEXNETWORKRUNTIME_API FFlexSyntheticGenerationResult
{
	int32 NumNodesCreated = 0;
	int32 NumSegmentsCreated = 0;

	// Diagnostics, useful for logging/toolkit status even though they're not the primary output.
	int32 NumStreamlinesTraced = 0;
	int32 NumRawSegmentsInserted = 0;
	/** True if Settings.MaxRawSegmentsToInsert was hit -- the source graph was incomplete (some streamlines were skipped) before chain extraction. */
	bool bSegmentBudgetExceeded = false;
	/** FArrangement2d::HasSelfIntersections() before chain extraction -- should be false. A true here means something is badly wrong; NumSegmentsCreated should not be trusted. */
	bool bHasSelfIntersectionsAfterPlanarization = false;
	double GenerationSeconds = 0.0;
};

namespace FlexSyntheticGraphBuilder
{
	/**
	 * Generates a synthetic road network and adds it directly to Subsystem via the same
	 * AddNode/AddSegment API every other network source uses. Wraps the whole run in
	 * BeginBatchUpdate()/EndBatchUpdate() so it triggers one rebuild instead of one per segment.
	 * ResolveProfile is called once per generated segment to pick its URoadTypeProfile from its
	 * tier (mirrors FlexOsmGraphBuilder::BuildFromOsm's own ResolveProfile callback shape) -- a
	 * segment whose callback returns nullptr is skipped (not added) and counted as a warning-worthy
	 * gap, not a hard failure.
	 */
	FLEXNETWORKRUNTIME_API FFlexSyntheticGenerationResult GenerateSyntheticNetwork(
		UFlexNetworkSubsystem& Subsystem,
		const FFlexSyntheticGenerationSettings& Settings,
		TFunctionRef<URoadTypeProfile*(EFlexSyntheticRoadTier)> ResolveProfile);

	/**
	 * Blends every region's contribution into a single direction (unit length) at P, using the
	 * doubled-angle tensor trick (Chen et al., "Interactive Procedural Street Modeling"): a
	 * direction-only (180-degree-periodic) field can't be averaged as a plain vector without
	 * opposite-pointing contributions cancelling, so each region's angle Theta is instead encoded
	 * as (cos 2*Theta, sin 2*Theta), summed linearly (which IS well-defined for this
	 * representation), then halved back via 0.5*atan2(...) to recover the blended direction.
	 * Exposed (not just an implementation detail) so a test can sample it directly.
	 */
	FLEXNETWORKRUNTIME_API FVector2d SampleFieldDirection(const TArray<FFlexSyntheticFieldRegion>& Regions, const FVector2d& P);

	/** The same two-region field (one Radial center, one 30-degree-rotated Grid district) Phase 1 built into the toolkit, scaled to fit within HalfDomainSize of the origin. Used when no UFlexSyntheticNetworkConfig is assigned, so existing setups keep working unchanged. */
	FLEXNETWORKRUNTIME_API TArray<FFlexSyntheticFieldRegion> MakeDefaultFieldRegions(double HalfDomainSize);
}
