#pragma once

#include "CoreMinimal.h"
#include "FlexNetworkTypes.h"
#include "FlexCurveTypes.h"

/**
 * One trimmed rail-segment end approaching a junction node -- the rail equivalent of
 * FFlexJunctionApproachInput, but built and consumed entirely by the rail pipeline; it is never
 * passed to FFlexIntersectionBuilder and never affects the road junction polygon.
 */
struct FLEXNETWORKRUNTIME_API FFlexTrackPort
{
	FVector Position = FVector::ZeroVector;

	/** Points away from the junction, back along the approach (i.e. the direction a tram leaves the junction on this track). */
	FVector Direction = FVector::ForwardVector;

	FVector Up = FVector::UpVector;

	float Gauge = 143.5f;
	float RailWidth = 15.6f;
	float MinTurnRadius = 1800.f;

	FFlexSegmentId SegmentId;

	/** True if this port sits on the segment's EndNodeId side (trimmed back from the segment's end); false for the StartNodeId side. */
	bool bAtSegmentEnd = false;

	/** Arc length along the segment (from its start, t=0) this port's trim point sits at. */
	float TrimArcLength = 0.f;
};

/**
 * One permitted movement between two ports at a junction, generating two physical rails once the
 * RailGraph is built. Ports are geometrically undirected (the offset rails don't care which port
 * is "from"); FromPortIndex/ToPortIndex just fix a consistent orientation for the movement curve.
 */
struct FLEXNETWORKRUNTIME_API FFlexTrackMovement
{
	int32 FromPortIndex = INDEX_NONE;
	int32 ToPortIndex = INDEX_NONE;

	FFlexBezierCurve Centerline;
	float MinimumRadius = 1800.f;

	/** Near-straight continuation (e.g. the main line through a turnout). */
	bool bIsStraight = false;

	/** A curving connection between two ports (e.g. the diverging leg of a turnout). */
	bool bIsTurnout = false;
};

/**
 * One rail junction node: its trimmed ports and every movement FFlexTrackJunctionSolver managed
 * to solve between them. A port pair that cannot clear minimum radius even at the maximum trim
 * distance is simply absent from Movements -- see FFlexTrackJunctionSolver's class comment.
 */
struct FLEXNETWORKRUNTIME_API FFlexTrackJunction
{
	FFlexNodeId NodeId;
	TArray<FFlexTrackPort> Ports;
	TArray<FFlexTrackMovement> Movements;
};
