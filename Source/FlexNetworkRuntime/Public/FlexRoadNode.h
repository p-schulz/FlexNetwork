#pragma once

#include "CoreMinimal.h"
#include "FlexNetworkTypes.h"
#include "FlexRoadNode.generated.h"

/**
 * A node in the planar road graph. Pure data -- position plus metadata. Connected segment IDs
 * are cached here for O(1) traversal but the graph (UFlexNetworkSubsystem) is what keeps this
 * list authoritative; never mutate ConnectedSegments directly from outside the subsystem.
 */
USTRUCT(BlueprintType)
struct FLEXNETWORKRUNTIME_API FFlexRoadNode
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FlexNetwork")
	FVector Position = FVector::ZeroVector;

	/** Local up vector, used to orient the cross-section at this node (supports banked/sloped terrain). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FlexNetwork")
	FVector UpVector = FVector::UpVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FlexNetwork")
	EFlexRoadElevationType ElevationType = EFlexRoadElevationType::Ground;

	/** Bitmask of EFlexNodeRole, recomputed by the subsystem whenever this node's connectivity changes. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "FlexNetwork")
	uint8 RoleFlags = static_cast<uint8>(EFlexNodeRole::None);

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "FlexNetwork")
	TArray<FFlexSegmentId> ConnectedSegments;

	/** Fixed corner-rounding radius override for this junction; <= 0 means "use UFlexNetworkSettings::DefaultFilletRadius". */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FlexNetwork")
	float FilletRadiusOverride = 0.f;

	bool HasRole(EFlexNodeRole Role) const { return (RoleFlags & static_cast<uint8>(Role)) != 0; }
	bool IsJunction() const { return HasRole(EFlexNodeRole::Junction); }
};
