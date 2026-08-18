#pragma once

#include "CoreMinimal.h"
#include "FlexNetworkTypes.generated.h"

/** Which transient world representation the subsystem maintains for authored/imported roads. */
UENUM(BlueprintType)
enum class EFlexNetworkVisualizationMode : uint8
{
	GeneratedGeometry UMETA(DisplayName = "Generated Geometry"),
	SegmentActors UMETA(DisplayName = "Segment Actors (Splines + PCG, No Geometry)"),
	Both UMETA(DisplayName = "Generated Geometry + Segment Actors")
};

/** Elevation/structure type of a node or the segment between two nodes. */
UENUM(BlueprintType)
enum class EFlexRoadElevationType : uint8
{
	Ground,
	Elevated,
	Bridge,
	Tunnel,
	Ramp
};

/** Role a node plays in the planar graph. A node can hold more than one of these at once. */
UENUM(BlueprintType, meta = (Bitflags))
enum class EFlexNodeRole : uint8
{
	None = 0,
	Endpoint = 1 << 0,			// Exactly one connected segment.
	Bend = 1 << 1,				// Exactly two connected segments, no real junction.
	Junction = 1 << 2,			// Three or more connected segments, or two segments meeting at a sharp angle/width mismatch.
	ElevationTransition = 1 << 3	// Node sits between two differing EFlexRoadElevationType segments (ramp on/off point).
};
ENUM_CLASS_FLAGS(EFlexNodeRole)

/** What a lane in a cross-section profile is used for. */
UENUM(BlueprintType)
enum class EFlexLaneType : uint8
{
	Vehicle,
	Parking,
	Bike,
	Sidewalk,
	Median
};

/** Direction of travel a lane allows, relative to the segment's start->end parameterization. */
UENUM(BlueprintType)
enum class EFlexLaneDirection : uint8
{
	Forward,
	Backward,
	Bidirectional,
	None	// Non-traversable lanes: median, decorative strips, etc.
};

/**
 * Lightweight, stable handle to a node in the road graph. Safe for external code (the traffic
 * simulation) to hold across frames without holding a raw pointer into a container that may
 * reallocate -- the Generation field lets TFlexIdAllocator detect and reject stale handles
 * after the slot they pointed to has been freed and reused.
 */
USTRUCT(BlueprintType)
struct FLEXNETWORKRUNTIME_API FFlexNodeId
{
	GENERATED_BODY()

	FFlexNodeId() = default;
	FFlexNodeId(uint32 InIndex, uint32 InGeneration) : Index(InIndex), Generation(InGeneration) {}

	// Not BlueprintReadOnly: uint32 has no Blueprint-compatible representation. The struct itself
	// stays BlueprintType so IDs can still be passed around/compared/stored in BP-visible arrays.
	UPROPERTY(VisibleAnywhere, Category = "FlexNetwork")
	uint32 Index = INDEX_NONE;

	UPROPERTY(VisibleAnywhere, Category = "FlexNetwork")
	uint32 Generation = 0;

	bool IsValid() const { return Index != static_cast<uint32>(INDEX_NONE); }
	static FFlexNodeId Invalid() { return FFlexNodeId(); }

	bool operator==(const FFlexNodeId& Other) const { return Index == Other.Index && Generation == Other.Generation; }
	bool operator!=(const FFlexNodeId& Other) const { return !(*this == Other); }

	friend uint32 GetTypeHash(const FFlexNodeId& Id) { return HashCombine(::GetTypeHash(Id.Index), ::GetTypeHash(Id.Generation)); }

	FString ToString() const { return FString::Printf(TEXT("Node[%u/%u]"), Index, Generation); }
};

/** Lightweight, stable handle to a segment in the road graph. See FFlexNodeId for rationale. */
USTRUCT(BlueprintType)
struct FLEXNETWORKRUNTIME_API FFlexSegmentId
{
	GENERATED_BODY()

	FFlexSegmentId() = default;
	FFlexSegmentId(uint32 InIndex, uint32 InGeneration) : Index(InIndex), Generation(InGeneration) {}

	// Not BlueprintReadOnly: uint32 has no Blueprint-compatible representation. The struct itself
	// stays BlueprintType so IDs can still be passed around/compared/stored in BP-visible arrays.
	UPROPERTY(VisibleAnywhere, Category = "FlexNetwork")
	uint32 Index = INDEX_NONE;

	UPROPERTY(VisibleAnywhere, Category = "FlexNetwork")
	uint32 Generation = 0;

	bool IsValid() const { return Index != static_cast<uint32>(INDEX_NONE); }
	static FFlexSegmentId Invalid() { return FFlexSegmentId(); }

	bool operator==(const FFlexSegmentId& Other) const { return Index == Other.Index && Generation == Other.Generation; }
	bool operator!=(const FFlexSegmentId& Other) const { return !(*this == Other); }

	friend uint32 GetTypeHash(const FFlexSegmentId& Id) { return HashCombine(::GetTypeHash(Id.Index), ::GetTypeHash(Id.Generation)); }

	FString ToString() const { return FString::Printf(TEXT("Segment[%u/%u]"), Index, Generation); }
};

/**
 * Generic free-list ID allocator shared by the node and segment tables. Handing out
 * index+generation pairs (rather than e.g. FGuid) keeps handle comparison/hashing cheap and
 * still lets us detect use of a stale handle after its slot has been freed and reused.
 */
template <typename TId>
class TFlexIdAllocator
{
public:
	TId Allocate()
	{
		uint32 Index;
		if (FreeIndices.Num() > 0)
		{
			Index = FreeIndices.Pop(EAllowShrinking::No);
		}
		else
		{
			Index = static_cast<uint32>(Generations.Add(0));
		}
		return TId(Index, Generations[Index]);
	}

	void Free(const TId& Id)
	{
		if (!ensure(IsValid(Id)))
		{
			return;
		}
		++Generations[Id.Index];
		FreeIndices.Add(Id.Index);
	}

	bool IsValid(const TId& Id) const
	{
		return Id.IsValid() && Generations.IsValidIndex(static_cast<int32>(Id.Index)) && Generations[Id.Index] == Id.Generation;
	}

	void Reset()
	{
		Generations.Reset();
		FreeIndices.Reset();
	}

private:
	TArray<uint32> Generations;
	TArray<uint32> FreeIndices;
};
