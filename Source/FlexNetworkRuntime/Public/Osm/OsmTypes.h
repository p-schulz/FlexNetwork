#pragma once

#include "CoreMinimal.h"
#include "OsmTypes.generated.h"

/** Which kind of OSM element an <member> of a <relation> refers to. */
UENUM(BlueprintType)
enum class EOsmElementType : uint8
{
	Node,
	Way,
	Relation
};

/** An OSM <bounds> element: the lat/lon extent the file's data was extracted for. Appears (if at all) as the first child of <osm>, before any <node>/<way>/<relation>. */
USTRUCT(BlueprintType)
struct FLEXNETWORKRUNTIME_API FOsmBounds
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "OSM")
	double MinLat = 0.0;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "OSM")
	double MinLon = 0.0;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "OSM")
	double MaxLat = 0.0;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "OSM")
	double MaxLon = 0.0;

	/** True once parsed from a <bounds> element; false for files that don't have one (e.g. hand-built extracts). */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "OSM")
	bool bIsValid = false;

	/** Midpoint of the bounds -- X = latitude, Y = longitude. */
	FVector2D GetCenter() const { return FVector2D((MinLat + MaxLat) * 0.5, (MinLon + MaxLon) * 0.5); }
};

/** An OSM <node>: a single lat/lon point plus its tags (most nodes have no tags -- they're just shape points on a way). */
USTRUCT(BlueprintType)
struct FLEXNETWORKRUNTIME_API FOsmNode
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "OSM")
	double Latitude = 0.0;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "OSM")
	double Longitude = 0.0;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "OSM")
	TMap<FString, FString> Tags;
};

/** An OSM <way>: an ordered list of node references plus tags (e.g. highway=primary, lanes=2). */
USTRUCT(BlueprintType)
struct FLEXNETWORKRUNTIME_API FOsmWay
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "OSM")
	TArray<int64> NodeRefs;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "OSM")
	TMap<FString, FString> Tags;
};

/** One <member> of an OSM <relation>. */
USTRUCT(BlueprintType)
struct FLEXNETWORKRUNTIME_API FOsmRelationMember
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "OSM")
	EOsmElementType Type = EOsmElementType::Node;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "OSM")
	int64 Ref = 0;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "OSM")
	FString Role;
};

/** An OSM <relation>: a set of member elements (nodes/ways/other relations) plus tags -- e.g. bus routes, turn restrictions, multipolygons. Parsed for completeness/future use; the road-graph importer only consumes ways. */
USTRUCT(BlueprintType)
struct FLEXNETWORKRUNTIME_API FOsmRelation
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "OSM")
	TArray<FOsmRelationMember> Members;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "OSM")
	TMap<FString, FString> Tags;
};
