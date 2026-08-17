#pragma once

#include "CoreMinimal.h"
#include "ProceduralMeshComponent.h"
#include "FlexMeshSectionData.generated.h"

/**
 * Plain-data mirror of one UProceduralMeshComponent mesh section, built off the game thread by
 * the mesh/intersection builders and applied via CreateMeshSection/UpdateMeshSection back on the
 * game thread. Keeping this as a POD result (rather than writing straight into a component)
 * is what makes background mesh generation possible.
 */
USTRUCT()
struct FLEXNETWORKRUNTIME_API FFlexMeshSectionData
{
	GENERATED_BODY()

	UPROPERTY()
	TArray<FVector> Vertices;

	UPROPERTY()
	TArray<int32> Triangles;

	UPROPERTY()
	TArray<FVector> Normals;

	UPROPERTY()
	TArray<FVector2D> UV0;

	UPROPERTY()
	TArray<FProcMeshTangent> Tangents;

	UPROPERTY()
	TArray<FColor> VertexColors;

	UPROPERTY()
	TObjectPtr<UMaterialInterface> Material = nullptr;

	UPROPERTY()
	bool bEnableCollision = true;

	bool IsEmpty() const { return Vertices.Num() == 0 || Triangles.Num() == 0; }

	/** Appends a single triangle (CCW when viewed from the side Normal points to), with UVs and a shared per-vertex normal/tangent -- used for fan-triangulated shapes like a corner island. */
	void AppendTriangle(const FVector& A, const FVector& B, const FVector& C, const FVector& Normal, const FVector& TangentDir, const FVector2D& UvA, const FVector2D& UvB, const FVector2D& UvC, const FColor& Color = FColor::White)
	{
		const int32 Base = Vertices.Num();
		Vertices.Add(A);
		Vertices.Add(B);
		Vertices.Add(C);

		for (int32 i = 0; i < 3; ++i)
		{
			Normals.Add(Normal);
			Tangents.Add(FProcMeshTangent(TangentDir, false));
			VertexColors.Add(Color);
		}

		UV0.Add(UvA);
		UV0.Add(UvB);
		UV0.Add(UvC);

		Triangles.Add(Base + 0);
		Triangles.Add(Base + 1);
		Triangles.Add(Base + 2);
	}

	/** Appends a quad (four corners, CCW when viewed from the side Normal points to) as two triangles, with UVs and a shared per-vertex normal/tangent. */
	void AppendQuad(const FVector& A, const FVector& B, const FVector& C, const FVector& D, const FVector& Normal, const FVector& TangentDir, const FVector2D& UvA, const FVector2D& UvB, const FVector2D& UvC, const FVector2D& UvD, const FColor& Color = FColor::White)
	{
		const int32 Base = Vertices.Num();
		Vertices.Add(A);
		Vertices.Add(B);
		Vertices.Add(C);
		Vertices.Add(D);

		for (int32 i = 0; i < 4; ++i)
		{
			Normals.Add(Normal);
			Tangents.Add(FProcMeshTangent(TangentDir, false));
			VertexColors.Add(Color);
		}

		UV0.Add(UvA);
		UV0.Add(UvB);
		UV0.Add(UvC);
		UV0.Add(UvD);

		Triangles.Add(Base + 0);
		Triangles.Add(Base + 1);
		Triangles.Add(Base + 2);
		Triangles.Add(Base + 0);
		Triangles.Add(Base + 2);
		Triangles.Add(Base + 3);
	}

	/** Same as AppendQuad, but with a separate normal per corner -- used along a curved extrusion so shading blends smoothly between samples instead of faceting per-quad. */
	void AppendQuadSmooth(const FVector& A, const FVector& B, const FVector& C, const FVector& D, const FVector& NA, const FVector& NB, const FVector& NC, const FVector& ND, const FVector& TangentDir, const FVector2D& UvA, const FVector2D& UvB, const FVector2D& UvC, const FVector2D& UvD, const FColor& Color = FColor::White)
	{
		const int32 Base = Vertices.Num();
		Vertices.Add(A);
		Vertices.Add(B);
		Vertices.Add(C);
		Vertices.Add(D);

		Normals.Add(NA);
		Normals.Add(NB);
		Normals.Add(NC);
		Normals.Add(ND);

		for (int32 i = 0; i < 4; ++i)
		{
			Tangents.Add(FProcMeshTangent(TangentDir, false));
			VertexColors.Add(Color);
		}

		UV0.Add(UvA);
		UV0.Add(UvB);
		UV0.Add(UvC);
		UV0.Add(UvD);

		Triangles.Add(Base + 0);
		Triangles.Add(Base + 1);
		Triangles.Add(Base + 2);
		Triangles.Add(Base + 0);
		Triangles.Add(Base + 2);
		Triangles.Add(Base + 3);
	}
};

/** Everything derived from one segment's Curve+Profile: the drivable-lane roadway strip and its sidewalk offset-curve strips. */
USTRUCT()
struct FLEXNETWORKRUNTIME_API FFlexSegmentMeshResult
{
	GENERATED_BODY()

	UPROPERTY()
	FFlexMeshSectionData Roadway;

	UPROPERTY()
	FFlexMeshSectionData Sidewalks;

	/** Arc length (cm), measured from the segment's start (t=0), at which the road/sidewalk strips begin -- 0 unless trimmed by a junction at the start node. */
	float TrimStartArcLength = 0.f;

	/** Arc length (cm), measured from the start, at which the strips end -- segment length unless trimmed by a junction at the end node. */
	float TrimEndArcLength = 0.f;
};

/** Everything derived from a node's junction: the visible drivable surface, crosswalk decal strips, and the rounded sidewalk-corner islands/bands that replace an abrupt sidewalk cutoff. */
USTRUCT()
struct FLEXNETWORKRUNTIME_API FFlexJunctionMeshResult
{
	GENERATED_BODY()

	UPROPERTY()
	FFlexMeshSectionData Surface;

	UPROPERTY()
	FFlexMeshSectionData Crosswalks;

	/** The curved sidewalk strip wrapped around each corner island's outside, connecting one approach's sidewalk to the next. */
	UPROPERTY()
	FFlexMeshSectionData SidewalkCorners;

	/** The raised/landscaped corner refuge each SidewalkCorners band wraps around. */
	UPROPERTY()
	FFlexMeshSectionData CornerIslands;
};
