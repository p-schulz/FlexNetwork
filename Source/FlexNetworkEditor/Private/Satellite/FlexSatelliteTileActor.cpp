#include "Satellite/FlexSatelliteTileActor.h"
#include "Satellite/FlexSatelliteImagerySettings.h"
#include "Mesh/FlexMeshSectionData.h"
#include "ProceduralMeshComponent.h"
#include "Materials/Material.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Engine/Texture2D.h"

AFlexSatelliteTileActor::AFlexSatelliteTileActor()
{
	PrimaryActorTick.bCanEverTick = false;
	bReplicates = false;

	MeshComponent = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("Mesh"));
	SetRootComponent(MeshComponent);
	MeshComponent->SetMobility(EComponentMobility::Movable);
	MeshComponent->SetCollisionEnabled(ECollisionEnabled::NoCollision);
}

void AFlexSatelliteTileActor::Initialize(
	const double SouthCm, const double NorthCm, const double WestCm, const double EastCm,
	UTexture2D* InAerialTexture, const FFlexLGLImageryLayerPreset& AerialLayer,
	UTexture2D* InLandUseTexture, const FFlexLGLImageryLayerPreset& LandUseLayer,
	const double LandUseOpacity, const FName LandUseOpacityParameterName,
	UMaterialInterface* TileMaterial)
{
	AerialTexture = InAerialTexture;
	LandUseTexture = InLandUseTexture;

	const FVector Sw(WestCm, SouthCm, 0.0);
	const FVector Se(EastCm, SouthCm, 0.0);
	const FVector Ne(EastCm, NorthCm, 0.0);
	const FVector Nw(WestCm, NorthCm, 0.0);

	FFlexMeshSectionData Section;
	// Sw->Nw->Ne->Se is what actually renders front-facing from above in this project's specific
	// setup -- confirmed by direct computation, not by analogy: cross(Se-Sw, Ne-Sw).Z =
	// (EastCm-WestCm)*(NorthCm-SouthCm), always positive for a valid bbox, and this project's
	// renderer treats dot(cross(edge1,edge2), Normal) > 0 as BACK-facing (established empirically
	// via direct user observation during the junction-mesh work in FlexIntersectionBuilder.cpp --
	// see its own winding comments). Sw->Se->Ne->Nw (the previous order here) has exactly that
	// positive dot product, so it was back-facing and invisible from a top-down view regardless of
	// material -- Sw->Nw->Ne->Se flips the sign and is the corrected order. (The previous comment
	// here reasoned by pattern-matching FFlexRoadMeshBuilder::AppendExtrudedStrip's own
	// Frenet-frame-relative winding, which doesn't actually transfer to a fixed world-aligned quad
	// -- "CCW" alone doesn't determine facing without also fixing which vectors CCW is computed
	// from.) WMS GetMap responses have image row 0 at the north edge, so V=0 sits at the north
	// corners.
	Section.AppendQuad(
		Sw, Nw, Ne, Se,
		FVector::UpVector, FVector::ForwardVector,
		FVector2D(0.f, 1.f), FVector2D(0.f, 0.f), FVector2D(1.f, 0.f), FVector2D(1.f, 1.f));

	MeshComponent->CreateMeshSection(0, Section.Vertices, Section.Triangles, Section.Normals, Section.UV0, TArray<FVector2D>(), TArray<FVector2D>(), TArray<FVector2D>(), Section.VertexColors, Section.Tangents, false);

	// Always ends up with SOME material applied -- never silently leaves the section with none --
	// so a tile that's still invisible after this points squarely at geometry/visibility (the bug
	// just fixed above) rather than leaving "is a material even applied?" as an open question the
	// next time this needs debugging. Exposed via AppliedMaterial (VisibleAnywhere) for the same
	// reason: inspectable directly on the spawned actor without digging into the component.
	UMaterialInterface* EffectiveMaterial = TileMaterial;
	if (!EffectiveMaterial)
	{
		EffectiveMaterial = UMaterial::GetDefaultMaterial(MD_Surface);
		UE_LOG(LogTemp, Warning, TEXT("FlexNetwork: satellite tile '%s' has no PreviewTileMaterial resolved (unset, or the configured asset failed to load) -- using the engine default material. Set UFlexSatelliteImagerySettings::PreviewTileMaterial in Project Settings."), *GetActorLabel());
	}

	UMaterialInstanceDynamic* Mid = UMaterialInstanceDynamic::Create(EffectiveMaterial, this);
	if (AerialTexture && !AerialLayer.TextureParameterName.IsNone())
	{
		Mid->SetTextureParameterValue(AerialLayer.TextureParameterName, AerialTexture);
	}
	if (LandUseTexture && !LandUseLayer.TextureParameterName.IsNone())
	{
		Mid->SetTextureParameterValue(LandUseLayer.TextureParameterName, LandUseTexture);
		Mid->SetScalarParameterValue(LandUseOpacityParameterName, static_cast<float>(LandUseOpacity));
	}
	AppliedMaterial = Mid;
	MeshComponent->SetMaterial(0, Mid);
}
