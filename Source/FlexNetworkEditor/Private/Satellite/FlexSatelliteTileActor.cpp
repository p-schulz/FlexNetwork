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

	// Unreal map convention: X=north, Y=east.
	const FVector Sw(SouthCm, WestCm, 0.0);
	const FVector Se(SouthCm, EastCm, 0.0);
	const FVector Ne(NorthCm, EastCm, 0.0);
	const FVector Nw(NorthCm, WestCm, 0.0);

	FFlexMeshSectionData Section;
	// Sw->Se->Ne->Nw has the negative-Z winding this project's mesh-section path renders from above
	// after mapping north onto world X and east onto world Y. WMS image row 0 is the north edge.
	Section.AppendQuad(
		Sw, Se, Ne, Nw,
		FVector::UpVector, FVector::RightVector,
		FVector2D(0.f, 1.f), FVector2D(1.f, 1.f), FVector2D(1.f, 0.f), FVector2D(0.f, 0.f));

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
