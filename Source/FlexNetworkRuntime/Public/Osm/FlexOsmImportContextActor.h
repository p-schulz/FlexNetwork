#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Osm/FlexOsmImportSettings.h"
#include "FlexOsmImportContextActor.generated.h"

class UOsmDataAsset;
class USceneComponent;
class UWorld;

/**
 * Level-owned OSM source and projection contract shared by FlexNetwork and other OSM consumers.
 * Keeping this in the level prevents transient editor-mode settings from silently drifting apart.
 */
UCLASS(BlueprintType, NotPlaceable)
class FLEXNETWORKRUNTIME_API AFlexOsmImportContextActor : public AActor
{
	GENERATED_BODY()

public:
	AFlexOsmImportContextActor();

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "OSM Context")
	TObjectPtr<UOsmDataAsset> OsmAsset;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "OSM Context", meta = (ShowOnlyInnerProperties))
	FFlexOsmImportSettings ImportSettings;

	/** Resolved WGS84 latitude/longitude whose projected world position is X=0, Y=0. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "OSM Context")
	FVector2D ResolvedOriginLatLon = FVector2D::ZeroVector;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "OSM Context")
	bool bHasResolvedOrigin = false;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "OSM Context")
	int32 ContextRevision = 0;

	/** Publishes a source/settings pair and refreshes the exact shared projection origin. */
	void SetContext(UOsmDataAsset* InOsmAsset, const FFlexOsmImportSettings& InImportSettings);

	static AFlexOsmImportContextActor* Find(UWorld* World);
	static AFlexOsmImportContextActor* FindOrCreate(UWorld* World);

private:
	UPROPERTY()
	TObjectPtr<USceneComponent> SceneRoot;
};
