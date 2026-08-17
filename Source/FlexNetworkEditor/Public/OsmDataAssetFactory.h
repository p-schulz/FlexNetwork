#pragma once

#include "CoreMinimal.h"
#include "Factories/Factory.h"
#include "OsmDataAssetFactory.generated.h"

/** Content Browser import support (drag-drop or right-click > Import) for .osm XML files -> UOsmDataAsset. Thin wrapper: all the actual parsing is FOsmXmlParser (FlexNetworkRuntime), so it isn't an editor-only capability. */
UCLASS()
class UOsmDataAssetFactory : public UFactory
{
	GENERATED_BODY()

public:
	UOsmDataAssetFactory();

	virtual UObject* FactoryCreateFile(UClass* InClass, UObject* InParent, FName InName, EObjectFlags Flags, const FString& Filename, const TCHAR* Parms, FFeedbackContext* Warn, bool& bOutOperationCanceled) override;
};
