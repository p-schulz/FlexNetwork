#include "OsmDataAssetFactory.h"
#include "Osm/OsmDataAsset.h"
#include "Osm/OsmXmlParser.h"

UOsmDataAssetFactory::UOsmDataAssetFactory()
{
	SupportedClass = UOsmDataAsset::StaticClass();
	Formats.Add(TEXT("osm;OpenStreetMap XML"));
	bCreateNew = false;
	bEditorImport = true;
	bText = true;
	ImportPriority = DefaultImportPriority;
}

UObject* UOsmDataAssetFactory::FactoryCreateFile(UClass* InClass, UObject* InParent, FName InName, EObjectFlags Flags, const FString& Filename, const TCHAR* Parms, FFeedbackContext* Warn, bool& bOutOperationCanceled)
{
	UOsmDataAsset* Asset = NewObject<UOsmDataAsset>(InParent, InClass, InName, Flags);

	TArray<FString> Warnings;
	if (!FOsmXmlParser::ParseFile(Filename, *Asset, &Warnings))
	{
		for (const FString& Warning : Warnings)
		{
			Warn->Log(ELogVerbosity::Error, Warning);
		}
		bOutOperationCanceled = true;
		return nullptr;
	}

	for (const FString& Warning : Warnings)
	{
		Warn->Log(ELogVerbosity::Warning, Warning);
	}

	UE_LOG(LogTemp, Display, TEXT("FlexNetwork: imported OSM asset '%s' -- %d node(s), %d way(s), %d relation(s)."), *InName.ToString(), Asset->Nodes.Num(), Asset->Ways.Num(), Asset->Relations.Num());

	return Asset;
}
