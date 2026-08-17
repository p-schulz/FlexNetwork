#include "FlexNetworkAssetUtils.h"
#include "RoadTypeProfile.h"
#include "Materials/MaterialInterface.h"
#include "AssetRegistry/AssetRegistryModule.h"
#include "AssetRegistry/IAssetRegistry.h"
#include "UObject/Package.h"
#include "UObject/SavePackage.h"
#include "Misc/PackageName.h"

URoadTypeProfile* FlexNetworkAssetUtils::CreateRoadTypeProfileAsset(const FString& PackagePath, const FString& AssetName, TFunctionRef<void(URoadTypeProfile&)> Configure)
{
	const FString FullPackagePath = PackagePath / AssetName;

	UPackage* Package = CreatePackage(*FullPackagePath);
	if (!Package)
	{
		return nullptr;
	}

	URoadTypeProfile* Profile = NewObject<URoadTypeProfile>(Package, URoadTypeProfile::StaticClass(), *AssetName, RF_Public | RF_Standalone);
	Configure(*Profile);

	FAssetRegistryModule::AssetCreated(Profile);
	Package->MarkPackageDirty();

	FSavePackageArgs SaveArgs;
	SaveArgs.TopLevelFlags = RF_Public | RF_Standalone;
	SaveArgs.SaveFlags = SAVE_NoError;

	const FString PackageFileName = FPackageName::LongPackageNameToFilename(FullPackagePath, FPackageName::GetAssetPackageExtension());
	UPackage::SavePackage(Package, Profile, *PackageFileName, SaveArgs);

	return Profile;
}

int32 FlexNetworkAssetUtils::ApplyMaterialsToAllProfiles(UMaterialInterface* RoadMaterial, UMaterialInterface* SidewalkMaterial, UMaterialInterface* JunctionMaterial, UMaterialInterface* MedianMaterial)
{
	IAssetRegistry& AssetRegistry = FModuleManager::LoadModuleChecked<FAssetRegistryModule>("AssetRegistry").Get();

	TArray<FAssetData> AssetDataList;
	AssetRegistry.GetAssetsByClass(URoadTypeProfile::StaticClass()->GetClassPathName(), AssetDataList);

	int32 NumModified = 0;
	for (const FAssetData& AssetData : AssetDataList)
	{
		URoadTypeProfile* Profile = Cast<URoadTypeProfile>(AssetData.GetAsset());
		if (!Profile)
		{
			continue;
		}

		bool bChanged = false;
		if (RoadMaterial && Profile->RoadMaterial != RoadMaterial)
		{
			Profile->RoadMaterial = RoadMaterial;
			bChanged = true;
		}
		if (SidewalkMaterial && Profile->SidewalkMaterial != SidewalkMaterial)
		{
			Profile->SidewalkMaterial = SidewalkMaterial;
			bChanged = true;
		}
		if (JunctionMaterial && Profile->JunctionMaterial != JunctionMaterial)
		{
			Profile->JunctionMaterial = JunctionMaterial;
			bChanged = true;
		}
		if (MedianMaterial && Profile->MedianMaterial != MedianMaterial)
		{
			Profile->MedianMaterial = MedianMaterial;
			bChanged = true;
		}

		if (!bChanged)
		{
			continue;
		}
		++NumModified;

		UPackage* Package = Profile->GetPackage();
		Package->MarkPackageDirty();

		FSavePackageArgs SaveArgs;
		SaveArgs.TopLevelFlags = RF_Public | RF_Standalone;
		SaveArgs.SaveFlags = SAVE_NoError;
		const FString PackageFileName = FPackageName::LongPackageNameToFilename(Package->GetName(), FPackageName::GetAssetPackageExtension());
		UPackage::SavePackage(Package, Profile, *PackageFileName, SaveArgs);
	}

	return NumModified;
}
