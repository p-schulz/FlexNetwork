#include "FlexNetworkMassSignalExporter.h"

#include "FlexNetworkSubsystem.h"
#include "Traffic/FlexTrafficSignal.h"
#include "MassTrafficLights.h"

#if WITH_EDITOR
#include "AssetRegistry/AssetRegistryModule.h"
#include "Misc/PackageName.h"
#include "UObject/Package.h"
#include "UObject/SavePackage.h"
#endif

namespace
{
	const TCHAR* GeneratedFolder = TEXT("/Game/FlexNetwork/Generated/");
	const TCHAR* CitySampleTypesPath = TEXT("/Game/AI/Traffic/TrafficLights/CitySampleTrafficLightTypes.CitySampleTrafficLightTypes");
}

#if WITH_EDITOR
namespace
{
	template <typename T>
	T* FindOrCreateGeneratedAsset(const FString& AssetName)
	{
		UPackage* Package = CreatePackage(*(FString(GeneratedFolder) + AssetName));
		Package->FullyLoad();
		if (T* Existing = FindObject<T>(Package, *AssetName))
		{
			return Existing;
		}
		if (T* Loaded = LoadObject<T>(Package, *AssetName))
		{
			return Loaded;
		}
		return NewObject<T>(Package, *AssetName, RF_Public | RF_Standalone);
	}

	bool SaveGeneratedAsset(UObject* Asset)
	{
		if (!Asset)
		{
			return false;
		}
		Asset->SetFlags(RF_Public | RF_Standalone);
		Asset->PostEditChange();
		FAssetRegistryModule::AssetCreated(Asset);
		UPackage* Package = Asset->GetOutermost();
		Package->MarkPackageDirty();
		const FString Filename = FPackageName::LongPackageNameToFilename(
			Package->GetName(), FPackageName::GetAssetPackageExtension());
		FSavePackageArgs Args;
		Args.TopLevelFlags = RF_Public | RF_Standalone;
		return UPackage::SavePackage(Package, Asset, *Filename, Args);
	}
}
#endif

FFlexMassSignalExportResult FFlexNetworkMassSignalExporter::GenerateOrUpdate(
	const UFlexNetworkSubsystem& Network,
	const FFlexMassSignalExportOptions& Options)
{
	FFlexMassSignalExportResult Result;
#if WITH_EDITOR
	FString AssetName = Options.AssetName.TrimStartAndEnd();
	if (AssetName.IsEmpty())
	{
		AssetName = TEXT("DA_FlexNetworkTrafficLights");
	}
	for (int32 Index = 0; Index < AssetName.Len(); ++Index)
	{
		TCHAR& Character = AssetName[Index];
		if (!FChar::IsAlnum(Character) && Character != TEXT('_'))
		{
			Character = TEXT('_');
		}
	}

	UMassTrafficLightTypesDataAsset* Types = Options.TrafficLightTypes;
	if (!Types)
	{
		Types = LoadObject<UMassTrafficLightTypesDataAsset>(nullptr, CitySampleTypesPath);
	}
	if (!Types)
	{
		Types = FindOrCreateGeneratedAsset<UMassTrafficLightTypesDataAsset>(AssetName + TEXT("_Types"));
		if (Types && Types->TrafficLightTypes.IsEmpty())
		{
			FMassTrafficLightTypeData& DefaultType = Types->TrafficLightTypes.AddDefaulted_GetRef();
			DefaultType.Name = TEXT("FlexNetwork_Default");
			DefaultType.NumLanes = 0;
		}
		if (!SaveGeneratedAsset(Types))
		{
			return FFlexMassSignalExportResult();
		}
	}

	UMassTrafficLightInstancesDataAsset* Instances =
		FindOrCreateGeneratedAsset<UMassTrafficLightInstancesDataAsset>(AssetName);
	if (!Instances)
	{
		return FFlexMassSignalExportResult();
	}
	Instances->TrafficLightTypesData = Types;
	Instances->TrafficLights.Reset();

	TArray<const FFlexTrafficSignal*> OrderedSignals;
	OrderedSignals.Reserve(Network.GetAllTrafficSignals().Num());
	for (const TPair<FGuid, FFlexTrafficSignal>& Pair : Network.GetAllTrafficSignals())
	{
		OrderedSignals.Add(&Pair.Value);
	}
	OrderedSignals.Sort([](const FFlexTrafficSignal& A, const FFlexTrafficSignal& B)
	{
		const FString AKey = A.SourceId.IsEmpty() ? A.Id.ToString() : A.SourceId;
		const FString BKey = B.SourceId.IsEmpty() ? B.Id.ToString() : B.SourceId;
		return AKey < BKey;
	});

	for (const FFlexTrafficSignal* SignalPtr : OrderedSignals)
	{
		const FFlexTrafficSignal& Signal = *SignalPtr;
		if (Signal.Type != EFlexTrafficControlType::TrafficLight || !Signal.bEnabled)
		{
			continue;
		}
		FFlexResolvedTrafficSignal Resolved;
		if (!Network.ResolveTrafficSignal(Signal, Resolved))
		{
			continue;
		}
		FMassTrafficLightInstanceDesc& Instance = Instances->TrafficLights.AddDefaulted_GetRef();
		Instance.Position = Resolved.Transform.GetLocation();
		Instance.ZRotation = Resolved.Transform.Rotator().Yaw;
		Instance.ControlledIntersectionSideMidpoint = Resolved.ControlledIntersectionSideMidpoint;
		Instance.TrafficLightTypeIndex = static_cast<int16>(FMath::Clamp(
			Signal.TrafficLightTypeIndex, static_cast<int32>(MIN_int16), static_cast<int32>(MAX_int16)));
	}
	Instances->NumTrafficLights = Instances->TrafficLights.Num();
	if (!SaveGeneratedAsset(Instances))
	{
		return FFlexMassSignalExportResult();
	}

	Result.Types = Types;
	Result.Instances = Instances;
	Result.NumTrafficLights = Instances->TrafficLights.Num();
#endif
	return Result;
}
