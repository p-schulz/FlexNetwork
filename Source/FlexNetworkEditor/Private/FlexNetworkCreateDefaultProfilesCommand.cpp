#include "HAL/IConsoleManager.h"
#include "RoadTypeProfile.h"
#include "AssetRegistry/AssetRegistryModule.h"
#include "UObject/Package.h"
#include "UObject/SavePackage.h"
#include "Misc/PackageName.h"

// The plugin ships no default URoadTypeProfile assets (a data-driven asset is inherently
// project-specific -- there's no single "correct" lane layout to bake in). This console command
// exists so a project adopting the plugin has real, inspectable starting points instead of an
// empty Content Browser folder and a blank asset creation dialog. Run it once from the Output
// Log: `FlexNetwork.CreateDefaultProfiles`.
namespace
{
	URoadTypeProfile* CreateProfileAsset(const FString& AssetName, TFunctionRef<void(URoadTypeProfile&)> Configure)
	{
		const FString PackagePath = TEXT("/FlexNetwork/Profiles/") + AssetName;

		UPackage* Package = CreatePackage(*PackagePath);
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

		const FString PackageFileName = FPackageName::LongPackageNameToFilename(PackagePath, FPackageName::GetAssetPackageExtension());
		UPackage::SavePackage(Package, Profile, *PackageFileName, SaveArgs);

		return Profile;
	}

	FRoadLaneDescriptor MakeVehicleLane(const TCHAR* Name, float LateralOffset, float Width, EFlexLaneDirection Direction, float SpeedLimitKmh)
	{
		FRoadLaneDescriptor Lane;
		Lane.LaneName = Name;
		Lane.LateralOffset = LateralOffset;
		Lane.Width = Width;
		Lane.Type = EFlexLaneType::Vehicle;
		Lane.Direction = Direction;
		Lane.SpeedLimit = SpeedLimitKmh * 100000.f / 3600.f; // km/h -> cm/s
		return Lane;
	}

	void CreateDefaultRoadProfiles()
	{
		int32 NumCreated = 0;

		if (CreateProfileAsset(TEXT("DA_Road_Residential"), [](URoadTypeProfile& P)
		{
			P.Lanes = {
				MakeVehicleLane(TEXT("Forward"), 175.f, 350.f, EFlexLaneDirection::Forward, 50.f),
				MakeVehicleLane(TEXT("Backward"), -175.f, 350.f, EFlexLaneDirection::Backward, 50.f)
			};
			P.SidewalkWidth = 200.f;
			P.CurbHeight = 15.f;
			P.MaxGrade = 0.08f;
			P.MinTurnRadius = 800.f;
		}))
		{
			++NumCreated;
		}

		if (CreateProfileAsset(TEXT("DA_Road_Arterial"), [](URoadTypeProfile& P)
		{
			P.Lanes = {
				MakeVehicleLane(TEXT("Forward_Inner"), 175.f, 350.f, EFlexLaneDirection::Forward, 70.f),
				MakeVehicleLane(TEXT("Forward_Outer"), 525.f, 350.f, EFlexLaneDirection::Forward, 70.f),
				MakeVehicleLane(TEXT("Backward_Inner"), -175.f, 350.f, EFlexLaneDirection::Backward, 70.f),
				MakeVehicleLane(TEXT("Backward_Outer"), -525.f, 350.f, EFlexLaneDirection::Backward, 70.f)
			};
			P.SidewalkWidth = 250.f;
			P.CurbHeight = 15.f;
			P.MaxGrade = 0.06f;
			P.MinTurnRadius = 1500.f;
		}))
		{
			++NumCreated;
		}

		if (CreateProfileAsset(TEXT("DA_Footpath"), [](URoadTypeProfile& P)
		{
			// No vehicle lanes at all: the walkable width comes entirely from the sidewalk offset
			// (see URoadTypeProfile::GetRoadwayHalfWidth -- an empty Lanes array makes it 0, so the
			// two sidewalk strips abut directly at the centerline instead of flanking a road).
			P.Lanes.Reset();
			P.SidewalkWidth = 300.f;
			P.CurbHeight = 0.f;
			P.MaxGrade = 0.12f;
			P.MinTurnRadius = 300.f;
		}))
		{
			++NumCreated;
		}

		UE_LOG(LogTemp, Display, TEXT("FlexNetwork: created %d default road profile asset(s) under /FlexNetwork/Profiles/. Assign one as the Active Profile in the Flex Network mode toolkit to start drawing."), NumCreated);
	}

	FAutoConsoleCommand CreateDefaultProfilesCommand(
		TEXT("FlexNetwork.CreateDefaultProfiles"),
		TEXT("Creates a small set of sample URoadTypeProfile data assets (residential, arterial, footpath) under /FlexNetwork/Profiles/ so there's something to pick in the Flex Network drawing tool out of the box."),
		FConsoleCommandDelegate::CreateStatic(&CreateDefaultRoadProfiles)
	);
}
