#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "RoadTypeProfile.h"
#include "Osm/FlexOsmGraphBuilder.h"

// OSM-imported profiles should pick up EFlexRoadDominanceLevel from their highway=* tag, and
// hand-authored profiles should default to Unclassified rather than an arbitrary/undefined value.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFlexRoadDominanceLevelTest, "FlexNetwork.Osm.RoadDominanceLevelFromHighwayTag", EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FFlexRoadDominanceLevelTest::RunTest(const FString& Parameters)
{
	auto DominanceLevelForHighwayTag = [](const FString& HighwayTag) -> EFlexRoadDominanceLevel
	{
		URoadTypeProfile* Profile = NewObject<URoadTypeProfile>(GetTransientPackage());
		FFlexOsmGraphBuilder::FLaneSignature Signature;
		Signature.HighwayTag = HighwayTag;
		FFlexOsmGraphBuilder::ConfigureProfileFromLaneSignature(*Profile, Signature);
		return Profile->RoadDominanceLevel;
	};

	TestTrue(TEXT("highway=motorway maps to Motorway"), DominanceLevelForHighwayTag(TEXT("motorway")) == EFlexRoadDominanceLevel::Motorway);
	TestTrue(TEXT("highway=primary maps to Primary"), DominanceLevelForHighwayTag(TEXT("primary")) == EFlexRoadDominanceLevel::Primary);
	TestTrue(TEXT("highway=secondary maps to Secondary"), DominanceLevelForHighwayTag(TEXT("secondary")) == EFlexRoadDominanceLevel::Secondary);
	TestTrue(TEXT("highway=tertiary maps to Tertiary"), DominanceLevelForHighwayTag(TEXT("tertiary")) == EFlexRoadDominanceLevel::Tertiary);
	TestTrue(TEXT("highway=residential maps to Residential"), DominanceLevelForHighwayTag(TEXT("residential")) == EFlexRoadDominanceLevel::Residential);
	TestTrue(TEXT("highway=living_street maps to Residential"), DominanceLevelForHighwayTag(TEXT("living_street")) == EFlexRoadDominanceLevel::Residential);
	TestTrue(TEXT("highway=service maps to Service"), DominanceLevelForHighwayTag(TEXT("service")) == EFlexRoadDominanceLevel::Service);
	TestTrue(TEXT("An unrecognized highway tag falls back to Unclassified"), DominanceLevelForHighwayTag(TEXT("bridleway")) == EFlexRoadDominanceLevel::Unclassified);

	URoadTypeProfile* FreshProfile = NewObject<URoadTypeProfile>(GetTransientPackage());
	TestTrue(TEXT("A freshly-authored profile defaults to Unclassified"), FreshProfile->RoadDominanceLevel == EFlexRoadDominanceLevel::Unclassified);

	TestTrue(TEXT("Motorway outranks Primary (lower enum value = higher priority)"),
		static_cast<uint8>(EFlexRoadDominanceLevel::Motorway) < static_cast<uint8>(EFlexRoadDominanceLevel::Primary));
	TestTrue(TEXT("Tertiary outranks Service"),
		static_cast<uint8>(EFlexRoadDominanceLevel::Tertiary) < static_cast<uint8>(EFlexRoadDominanceLevel::Service));

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
