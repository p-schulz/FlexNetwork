#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Osm/OsmDataAsset.h"
#include "Osm/OsmXmlParser.h"
#include "Osm/FlexOsmGraphBuilder.h"
#include "FlexNetworkSubsystem.h"
#include "RoadTypeProfile.h"
#include "Editor.h"

namespace
{
	// Two ways sharing a node (Karlsruhe-ish coordinates) forming an L-shaped junction, plus a
	// third way sharing that same junction node -- enough to exercise parsing, way-to-graph
	// conversion, and the multi-way junction that should naturally fall out of it (no
	// special-casing needed: node 20 is referenced by all three ways). Nodes 10/11/12 also sit
	// within a few centimeters of each other (~1e-5 degrees apart) to exercise the junction-merge
	// clustering pass. Attribute values use double quotes deliberately -- UE's lightweight XML
	// tokenizer (FXmlFile) only recognizes double-quoted attributes, unlike a full XML parser.
	// FXmlFile's own source comment: "Assumptions/Misc: Well-formatted file with 1 entry per
	// line" -- its <?xml.../<!DOCTYPE line-stripping pass operates per line, so cramming the
	// whole document onto one line (no real OSM export does that) makes it strip everything, not
	// just the declaration. Every element below deliberately starts on its own line.
	const TCHAR* kSyntheticOsmXml =
		TEXT("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n")
		TEXT("<osm version=\"0.6\">\n")
		TEXT("<node id=\"10\" lat=\"49.00900\" lon=\"8.40400\"/>\n")
		TEXT("<node id=\"11\" lat=\"49.00901\" lon=\"8.40401\"/>\n")
		TEXT("<node id=\"12\" lat=\"49.00899\" lon=\"8.40399\"/>\n")
		TEXT("<node id=\"20\" lat=\"49.01000\" lon=\"8.40500\"/>\n")
		TEXT("<node id=\"30\" lat=\"49.01100\" lon=\"8.40600\"/>\n")
		TEXT("<way id=\"100\">\n")
		TEXT("<nd ref=\"10\"/>\n")
		TEXT("<nd ref=\"20\"/>\n")
		TEXT("<tag k=\"highway\" v=\"primary\"/>\n")
		TEXT("<tag k=\"lanes\" v=\"4\"/>\n")
		TEXT("<tag k=\"width\" v=\"14\"/>\n")
		TEXT("<tag k=\"maxspeed\" v=\"70\"/>\n")
		TEXT("</way>\n")
		TEXT("<way id=\"101\">\n")
		TEXT("<nd ref=\"11\"/>\n")
		TEXT("<nd ref=\"20\"/>\n")
		TEXT("<tag k=\"highway\" v=\"secondary\"/>\n")
		TEXT("<tag k=\"lanes\" v=\"2\"/>\n")
		TEXT("</way>\n")
		TEXT("<way id=\"102\">\n")
		TEXT("<nd ref=\"12\"/>\n")
		TEXT("<nd ref=\"20\"/>\n")
		TEXT("<nd ref=\"30\"/>\n")
		TEXT("<tag k=\"highway\" v=\"primary\"/>\n")
		TEXT("<tag k=\"oneway\" v=\"yes\"/>\n")
		TEXT("<tag k=\"lanes\" v=\"2\"/>\n")
		TEXT("</way>\n")
		TEXT("<way id=\"999\">\n") // Not in the default highway filter -- must be skipped entirely.
		TEXT("<nd ref=\"20\"/>\n")
		TEXT("<nd ref=\"30\"/>\n")
		TEXT("<tag k=\"highway\" v=\"footway\"/>\n")
		TEXT("</way>\n")
		TEXT("<relation id=\"500\">\n")
		TEXT("<member type=\"way\" ref=\"100\" role=\"\"/>\n")
		TEXT("<tag k=\"type\" v=\"route\"/>\n")
		TEXT("</relation>\n")
		TEXT("</osm>\n");
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFlexOsmParseTest, "FlexNetwork.Osm.ParseXml", EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FFlexOsmParseTest::RunTest(const FString& Parameters)
{
	UOsmDataAsset* Asset = NewObject<UOsmDataAsset>(GetTransientPackage());
	TArray<FString> Warnings;
	const bool bParsed = FOsmXmlParser::ParseText(kSyntheticOsmXml, *Asset, &Warnings);

	TestTrue(TEXT("Parsing succeeds"), bParsed);
	TestEqual(TEXT("All 5 nodes parsed"), Asset->Nodes.Num(), 5);
	TestEqual(TEXT("All 4 ways parsed"), Asset->Ways.Num(), 4);
	TestEqual(TEXT("The relation is parsed too"), Asset->Relations.Num(), 1);

	if (const FOsmWay* Way100 = Asset->Ways.Find(100))
	{
		TestEqual(TEXT("Way 100 has 2 node refs"), Way100->NodeRefs.Num(), 2);
		TestEqual(TEXT("Way 100's highway tag"), Way100->Tags.FindRef(TEXT("highway")), FString(TEXT("primary")));
		TestEqual(TEXT("Way 100's lanes tag"), Way100->Tags.FindRef(TEXT("lanes")), FString(TEXT("4")));
	}
	else
	{
		AddError(TEXT("Way 100 missing after parse"));
	}

	if (const FOsmRelation* Relation = Asset->Relations.Find(500))
	{
		TestEqual(TEXT("Relation 500 has 1 member"), Relation->Members.Num(), 1);
		if (Relation->Members.Num() == 1)
		{
			TestTrue(TEXT("Relation member type is Way"), Relation->Members[0].Type == EOsmElementType::Way);
			TestEqual(TEXT("Relation member ref"), Relation->Members[0].Ref, static_cast<int64>(100));
		}
	}
	else
	{
		AddError(TEXT("Relation 500 missing after parse"));
	}

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFlexOsmGraphBuilderTest, "FlexNetwork.Osm.BuildGraph", EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FFlexOsmGraphBuilderTest::RunTest(const FString& Parameters)
{
	if (!GEditor)
	{
		AddError(TEXT("Test requires a running editor context (GEditor)."));
		return false;
	}
	UWorld* World = GEditor->GetEditorWorldContext().World();
	if (!TestNotNull(TEXT("Editor world available"), World))
	{
		return false;
	}
	UFlexNetworkSubsystem* Subsystem = World->GetSubsystem<UFlexNetworkSubsystem>();
	if (!TestNotNull(TEXT("Subsystem available"), Subsystem))
	{
		return false;
	}

	UOsmDataAsset* Asset = NewObject<UOsmDataAsset>(GetTransientPackage());
	TArray<FString> ParseWarnings;
	if (!TestTrue(TEXT("Synthetic XML parses"), FOsmXmlParser::ParseText(kSyntheticOsmXml, *Asset, &ParseWarnings)))
	{
		return false;
	}

	FFlexOsmImportSettings Settings;
	Settings.JunctionMergeRadius = 500.f; // Comfortably covers the ~1-2m spread between nodes 10/11/12.

	// Transient (unsaved) profiles for this test -- exercises the same dedup contract
	// (ResolveProfile called once per distinct signature) that the editor's asset-saving resolver
	// relies on, without touching disk.
	TMap<FString, URoadTypeProfile*> ProfileCache;
	int32 NumResolveCalls = 0;
	const FFlexOsmGraphBuilder::FImportResult Result = FFlexOsmGraphBuilder::BuildFromOsm(*Subsystem, *Asset, Settings,
		[&ProfileCache, &NumResolveCalls](const FFlexOsmGraphBuilder::FLaneSignature& Signature) -> URoadTypeProfile*
		{
			++NumResolveCalls;
			const FString Key = Signature.ToKey();
			if (URoadTypeProfile** Existing = ProfileCache.Find(Key))
			{
				return *Existing;
			}
			URoadTypeProfile* NewProfile = NewObject<URoadTypeProfile>(GetTransientPackage());
			FFlexOsmGraphBuilder::ConfigureProfileFromLaneSignature(*NewProfile, Signature);
			ProfileCache.Add(Key, NewProfile);
			return NewProfile;
		});

	TestEqual(TEXT("3 of the 4 ways match the default highway filter (footway is excluded)"), Result.NumWaysImported, 3);
	TestTrue(TEXT("At least one junction cluster merged (nodes 10/11/12 are within the merge radius)"), Result.NumJunctionsMerged >= 1);
	TestTrue(TEXT("Some FlexNetwork nodes were created"), Result.NumNodesCreated > 0);
	TestTrue(TEXT("Some FlexNetwork segments were created"), Result.NumSegmentsCreated > 0);
	TestEqual(TEXT("3 distinct lane signatures (primary/4lanes, secondary/2lanes, primary-oneway/2lanes)"), Result.NumDistinctLaneSignatures, 3);
	// All 3 ways happen to have distinct signatures here, so this doesn't yet prove dedup kicks in
	// for a *repeated* signature -- it does (BuildFromOsm caches ResolveProfile's result by
	// Signature.ToKey() internally) but this synthetic dataset doesn't exercise that path.
	TestEqual(TEXT("ResolveProfile called once per distinct signature"), NumResolveCalls, 3);

	// Node 20 is shared by all three imported ways (way 102 passes through it, contributing two
	// segments on its own, plus one each from ways 100 and 101) -- confirm a real multi-way
	// junction (3+ segment connections) exists where they meet.
	bool bFoundMultiWayJunction = false;
	for (const TPair<FFlexNodeId, FFlexRoadNode>& Pair : Subsystem->GetAllNodes())
	{
		if (Pair.Value.ConnectedSegments.Num() >= 3)
		{
			bFoundMultiWayJunction = true;
			break;
		}
	}
	TestTrue(TEXT("A multi-way junction exists where the three imported ways meet"), bFoundMultiWayJunction);

	return true;
}

// Sanity-checks the parser + graph builder against a real-world extract (one of the .osm sample
// files this project happens to ship at its root), not just the hand-built synthetic XML above --
// real data has messier tagging (missing lanes/width, varied maxspeed formats, multi-hundred-node
// ways, ...) that's worth exercising even though it isn't part of the plugin itself. Skips
// (doesn't fail) if the file isn't present, since it's project sample data, not plugin content.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFlexOsmRealFileSmokeTest, "FlexNetwork.Osm.RealFileSmokeTest", EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FFlexOsmRealFileSmokeTest::RunTest(const FString& Parameters)
{
	const FString FilePath = FPaths::Combine(FPaths::ProjectDir(), TEXT("Small.osm"));
	if (!FPaths::FileExists(FilePath))
	{
		AddInfo(FString::Printf(TEXT("Skipping: '%s' not present (project-specific sample data, not part of the plugin)."), *FilePath));
		return true;
	}

	if (!GEditor)
	{
		AddError(TEXT("Test requires a running editor context (GEditor)."));
		return false;
	}
	UWorld* World = GEditor->GetEditorWorldContext().World();
	if (!TestNotNull(TEXT("Editor world available"), World))
	{
		return false;
	}
	UFlexNetworkSubsystem* Subsystem = World->GetSubsystem<UFlexNetworkSubsystem>();
	if (!TestNotNull(TEXT("Subsystem available"), Subsystem))
	{
		return false;
	}

	UOsmDataAsset* Asset = NewObject<UOsmDataAsset>(GetTransientPackage());
	TArray<FString> ParseWarnings;
	if (!TestTrue(TEXT("Real .osm file parses"), FOsmXmlParser::ParseFile(FilePath, *Asset, &ParseWarnings)))
	{
		for (const FString& Warning : ParseWarnings)
		{
			AddError(Warning);
		}
		return false;
	}

	AddInfo(FString::Printf(TEXT("Parsed '%s': %d node(s), %d way(s), %d relation(s)."), *FilePath, Asset->Nodes.Num(), Asset->Ways.Num(), Asset->Relations.Num()));
	TestTrue(TEXT("Parsed at least one node"), Asset->Nodes.Num() > 0);
	TestTrue(TEXT("Parsed at least one way"), Asset->Ways.Num() > 0);

	FFlexOsmImportSettings Settings; // defaults: primary/secondary/tertiary/residential, 500cm merge radius

	TMap<FString, URoadTypeProfile*> ProfileCache;
	const FFlexOsmGraphBuilder::FImportResult Result = FFlexOsmGraphBuilder::BuildFromOsm(*Subsystem, *Asset, Settings,
		[&ProfileCache](const FFlexOsmGraphBuilder::FLaneSignature& Signature) -> URoadTypeProfile*
		{
			const FString Key = Signature.ToKey();
			if (URoadTypeProfile** Existing = ProfileCache.Find(Key))
			{
				return *Existing;
			}
			URoadTypeProfile* NewProfile = NewObject<URoadTypeProfile>(GetTransientPackage());
			FFlexOsmGraphBuilder::ConfigureProfileFromLaneSignature(*NewProfile, Signature);
			ProfileCache.Add(Key, NewProfile);
			return NewProfile;
		});

	for (const FString& Warning : Result.Warnings)
	{
		AddWarning(Warning);
	}

	AddInfo(FString::Printf(TEXT("Import result: %d way(s), %d node(s), %d segment(s), %d junction(s) merged, %d distinct lane signature(s)."),
		Result.NumWaysImported, Result.NumNodesCreated, Result.NumSegmentsCreated, Result.NumJunctionsMerged, Result.NumDistinctLaneSignatures));

	if (Result.NumWaysImported > 0)
	{
		TestTrue(TEXT("Importing matched ways produced FlexNetwork nodes"), Result.NumNodesCreated > 0);
		TestTrue(TEXT("Importing matched ways produced FlexNetwork segments"), Result.NumSegmentsCreated > 0);
		TestTrue(TEXT("At least one lane profile was resolved"), Result.NumDistinctLaneSignatures > 0);

		// Every generated node's world-space position should be finite and within a sane range for
		// a city-district-scale extract -- catches a broken projection (e.g. NaN/huge coordinates)
		// without needing to know this specific file's exact extents.
		bool bAllFinite = true;
		bool bAllReasonable = true;
		for (const TPair<FFlexNodeId, FFlexRoadNode>& Pair : Subsystem->GetAllNodes())
		{
			const FVector& Pos = Pair.Value.Position;
			if (!FMath::IsFinite(Pos.X) || !FMath::IsFinite(Pos.Y) || !FMath::IsFinite(Pos.Z))
			{
				bAllFinite = false;
			}
			if (Pos.Size() > 5000.0 * 100.0 * 1000.0) // 5000 km, generously beyond any single city extract
			{
				bAllReasonable = false;
			}
		}
		TestTrue(TEXT("All generated node positions are finite"), bAllFinite);
		TestTrue(TEXT("All generated node positions are within a sane city-scale range"), bAllReasonable);
	}
	else
	{
		AddWarning(TEXT("No ways in this file matched the default highway filter -- projection/lane-derivation logic wasn't exercised by this run."));
	}

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
