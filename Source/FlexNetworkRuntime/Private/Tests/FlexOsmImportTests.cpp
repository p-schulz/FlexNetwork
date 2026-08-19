#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Osm/OsmDataAsset.h"
#include "Osm/OsmXmlParser.h"
#include "Osm/FlexOsmGraphBuilder.h"
#include "FlexNetworkSubsystem.h"
#include "RoadTypeProfile.h"
#include "Mesh/FlexMeshSectionData.h"
#include "Editor.h"
#include "Misc/ScopeExit.h"

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
		TEXT("<tag k=\"placement\" v=\"middle_of:1\"/>\n")
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

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFlexOsmSharedOriginTest, "FlexNetwork.Osm.SharedDeterministicOrigin", EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FFlexOsmSharedOriginTest::RunTest(const FString& Parameters)
{
	UOsmDataAsset* Asset = NewObject<UOsmDataAsset>(GetTransientPackage());

	FOsmNode UnrelatedNode;
	UnrelatedNode.Latitude = 1.0;
	UnrelatedNode.Longitude = 2.0;
	Asset->Nodes.Add(1, UnrelatedNode);
	FOsmNode FirstRoadNode;
	FirstRoadNode.Latitude = 49.0;
	FirstRoadNode.Longitude = 8.0;
	Asset->Nodes.Add(10, FirstRoadNode);
	FOsmNode LaterRoadNode;
	LaterRoadNode.Latitude = 49.1;
	LaterRoadNode.Longitude = 8.1;
	Asset->Nodes.Add(20, LaterRoadNode);

	// Insert in the opposite order to the IDs: origin selection must not depend on TMap order.
	FOsmWay LaterWay;
	LaterWay.NodeRefs = { 20 };
	LaterWay.Tags.Add(TEXT("highway"), TEXT("primary"));
	Asset->Ways.Add(200, LaterWay);
	FOsmWay FirstWay;
	FirstWay.NodeRefs = { 10 };
	FirstWay.Tags.Add(TEXT("highway"), TEXT("primary"));
	Asset->Ways.Add(100, FirstWay);

	FFlexOsmImportSettings Settings;
	double OriginLat = 0.0;
	double OriginLon = 0.0;
	TestTrue(TEXT("Shared origin resolves"), FFlexOsmGraphBuilder::ResolveOrigin(*Asset, Settings, OriginLat, OriginLon));
	TestEqual(TEXT("Lowest-ID matching way supplies latitude"), OriginLat, FirstRoadNode.Latitude);
	TestEqual(TEXT("Lowest-ID matching way supplies longitude"), OriginLon, FirstRoadNode.Longitude);

	FVector2D MinLocal;
	FVector2D MaxLocal;
	double ExtentOriginLat = 0.0;
	double ExtentOriginLon = 0.0;
	TestTrue(TEXT("Matching-road extent resolves"), FFlexOsmGraphBuilder::ComputeMatchingRoadExtent(
		*Asset, Settings, ExtentOriginLat, ExtentOriginLon, MinLocal, MaxLocal));
	TestEqual(TEXT("Extent and geometry use identical latitude origin"), ExtentOriginLat, OriginLat);
	TestEqual(TEXT("Extent and geometry use identical longitude origin"), ExtentOriginLon, OriginLon);
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

	URoadTypeProfile* OneWayPlacementProfile = nullptr;
	for (const TPair<FString, URoadTypeProfile*>& Pair : ProfileCache)
	{
		if (Pair.Value && Pair.Value->Lanes.Num() == 2
			&& Pair.Value->Lanes[0].Direction == EFlexLaneDirection::Forward
			&& Pair.Value->Lanes[1].Direction == EFlexLaneDirection::Forward)
		{
			OneWayPlacementProfile = Pair.Value;
			break;
		}
	}
	if (TestNotNull(TEXT("One-way placement profile was generated"), OneWayPlacementProfile))
	{
		TestEqual(TEXT("One-way roadway keeps its tagged 7m physical width"), OneWayPlacementProfile->GetRoadwayWidth(), 700.f);
		TestEqual(TEXT("middle_of:1 places the first curb 1.75m left of the OSM line"), OneWayPlacementProfile->GetRoadwayMinOffset(), -175.f);
		TestEqual(TEXT("middle_of:1 places the opposite curb 5.25m right of the OSM line"), OneWayPlacementProfile->GetRoadwayMaxOffset(), 525.f);
	}

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

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFlexOsmComplexIntersectionCollapseTest,
	"FlexNetwork.Osm.CollapsesComplexIntersectionInterior",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FFlexOsmComplexIntersectionCollapseTest::RunTest(const FString& Parameters)
{
	if (!GEditor)
	{
		AddError(TEXT("Test requires a running editor context (GEditor)."));
		return false;
	}
	UWorld* World = GEditor->GetEditorWorldContext().World();
	UFlexNetworkSubsystem* Subsystem = World ? World->GetSubsystem<UFlexNetworkSubsystem>() : nullptr;
	if (!TestNotNull(TEXT("Subsystem available"), Subsystem))
	{
		return false;
	}
	Subsystem->ClearNetwork();
	ON_SCOPE_EXIT { Subsystem->ClearNetwork(); };

	UOsmDataAsset* Asset = NewObject<UOsmDataAsset>(GetTransientPackage());
	auto AddNode = [Asset](int64 Id, double Lat, double Lon)
	{
		FOsmNode Node;
		Node.Latitude = Lat;
		Node.Longitude = Lon;
		Asset->Nodes.Add(Id, Node);
	};
	auto AddRoad = [Asset](int64 Id, int64 A, int64 B)
	{
		FOsmWay Way;
		Way.NodeRefs = { A, B };
		Way.Tags.Add(TEXT("highway"), TEXT("residential"));
		Way.Tags.Add(TEXT("lanes"), TEXT("2"));
		Asset->Ways.Add(Id, MoveTemp(Way));
	};

	// Four nearby degree-4 junctions surrounding one mapped intersection interior, with two
	// external approaches at each corner. This is the same topological pattern as Block_Gerwig.
	AddNode(1, 49.00000, 8.00000); AddNode(2, 49.00000, 8.00015);
	AddNode(3, 49.00010, 8.00015); AddNode(4, 49.00010, 8.00000);
	AddNode(51, 49.00000, 8.000075); AddNode(52, 49.00005, 8.00015);
	AddNode(53, 49.00010, 8.000075); AddNode(54, 49.00005, 8.00000);
	AddNode(11, 49.00000, 7.99975); AddNode(12, 48.99980, 8.00000);
	AddNode(21, 49.00000, 8.00040); AddNode(22, 48.99980, 8.00015);
	AddNode(31, 49.00010, 8.00040); AddNode(32, 49.00030, 8.00015);
	AddNode(41, 49.00010, 7.99975); AddNode(42, 49.00030, 8.00000);
	AddRoad(100, 1, 51); AddRoad(104, 51, 2);
	AddRoad(101, 2, 52); AddRoad(105, 52, 3);
	AddRoad(102, 3, 53); AddRoad(106, 53, 4);
	AddRoad(103, 4, 54); AddRoad(107, 54, 1);
	AddRoad(110, 1, 11); AddRoad(111, 1, 12); AddRoad(120, 2, 21); AddRoad(121, 2, 22);
	AddRoad(130, 3, 31); AddRoad(131, 3, 32); AddRoad(140, 4, 41); AddRoad(141, 4, 42);

	FFlexOsmImportSettings Settings;
	Settings.JunctionMergeRadius = 0.f;
	Settings.ComplexIntersectionInternalEdgeLength = 2000.f;
	Settings.ComplexIntersectionMaxDiameter = 5000.f;
	TMap<FString, URoadTypeProfile*> Profiles;
	const FFlexOsmGraphBuilder::FImportResult Result = FFlexOsmGraphBuilder::BuildFromOsm(*Subsystem, *Asset, Settings,
		[&Profiles](const FFlexOsmGraphBuilder::FLaneSignature& Signature)
		{
			URoadTypeProfile*& Profile = Profiles.FindOrAdd(Signature.ToKey());
			if (!Profile)
			{
				Profile = NewObject<URoadTypeProfile>(GetTransientPackage());
				FFlexOsmGraphBuilder::ConfigureProfileFromLaneSignature(*Profile, Signature);
			}
			return Profile;
		});

	TestEqual(TEXT("The short four-junction interior is detected as one region"), Result.NumComplexIntersectionsCollapsed, 1);
	TArray<FFlexNodeId> RegionNodes;
	int32 RegionIndex = INDEX_NONE;
	for (const TPair<FFlexNodeId, FFlexRoadNode>& Pair : Subsystem->GetAllNodes())
	{
		if (Pair.Value.ComplexIntersectionRegionIndex != INDEX_NONE)
		{
			RegionNodes.Add(Pair.Key);
			RegionIndex = Pair.Value.ComplexIntersectionRegionIndex;
			TestEqual(TEXT("Each retained portal remains a four-way routing node"), Pair.Value.ConnectedSegments.Num(), 4);
		}
	}
	TestEqual(TEXT("All four original junction portals are retained"), RegionNodes.Num(), 4);
	bool bAllPortalsShareRegion = RegionIndex != INDEX_NONE;
	for (const FFlexNodeId NodeId : RegionNodes)
	{
		const FFlexRoadNode* Node = Subsystem->GetNode(NodeId);
		bAllPortalsShareRegion &= Node && Node->ComplexIntersectionRegionIndex == RegionIndex;
	}
	TestTrue(TEXT("The retained portals share a complex-region index"), bAllPortalsShareRegion);
	TestEqual(TEXT("Interior degree-2 shape nodes are removed, leaving eight external and four portal nodes"),
		Subsystem->GetAllNodes().Num(), 12);
	TestEqual(TEXT("Eight approaches and four routing-only internal links remain"), Subsystem->GetAllSegments().Num(), 12);
	TestEqual(TEXT("Import counters preserve the navigable OSM graph"), Result.NumSegmentsCreated, 12);

	int32 NumInternalRoutingLinks = 0;
	for (const TPair<FFlexSegmentId, FFlexRoadSegment>& Pair : Subsystem->GetAllSegments())
	{
		const FFlexRoadNode* Start = Subsystem->GetNode(Pair.Value.StartNodeId);
		const FFlexRoadNode* End = Subsystem->GetNode(Pair.Value.EndNodeId);
		if (Start && End && Start->ComplexIntersectionRegionIndex == RegionIndex
			&& End->ComplexIntersectionRegionIndex == RegionIndex)
		{
			++NumInternalRoutingLinks;
			FFlexSegmentMeshResult InternalMesh;
			TestFalse(TEXT("An internal routing link emits no independent road/sidewalk mesh"),
				Subsystem->BuildSegmentMeshResult(Pair.Key, InternalMesh));
		}
	}
	TestEqual(TEXT("The four internal links remain routing-only"), NumInternalRoutingLinks, 4);

	int32 NumRegionSurfaces = 0;
	bool bFoundTurningConnector = false;
	for (const FFlexNodeId RegionNodeId : RegionNodes)
	{
		const FFlexJunctionData* Junction = Subsystem->GetJunctionData(RegionNodeId);
		if (TestNotNull(TEXT("The retained portal builds junction data"), Junction))
		{
			TestTrue(TEXT("Each portal generates local lane connectors"), Junction->LaneConnectors.Num() > 0);
			bFoundTurningConnector |= Junction->LaneConnectors.ContainsByPredicate([](const FFlexLaneConnector& Connector)
			{
				return Connector.TurnAngleDegrees > 30.f;
			});
		}
		FFlexJunctionMeshResult Mesh;
		if (Subsystem->BuildJunctionMeshResult(RegionNodeId, Mesh) && !Mesh.Surface.IsEmpty())
		{
			++NumRegionSurfaces;
		}
	}
	TestTrue(TEXT("The retained routing graph includes vehicle turns"), bFoundTurningConnector);
	TestEqual(TEXT("Exactly one portal owner emits the shared filled region surface"), NumRegionSurfaces, 1);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFlexOsmRailwayImportTest,
	"FlexNetwork.Osm.ImportsTrainAndTramRails",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FFlexOsmRailwayImportTest::RunTest(const FString& Parameters)
{
	if (!GEditor)
	{
		AddError(TEXT("Test requires a running editor context (GEditor)."));
		return false;
	}
	UWorld* World = GEditor->GetEditorWorldContext().World();
	UFlexNetworkSubsystem* Subsystem = World ? World->GetSubsystem<UFlexNetworkSubsystem>() : nullptr;
	if (!TestNotNull(TEXT("Subsystem available"), Subsystem))
	{
		return false;
	}
	Subsystem->ClearNetwork();
	ON_SCOPE_EXIT { Subsystem->ClearNetwork(); };

	UOsmDataAsset* Asset = NewObject<UOsmDataAsset>(GetTransientPackage());
	auto AddNode = [Asset](int64 Id, double Lat, double Lon)
	{
		FOsmNode Node;
		Node.Latitude = Lat;
		Node.Longitude = Lon;
		Asset->Nodes.Add(Id, Node);
	};
	AddNode(1, 49.00000, 8.00000); AddNode(2, 49.00010, 8.00000);
	AddNode(3, 49.00000, 8.00001); AddNode(4, 49.00010, 8.00001);

	FOsmWay TrainWay;
	TrainWay.NodeRefs = { 1, 2 };
	TrainWay.Tags.Add(TEXT("railway"), TEXT("rail"));
	TrainWay.Tags.Add(TEXT("tracks"), TEXT("2"));
	TrainWay.Tags.Add(TEXT("gauge"), TEXT("1435"));
	Asset->Ways.Add(200, MoveTemp(TrainWay));

	FOsmWay TramWay;
	TramWay.NodeRefs = { 3, 4 };
	TramWay.Tags.Add(TEXT("railway"), TEXT("tram"));
	TramWay.Tags.Add(TEXT("gauge"), TEXT("1000"));
	Asset->Ways.Add(201, MoveTemp(TramWay));

	FFlexOsmImportSettings Settings;
	Settings.JunctionMergeRadius = 500.f; // The parallel ways are <1m apart and must remain distinct.
	TMap<FString, URoadTypeProfile*> Profiles;
	const FFlexOsmGraphBuilder::FImportResult Result = FFlexOsmGraphBuilder::BuildFromOsm(*Subsystem, *Asset, Settings,
		[&Profiles](const FFlexOsmGraphBuilder::FLaneSignature& Signature)
		{
			URoadTypeProfile*& Profile = Profiles.FindOrAdd(Signature.ToKey());
			if (!Profile)
			{
				Profile = NewObject<URoadTypeProfile>(GetTransientPackage());
				FFlexOsmGraphBuilder::ConfigureProfileFromLaneSignature(*Profile, Signature);
			}
			return Profile;
		});

	TestEqual(TEXT("Both railway ways are imported"), Result.NumRailwayWaysImported, 2);
	TestEqual(TEXT("One segment is generated per two-node railway way"), Subsystem->GetAllSegments().Num(), 2);
	TestEqual(TEXT("Nearby parallel railway nodes are not proximity-merged"), Subsystem->GetAllNodes().Num(), 4);
	TestEqual(TEXT("No road proximity junction is reported for parallel tracks"), Result.NumJunctionsMerged, 0);
	TestEqual(TEXT("Train and tram configurations produce distinct profiles"), Profiles.Num(), 2);

	bool bFoundStandardGaugeTwoTrack = false;
	bool bFoundMeterGaugeTram = false;
	for (const TPair<FString, URoadTypeProfile*>& Pair : Profiles)
	{
		const URoadTypeProfile* Profile = Pair.Value;
		if (!TestNotNull(TEXT("Generated railway profile"), Profile))
		{
			continue;
		}
		TestTrue(TEXT("Generated railway profile is marked as rail"), Profile->bIsRailProfile);
		TestEqual(TEXT("Railway profile has no sidewalks"), Profile->SidewalkWidth, 0.f);
		TestEqual(TEXT("Railway profile has no curb"), Profile->CurbHeight, 0.f);
		const bool bIsStandardGaugeTrain = FMath::IsNearlyEqual(Profile->RailGauge, 143.5f) && Profile->Lanes.Num() == 2;
		const bool bIsMeterGaugeTram = FMath::IsNearlyEqual(Profile->RailGauge, 100.f) && Profile->Lanes.Num() == 1;
		bFoundStandardGaugeTwoTrack |= bIsStandardGaugeTrain;
		bFoundMeterGaugeTram |= bIsMeterGaugeTram;
		if (bIsStandardGaugeTrain)
		{
			TestFalse(TEXT("Ordinary railway keeps the ungrooved solid profile"), Profile->bUseGroovedRailProfile);
		}
		if (bIsMeterGaugeTram)
		{
			TestTrue(TEXT("railway=tram enables the boolean groove profile"), Profile->bUseGroovedRailProfile);
			TestEqual(TEXT("Default tram rail base follows the 156 mm reference"), Profile->RailWidth, 15.6f);
			TestEqual(TEXT("Default tram rail crown follows the 115 mm reference"), Profile->RailTopWidth, 11.5f);
			TestEqual(TEXT("Default tram rail height follows the 72 mm reference"), Profile->RailHeight, 7.2f);
		}
		for (const FRoadLaneDescriptor& Lane : Profile->Lanes)
		{
			TestTrue(TEXT("Every railway profile lane has Rail type"), Lane.Type == EFlexLaneType::Rail);
		}
	}
	TestTrue(TEXT("OSM tracks=2 and gauge=1435 are applied"), bFoundStandardGaugeTwoTrack);
	TestTrue(TEXT("OSM tram gauge=1000 is converted from mm to cm"), bFoundMeterGaugeTram);

	for (const TPair<FFlexSegmentId, FFlexRoadSegment>& Pair : Subsystem->GetAllSegments())
	{
		FFlexSegmentMeshResult Mesh;
		TestTrue(TEXT("Rail segment produces a mesh"), Subsystem->BuildSegmentMeshResult(Pair.Key, Mesh));
		TestFalse(TEXT("Rail segment mesh is not a roadway slab"), Mesh.Roadway.IsEmpty());
		TestTrue(TEXT("Rail segment mesh has no sidewalk section"), Mesh.Sidewalks.IsEmpty());
	}
	TArray<FFlexMeshSectionData> UnifiedRailMeshes;
	Subsystem->BuildRailMeshResults(UnifiedRailMeshes);
	TestEqual(TEXT("The two different rail profiles produce two unified mesh sections"), UnifiedRailMeshes.Num(), 2);
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

	AddInfo(FString::Printf(TEXT("Import result: %d way(s), including %d railway way(s), %d node(s), %d segment(s), %d proximity junction(s) merged, %d complex intersection surface region(s), %d distinct lane signature(s)."),
		Result.NumWaysImported, Result.NumRailwayWaysImported, Result.NumNodesCreated, Result.NumSegmentsCreated, Result.NumJunctionsMerged,
		Result.NumComplexIntersectionsCollapsed, Result.NumDistinctLaneSignatures));

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
