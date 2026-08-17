#include "Osm/OsmXmlParser.h"
#include "Osm/OsmDataAsset.h"
#include "XmlFile.h"
#include "XmlNode.h"

namespace
{
	void ParseTags(const FXmlNode* ElementNode, TMap<FString, FString>& OutTags)
	{
		for (const FXmlNode* Child : ElementNode->GetChildrenNodes())
		{
			if (Child->GetTag() == TEXT("tag"))
			{
				const FString Key = Child->GetAttribute(TEXT("k"));
				if (!Key.IsEmpty())
				{
					OutTags.Add(Key, Child->GetAttribute(TEXT("v")));
				}
			}
		}
	}

	void ParseBoundsElement(const FXmlNode* BoundsElement, UOsmDataAsset& OutAsset)
	{
		FOsmBounds Bounds;
		Bounds.MinLat = FCString::Atod(*BoundsElement->GetAttribute(TEXT("minlat")));
		Bounds.MinLon = FCString::Atod(*BoundsElement->GetAttribute(TEXT("minlon")));
		Bounds.MaxLat = FCString::Atod(*BoundsElement->GetAttribute(TEXT("maxlat")));
		Bounds.MaxLon = FCString::Atod(*BoundsElement->GetAttribute(TEXT("maxlon")));
		Bounds.bIsValid = true;
		OutAsset.Bounds = Bounds;
	}

	void ParseNodeElement(const FXmlNode* NodeElement, UOsmDataAsset& OutAsset)
	{
		const int64 Id = FCString::Atoi64(*NodeElement->GetAttribute(TEXT("id")));
		FOsmNode Node;
		Node.Latitude = FCString::Atod(*NodeElement->GetAttribute(TEXT("lat")));
		Node.Longitude = FCString::Atod(*NodeElement->GetAttribute(TEXT("lon")));
		ParseTags(NodeElement, Node.Tags);
		OutAsset.Nodes.Add(Id, MoveTemp(Node));
	}

	void ParseWayElement(const FXmlNode* WayElement, UOsmDataAsset& OutAsset)
	{
		const int64 Id = FCString::Atoi64(*WayElement->GetAttribute(TEXT("id")));
		FOsmWay Way;
		for (const FXmlNode* Child : WayElement->GetChildrenNodes())
		{
			if (Child->GetTag() == TEXT("nd"))
			{
				const FString RefStr = Child->GetAttribute(TEXT("ref"));
				if (!RefStr.IsEmpty())
				{
					Way.NodeRefs.Add(FCString::Atoi64(*RefStr));
				}
			}
		}
		ParseTags(WayElement, Way.Tags);
		OutAsset.Ways.Add(Id, MoveTemp(Way));
	}

	void ParseRelationElement(const FXmlNode* RelationElement, UOsmDataAsset& OutAsset)
	{
		const int64 Id = FCString::Atoi64(*RelationElement->GetAttribute(TEXT("id")));
		FOsmRelation Relation;
		for (const FXmlNode* Child : RelationElement->GetChildrenNodes())
		{
			if (Child->GetTag() == TEXT("member"))
			{
				FOsmRelationMember Member;
				const FString TypeStr = Child->GetAttribute(TEXT("type"));
				if (TypeStr == TEXT("way"))
				{
					Member.Type = EOsmElementType::Way;
				}
				else if (TypeStr == TEXT("relation"))
				{
					Member.Type = EOsmElementType::Relation;
				}
				else
				{
					Member.Type = EOsmElementType::Node;
				}
				Member.Ref = FCString::Atoi64(*Child->GetAttribute(TEXT("ref")));
				Member.Role = Child->GetAttribute(TEXT("role"));
				Relation.Members.Add(Member);
			}
		}
		ParseTags(RelationElement, Relation.Tags);
		OutAsset.Relations.Add(Id, MoveTemp(Relation));
	}

	bool ParseRoot(const FXmlNode* RootNode, UOsmDataAsset& OutAsset, TArray<FString>* OutWarnings)
	{
		if (!RootNode)
		{
			if (OutWarnings)
			{
				OutWarnings->Add(TEXT("OSM XML has no root element."));
			}
			return false;
		}

		OutAsset.Bounds = FOsmBounds();
		OutAsset.Nodes.Reset();
		OutAsset.Ways.Reset();
		OutAsset.Relations.Reset();

		for (const FXmlNode* Child : RootNode->GetChildrenNodes())
		{
			const FString& Tag = Child->GetTag();
			if (Tag == TEXT("bounds"))
			{
				ParseBoundsElement(Child, OutAsset);
			}
			else if (Tag == TEXT("node"))
			{
				ParseNodeElement(Child, OutAsset);
			}
			else if (Tag == TEXT("way"))
			{
				ParseWayElement(Child, OutAsset);
			}
			else if (Tag == TEXT("relation"))
			{
				ParseRelationElement(Child, OutAsset);
			}
		}

		return true;
	}
}

bool FOsmXmlParser::ParseFile(const FString& Path, UOsmDataAsset& OutAsset, TArray<FString>* OutWarnings)
{
	FXmlFile XmlFile;
	if (!XmlFile.LoadFile(Path, EConstructMethod::ConstructFromFile))
	{
		if (OutWarnings)
		{
			OutWarnings->Add(FString::Printf(TEXT("Failed to load/parse '%s': %s"), *Path, *XmlFile.GetLastError()));
		}
		return false;
	}

	const bool bSuccess = ParseRoot(XmlFile.GetRootNode(), OutAsset, OutWarnings);
	if (bSuccess)
	{
		OutAsset.SourceFilePath = Path;
	}
	return bSuccess;
}

bool FOsmXmlParser::ParseText(const FString& XmlText, UOsmDataAsset& OutAsset, TArray<FString>* OutWarnings)
{
	FXmlFile XmlFile;
	if (!XmlFile.LoadFile(XmlText, EConstructMethod::ConstructFromBuffer))
	{
		if (OutWarnings)
		{
			OutWarnings->Add(FString::Printf(TEXT("Failed to parse OSM XML text: %s"), *XmlFile.GetLastError()));
		}
		return false;
	}

	return ParseRoot(XmlFile.GetRootNode(), OutAsset, OutWarnings);
}
