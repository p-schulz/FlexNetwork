# FlexNetwork

Interactive, graph-based procedural road/sidewalk/intersection authoring for UE 5.8. A planar
graph of nodes and cubic-Bezier segments is the single source of truth; visible meshes, terrain
conforming, and a queryable lane-connector graph are all *derived* from that graph, so the
external traffic simulation can ask "what lanes exist, where do they lead, what are their
curves" without depending on any rendering code.

## Module split

- **`FlexNetworkRuntime`** -- the graph, curve/geometry math, mesh generation, intersection +
  lane-connector construction, terrain conforming, and the query/mutation API. No editor-only
  dependencies (`UnrealEd`, `Slate`), so it packages into shipping/server builds; a project can
  regenerate roads at runtime, not just in-editor.
- **`FlexNetworkEditor`** -- the interactive drawing/editing tool (a legacy `FEdMode` -- see
  `FlexNetworkEditor.Build.cs` for why, over the modern `UEdMode`/Interactive-Tools-Framework
  stack), ghost-preview rendering, a real world raycast for cursor placement (landscape/mesh
  collision, ortho-aware, with a flat-plane fallback -- see `FFlexNetworkEdMode::TraceCursorToWorld`),
  classic-gizmo node selection/movement (the same `HHitProxy` + transform-widget mechanism
  Landscape/BSP modes use), undo/redo via `FScopedTransaction`, and a minimal toolkit panel.
  Contains no algorithmic logic of its own -- every commit ends by calling into
  `UFlexNetworkRuntime`'s `UFlexNetworkSubsystem`.

FlexNetwork is intentionally independent of this project's other road-related plugin
(`ProceduralRoads`, which imports OSM/OpenDRIVE data) -- different philosophy (interactive
authoring vs. import), no shared code.

## Using the drawing tool

Enable the plugin, open a level, switch to the **Flex Network** editor mode (Modes panel), and
pick an **Active Profile** (a `URoadTypeProfile` data asset -- see `FlexNetwork.CreateDefaultProfiles`
below if you don't have one yet) in the mode's toolkit. The toolkit has a **Draw Mode** toggle
that switches between two gestures, modeled on Landscape Splines rather than a single overloaded
click-drag:

**Draw Mode (default, click-click -- like Landscape Splines' "Add Control Point" tool):**
- Click once to start a road. Move the mouse to preview it (a real raycast against landscape/mesh
  collision, so the preview follows terrain height -- not a flat-plane guess). Click again to
  commit the segment; the new endpoint immediately becomes the start of the next one, so drawing a
  winding street is click-click-click, not one click-drag per segment.
- Clicking on/near an existing node snaps and connects to it; clicking onto an existing road's
  midspan splits it and connects there.
- Drawing a new road across an existing one automatically splits the existing one at the crossing
  point and threads the new road through the resulting junction node (this is how two crossing
  roads become a junction without an explicit "build junction" action).
- The preview curve renders green when valid and red when invalid (too short, tighter than the
  active profile's minimum turn radius, or self-intersecting). Right-click or Escape cancels.

**Select/Move Mode (toggle Draw Mode off):**
- Click an existing node to select it (turns yellow) -- this uses the same hit-proxy click
  mechanism as every other legacy editor mode, not a nearest-point heuristic, so it's exact.
- Drag the viewport's move gizmo to reposition it; connected segments' curves update live, undoable
  as one transaction per drag.
- Click on a segment's curve (a thin line runs along every road specifically for this, faint when
  unselected, bright yellow when selected) to select it instead -- no gizmo for a segment, since it
  has no single position to drag.
- **Delete** deletes whichever is currently selected: a selected node cascades to remove every
  segment still connected to it; a selected segment removes just that segment, leaving both its
  endpoint nodes in place (even if a node ends up with no segments left). Undoable.

Three or more roads meeting at a node -- or two meeting at a sharp angle or a significant width
mismatch -- automatically get a generated junction polygon, trimmed road meshes, crosswalks, and
an invisible lane-connector graph; a node with exactly two roads meeting nearly head-on stays a
smooth pass-through with no junction geometry.

At each junction corner where both flanking roads have a sidewalk, the pavement corner itself now
sweeps through a long, smooth curb-return arc (`UFlexNetworkSettings::CurbReturnRadius`, default
450cm) instead of a sharp point, and the sidewalk continues around the outside of that curve as a
constant-width band instead of stopping abruptly. A landscaped corner island fills the pocket just
beyond the sidewalk, rendered with `URoadTypeProfile::MedianMaterial` (falls back to
`SidewalkMaterial` if unset). Lane connectors through a junction also sweep through a wider turn
than before (bezier handle length raised from ~1/3 to ~0.45 of the connector's chord distance).
Corners where either flanking road has no sidewalk keep the previous sharp-or-small-fillet
behavior (`DefaultFilletRadius`), since there's no sidewalk/island geometry to blend with there.
Each approach's road/sidewalk mesh is trimmed back exactly to its corners' own reach -- no
independent margin is added there, since the road mesh and the junction polygon boundary have to
end at the same point or a gap opens up between them; more visual clearance before a junction has
to come from growing the junction geometry itself (e.g. `CurbReturnRadius`), not from trimming the
road shorter in isolation. Getting the *sidewalk* half of that right takes one extra step: unlike
the drivable polygon (one continuous ring around the whole junction, so the straight run along an
approach's own curb line between its two corners comes for free as a polygon edge), each corner's
sidewalk band is built in isolation and only covers its own arc sweep. `BuildJunctionCornersForRadius`
extends a band's arcs with a straight point on whichever side falls short of that approach's own
final trim distance (only known once every corner touching it has been visited), so the curved
band always reaches out to meet the segment's straight sidewalk exactly, with no gap and no
overlap.

Two further rules keep this safe and sensible everywhere:

- **Near-parallel approaches never get curb-return sidewalks.** Two roads meeting at a junction
  corner narrower than `UFlexNetworkSettings::ParallelApproachAngleToleranceDegrees` (default
  30°) -- e.g. two carriageways of a divided road -- are treated as continuing in essentially the
  same direction rather than turning a corner, and get no sidewalk band/island bridging them.
  Sidewalks only ever appear at genuine corners, at the edges of an intersection, never cutting
  across a gap between roads that are really the same road.
- **The curb-return radius is always geometrically safe.** A single global `CurbReturnRadius`
  can't be right for every junction -- an ordinary, perfectly symmetric 90° 4-way at default
  settings can already self-intersect at the node's center (each corner's fillet circle reaches
  far enough to overlap its neighbors), which no per-corner heuristic can predict, since it
  depends on how many roads share the node, not any one corner's own width or angle. Every
  junction's polygon is validated (`FlexGeometry2D::IsSimplePolygon` -- no two of its edges may
  cross) after being built; if the full radius produces an invalid result, it's bisected down
  until the polygon is valid, falling back to the ordinary sharp/`DefaultFilletRadius` corners
  (radius 0) if nothing smaller works either. Crosswalk placement and lane-connector endpoints
  shift accordingly if a junction's radius ends up auto-shrunk -- that's expected, not a bug.
  That sharp/`DefaultFilletRadius` fallback is capped the same way the curb-return radius is
  (proportional to the corner's own road width and its flanking segments' length): both grow like
  `1/tan(angle/2)` as two adjacent roads approach parallel, so an ungated fallback would just move
  the same unbounded-reach problem there instead of fixing it -- this is what stops an
  unevenly-angled junction (particularly one with a near-parallel pair) from producing a long
  stretched-out polygon spike with part of the junction mesh missing.

## Importing roads from OpenStreetMap

1. **Import a `.osm` file as a `UOsmDataAsset`**: drag a `.osm` XML file (e.g. any of the extracts
   in the project root, or one exported from [openstreetmap.org](https://www.openstreetmap.org) /
   the Overpass API) into the Content Browser, or right-click > Import To. `UOsmDataAssetFactory`
   parses it via `FOsmXmlParser` into a generic asset holding every `<node>`, `<way>`, and
   `<relation>` with their tags -- not road-import-specific, so other systems can read the raw OSM
   data from it too.
2. **Set it as the OSM Asset** in the Flex Network edit mode's toolkit, under the **OSM Import**
   section.
3. Configure (or leave at their defaults):
   - **Highway Tags** -- `highway=<value>` tags to import; defaults to `primary`, `secondary`,
     `tertiary`, `residential`. Add/remove entries for the OSM highway classes you want (e.g. add
     `unclassified` or `trunk`, remove `residential` to skip dense street grids).
   - **Junction Merge Radius** -- world-space distance (cm) within which distinct OSM nodes are
     combined into one FlexNetwork junction node. Real-world OSM data frequently has several
     slightly-offset nodes at what's physically one intersection (survey noise, dual carriageways
     modeled as separate ways, ...); this cleans that up. Only merges nodes created by *this*
     import -- it never reconnects into roads you've already drawn by hand.
   - **Default Lane Width / Default Lane Count / Default Speed Limit Kmh** -- fallbacks used when
     a way is missing the corresponding OSM tag (`width`/`lanes:width`, `lanes`, `maxspeed`).
4. Click **Generate Roads From OSM**. This:
   - Projects each way's nodes from lat/lon to local world-space centimeters (a self-contained
     local tangent-plane projection -- no dependency on the project's GeoReferencing plugin, so the
     generated network can be freely moved/aligned afterward like anything else). The projection's
     origin is the midpoint of the file's `<bounds>` element (its first child, present in any
     Overpass/JOSM export) when the asset has one, falling back to the first referenced node's
     coordinates for files without a `<bounds>` element; `UFlexOsmImportSettings::OriginLatLon`
     (with `bUseOriginOverride`) overrides both.
   - Reads each way's `lanes`, `lanes:forward`/`lanes:backward`, `width`, and `maxspeed` tags to
     derive a lane configuration (how many lanes each direction, how wide, what speed limit).
   - Creates one `URoadTypeProfile` asset per *distinct* lane configuration encountered (not per
     way) under `/FlexNetwork/Profiles/OSM/`, named after that configuration (e.g.
     `DA_OSM_primary_F2_B1_W350`), and assigns it to every way that matches it.
   - Builds the FlexNetwork graph through the normal `UFlexNetworkSubsystem` mutation API (so
     everything downstream -- mesh generation, junction polygons, lane connectors, terrain
     conforming -- just runs as usual), batched into a single incremental rebuild rather than one
     per node/segment (see `UFlexNetworkSubsystem::BeginBatchUpdate`), since a real OSM extract can
     mean hundreds or thousands of segments.
   - Fits a smooth curve through each way's shape points (Catmull-Rom-derived Bezier tangent
     handles) rather than a jagged chain of straight segments between consecutive OSM nodes.
   - Reads each way's `bridge`, `tunnel`, and `layer` tags to place it vertically -- see below.

See `FFlexOsmGraphBuilder::BuildFromOsm` (FlexNetworkRuntime) for the algorithm and
`UFlexNetworkEdModeSettings::GenerateRoadsFromOsm` (FlexNetworkEditor) for how the edit mode
wires a profile-creating resolver into it -- the graph-building logic itself has no editor-only
dependency, so a project could drive the same import at runtime from streamed OSM data using an
in-memory (non-asset-saving) profile resolver instead.

### Bridge / tunnel / layer

Each way's `bridge=*`, `tunnel=*`, and `layer=<n>` tags are read to derive a target height offset
(cm, relative to ground datum) and an `EFlexRoadElevationType` (`Ground`/`Bridge`/`Tunnel`/
`Elevated`):

- An explicit `layer=<n>` is authoritative when present -- its sign already encodes above/below
  ground per OSM convention, so the offset is simply `n * UFlexOsmImportSettings::LayerHeightStep`
  (default 500cm/layer).
- `bridge=yes` (or any non-`no` value) without a `layer` tag lifts the way by `DefaultBridgeHeight`
  (default 500cm); `tunnel=yes` without a `layer` tag sinks it by `DefaultTunnelDepth` (default
  500cm).
- A way with none of these tags stays `Ground` at offset 0.

Real OSM data models a vertical transition (a road climbing onto a bridge, or dropping into a
tunnel) as two separate `<way>` elements meeting at a shared node, with the tag change happening
abruptly right at that node -- there's no gradual "ramp" way in the source data. FlexNetwork
synthesizes one: when a way's target offset differs from the height already established at its
first (shared) node, every *new* node that way creates eases from that entry height to its own
target over `ElevationTransitionLength` (default 1500cm) of the way's own arc length (clamped to
the way's total length, so a short bridge/tunnel approach still always reaches its target by its
last node), using a smoothstep ease so the grade doesn't kink at either end. Nodes/segments still
inside that eased stretch get `EFlexRoadElevationType::Ramp`; a way with no incoming transition
(the very start of an imported chain, or one whose neighbor is already at the same height) is
placed flat with no ramp at all. Ground segments have their terrain flattened to match as usual;
Bridge/Elevated/Tunnel/Ramp segments are left alone (see Terrain section below) so the landscape
doesn't get pulled up to meet a bridge deck.

### Placement

A `placement=<right_of|left_of|middle_of>:<lane>` tag (or `placement:forward`/`placement:backward`,
checked in that order if the plain tag isn't present) means the way wasn't digitized along the
road's true centerline -- OSM mappers often trace one edge of a multi-lane road rather than its
middle. FlexNetwork corrects for this by shifting every node the way creates sideways
(perpendicular to the way's own local direction at that point, using the same 2D "right" convention
as everywhere else in the plugin) by however far off-center the tag says it was traced, so the
generated cross-section actually sits over the real road instead of over wherever OSM's line
happens to run.

OSM numbers lanes 1..N left-to-right across the *whole* way regardless of travel direction, so
`right_of:2` means "traced along the right edge of lane 2" and `left_of:2` means its left edge;
`middle_of:2` its centerline. FlexNetwork's own lane origin (`LateralOffset` 0) sits at the
boundary between the backward lanes (left) and forward lanes (right) -- see
`ConfigureProfileFromLaneSignature` -- so a `left_of`/small-lane-number placement (typically left of
that origin) shifts nodes right to compensate, and a `right_of`/large-lane-number placement shifts
them left; see `ComputeWayPlacementOffset` in `FlexOsmGraphBuilder.cpp` for the exact derivation. A
way with no placement tag at all is assumed already correct and is left untouched. Like the
bridge/tunnel/layer height offset, this only affects nodes a way is *creating* for the first time --
a shared junction node another way already placed is left alone, since which way's correction
should win there is inherently ambiguous.

## Satellite / land-use imagery

FlexNetwork can fetch real aerial photo and (optionally) official land-use imagery from LGL-BW
(Landesamt fuer Geoinformation und Landentwicklung Baden-Wuerttemberg -- German state open geodata,
Baden-Wuerttemberg coverage only, roughly lon 7.2-10.7 / lat 47.4-50.0) and lay it out under an
imported OSM road network for reference, or bake it into permanent, landscape-ready texture/material
assets. Ported from ProceduralRoads' own satellite-imagery module, adapted to FlexNetwork's
conventions (see below) rather than reused as-is.

- Set **OSM Asset**/**OSM Import Settings** in the toolkit (the same fields "Generate Roads From
  OSM" uses) and click **Import Satellite/Landuse Imagery**. This fetches one WMS tile per
  `UFlexSatelliteImagerySettings::TileRadiusM`-square patch covering the extent of the roads
  `OsmImportSettings.HighwayTags` would actually import (`FFlexOsmGraphBuilder::ComputeMatchingRoadExtent`
  -- the *matching ways'* own node extent, not the file's raw `<bounds>` declaration, which a
  typical Overpass export makes deliberately larger than the specific roads a tag filter ends up
  keeping; laying the tile grid out over that whole nominal area instead of where the roads
  actually are is what made tiles look misplaced relative to the imported network) and spawns a
  flat preview quad (`AFlexSatelliteTileActor`) per tile -- runs asynchronously (one or two HTTP
  requests per tile), so check the Output Log for the completion message rather than expecting
  tiles to appear the instant the button returns.
- Click **Bake Imagery To Content** afterward to save every spawned tile's textures as permanent
  `UTexture2D` assets under `UFlexSatelliteImagerySettings::BakePackagePath` (in a subfolder named
  after the OSM asset) -- landscape-appropriate settings (`TC_Default`, `TEXTUREGROUP_World`,
  regenerated mips), separate sRGB toggles for the aerial vs. land-use layer since land-use's flat
  fill colors are really discrete category IDs, not photographic color. Enable
  `bResizeToPowerOfTwo` to bilinearly resize each baked texture to a square `BakeTextureSize`
  (default 2048, always rounded up to the nearest power of two) first -- the WMS-fetched native
  resolution (`2*TileRadiusM/ResolutionM`, clamped by `MaxPixelsPerSide`) is essentially never a
  power of two, which full mip-chain generation and most GPU compression formats want. If
  `UFlexSatelliteImagerySettings::BaseLandscapeMaterial` is set, one `UMaterialInstanceConstant` per
  tile is also created with the baked textures wired up, ready to assign straight to a Landscape's
  Material slot (author that base material to read its texture parameters from a Landscape
  Coords/world-aligned UV node, not a mesh UV channel).
- **Project Settings > Flex Network - Satellite Imagery** (`UFlexSatelliteImagerySettings`) holds
  the WMS service URLs/layer names/styles for both layers, `bFetchLandUseOverlay`, tiling parameters
  (`TileRadiusM`, `ResolutionM` -- native DOP20 resolution is 20cm, requesting finer just upsamples),
  and the preview/baked material references. `PreviewTileMaterial` defaults to ProceduralRoads' own
  `/ProceduralRoads/Material/M_Satellite` (a real content reference, not a FlexNetwork-owned copy --
  `AerialLayer`/`LandUseLayer`'s own `TextureParameterName` defaults, `TileTexture`/`LanduseTexture`,
  are named to match its actual parameters) so imagery previews correctly with zero setup as long as
  the ProceduralRoads plugin is enabled; if it isn't, this default just stays unresolved and
  `PreviewTileMaterial` needs pointing at a material of your own instead.
- **Projection/alignment**: tiles are positioned with the *exact same*
  `FFlexOsmGraphBuilder::ProjectLatLonToLocalCm` formula (and the exact same origin-resolution
  priority -- `FFlexOsmGraphBuilder::ResolveOrigin`, factored out specifically so both call sites
  can't drift apart) that `GenerateRoadsFromOsm` uses for the road network itself, so imagery
  imported from the same OSM asset/import settings lines up with the roads exactly, with no separate
  alignment step. This is the one deliberate change from ProceduralRoads' own version, which projects
  onto the *opposite* world-space axis pair (UE.X=north/UE.Y=east there vs. UE.X=east/UE.Y=north
  here) and defaults to a different origin-resolution rule -- copying that module's math as-is would
  have misaligned FlexNetwork's own roads by a 90-degree rotation.
- The WMS protocol itself only accepts `CRS=EPSG:3857` (Web Mercator) requests -- confirmed
  empirically, `EPSG:4326` requests come back blank -- but this is purely a wire-format detail: every
  response is converted back to lat/lon and then into the same local projection above before it ever
  reaches Unreal world space, so Web Mercator distortion never leaks into tile placement.
- LGL-BW's data is Open Data under "Datenlizenz Deutschland - Namensnennung 2.0", which requires
  attribution ("(c) LGL Baden-Wuerttemberg (www.lgl-bw.de), Datenlizenz Deutschland - Namensnennung
  2.0") wherever the imagery is used/displayed -- the import-complete log message includes this text
  as a reminder, but attaching it to any actual publication/screenshot is on you.

See `Satellite/FlexSatelliteImport.h` (orchestration), `Satellite/FlexSatelliteTileFetcher.h` (WMS
fetch + decode), `Satellite/FlexSatelliteTileActor.h` (preview quad), and
`Satellite/FlexSatelliteImageBaker.h` (asset baking), all in `FlexNetworkEditor`.

## Terrain commands

Ground segments already get the landscape auto-flattened to their height on every edit (see
`IFlexTerrainConformer`/`FFlexLandscapeConformer`). Two buttons in the edit mode's toolkit, under
**Terrain**, drive that relationship explicitly in either direction:

- **Conform Terrain To Roads** -- force-reapplies landscape flattening under every Ground road,
  regardless of what's actually dirty. Useful after hand-sculpting the landscape, or after
  changing terrain-conforming settings/materials, when nothing about the road graph itself
  changed (so the normal per-edit auto-conform wouldn't otherwise re-run).
- **Fit Roads To Terrain** -- the reverse: samples the landscape's existing height under every
  Ground node and snaps the node there, draping the network onto the terrain's actual surface
  instead of flattening the terrain to the network. Bridge/Elevated/Tunnel/Ramp nodes are left
  untouched either way, since their elevation is deliberately offset from ground rather than
  meant to track it.

Both operate on the whole network regardless of the current Select/Move Mode selection (there's no
per-selection scoping for these two) and are undoable as one transaction. Both require an
`ALandscape` in the level; `IFlexTerrainConformer`
is still the extension point for a project using a different terrain system, and now additionally
needs `SampleHeight` implemented (`FFlexLandscapeConformer`'s reads straight from the landscape
heightmap, editor-only like the rest of that class -- see its header for why).

Because Ground segments get the landscape flattened to *exactly* their own height, the generated
road/junction meshes and the landscape underneath them would otherwise sit perfectly coplanar and
z-fight. Every generated mesh is raised `UFlexNetworkSettings::MeshZFightOffset` (default 1cm)
above its own logical height to avoid this -- applied once, as a component-level relative offset
in `AFlexNetworkMeshActor::GetOrCreateComponent`, not baked into any vertex data, so it covers
every section (road, sidewalk, junction surface, crosswalks, corner bands/islands) uniformly with
a single tunable.

## Materials

Setting materials on dozens of auto-generated OSM-import profiles one at a time doesn't scale, so
the toolkit's **Materials** section has a batch alternative: set any of **Standard Road Material**
/ **Standard Sidewalk Material** / **Standard Junction Material** / **Standard Median Material**
and click **Apply Materials To All Profiles**. This overwrites that slot on *every*
`URoadTypeProfile` asset in the project (found via the asset registry, not just ones from the
current session) and saves each modified one to disk; leaving a slot unset leaves that material
untouched on every profile, so e.g. setting only Road Material doesn't disturb per-profile
Sidewalk/Junction/Median choices. See `FlexNetworkAssetUtils::ApplyMaterialsToAllProfiles`.

## Curbstones

Set a **Curbstone Mesh** in the toolkit's **Curbstones** section (author it with its long axis as
local X -- that's the axis `USplineMeshComponent` fits along the curb line) and click
**Generate Curbstones**. This is a manual step, not run automatically on every edit, since it can
place a lot of components for a large network:

- One curbstone line along each side of every road that has a sidewalk, clipped to the same
  trimmed range (`FFlexJunctionData::TrimArcLengthBySegment`) the road's own roadway mesh uses at
  each end, so it stops at the junction boundary instead of continuing across the intersection.
- One closed curbstone loop around every junction's own drivable polygon boundary.

See `AFlexNetworkMeshActor::ApplyCurbstones` (spline-mesh creation/teardown) and
`UFlexNetworkEdModeSettings::GenerateCurbstones` (gathers the curb lines from the live network).

## `URoadTypeProfile` schema

A `URoadTypeProfile` is the data-driven cross-section "recipe" shared by mesh generation, the
lane-connector graph, and validation -- one asset per road type (highway, arterial, residential,
footpath, ...):

| Field | Meaning |
|---|---|
| `Lanes` (`TArray<FRoadLaneDescriptor>`) | Ordered list of lanes/strips. Each has `LateralOffset` (from centerline, +right), `Width`, `Type` (`Vehicle`/`Parking`/`Bike`/`Sidewalk`/`Median`), `Direction` (`Forward`/`Backward`/`Bidirectional`/`None`), `SpeedLimit`. |
| `SidewalkWidth` / `CurbHeight` | Offset-curve sidewalk generated beyond the outermost drivable lane on each side; 0 width disables sidewalks. |
| `MaxGrade`, `MinTurnRadius`, `MinSegmentLengthOverride` | Per-type constraints enforced live by the drawing tool (`UFlexNetworkSubsystem::ValidateProposedSegment`). |
| `RoadMaterial` / `SidewalkMaterial` / `JunctionMaterial` | Applied to the corresponding generated mesh sections. |
| `MedianMaterial` | Applied to landscaped junction corner islands (falls back to `SidewalkMaterial` if unset). |

Global tunables that aren't per-road-type (snap radius, angle-snap increment, default fillet
radius, curb-return radius, arc-length sampling step, terrain falloff, ...) live in
`UFlexNetworkSettings` (`Project Settings > Plugins > Flex Network`), a `UDeveloperSettings` class.

## Querying the network (for the traffic simulation)

Everything a consuming C++ module needs is on `UFlexNetworkSubsystem` (a `UWorldSubsystem` --
fetch it via `World->GetSubsystem<UFlexNetworkSubsystem>()`), with no rendering dependency:

```cpp
UFlexNetworkSubsystem* Network = World->GetSubsystem<UFlexNetworkSubsystem>();

// Position/tangent/right/up at an arc-length distance along a segment's curve -- offset
// laterally by a lane's LateralOffset to get that lane's own position/tangent.
FFlexCurveFrame Frame = Network->SampleSegmentAtArcLength(SegmentId, ArcLength);

// The invisible lane-to-lane Bezier curves through a junction -- what pathfinding follows.
TArray<FFlexLaneConnector> Turns = Network->GetLaneConnectorsAtNode(NodeId);

// Read-only graph access.
const FFlexRoadNode* Node = Network->GetNode(NodeId);
const FFlexRoadSegment* Segment = Network->GetSegment(SegmentId);
```

Subscribe to `Network->OnRoadNetworkChanged` (native `TMulticastDelegate`, or
`OnRoadNetworkChangedBP` from Blueprint) to get the exact set of node/segment IDs touched by each
graph edit, so cached routes can be invalidated incrementally instead of polling or rebuilding
everything.

`FFlexNodeId`/`FFlexSegmentId` are stable, cheap-to-hold handles (index+generation pairs, not raw
pointers) -- safe to cache across frames; `IsValid()`/the owning `TFlexIdAllocator` reject a
handle whose slot has since been freed and reused.

### Extension points

- **`IFlexTerrainConformer`** -- swap in a different terrain integration (the default,
  `FFlexLandscapeConformer`, targets `ALandscape`; a project without Landscape can supply its
  own) via `UFlexNetworkSubsystem::SetTerrainConformer`.
- **`IFlexNetworkExporter`** -- left unimplemented on purpose. The direct query API above is the
  primary integration path; if a ZoneGraph export is wanted later, implement it against this
  interface (e.g. in a new `FlexNetworkZoneGraph` module) and register it via
  `UFlexNetworkSubsystem::RegisterExporter` rather than adding a hard ZoneGraph dependency to the
  runtime module.

## Known limitations / deliberate scope cuts

- **Cross-segment frame continuity at bends**: each segment's rotation-minimizing frame is seeded
  independently at its own start; two segments sharing a bend node with aligned tangents match
  exactly there, but a segment's *far* end can drift slightly from a fresh seed on a strongly
  curved segment. See the comment in `FlexRotationMinimizingFrame.cpp`.
- **Landscape terrain conforming edits the active heightmap directly** (with save/restore for
  reversibility) rather than through a dedicated non-destructive Edit Layer, because this
  engine build's edit-layer identity API (`FLandscapeLayer::Guid`/`Name`) is marked `_DEPRECATED`
  in favor of a newer system that couldn't be safely guessed without live-editor testing. See
  `FlexLandscapeConformer.h`.
- **Junction trim distances** are derived from the junction polygon's corner points projected
  onto each approach's outward direction, not an exact curve/polygon intersection -- exact for
  straight roads, a close approximation for curved ones near the node.
- **No tangent-handle editing yet**: Select/Move mode only lets you drag a node's position, not
  the Bezier tangent handles authored implicitly by the drag direction when the segment was drawn.
  Redrawing through a node (Draw Mode) is the current way to reshape a curve.
- **Node elevation/type editing has no dedicated UI yet**: `UFlexNetworkSubsystem::SetNodeElevationType`
  exists and works, but the toolkit doesn't expose a picker for a *selected* node's elevation the
  way `ActiveElevationType` picks it for *new* nodes. Drive it via console/Python for now if needed.
- **OSM import isn't idempotent**: re-running "Generate Roads From OSM" on the same asset creates
  a second, overlapping copy of the network -- there's no tracking of which OSM node/way IDs were
  already imported. If you need to tweak import settings, undo the previous import first.
- **OSM import always imports flat** (all generated nodes at Z=0, `EFlexRoadElevationType::Ground`)
  -- OSM node elevation (`ele` tag) isn't read. Use Select/Move mode's gizmo afterward for roads
  that need to follow terrain height, or conform terrain to the flat network instead.
- **`<relation>` elements are parsed into `UOsmDataAsset` but not consumed** by the road importer
  -- per the project's request, they're stored for potential future use (turn restrictions, bus
  routes, multipolygons) but `FFlexOsmGraphBuilder` only reads `<way>` data today.

## Testing

- Automation tests (`FlexNetwork.Math.*`, `FlexNetwork.Pipeline.*`) run headless via the UE
  automation framework; the pipeline test drives `UFlexNetworkSubsystem` through the same
  scenario (straight road, curved road, 3-way junction, 4-way junction with mismatched widths,
  elevated ramp, crossing-road auto-split) the editor tool would produce, since this environment
  has no way to drive the editor viewport's mouse input directly.
- To manually verify the actual click-click drawing and gizmo node-move UX: open the editor,
  enable the Flex Network mode, and reproduce that same scenario by hand -- watch for the
  incremental-rebuild log line (`LogTemp: Verbose: FlexNetwork: incremental rebuild touched N
  node(s), M segment(s).`) staying small after a single-segment edit instead of touching the
  whole network.
