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
- **`FlexNetworkEditor`** -- the interactive drag-to-draw tool (a legacy `FEdMode` -- see
  `FlexNetworkEditor.Build.cs` for why, over the modern `UEdMode`/Interactive-Tools-Framework
  stack), ghost-preview rendering, undo/redo via `FScopedTransaction`, and a minimal toolkit
  panel. Contains no algorithmic logic of its own -- every drag ends by calling into
  `UFlexNetworkRuntime`'s `UFlexNetworkSubsystem`.

FlexNetwork is intentionally independent of this project's other road-related plugin
(`ProceduralRoads`, which imports OSM/OpenDRIVE data) -- different philosophy (interactive
authoring vs. import), no shared code.

## Using the drawing tool

Enable the plugin, open a level, switch to the **Flex Network** editor mode (Modes panel), pick
an **Active Profile** (a `URoadTypeProfile` data asset) in the mode's toolkit, then click-drag in
the viewport on the ground plane to draw a road:

- Dragging from/onto an existing node snaps and connects to it.
- Dragging onto an existing road's midspan splits it and connects there.
- Dragging a new road across an existing one automatically splits the existing one at the
  crossing point and threads the new road through the resulting junction node (this is how two
  crossing roads become a junction without an explicit "build junction" action).
- The preview curve renders green when valid and red when invalid (too short, tighter than the
  active profile's minimum turn radius, or self-intersecting).
- Click an existing node (without dragging) to select it (yellow).

Three or more roads meeting at a node -- or two meeting at a sharp angle or a significant width
mismatch -- automatically get a generated junction polygon, trimmed road meshes, crosswalks, and
an invisible lane-connector graph; a node with exactly two roads meeting nearly head-on stays a
smooth pass-through with no junction geometry.

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

Global tunables that aren't per-road-type (snap radius, angle-snap increment, default fillet
radius, arc-length sampling step, terrain falloff, ...) live in `UFlexNetworkSettings`
(`Project Settings > Plugins > Flex Network`), a `UDeveloperSettings` class.

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

## Testing

- Automation tests (`FlexNetwork.Math.*`, `FlexNetwork.Pipeline.*`) run headless via the UE
  automation framework; the pipeline test drives `UFlexNetworkSubsystem` through the same
  scenario (straight road, curved road, 3-way junction, 4-way junction with mismatched widths,
  elevated ramp, crossing-road auto-split) the editor tool would produce, since this environment
  has no way to drive the editor viewport's mouse input directly.
- To manually verify the actual click-drag UX: open the editor, enable the Flex Network mode, and
  reproduce that same scenario by hand -- watch for the incremental-rebuild log line
  (`LogTemp: Verbose: FlexNetwork: incremental rebuild touched N node(s), M segment(s).`) staying
  small after a single-segment edit instead of touching the whole network.
