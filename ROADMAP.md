# Bitruvius Roadmap

Tracks implemented milestones and upcoming development phases for the Bitruvius rigging engine, workspace studio, and Export IR pipeline.

## Completed Milestones

### V1.0 — Canonical Manual Stage & 43-Joint Closed-Chain Rig
- Endless procedural paper & back-cover SVG stage with symmetrical borders, responsive rulers, 8-head bands, and Vitruvian circle/square canons (`1.00 PYX = 100 PX`).
- Full 43-joint procedural mannequin with interactive IK/FK handles, ground/floor snapping, world pins, soft joint limits, and multi-joint physics modes (`rig`/`rigid`).

### V1.1 — Non-Elastic Collar & Hip Toggle Hinges
- Replaced elastic stretch on collar (`braceR`/`braceL`, `clavOuterR`/`clavOuterL`) and hip (`hipR`/`hipL`) links with fixed-length 2-link toggle triangles matching the full canonical reach cleanly.

### V1.2 — Per-Arm Reverse Swivel & Live Pose Interpolation
- Added dedicated per-arm reverse swivel controls (`armSwivelR`, `armSwivelL`) with drag + quick toggle for clean shoulder/elbow inversions between poses.
- Added smooth time-based pose interpolation (`computePoseTransitionFrame`) so presets, mirrors, resets, hinge flips, and swivels animate smoothly on canvas.

### V1.3 — Versioned Export IR & Deterministic Format Adapters
- Defined versioned Intermediate Representation schema (`src/types/exportIR.ts`, schema `1.0.0`) capturing bone hierarchy, rigid lengths in PYX, world/local transforms, and interpolated timeline keyframes.
- Implemented pure deterministic adapters (`src/utils/exportAdapters.ts`) for Canonical JSON, Layered SVG, Spine JSON, and DragonBones JSON, covered by unit tests.

### V1.4 — Modular Workspace, Performance Audit & VSC Readiness *(Current)*
- Replaced invasive fixed side drawer and stacked bottom strips with a minimal system-only top bar, a slim right vertical quick-toggle + launcher rail, stackable/detachable edge-snapping model modules, and a single collapsible bottom animation drawer (`Timeline`, `Pose Snapshots`, `Animation & Export`).
- Audited rendering hot paths: stabilized `ResizeObserver` callbacks, memoized heavy SVG layers (`BookStageCanvas`, `DigitalMannequin`, `ScanBorderLayer`), hoisted static guide geometry, and removed obsolete duplicate UI components.
- Added `.vscode/` config, `npm test` script, updated `.gitignore`, `README.md`, and `ROADMAP.md` for smooth VS Code + Git onboarding.

## Priority Path — Shared-Rig Simple & Advanced Masks

The advanced skeletal model is a richer geometry/mask option for the existing rig, not a second rigging system. `ArmatureState`, computed joint transforms, kinematic solvers, constraints, pose history, scenes, and exports remain authoritative. Switching masks must never change pose or alter the rules that produced it.

Playground remains the context intake and prototype area. Contexts are promoted only after they fit this shared-rig contract; shipped components belong in app-owned feature/component areas rather than `components/playground`.

### P0 — Finish in-flight interaction work
- Complete and validate the other active chat's neck/collar magnet behavior and animation-first shot gestures before changing shared rig or rendering architecture.
- Preserve the agreed interaction semantics: double-clicking a shot copies the previous shot's pose; triple-clicking sets that shot to T-pose; these edit timeline data rather than snapping the live canvas.
- Record any remaining user-defined solver constraints before treating the magnet behavior as settled. Keep this work isolated from mask integration.

### P1 — Define the shared mask contract
- Inventory the current 43-joint rig and the advanced model in [Bitruvius-Evolved-v2](https://github.com/cafeetc/Bitruvius-Evolved-v2); use [Bitruvian.Human.Puppyt](https://github.com/cafeetc/Bitruvian.Human.Puppyt) as a source of context, not as a replacement state model.
- Map advanced geometry to existing joint IDs and computed transforms through explicit attachment data. The older 18-joint procedural model is a behavior/asset reference, not a parallel solver to merge.
- Keep advanced-only details visual unless a new articulation is explicitly specified and shown to preserve current constraints.
- Decide whether the advanced mask replaces or layers over the simple mask as a presentation choice; neither choice may affect pose state or solver behavior.

### P2 — Prototype the adapter in isolation
- Keep imported assets, mapping tables, and experiments in Playground while validating attachment points, orientation, scale, and silhouette against canonical poses.
- Add an app-owned renderer/adapter boundary that consumes the existing computed rig output. Do not fork joint state, interpolation, IK, ground support, or constraints.
- Test representative compound poses and edge cases before mounting the advanced renderer in the production workspace.

### P3 — Add reversible mask switching
- Expose simple and advanced masks as render profiles over the same armature and controls.
- Switching profiles must preserve the exact armature, selected joint, pins, active scene, and timeline position. Dragging and controls must continue to call the same handlers.
- Keep any comparison overlay diagnostic-only; it must not introduce a second interactive skeleton or write to pose state.

### P4 — Promotion and regression gate
- Verify pose-state equality before and after repeated mode switches; verify identical shared-joint transforms and constraint outcomes for the same input pose.
- Run existing solver, interpolation, scene, and export tests, plus focused mask attachment and mode-switch tests. Smoke-test interactions in both profiles.
- Measure rendering cost before optimizing. Promote only the validated adapter, assets, and tests out of Playground; leave rejected or incomplete contexts staged with their status recorded.

**Acceptance invariant:** for a fixed `ArmatureState` and scene, selecting a different mask may change rendered geometry only. It must not change armature values, computed shared-joint transforms, solver results, pose history, or export data.

The existing V1.5–V1.7 items below remain long-term backlog and follow this shared-mask foundation unless reprioritized.

## Context Promotion Priority System

Score each upstream context against this app before adapting it:

- **Relevance (0–5):** how directly it supports the current studio and the shared-rig simple/advanced mask goal.
- **Gap (0–2):** `0` duplicates a working app feature, `1` extends one, `2` fills a real capability gap.
- **Compatibility gate:** `Pass` means it uses current state/rules as-is; `Adapt` means a bounded mapping is needed; `Hold` means it forks the rig, changes constraints, or depends on unresolved rules. `Hold` cannot be promoted regardless of score.
- **Priority score = Relevance + Gap** (maximum 7). P1 = 6–7, P2 = 4–5, P3 = 1–3. Duplicate or incompatible work stays out of the implementation queue. Estimate effort separately; use it only to order items within a priority.

| Context from upstream | Relevance + gap | Gate | Priority and disposition |
| --- | ---: | --- | --- |
| Evolved 43-joint skeletal artwork as a mask over the existing armature | 5 + 2 = 7 | Adapt | **P1.** Core goal. Map geometry to existing joints/transforms; do not port its solver or create a second pose state. |
| Geometry attachment/seam validation | 5 + 2 = 7 | Pass | **P1.** Add focused attachment and continuity checks alongside the mask adapter. |
| Simple/advanced inspection presentation (shadow, purple, hybrid, wireframe) | 4 + 2 = 6 | Adapt | **P2.** Useful for comparing skins and validating the switch; add only after the advanced renderer exists. |
| Procedural per-part masks, visibility, and proportions | 4 + 2 = 6 | Adapt | **P2.** Consider as render configuration only. Do not import global scale controls that alter canonical bone lengths or pose rules. |
| Scene-planner command grammar and ZIP vector-sequence import | 2 + 2 = 4 | Pass | **P3.** Adjacent workflow ideas; current scenes, keyframes, and exports take precedence. |
| Extra PNG/Lottie export formats | 2 + 1 = 3 | Pass | **P3.** Existing versioned Export IR already covers JSON, SVG, Spine, and DragonBones; add formats only against a demonstrated workflow need. |
| Older 18-joint FK/FABRIK engine and its pose/tween library | 1 + 0 = 1 | Hold | **Do not port.** The topology and overlapping pose/tween behavior conflict with or duplicate the current 43-joint system. |

Promote one bounded capability at a time: document its mapping, implement behind the existing state/solver boundary, add a focused regression check, validate in the app, then record the outcome before taking the next P1/P2 item. Re-score when the app or user goal changes; do not bulk-merge upstream repositories.

## Deferred Upcoming Phases

### V1.5 — Binary & Streaming Export Formats
- Add binary Spine `.skel` encoder, multi-frame SVG/SpriteSheet strip packing, and lightweight WebM/PNG sequence frame capture driven by the versioned Export IR.
- Extend snapshot & property tests across all binary serializers.

### V1.6 — Onion-Skinning & Audio Beat Scrubbing
- Add toggleable ghost pose overlays (previous/next keyframe and Breakdown mid-frames) on the canvas stage.
- Optional local audio waveform markers in the bottom `Timeline` tab for beat-aligned keyframe placement.

### V1.7 — Multi-Figure Composition & Saved Rig Profiles
- Support multiple independent `ArmatureState` actors sharing one calibrated scene backdrop and world-pin plane.
- Custom proportional rig presets (heroic 8.5-head, classical 8-head, stylized proportions) serialized alongside `ExportIR`.
