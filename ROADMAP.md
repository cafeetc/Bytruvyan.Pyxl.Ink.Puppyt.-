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
