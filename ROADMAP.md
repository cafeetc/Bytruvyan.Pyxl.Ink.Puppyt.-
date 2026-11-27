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

## Upcoming Phases

### V1.5 — Binary & Streaming Export Formats
- Add binary Spine `.skel` encoder, multi-frame SVG/SpriteSheet strip packing, and lightweight WebM/PNG sequence frame capture driven by the versioned Export IR.
- Extend snapshot & property tests across all binary serializers.

### V1.6 — Onion-Skinning & Audio Beat Scrubbing
- Add toggleable ghost pose overlays (previous/next keyframe and Breakdown mid-frames) on the canvas stage.
- Optional local audio waveform markers in the bottom `Timeline` tab for beat-aligned keyframe placement.

### V1.7 — Multi-Figure Composition & Saved Rig Profiles
- Support multiple independent `ArmatureState` actors sharing one calibrated scene backdrop and world-pin plane.
- Custom proportional rig presets (heroic 8.5-head, classical 8-head, stylized proportions) serialized alongside `ExportIR`.
