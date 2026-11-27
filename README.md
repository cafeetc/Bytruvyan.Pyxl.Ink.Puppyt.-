# Bitruvius

Procedural 43-joint Vitruvian figure rigging, closed-chain inverse kinematics, timeline keyframing, and deterministic Export IR studio built with React, TypeScript, Vite, and Tailwind CSS.

## Quick Start

```bash
npm install
npm run dev
```

Open `http://localhost:3000` in your browser.

### Live Pose Tracking

For manual, offline pose usage tracking, open **Pose Data**:

1. Save a current snapshot as JSON.
2. Select `data/pose-updates.json` as the project document.
3. Ingest each snapshot file. The selected input clears after a successful update.

The document stores only per-pose sample counts and min/max joint angle ranges; it does not retain raw snapshots or collect data in the background. Direct project-file writes use the browser's file picker and require a Chromium-based browser.

The optional **Bitruvius Live Pose Tracker** canvas can still monitor the complete 43-joint rig in real time:

1. Open the canvas with `workspaceId: "bitruvius-ink-manual-kinematics-studio"`.
2. In the canvas, copy the receiver endpoint and access key.
3. In the app's **Pose Stream** menu, paste both values and select **Connect**.

The app sends the current armature, scene/timeline values, per-joint runtime settings, computed joint positions, angles, matrices, and constraint status when pose data changes. The canvas displays all joints and refreshes the latest snapshot. Its receiver binds to loopback only, accepts app requests from localhost, and requires the per-canvas access key. The latest snapshot is stored privately under the Copilot home directory in `extension-data/bitruvius-live-pose-tracker/`; it is not written into the repository. Stream settings are kept in the current browser's local storage and can be removed with **Disconnect**.

### Scripts

| Command | Purpose |
| --- | --- |
| `npm run dev` | Start Vite dev server on port `3000` |
| `npm test` | Run Node/TSX unit tests (`src/utils/*.test.ts`) |
| `npm run lint` | Run TypeScript typechecking (`tsc --noEmit`) |
| `npm run build` | Build production static bundle into `dist/` |
| `npm run preview` | Preview the production build locally |

## Workspace Architecture

- **Top System Bar**: Minimal viewport utilities (`Fit View`, `Grid`, `Guides`, `Rulers`, `Ink Strength`, `Modules` rail toggle, `Reset Layout`, `Dark/Light` theme).
- **Right Vertical Icon Rail**: Single-click model quick toggles (`All IK/FK`, `Soft Limits`, `Floor Snap`, `Pin Anchor`, `360° Spin`, `Perspective Horizon`, `Mirror Pose`, `Reset T-Pose`) plus launchers for the 4 modular model panels (`Rig & Joint`, `Hands & Digits`, `Perspective`, `Scene & Backdrop`).
- **Stackable & Attachable Model Modules**: Modular model controls dock cleanly along the right rail without shrinking the canvas stage, or detach and drag anywhere over the viewport with edge snapping (`right-dock`, `left`, `top`, `bottom`) and `localStorage` persistence.
- **Canvas Gestures**: `Command+Shift`-drag pans the camera. Scrolling stays centered on the X/Y origin and cannot zoom below the default fitted view; `Command+Shift`-scroll bypasses the zoom-out floor and figure-edge resistance. Unmodified drags on empty canvas leave the pose and camera unchanged. Skull drags rotate the head; faceplate drags engage constrained look IK, slide the faceplate within half its dimensions, lift the neck from its collar joint without scaling the neck bone, and favor stable collar rotations while the look direction is steady. The torso-center handle balances the torso against the planted feet, with additional spine compensation for vertical movement; waist dragging also engages thorax and neck compensation. A static ground reference sits behind the pressure plate, which follows the lowest ankle or toe contact to distinguish the rest ground from current support. Hand/claw drags rotate the finger and thumb together; hovering the thumb highlights its separate control. Lumbar, thorax, and neck lean controls counter-adjust to preserve overall spine balance. Long right-click copies pose JSON; double right-click exports a PNG.
- **Collapsible Bottom Animation Drawer**: Houses the `Timeline` (`ABC` keyframe slots, scrubber, play/pause, speed, curve, procedural Breakdown synthesis), `Pose Snapshots` (calibrated `T-Pose` base + custom saved pose library), and `Animation & Export` (multi-shot scene sequencer, shot FPS throttle, and versioned Export IR adapters).
- **Engine & Export IR**:
  - `src/utils/kinematics.ts` — 43-joint closed-chain solver (non-elastic collar & hip toggle hinges, per-arm reverse swivel, analytical/CCD/FABRIK IK, soft limits, live pose interpolation).
  - `src/utils/proportions.ts` — Canonical PYX proportions (`1.00 PYX = 100 PX`), 8-head bands, working square, and symmetrical stage borders.
  - `src/utils/scene.ts` — Multi-shot scene calibration, video/image backdrop plate sync, and world pins.
  - `src/types/exportIR.ts` & `src/utils/exportAdapters.ts` — Versioned Export IR (`1.0.0`) with deterministic adapters for Canonical JSON, Layered SVG, Spine JSON, and DragonBones JSON.
- **Performance & Pose Integrity**: Canvas pointer input is coalesced to the latest position per animation frame, with the final drag position flushed on release. Kinematics tests sample compound pose transitions to check finite, drawable geometry and guard against abrupt per-frame movement.

## Canonical Pose Policy

- Canonical presets are cleared strictly to `T-Pose` (`'tpose'`).
- **No LLM should ever add default poses without explicit user prompt or approval.**
- All poses added or updated in the future must be calibrated directly against the latest rig geometry and constraint updates.

## VS Code Setup

Shared workspace files are included in `.vscode/`:

- **Extensions**: ESLint, Pretty TypeScript Errors, Error Lens, Tailwind CSS IntelliSense, Prettier, SVG, Color Highlight, GitLens, and JSON Crack (`extensions.json`).
- **Formatting**: Format on save with Prettier, workspace TypeScript SDK, and explicit ESLint fix-all on save (`settings.json`).
- **Tasks**: Run **Tasks: Run Task** (`Dev Server (Port 3000)`, `Run Unit Tests`, `Typecheck (tsc --noEmit)`, `Build Production Bundle`).
- **Debugging**: Press **F5** (`Debug Bitruvius in Chrome`) to start the dev server and attach Chrome to `http://localhost:3000`.

Tests currently use Node's test runner through `tsx` (`npm test`), not Vitest or Jest, so no framework-specific test explorer extension is recommended.
