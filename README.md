# Bitruvius

Procedural 43-joint Vitruvian figure rigging, closed-chain inverse kinematics, timeline keyframing, and deterministic Export IR studio built with React, TypeScript, Vite, and Tailwind CSS.

## Quick Start

```bash
npm install
npm run dev
```

Open `http://localhost:3000` in your browser.

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
- **Collapsible Bottom Animation Drawer**: Houses the `Timeline` (`ABC` keyframe slots, scrubber, play/pause, speed, curve, procedural Breakdown synthesis), `Pose Snapshots` (calibrated `T-Pose` base + custom saved pose library), and `Animation & Export` (multi-shot scene sequencer, shot FPS throttle, and versioned Export IR adapters).
- **Engine & Export IR**:
  - `src/utils/kinematics.ts` — 43-joint closed-chain solver (non-elastic collar & hip toggle hinges, per-arm reverse swivel, analytical/CCD/FABRIK IK, soft limits, live pose interpolation).
  - `src/utils/proportions.ts` — Canonical PYX proportions (`1.00 PYX = 100 PX`), 8-head bands, working square, and symmetrical stage borders.
  - `src/utils/scene.ts` — Multi-shot scene calibration, video/image backdrop plate sync, and world pins.
  - `src/types/exportIR.ts` & `src/utils/exportAdapters.ts` — Versioned Export IR (`1.0.0`) with deterministic adapters for Canonical JSON, Layered SVG, Spine JSON, and DragonBones JSON.

## Canonical Pose Policy

- Canonical presets are cleared strictly to `T-Pose` (`'tpose'`).
- **No LLM should ever add default poses without explicit user prompt or approval.**
- All poses added or updated in the future must be calibrated directly against the latest rig geometry and constraint updates.

## VS Code Setup

Shared workspace files are included in `.vscode/`:

- **Extensions**: ESLint, Tailwind CSS IntelliSense, Prettier (`extensions.json`).
- **Tasks**: Run **Tasks: Run Task** (`Dev Server (Port 3000)`, `Run Unit Tests`, `Typecheck (tsc --noEmit)`, `Build Production Bundle`).
- **Debugging**: Press **F5** (`Debug Bitruvius in Chrome`) to start the dev server and attach Chrome to `http://localhost:3000`.
