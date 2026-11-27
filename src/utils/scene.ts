import {
  ArmatureState,
  JointId,
  MotionCurve,
  Point2D,
  PosePresetId,
  RigSystemsConfig,
} from '../types/rig';
import {
  CompositeFilter,
  SamplePlateId,
  Scene,
  SceneMedia,
  StageTransform,
  WorldPin,
} from '../types/scene';
import { ScreenFpsOption } from './frameThrottle';
import {
  computeArmature,
  createPosePreset,
  interpolateArmatures,
  solveJointIK,
} from './kinematics';
import {
  PAPER_HEIGHT,
  PAPER_WIDTH,
  computeSymmetricalCenterPan,
  computeSymmetricalFitZoom,
} from './proportions';

const DEG_TO_RAD = Math.PI / 180;

export const DEFAULT_STAGE_TRANSFORM: StageTransform = {
  originX: 0,
  originY: 0,
  pyxScale: 1.0,
  rotation: 0,
};

export const DEFAULT_COMPOSITE_FILTER: CompositeFilter = {
  blendMode: 'multiply',
  inkOpacity: 0.94,
  edgeThreshold: 38,
  paperGrain: 0.35,
};

export const DEFAULT_SCENE_DURATION_SEC = 4.0;

/**
 * Deterministic inline architectural / horizon plates so users can test external-plate
 * grid calibration, WorldPin grounding, and multiply ink compositing immediately offline.
 */
export const SAMPLE_PLATES: Record<
  SamplePlateId,
  {
    id: SamplePlateId;
    name: string;
    naturalWidth: number;
    naturalHeight: number;
    src: string;
    recommendedTransform: StageTransform;
    recommendedPreset: PosePresetId;
  }
> = {
  stone_archway: {
    id: 'stone_archway',
    name: 'Stone Archway Studio',
    naturalWidth: 1600,
    naturalHeight: 1200,
    src: `data:image/svg+xml;utf8,${encodeURIComponent(
      `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 1600 1200">
        <defs>
          <linearGradient id="sky" x1="0" y1="0" x2="0" y2="1">
            <stop offset="0%" stop-color="#D7D2C7"/>
            <stop offset="68%" stop-color="#ECE7DC"/>
            <stop offset="100%" stop-color="#C8C0B2"/>
          </linearGradient>
          <linearGradient id="floor" x1="0" y1="0" x2="0" y2="1">
            <stop offset="0%" stop-color="#B9B0A0"/>
            <stop offset="100%" stop-color="#9A907F"/>
          </linearGradient>
        </defs>
        <rect width="1600" height="1200" fill="url(#sky)"/>
        <polygon points="0,840 1600,840 1600,1200 0,1200" fill="url(#floor)"/>
        <line x1="0" y1="840" x2="1600" y2="840" stroke="#7A7062" stroke-width="2" stroke-dasharray="10 8" opacity="0.55"/>
        <path d="M220,1200 L220,380 A580,580 0 0 1 1380,380 L1380,1200 L1220,1200 L1220,410 A420,420 0 0 0 380,410 L380,1200 Z" fill="#C4BBAE" stroke="#8C8272" stroke-width="4"/>
        <path d="M380,1200 L380,410 A420,420 0 0 1 1220,410 L1220,1200 Z" fill="#EAE4D8" opacity="0.68"/>
        <g stroke="#8E8474" stroke-width="2" opacity="0.48">
          <line x1="800" y1="840" x2="0" y2="1180"/>
          <line x1="800" y1="840" x2="360" y2="1200"/>
          <line x1="800" y1="840" x2="1240" y2="1200"/>
          <line x1="800" y1="840" x2="1600" y2="1180"/>
          <line x1="160" y1="920" x2="1440" y2="920"/>
          <line x1="70" y1="1020" x2="1530" y2="1020"/>
        </g>
        <rect x="460" y="840" width="680" height="36" fill="#B2A898" stroke="#7D7364" stroke-width="2"/>
      </svg>`
    )}`,
    recommendedTransform: { originX: 0, originY: -40, pyxScale: 0.78, rotation: 0 },
    recommendedPreset: 'tpose',
  },
  botanical_pergola: {
    id: 'botanical_pergola',
    name: 'Sunlight Courtyard Plate',
    naturalWidth: 1600,
    naturalHeight: 1200,
    src: `data:image/svg+xml;utf8,${encodeURIComponent(
      `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 1600 1200">
        <defs>
          <linearGradient id="warmPaper" x1="0" y1="0" x2="1" y2="1">
            <stop offset="0%" stop-color="#E4DFD3"/>
            <stop offset="55%" stop-color="#D4CEC0"/>
            <stop offset="100%" stop-color="#B9B1A2"/>
          </linearGradient>
        </defs>
        <rect width="1600" height="1200" fill="url(#warmPaper)"/>
        <polygon points="0,875 1600,825 1600,1200 0,1200" fill="#B6ADA0" opacity="0.88"/>
        <line x1="0" y1="875" x2="1600" y2="825" stroke="#6E655A" stroke-width="2.5" opacity="0.5"/>
        <g fill="#ADA494" stroke="#787065" stroke-width="2" opacity="0.65">
          <rect x="180" y="140" width="76" height="725"/>
          <rect x="1340" y="120" width="76" height="710"/>
          <rect x="140" y="110" width="1320" height="44"/>
        </g>
        <g stroke="#7A7268" stroke-width="2" opacity="0.35">
          <line x1="256" y1="220" x2="1340" y2="190"/>
          <line x1="256" y1="300" x2="1340" y2="270"/>
          <line x1="0" y1="1030" x2="1600" y2="980"/>
        </g>
        <circle cx="800" cy="480" r="245" fill="#EDE8DC" opacity="0.45"/>
      </svg>`
    )}`,
    recommendedTransform: { originX: 0, originY: -50, pyxScale: 0.74, rotation: -1.5 },
    recommendedPreset: 'tpose',
  },
};

export function createDefaultSceneMedia(): SceneMedia {
  return {
    kind: 'paper',
    paperVariant: 'cover',
    src: '/assets/manual/page_22_cover.png',
    name: '1901 Manual Cover',
    duration: DEFAULT_SCENE_DURATION_SEC,
    fps: 24,
    naturalWidth: PAPER_WIDTH,
    naturalHeight: PAPER_HEIGHT,
    loop: true,
    playbackOffset: 0,
  };
}

export function createSamplePlateMedia(sampleId: SamplePlateId): SceneMedia {
  const plate = SAMPLE_PLATES[sampleId];
  return {
    kind: 'image',
    sampleId,
    src: plate.src,
    name: plate.name,
    duration: DEFAULT_SCENE_DURATION_SEC,
    fps: 24,
    naturalWidth: plate.naturalWidth,
    naturalHeight: plate.naturalHeight,
    loop: true,
    playbackOffset: 0,
  };
}

export function createDefaultScene(
  index = 0,
  viewport: { width: number; height: number } = { width: 1440, height: 780 }
): Scene {
  const fitZoom = computeSymmetricalFitZoom(viewport.width, viewport.height);
  const centerPan = computeSymmetricalCenterPan(
    viewport.width,
    viewport.height,
    PAPER_WIDTH / 2,
    PAPER_HEIGHT / 2,
    fitZoom
  );

  return {
    id: `scene-${index + 1}-${Math.random().toString(36).slice(2, 6)}`,
    name: `Shot ${index + 1}`,
    durationSec: DEFAULT_SCENE_DURATION_SEC,
    media: createDefaultSceneMedia(),
    stageTransform: { ...DEFAULT_STAGE_TRANSFORM },
    compositeFilter: { ...DEFAULT_COMPOSITE_FILTER },
    worldPins: [],
    armature: createPosePreset('tpose'),
    activePreset: 'tpose',
    slots: {
      A: createPosePreset('tpose'),
      B: null,
      C: null,
    },
    activeSlot: 'A',
    timelineT: 0,
    motionCurve: 'flow',
    playbackSpeed: 1,
    showPerspective: false,
    vanishingPoint: {
      x: PAPER_WIDTH / 2,
      y: PAPER_HEIGHT / 2 - 110,
    },
    camera: {
      zoom: fitZoom,
      pan: centerPan,
      userEdited: false,
    },
  };
}

export function cloneSceneForNewShot(baseScene: Scene, nextIndex: number): Scene {
  return {
    ...structuredClone(baseScene),
    id: `scene-${nextIndex + 1}-${Math.random().toString(36).slice(2, 6)}`,
    name: `Shot ${nextIndex + 1}`,
  };
}

export function createShotBridgeSlots(
  previousEndPose: ArmatureState,
  nextStartPose: ArmatureState,
  motionCurve: MotionCurve
): Scene['slots'] {
  return {
    A: structuredClone(previousEndPose),
    B: interpolateArmatures(previousEndPose, nextStartPose, 0.5, motionCurve),
    C: structuredClone(nextStartPose),
  };
}

/**
 * Computes normalized media-plate frame dimensions inside the SVG world coordinate plane,
 * centered around `canvasOrigin` (PAPER_WIDTH/2, PAPER_HEIGHT/2) so image/video plates
 * maintain natural aspect ratio cleanly within the stage.
 */
export function computeMediaPlateBox(
  media: SceneMedia,
  canvasOrigin: Point2D = { x: PAPER_WIDTH / 2, y: PAPER_HEIGHT / 2 }
): { x: number; y: number; width: number; height: number } {
  const nw = Math.max(1, media.naturalWidth || PAPER_WIDTH);
  const nh = Math.max(1, media.naturalHeight || PAPER_HEIGHT);
  const aspect = nw / nh;
  const targetSpan = 1360;
  const width = aspect >= 1 ? targetSpan : targetSpan * aspect;
  const height = aspect >= 1 ? targetSpan / aspect : targetSpan;
  return {
    x: canvasOrigin.x - width / 2,
    y: canvasOrigin.y - height / 2,
    width,
    height,
  };
}

/**
 * Converts a point in uncalibrated world/media space (centered around canvasOrigin)
 * into calibrated rig world coordinates after StageTransform (`originX`, `originY`, `pyxScale`, `rotation`).
 */
export function mediaToWorld(
  mediaPt: Point2D,
  transform: StageTransform,
  canvasOrigin: Point2D = { x: PAPER_WIDTH / 2, y: PAPER_HEIGHT / 2 }
): Point2D {
  const dx = (mediaPt.x - canvasOrigin.x) * transform.pyxScale;
  const dy = (mediaPt.y - canvasOrigin.y) * transform.pyxScale;
  const rad = transform.rotation * DEG_TO_RAD;
  const c = Math.cos(rad);
  const s = Math.sin(rad);
  return {
    x: canvasOrigin.x + transform.originX + dx * c - dy * s,
    y: canvasOrigin.y + transform.originY + dx * s + dy * c,
  };
}

/**
 * Inverse transform: converts a point in calibrated rig world coordinates (e.g. a joint's
 * SVG world coordinate) into uncalibrated stage/media-plate space.
 */
export function worldToMedia(
  worldPt: Point2D,
  transform: StageTransform,
  canvasOrigin: Point2D = { x: PAPER_WIDTH / 2, y: PAPER_HEIGHT / 2 }
): Point2D {
  const safeScale = Math.max(0.05, transform.pyxScale);
  const dx = worldPt.x - canvasOrigin.x - transform.originX;
  const dy = worldPt.y - canvasOrigin.y - transform.originY;
  const rad = -transform.rotation * DEG_TO_RAD;
  const c = Math.cos(rad);
  const s = Math.sin(rad);
  return {
    x: canvasOrigin.x + (dx * c - dy * s) / safeScale,
    y: canvasOrigin.y + (dx * s + dy * c) / safeScale,
  };
}

/**
 * Formats SVG transform attribute applying StageTransform cleanly around canvasOrigin.
 */
export function buildStageSvgTransform(
  transform: StageTransform,
  canvasOrigin: Point2D = { x: PAPER_WIDTH / 2, y: PAPER_HEIGHT / 2 }
): string {
  return `translate(${canvasOrigin.x + transform.originX}, ${canvasOrigin.y + transform.originY}) rotate(${transform.rotation}) scale(${transform.pyxScale}) translate(${-canvasOrigin.x}, ${-canvasOrigin.y})`;
}

/**
 * Evaluates active WorldPins whose normalized `frameRange` covers `timelineT`.
 * Converts each pin's `mediaSpaceXY` (stage/plate coords) into the rig's local calibrated space
 * via `worldToMedia` so effectors stay locked to background features regardless of StageTransform!
 */
export function solveWorldPins(
  armature: ArmatureState,
  worldPins: readonly WorldPin[],
  timelineT: number,
  stageTransform: StageTransform,
  canvasOrigin: Point2D = { x: PAPER_WIDTH / 2, y: PAPER_HEIGHT / 2 },
  rigSystems?: Pick<RigSystemsConfig, 'solver' | 'softLimits'>
): ArmatureState {
  const activePins = worldPins.filter(
    (pin) =>
      pin.enabled &&
      timelineT + 1e-6 >= pin.frameRange[0] &&
      timelineT - 1e-6 <= pin.frameRange[1]
  );
  if (activePins.length === 0) return armature;

  let next = armature;
  for (let pass = 0; pass < 2; pass++) {
    for (const pin of activePins) {
      const rigTarget = worldToMedia(pin.mediaSpaceXY, stageTransform, canvasOrigin);
      next = solveJointIK(next, pin.jointId, rigTarget.x, rigTarget.y, canvasOrigin, {
        solver: rigSystems?.solver ?? 'analytical',
        softDamp: false,
        constrainLimits: rigSystems?.softLimits ?? false,
      });
    }
  }
  return next;
}

/**
 * Creates or updates a WorldPin for a joint locked at its current background plate position.
 */
export function createWorldPinForJoint(
  armature: ArmatureState,
  jointId: JointId,
  stageTransform: StageTransform,
  canvasOrigin: Point2D = { x: PAPER_WIDTH / 2, y: PAPER_HEIGHT / 2 },
  frameRange: readonly [number, number] = [0, 1]
): WorldPin {
  const computed = computeArmature(armature, canvasOrigin);
  const joint = computed[jointId];
  const platePt = mediaToWorld({ x: joint.x, y: joint.y }, stageTransform, canvasOrigin);
  return {
    id: `pin-${jointId}`,
    jointId,
    mediaSpaceXY: { x: platePt.x, y: platePt.y },
    frameRange: [frameRange[0], frameRange[1]],
    enabled: true,
  };
}

/**
 * Synchronizes a <video> media plate's target timestamp with the normalized timelineT,
 * stepped cleanly to the configured cutout aesthetic FPS (12 / 15 / 24 / 30).
 */
export function resolveVideoTimeForTimeline(
  timelineT: number,
  sceneDurationSec: number,
  mediaDurationSec: number,
  fps: ScreenFpsOption,
  loop = true,
  playbackOffset = 0
): number {
  const safeMediaDur = Math.max(0.05, mediaDurationSec);
  const safeSceneDur = Math.max(0.05, sceneDurationSec);
  const safeFps = Math.max(1, fps);
  const clampedT = Math.max(0, Math.min(1, timelineT));

  const rawTime = clampedT * safeSceneDur + playbackOffset;
  const snappedTime = Math.floor(rawTime * safeFps) / safeFps;

  if (loop) {
    const wrapped = ((snappedTime % safeMediaDur) + safeMediaDur) % safeMediaDur;
    return Number(wrapped.toFixed(4));
  }
  return Number(Math.max(0, Math.min(safeMediaDur - 0.001, snappedTime)).toFixed(4));
}

export function advanceSequencePlayback(
  scenes: readonly Scene[],
  activeSceneId: string,
  currentT: number,
  dtSec: number,
  playbackSpeed: number,
  mode: 'scene' | 'sequence'
): { nextSceneId: string; nextT: number } {
  if (scenes.length === 0) {
    return { nextSceneId: activeSceneId, nextT: currentT };
  }
  const idx = Math.max(
    0,
    scenes.findIndex((s) => s.id === activeSceneId)
  );
  const active = scenes[idx];
  const duration = Math.max(0.5, active.durationSec);
  const nextT = currentT + (dtSec * playbackSpeed) / duration;

  if (mode === 'scene') {
    const wrapped = nextT >= 1 ? nextT - Math.floor(nextT) : nextT;
    return { nextSceneId: active.id, nextT: wrapped };
  }

  if (nextT < 1) {
    return { nextSceneId: active.id, nextT };
  }

  const nextIdx = (idx + 1) % scenes.length;
  return {
    nextSceneId: scenes[nextIdx].id,
    nextT: nextT - 1,
  };
}
