import {
  COLLAR_LINK_SCALE,
  FACEPLATE_SLIDE_LIMIT_SVG,
  PUPPYT_LANDMARKS,
  TORSO_HANDLE_CENTER_SVG,
  JOINT_SPECS,
  JOINT_SPEC_MAP,
  CANONICAL_RIG_PARENTS,
  SVG_MODEL_CONSTANTS,
} from '../data/digitalRig';
import {
  ANGLE_PRECISION_STEP,
  ArmatureState,
  BONE_SCALE_MAX,
  BONE_SCALE_MIN,
  CaveArtLine,
  CaveArtPolygon,
  CaveArtRigGeometry,
  ComputedJoint,
  EnvironmentalAnchorId,
  GroundContactPin,
  JointId,
  JointMode,
  JointRuntimeState,
  JointSpec,
  PhysicsModeId,
  Point2D,
  PosePresetId,
  RigHierarchyAuditReport,
  ToggleHinge2LinkResult,
  ToggleHingeFoldState,
} from '../types/rig';
import {
  PAPER_HEIGHT,
  PAPER_WIDTH,
  PUPPYT_TO_EVOLVED_SCALE,
  computeProportionalCanons,
} from './proportions';

export type Affine2D = readonly [number, number, number, number, number, number];

const DEG_TO_RAD = Math.PI / 180;
const RAD_TO_DEG = 180 / Math.PI;

function softSaturate(value: number, limit: number): number {
  return limit * Math.tanh(value / limit);
}

export function mulAffine(a: Affine2D, b: Affine2D): Affine2D {
  return [
    a[0] * b[0] + a[2] * b[1],
    a[1] * b[0] + a[3] * b[1],
    a[0] * b[2] + a[2] * b[3],
    a[1] * b[2] + a[3] * b[3],
    a[0] * b[4] + a[2] * b[5] + a[4],
    a[1] * b[4] + a[3] * b[5] + a[5],
  ];
}

export function applyAffine(m: Affine2D, x: number, y: number): Point2D {
  return {
    x: m[0] * x + m[2] * y + m[4],
    y: m[1] * x + m[3] * y + m[5],
  };
}

export function composeBoneMatrix(
  originX: number,
  originY: number,
  worldDeg: number,
  lenScale: number,
  widScale: number,
  restVecDeg: number,
  pivotX: number,
  pivotY: number
): Affine2D {
  const wr = worldDeg * DEG_TO_RAD;
  const rr = -restVecDeg * DEG_TO_RAD;
  const cw = Math.cos(wr);
  const sw = Math.sin(wr);
  const cr = Math.cos(rr);
  const sr = Math.sin(rr);

  const tOrigin: Affine2D = [1, 0, 0, 1, originX, originY];
  const rWorld: Affine2D = [cw, sw, -sw, cw, 0, 0];
  const scale: Affine2D = [lenScale, 0, 0, widScale, 0, 0];
  const rRestInv: Affine2D = [cr, sr, -sr, cr, 0, 0];
  const tPivotInv: Affine2D = [1, 0, 0, 1, -pivotX, -pivotY];

  return mulAffine(tOrigin, mulAffine(rWorld, mulAffine(scale, mulAffine(rRestInv, tPivotInv))));
}

const SVG_TO_PUPPYT_AFFINE: Affine2D = [
  SVG_MODEL_CONSTANTS.puppytScaleFactor,
  0,
  0,
  SVG_MODEL_CONSTANTS.puppytScaleFactor,
  SVG_MODEL_CONSTANTS.pelvisPuppytPivot[0] -
    SVG_MODEL_CONSTANTS.midX * SVG_MODEL_CONSTANTS.puppytScaleFactor,
  SVG_MODEL_CONSTANTS.pelvisPuppytPivot[1] -
    SVG_MODEL_CONSTANTS.waistY * SVG_MODEL_CONSTANTS.puppytScaleFactor,
];

const SVG_MIRROR_X_AFFINE: Affine2D = [-1, 0, 0, 1, SVG_MODEL_CONSTANTS.midX * 2, 0];

/**
 * Maps SVG coordinates from body_pose_full_b3b6bf24.svg directly through a joint's live
 * affine bone matrix into stage coordinates, optionally mirrored across the SVG midline.
 */
export function computeSvgBoneMatrix(boneMatrix: Affine2D, mirrorX = false): Affine2D {
  const local = mirrorX
    ? mulAffine(SVG_TO_PUPPYT_AFFINE, SVG_MIRROR_X_AFFINE)
    : SVG_TO_PUPPYT_AFFINE;
  return mulAffine(boneMatrix, local);
}

function vecAngleDeg(a: readonly [number, number], b: readonly [number, number]): number {
  return Math.atan2(b[1] - a[1], b[0] - a[0]) * RAD_TO_DEG;
}

export const PHYSICS_MODE_INFO: Record<
  PhysicsModeId,
  { label: string; shortLabel: string; ratio: number | null; description: string }
> = {
  twins: {
    label: 'Twins',
    shortLabel: '1.00×',
    ratio: 1.0,
    description: '1:1 direct rotational input response',
  },
  elastic_rubber: {
    label: 'Elastic',
    shortLabel: '1.50×',
    ratio: 1.5,
    description: '150% amplified sweep ratio',
  },
  damp: {
    label: 'Damp',
    shortLabel: '0.50×',
    ratio: 0.5,
    description: '50% precision resistance ratio',
  },
  rigid: {
    label: 'Rigid',
    shortLabel: '0.00×',
    ratio: 0.0,
    description: '0% locked freeze preventing angular change',
  },
  mode_e: {
    label: 'Mode E',
    shortLabel: 'Gear',
    ratio: null,
    description: 'Custom steering gear ratio (0.50× to 1.50×)',
  },
};

export const MODE_E_MIN = 0.5;
export const MODE_E_MAX = 1.5;

/**
 * POLICY (Canonical Poses):
 * - Clear to strictly `'tpose'` unless explicitly requested/approved by the user.
 * - NO LLM may ever add default poses without user prompt or approval.
 * - All poses must always be calibrated to current rig updates.
 */
export const POSE_PRESET_LABELS: Record<PosePresetId, string> = {
  tpose: 'T-Pose',
};

export function auditRigHierarchy(
  specs: readonly JointSpec[] = JOINT_SPECS
): RigHierarchyAuditReport {
  const byId = new Map<JointId, JointSpec>();
  const idxById = new Map<JointId, number>();
  specs.forEach((s, i) => {
    byId.set(s.id, s);
    idxById.set(s.id, i);
  });

  const orphanNodes: string[] = [];
  const cycles: string[] = [];
  const topologicalViolations: string[] = [];
  const aimOrderViolations: string[] = [];
  const parentMismatches: Array<{ id: string; expected: string | null; actual: string | null }> = [];

  specs.forEach((s, i) => {
    if (s.parentId) {
      if (!byId.has(s.parentId)) {
        orphanNodes.push(s.id);
      } else if ((idxById.get(s.parentId) ?? -1) >= i) {
        topologicalViolations.push(s.id);
      }
    }
    if (s.aimTarget?.to) {
      const tIdx = idxById.get(s.aimTarget.to);
      if (tIdx === undefined) {
        orphanNodes.push(s.id);
      } else if (tIdx >= i) {
        aimOrderViolations.push(s.id);
      }
    }
    const expectedParent = CANONICAL_RIG_PARENTS[s.id];
    if (expectedParent !== s.parentId) {
      parentMismatches.push({
        id: s.id,
        expected: expectedParent,
        actual: s.parentId,
      });
    }
    const seen = new Set<JointId>([s.id]);
    let cur = s.parentId ? byId.get(s.parentId) : undefined;
    while (cur) {
      if (seen.has(cur.id)) {
        cycles.push(s.id);
        break;
      }
      seen.add(cur.id);
      cur = cur.parentId ? byId.get(cur.parentId) : undefined;
    }
  });

  return {
    valid:
      specs.length === 43 &&
      orphanNodes.length === 0 &&
      cycles.length === 0 &&
      topologicalViolations.length === 0 &&
      aimOrderViolations.length === 0 &&
      parentMismatches.length === 0,
    totalJoints: specs.length,
    orphanNodes,
    cycles,
    topologicalViolations,
    aimOrderViolations,
    parentMismatches,
  };
}

export function snapAnglePrecision(deg: number, step = ANGLE_PRECISION_STEP): number {
  return Math.round(deg / step) * step;
}

export function clampAngle180(deg: number): number {
  return Math.max(-180, Math.min(180, deg));
}

export function normalizeAngleDeg(deg: number): number {
  let a = (((deg + 180) % 360) + 360) % 360 - 180;
  if (a === -180) a = 180;
  return Math.abs(a) < 1e-6 ? 0 : a;
}

/**
 * Shortest-path angular wrapping formula:
 * Δθ = ((θ_target - θ_current + 540°) mod 360°) - 180°
 */
export function shortestAngleDelta(currentDeg: number, targetDeg: number): number {
  return ((((targetDeg - currentDeg + 540) % 360) + 360) % 360) - 180;
}

export function applyJointAngleConstraint(deg: number, continuousSpin: boolean): number {
  const snapped = snapAnglePrecision(deg);
  return continuousSpin ? snapped : clampAngle180(snapped);
}

export function clampModeERatio(ratio: number): number {
  if (!Number.isFinite(ratio)) return 1.0;
  return Math.max(MODE_E_MIN, Math.min(MODE_E_MAX, Math.round(ratio * 100) / 100));
}

export function getPhysicsMultiplier(runtime: JointRuntimeState): number {
  switch (runtime.physicsMode) {
    case 'twins':
      return 1.0;
    case 'elastic_rubber':
      return 1.5;
    case 'damp':
      return 0.5;
    case 'rigid':
      return 0.0;
    case 'mode_e':
      return clampModeERatio(runtime.modeERatio);
  }
}

export function formatSignedDeg(deg: number): string {
  const abs = Math.abs(deg).toFixed(2);
  const sign = deg < -0.005 ? '-' : '+';
  return `${sign}${abs}°`;
}

/**
 * Builds a calibrated pose preset.
 * POLICY: No LLM should ever add default poses without explicit user prompt or approval;
 * every preset must be calibrated directly against current rig updates.
 */
export function createPosePreset(
  preset: PosePresetId,
  existingState?: ArmatureState
): ArmatureState {
  const joints = {} as Record<JointId, JointRuntimeState>;
  for (const spec of JOINT_SPECS) {
    const prev = existingState?.joints[spec.id];
    joints[spec.id] = {
      localAngle: spec.defaultLocalAngle,
      mode: prev?.mode ?? spec.defaultMode,
      physicsMode: prev?.physicsMode ?? 'twins',
      modeERatio: prev?.modeERatio ?? 1.0,
      scaleLength: prev?.scaleLength ?? 1.0,
      scaleWidth: prev?.scaleWidth ?? 1.0,
    };
  }

  const base: ArmatureState = {
    rootX: existingState?.rootX ?? PAPER_WIDTH / 2,
    rootY: existingState?.rootY ?? PAPER_HEIGHT / 2,
    rootRotation: 0,
    hipToggleSign: existingState?.hipToggleSign ?? 1,
    armSwivelR: 0,
    armSwivelL: 0,
    faceplateOffsetX: 0,
    faceplateOffsetY: 0,
    neckLift: 0,
    neckCollarSeparation: { x: 0, y: 0 },
    continuousSpin: existingState?.continuousSpin ?? true,
    internalKeystone: existingState?.internalKeystone ?? 'navel',
    environmentalAnchor: existingState?.environmentalAnchor ?? 'navel',
    isAnchorPinned: existingState?.isAnchorPinned ?? true,
    joints,
  };

  if (preset === 'tpose') {
    // Matches calibrated body_pose_full_b3b6bf24.svg rest pose (horizontal T-pose arms + wide A-stance legs + outward feet)
  }

  return base;
}

/**
 * Rigid equal-link half-span for the 2-bone hip toggle bridge (`L1 = L2 = 86.0` SVG units),
 * lengthened slightly to cover full static & leg-swing spread rigidly without elastic stretching.
 */
export const HIP_BRIDGE_LINK_SVG = 86.0;
export const HIP_BRIDGE_LINK_STAGE_PX =
  HIP_BRIDGE_LINK_SVG * SVG_MODEL_CONSTANTS.stageScale;

export const COLLAR_INWARD_NORMAL_LIMIT_DEG = 16;
export const COLLAR_INWARD_RESISTANCE_START_DEG = 11.5;
export const COLLAR_INWARD_BREAKAWAY_DEG = 21.5;
const NECK_LIFT_MAX_SWIVEL_SPLAY_DEG = 16;
const COLLAR_EXPLICIT_UP_THRESHOLD_DEG = 1;
export const COLLAR_UNSNAP_HYSTERESIS_DEG = 18.0;
export const COLLAR_SWIVEL_MAX_INWARD_DEG = 48;
export const SWIVEL_REST_OFFSET_DEG = Math.atan2(99, 267) * RAD_TO_DEG;

export interface CollarInwardBehaviorResult {
  snapState: 'normal' | 'resisting' | 'snapped';
  resistanceRatio: number;
  effectiveLocalAngle: number;
}

/**
 * Evaluates a collar swivel's inward state:
 * - `[0°, 11.5°)` inward: normal shoulder range
 * - `[11.5°, 16°]` inward: tactile resistance state near the end of its inward limit
 * - `> 16.5°` inward: snapped across onto the opposite shoulder's collar for clean side poses
 */
export function resolveCollarInwardBehavior(
  side: 'R' | 'L',
  localAngle: number,
  _otherSwivelLocalAngle?: number
): CollarInwardBehaviorResult {
  const minAngle = side === 'R' ? -COLLAR_INWARD_NORMAL_LIMIT_DEG : -COLLAR_SWIVEL_MAX_INWARD_DEG;
  const maxAngle = side === 'R' ? COLLAR_SWIVEL_MAX_INWARD_DEG : COLLAR_INWARD_NORMAL_LIMIT_DEG;
  const clamped = Math.max(minAngle, Math.min(maxAngle, localAngle));
  const inward = side === 'R' ? clamped : -clamped;

  const resistanceRatio =
    inward >= COLLAR_INWARD_RESISTANCE_START_DEG
      ? Math.max(
          0,
          Math.min(
            1,
            (inward - COLLAR_INWARD_RESISTANCE_START_DEG) /
              Math.max(1e-4, COLLAR_SWIVEL_MAX_INWARD_DEG - COLLAR_INWARD_RESISTANCE_START_DEG)
          )
        )
      : 0;

  return {
    snapState: inward > COLLAR_INWARD_NORMAL_LIMIT_DEG ? 'resisting' : 'normal',
    resistanceRatio,
    effectiveLocalAngle: clamped,
  };
}

/**
 * Applies interactive FK drag input to `swivelR` / `swivelL` with smooth continuous interpolation
 * directly following pointer movement without discrete snap jumps.
 */
export function applyCollarSwivelDragInput(
  state: ArmatureState,
  side: 'R' | 'L',
  rawDragAngle: number
): { nextAngle: number; nextRawAngle: number; behavior: CollarInwardBehaviorResult } {
  const minAngle = side === 'R' ? -COLLAR_INWARD_NORMAL_LIMIT_DEG : -COLLAR_SWIVEL_MAX_INWARD_DEG;
  const maxAngle = side === 'R' ? COLLAR_SWIVEL_MAX_INWARD_DEG : COLLAR_INWARD_NORMAL_LIMIT_DEG;
  const clampedRaw = Math.max(minAngle, Math.min(maxAngle, rawDragAngle));
  const behavior = resolveCollarInwardBehavior(side, clampedRaw);
  const nextAngle = snapAnglePrecision(clampedRaw);
  return {
    nextAngle,
    nextRawAngle: clampedRaw,
    behavior,
  };
}

export function setArmReverseSwivel(
  state: ArmatureState,
  side: 'R' | 'L',
  deg: number
): ArmatureState {
  const normalized = normalizeAngleDeg(deg);
  return side === 'R'
    ? { ...state, armSwivelR: normalized }
    : { ...state, armSwivelL: normalized };
}

export function toggleArmReverseSwivel(
  state: ArmatureState,
  side: 'R' | 'L'
): ArmatureState {
  const cur = side === 'R' ? (state.armSwivelR ?? 0) : (state.armSwivelL ?? 0);
  const isReversed = Math.cos(cur * DEG_TO_RAD) < 0;
  return setArmReverseSwivel(state, side, isReversed ? 0 : 180);
}

/**
 * Analytical 2-link toggle-hinge solver (Law of Cosines via acos).
 * Strictly rigid (zero elasticity): link lengths never stretch beyond `linkLength`.
 * - When d >= 2 * L (within deadband), snaps to 180° Single Line (extended).
 * - When d < 2 * L, folds P_mid along the vertical bisector using foldSign (+1 = A-shape, -1 = V-shape).
 */
export function solveToggleHinge2Link(
  pLeft: Point2D,
  pRight: Point2D,
  linkLength: number,
  foldSign: 1 | -1 = 1,
  upHint?: Point2D,
  deadbandRatio = 0.997
): ToggleHinge2LinkResult {
  const safeL = Math.max(1e-3, linkLength);
  const dx = pRight.x - pLeft.x;
  const dy = pRight.y - pLeft.y;
  const rawDist = Math.hypot(dx, dy);
  const ux = rawDist > 1e-4 ? dx / rawDist : 1;
  const uy = rawDist > 1e-4 ? dy / rawDist : 0;

  const midX = (pLeft.x + pRight.x) * 0.5;
  const midY = (pLeft.y + pRight.y) * 0.5;

  // Choose perpendicular unit normal pointing "up" (toward upHint if provided, else -Y)
  const n1x = -uy;
  const n1y = ux;
  const refDx = upHint ? upHint.x - midX : 0;
  const refDy = upHint ? upHint.y - midY : -1;
  const dot = n1x * refDx + n1y * refDy;
  const upNx = dot >= 0 ? n1x : -n1x;
  const upNy = dot >= 0 ? n1y : -n1y;

  const maxSpan = safeL * 2;
  const span = Math.min(maxSpan, rawDist);

  if (rawDist >= maxSpan * deadbandRatio) {
    const pMid = { x: midX, y: midY };
    const clampedLeft =
      rawDist > maxSpan + 1e-4
        ? { x: midX - ux * safeL, y: midY - uy * safeL }
        : pLeft;
    const clampedRight =
      rawDist > maxSpan + 1e-4
        ? { x: midX + ux * safeL, y: midY + uy * safeL }
        : pRight;
    const lineAngle = Math.atan2(dy, dx) * RAD_TO_DEG;
    return {
      pLeft: clampedLeft,
      pMid,
      pRight: clampedRight,
      linkLength: safeL,
      span: maxSpan,
      state: 'extended',
      foldSign,
      interiorAngleDeg: 180,
      leftAngleDeg: lineAngle,
      rightAngleDeg: lineAngle,
    };
  }

  const cosAlpha = Math.max(0, Math.min(1, span / maxSpan));
  const alphaRad = Math.acos(cosAlpha);
  const h = safeL * Math.sin(alphaRad);
  const pMid = {
    x: midX + foldSign * h * upNx,
    y: midY + foldSign * h * upNy,
  };
  const interiorAngleDeg = 180 - 2 * alphaRad * RAD_TO_DEG;
  const state: ToggleHingeFoldState = foldSign >= 0 ? 'A' : 'V';

  return {
    pLeft,
    pMid,
    pRight,
    linkLength: safeL,
    span,
    state,
    foldSign,
    interiorAngleDeg,
    leftAngleDeg: Math.atan2(pMid.y - pLeft.y, pMid.x - pLeft.x) * RAD_TO_DEG,
    rightAngleDeg: Math.atan2(pRight.y - pMid.y, pRight.x - pMid.x) * RAD_TO_DEG,
  };
}

/**
 * Couples hip swivel smoothly as an upper anatomical extension of the thigh bone:
 * outward leg swings engage up to 36% hip rotation (capped to preserve pelvis integrity),
 * while inward swings engage 16% without collapsing the inter-hip bridge.
 */
export function computeHipThighExtension(hipId: 'hipR' | 'hipL', femurLocalDeg: number): number {
  if (Math.abs(femurLocalDeg) < 1e-4) return 0;
  if (hipId === 'hipR') {
    return femurLocalDeg >= 0
      ? Math.min(24, femurLocalDeg * 0.36)
      : Math.max(-7, femurLocalDeg * 0.16);
  }
  return femurLocalDeg <= 0
    ? Math.max(-24, femurLocalDeg * 0.36)
    : Math.min(7, femurLocalDeg * 0.16);
}

/**
 * Resolves collarbone toggle-hinge fold sign & effective angle magnitude so collarbones
 * favor rotating downward (`-1`, V-shape) by default, and smoothly ease through horizontal
 * (`180°`) upward (`+1`, A-shape) whenever overhead arm elevation or high shoulder lift
 * makes downward collarbone rotation impossible.
 */
export function resolveCollarToggleFold(
  side: 'R' | 'L',
  effLocalAngle: number,
  armElevDeg: number,
  acroLiftPx = 0
): { foldSign: 1 | -1; effectiveFoldMag: number; effectiveLocalAngle: number; upEase: number } {
  const clampedInput = Math.max(-65, Math.min(65, effLocalAngle));
  const explicitUp =
    side === 'R'
      ? clampedInput > COLLAR_EXPLICIT_UP_THRESHOLD_DEG
      : clampedInput < -COLLAR_EXPLICIT_UP_THRESHOLD_DEG;

  // Smoothly ramp upward ease (0 -> 1) when arm lifts past +22° overhead or shoulder elevates past +10px
  const armEase = Math.max(0, Math.min(1, (armElevDeg - 22) / 30));
  const liftEase = Math.max(0, Math.min(1, (acroLiftPx - 10) / 14));
  const upEase = Math.max(armEase, liftEase);

  if (explicitUp) {
    return {
      foldSign: 1,
      effectiveFoldMag: Math.abs(clampedInput),
      effectiveLocalAngle: clampedInput,
      upEase: 1,
    };
  }

  // Downward-favored default (-1) unless impossible (upEase > 0.5 eases smoothly into +1)
  const downBias = 1 - 2 * upEase; // +1 (down) -> 0 (180° extended) -> -1 (up)
  const inputMag = Math.abs(clampedInput);

  if (downBias >= 0) {
    const foldMag = inputMag * downBias;
    return {
      foldSign: -1,
      effectiveFoldMag: foldMag,
      effectiveLocalAngle: side === 'R' ? -foldMag : foldMag,
      upEase,
    };
  }

  const upwardStrength = -downBias;
  const foldMag = Math.max(inputMag, upEase * 14) * upwardStrength;
  return {
    foldSign: 1,
    effectiveFoldMag: foldMag,
    effectiveLocalAngle: side === 'R' ? foldMag : -foldMag,
    upEase,
  };
}

export function clampNeckCollarSeparation(separation: Point2D): Point2D {
  const neckSpec = JOINT_SPEC_MAP.neck;
  const neckLengthSvg = neckSpec.restTip
    ? Math.hypot(
        neckSpec.restTip[0] - neckSpec.restPivot[0],
        neckSpec.restTip[1] - neckSpec.restPivot[1]
      ) / SVG_MODEL_CONSTANTS.puppytScaleFactor
    : 0;
  const maxSeparationPx =
    (neckLengthSvg / 2) *
    SVG_MODEL_CONSTANTS.puppytScaleFactor *
    PUPPYT_TO_EVOLVED_SCALE;
  const separationLength = Math.hypot(separation.x, separation.y);
  if (separationLength <= maxSeparationPx || separationLength <= 1e-8) return separation;
  const scale = maxSeparationPx / separationLength;
  return { x: separation.x * scale, y: separation.y * scale };
}

export function setNeckCollarSeparation(
  state: ArmatureState,
  separation: Point2D
): ArmatureState {
  return { ...state, neckCollarSeparation: clampNeckCollarSeparation(separation) };
}

function evaluateRawArmature(state: ArmatureState): Record<JointId, ComputedJoint> {
  const computed = {} as Record<JointId, ComputedJoint>;
  const neckSpec = JOINT_SPEC_MAP.neck;
  const neckLengthSvg = neckSpec.restTip
    ? Math.hypot(
        neckSpec.restTip[0] - neckSpec.restPivot[0],
        neckSpec.restTip[1] - neckSpec.restPivot[1]
      ) / SVG_MODEL_CONSTANTS.puppytScaleFactor
    : 0;
  const maxNeckLift = neckLengthSvg / 2;
  const neckSeparation = clampNeckCollarSeparation(state.neckCollarSeparation ?? { x: 0, y: 0 });
  const neckLiftRatio =
    maxNeckLift > 0 ? Math.max(0, Math.min(1, (state.neckLift ?? 0) / maxNeckLift)) : 0;
  const maxSeparationPx =
    maxNeckLift * SVG_MODEL_CONSTANTS.puppytScaleFactor * PUPPYT_TO_EVOLVED_SCALE;
  const separationRatio =
    maxSeparationPx > 0
      ? Math.max(0, Math.min(1, Math.hypot(neckSeparation.x, neckSeparation.y) / maxSeparationPx))
      : 0;
  const neckCollarPressureRatio = Math.max(neckLiftRatio, separationRatio);
  const easedNeckLift = neckCollarPressureRatio ** 2 * (3 - 2 * neckCollarPressureRatio);

  // Pre-compute rigid non-stretching hip pair so 2-bone hip bridge never stretches beyond maxSpan
  const pelvisScaleLen = Math.max(
    BONE_SCALE_MIN,
    Math.min(BONE_SCALE_MAX, state.joints.pelvis?.scaleLength ?? 1)
  );
  const pelvisScaleWid = Math.max(
    BONE_SCALE_MIN,
    Math.min(BONE_SCALE_MAX, state.joints.pelvis?.scaleWidth ?? 1)
  );
  const baseHipR = Math.max(-10, Math.min(21, state.joints.hipR?.localAngle ?? 0));
  const baseHipL = Math.max(-21, Math.min(10, state.joints.hipL?.localAngle ?? 0));
  const reqExtR = computeHipThighExtension('hipR', state.joints.femurR?.localAngle ?? 0);
  const reqExtL = computeHipThighExtension('hipL', state.joints.femurL?.localAngle ?? 0);
  let resolvedHipR = Math.max(-14, Math.min(34, baseHipR + reqExtR));
  let resolvedHipL = Math.max(-34, Math.min(14, baseHipL + reqExtL));
  const maxHipSpanStage = 2 * HIP_BRIDGE_LINK_STAGE_PX * pelvisScaleWid;
  const hipRadiusStage =
    Math.hypot(37, 145) * SVG_MODEL_CONSTANTS.stageScale * pelvisScaleLen;
  const restHipOutwardDeg = Math.atan2(37, 145) * RAD_TO_DEG;
  const sinHalfHipMax = Math.min(
    0.9999,
    maxHipSpanStage / (2 * Math.max(1e-4, hipRadiusStage))
  );
  const maxHipOutwardSum =
    2 * (Math.asin(sinHalfHipMax) * RAD_TO_DEG - restHipOutwardDeg);
  const currentHipOutwardSum = resolvedHipR - resolvedHipL;
  if (currentHipOutwardSum > maxHipOutwardSum) {
    const excess = currentHipOutwardSum - maxHipOutwardSum;
    const outR = Math.max(0, resolvedHipR);
    const outL = Math.max(0, -resolvedHipL);
    const totalOut = outR + outL;
    if (totalOut > 1e-4) {
      const ratioR = outR / totalOut;
      const ratioL = outL / totalOut;
      resolvedHipR -= excess * ratioR;
      resolvedHipL += excess * ratioL;
    } else {
      resolvedHipR -= excess * 0.5;
      resolvedHipL += excess * 0.5;
    }
  }
  const appliedHipExtR = resolvedHipR - baseHipR;
  const appliedHipExtL = resolvedHipL - baseHipL;

  const armFlipCosR = Math.cos((state.armSwivelR ?? 0) * DEG_TO_RAD);
  const armFlipCosL = Math.cos((state.armSwivelL ?? 0) * DEG_TO_RAD);
  const armVisualFlipR =
    Math.abs(armFlipCosR) < 0.14 ? (armFlipCosR >= 0 ? 0.14 : -0.14) : armFlipCosR;
  const armVisualFlipL =
    Math.abs(armFlipCosL) < 0.14 ? (armFlipCosL >= 0 ? 0.14 : -0.14) : armFlipCosL;

  for (const spec of JOINT_SPECS) {
    const runtime = state.joints[spec.id] ?? {
      localAngle: spec.defaultLocalAngle,
      mode: spec.defaultMode,
      physicsMode: 'twins' as PhysicsModeId,
      modeERatio: 1.0,
      scaleLength: 1.0,
      scaleWidth: 1.0,
    };

    const scaleLen = Math.max(BONE_SCALE_MIN, Math.min(BONE_SCALE_MAX, runtime.scaleLength));
    const scaleWid = Math.max(BONE_SCALE_MIN, Math.min(BONE_SCALE_MAX, runtime.scaleWidth));
    const effLocalAngle = spec.locked ? 0 : runtime.localAngle;

    if (spec.parentId === null) {
      const restWorldDeg = spec.restTip ? vecAngleDeg(spec.restPivot, spec.restTip) : -90;
      const pelvisMatrix = composeBoneMatrix(
        state.rootX,
        state.rootY,
        state.rootRotation,
        PUPPYT_TO_EVOLVED_SCALE * scaleWid,
        PUPPYT_TO_EVOLVED_SCALE * scaleLen,
        0,
        spec.restPivot[0],
        spec.restPivot[1]
      );
      const endPt = spec.restTip
        ? applyAffine(pelvisMatrix, spec.restTip[0], spec.restTip[1])
        : { x: state.rootX, y: state.rootY };

      computed[spec.id] = {
        id: spec.id,
        spec,
        x: state.rootX,
        y: state.rootY,
        endX: endPt.x,
        endY: endPt.y,
        parentX: null,
        parentY: null,
        localAngle: state.rootRotation,
        worldAngle: restWorldDeg + state.rootRotation,
        restWorldDeg,
        cumulativeDeltaDeg: state.rootRotation,
        effectiveLengthPx: Math.hypot(endPt.x - state.rootX, endPt.y - state.rootY),
        effectiveWidthScale: scaleWid,
        mode: runtime.mode,
        boneMatrix: pelvisMatrix,
      };
      continue;
    }

    const parent = computed[spec.parentId];
    let pivotX = spec.restPivot[0];
    let pivotY = spec.restPivot[1];
    if (spec.id === 'neck') {
      const neckLength = spec.restTip
        ? Math.hypot(spec.restTip[0] - spec.restPivot[0], spec.restTip[1] - spec.restPivot[1]) /
          SVG_MODEL_CONSTANTS.puppytScaleFactor
        : 0;
      const neckLift = Math.max(0, Math.min(neckLength / 2, state.neckLift ?? 0));
      pivotY -= neckLift * SVG_MODEL_CONSTANTS.puppytScaleFactor;
    } else if (spec.id === 'lip') {
      const offsetX = Math.max(
        -FACEPLATE_SLIDE_LIMIT_SVG,
        Math.min(FACEPLATE_SLIDE_LIMIT_SVG, state.faceplateOffsetX ?? 0)
      );
      const offsetY = Math.max(
        -FACEPLATE_SLIDE_LIMIT_SVG,
        Math.min(FACEPLATE_SLIDE_LIMIT_SVG, state.faceplateOffsetY ?? 0)
      );
      pivotX += offsetX * SVG_MODEL_CONSTANTS.puppytScaleFactor;
      pivotY += offsetY * SVG_MODEL_CONSTANTS.puppytScaleFactor;
    }
    const originPt = applyAffine(parent.boneMatrix, pivotX, pivotY);
    if (
      spec.id === 'neck' ||
      spec.id === 'swivelR' ||
      spec.id === 'swivelL'
    ) {
      originPt.x += neckSeparation.x;
      originPt.y += neckSeparation.y;
    }
    const restWorldDeg = spec.restTip
      ? vecAngleDeg(spec.restPivot, spec.restTip)
      : parent.restWorldDeg;

    if (spec.aimTarget) {
      const targetJoint = computed[spec.aimTarget.to];
      const aimDx = targetJoint.x - originPt.x;
      const aimDy = targetJoint.y - originPt.y;
      const dist = Math.hypot(aimDx, aimDy);
      const restDist =
        Math.hypot(
          targetJoint.spec.restPivot[0] - spec.restPivot[0],
          targetJoint.spec.restPivot[1] - spec.restPivot[1]
        ) * PUPPYT_TO_EVOLVED_SCALE;

      // Keep collarbone/aim bones rigidly locked onto their target socket so the triangle never breaks
      const worldAimDeg =
        dist > 1e-3
          ? Math.atan2(aimDy, aimDx) * RAD_TO_DEG
          : restWorldDeg + parent.cumulativeDeltaDeg;
      const cumulativeDeltaDeg = normalizeAngleDeg(worldAimDeg - restWorldDeg);
      const stretchRatio =
        restDist > 1e-3 ? dist / (restDist * Math.max(0.5, scaleWid)) : 1;
      const isStrained =
        stretchRatio < spec.aimTarget.minRatio || stretchRatio > spec.aimTarget.maxRatio;

      const dynamicLenScale =
        restDist > 1e-3 ? Math.max(0.4, Math.min(2.0, dist / restDist)) : scaleLen;

      const matrix = composeBoneMatrix(
        originPt.x,
        originPt.y,
        worldAimDeg,
        PUPPYT_TO_EVOLVED_SCALE * dynamicLenScale,
        PUPPYT_TO_EVOLVED_SCALE * scaleWid,
        restWorldDeg,
        spec.restPivot[0],
        spec.restPivot[1]
      );

      computed[spec.id] = {
        id: spec.id,
        spec,
        x: originPt.x,
        y: originPt.y,
        endX: targetJoint.x,
        endY: targetJoint.y,
        parentX: parent.x,
        parentY: parent.y,
        localAngle: 0,
        worldAngle: worldAimDeg,
        restWorldDeg,
        cumulativeDeltaDeg,
        effectiveLengthPx: dist,
        effectiveWidthScale: scaleWid,
        mode: runtime.mode,
        boneMatrix: matrix,
        isStrained,
        tensionRatio: stretchRatio,
      };
      continue;
    }

    if (spec.id === 'braceR' || spec.id === 'braceL') {
      const isRight = spec.id === 'braceR';
      const side: 'R' | 'L' = isRight ? 'R' : 'L';
      const thorax = computed.thorax;
      const collarAnchor = {
        x: thorax.x + neckSeparation.x,
        y: thorax.y + neckSeparation.y,
      };
      const lumbar = computed.lumbar;
      const acromion = computed[isRight ? 'acromionR' : 'acromionL'];
      const halfClavLen =
        Math.hypot(
          (spec.restTip?.[0] ?? spec.restPivot[0]) - spec.restPivot[0],
          (spec.restTip?.[1] ?? spec.restPivot[1]) - spec.restPivot[1]
        ) *
        PUPPYT_TO_EVOLVED_SCALE *
        COLLAR_LINK_SCALE;

      const spineDx = thorax.x - lumbar.x;
      const spineDy = thorax.y - lumbar.y;
      const spineLen = Math.max(1e-4, Math.hypot(spineDx, spineDy));
      const spineUx = spineDx / spineLen;
      const spineUy = spineDy / spineLen;
      const spineUpHint: Point2D = {
        x: thorax.x + spineDx,
        y: thorax.y + spineDy,
      };
      const acroLiftPx =
        (acromion.x - thorax.x) * spineUx + (acromion.y - thorax.y) * spineUy;
      const pairedHumerusAngle =
        state.joints[isRight ? 'humerusR' : 'humerusL']?.localAngle ?? 0;
      const armElevDeg = isRight ? pairedHumerusAngle : -pairedHumerusAngle;

      const { foldSign, effectiveLocalAngle } = resolveCollarToggleFold(
        side,
        effLocalAngle,
        armElevDeg,
        acroLiftPx
      );
      const toggle = solveToggleHinge2Link(
        collarAnchor,
        acromion,
        halfClavLen,
        foldSign,
        spineUpHint
      );

      const midX = toggle.pMid.x;
      const midY = toggle.pMid.y;

      const worldAngle = Math.atan2(midY - collarAnchor.y, midX - collarAnchor.x) * RAD_TO_DEG;
      const cumulativeDeltaDeg = normalizeAngleDeg(worldAngle - restWorldDeg);
      const matrix = composeBoneMatrix(
        collarAnchor.x,
        collarAnchor.y,
        worldAngle,
        PUPPYT_TO_EVOLVED_SCALE * COLLAR_LINK_SCALE,
        PUPPYT_TO_EVOLVED_SCALE * scaleWid,
        restWorldDeg,
        spec.restPivot[0],
        spec.restPivot[1]
      );

      computed[spec.id] = {
        id: spec.id,
        spec,
        x: collarAnchor.x,
        y: collarAnchor.y,
        endX: midX,
        endY: midY,
        parentX: parent.x,
        parentY: parent.y,
        localAngle: effectiveLocalAngle,
        worldAngle,
        restWorldDeg,
        cumulativeDeltaDeg,
        effectiveLengthPx: halfClavLen,
        effectiveWidthScale: scaleWid,
        mode: runtime.mode,
        boneMatrix: matrix,
      };
      continue;
    }

    if (spec.id === 'clavOuterR' || spec.id === 'clavOuterL') {
      const isRight = spec.id === 'clavOuterR';
      const acromion = computed[isRight ? 'acromionR' : 'acromionL'];
      const halfClavLen =
        Math.hypot(
          (spec.restTip?.[0] ?? spec.restPivot[0]) - spec.restPivot[0],
          (spec.restTip?.[1] ?? spec.restPivot[1]) - spec.restPivot[1]
        ) *
        PUPPYT_TO_EVOLVED_SCALE *
        COLLAR_LINK_SCALE;

      const worldAngle =
        Math.atan2(acromion.y - originPt.y, acromion.x - originPt.x) * RAD_TO_DEG;
      const cumulativeDeltaDeg = normalizeAngleDeg(worldAngle - restWorldDeg);
      const hingeAngle = normalizeAngleDeg(cumulativeDeltaDeg - parent.cumulativeDeltaDeg);
      const matrix = composeBoneMatrix(
        originPt.x,
        originPt.y,
        worldAngle,
        PUPPYT_TO_EVOLVED_SCALE * COLLAR_LINK_SCALE,
        PUPPYT_TO_EVOLVED_SCALE * scaleWid,
        restWorldDeg,
        spec.restPivot[0],
        spec.restPivot[1]
      );

      computed[spec.id] = {
        id: spec.id,
        spec,
        x: originPt.x,
        y: originPt.y,
        endX: acromion.x,
        endY: acromion.y,
        parentX: parent.x,
        parentY: parent.y,
        localAngle: hingeAngle,
        worldAngle,
        restWorldDeg,
        cumulativeDeltaDeg,
        effectiveLengthPx: halfClavLen,
        effectiveWidthScale: scaleWid,
        mode: runtime.mode,
        boneMatrix: matrix,
      };
      continue;
    }

    let bondedLocalAngle = effLocalAngle;
    let collarSnapState: 'normal' | 'resisting' | 'snapped' | undefined;
    let collarResistanceRatio: number | undefined;
    let armReverseFactor: number | undefined;
    let effectiveWidMultiplier = 1;

    if (spec.id === 'swivelR' || spec.id === 'swivelL') {
      const isRight = spec.id === 'swivelR';
      const side: 'R' | 'L' = isRight ? 'R' : 'L';
      const otherSwivelAngle =
        state.joints[isRight ? 'swivelL' : 'swivelR']?.localAngle ?? 0;
      const behavior = resolveCollarInwardBehavior(
        side,
        effLocalAngle,
        otherSwivelAngle
      );
      collarSnapState = behavior.snapState;
      collarResistanceRatio = behavior.resistanceRatio;

      const thorax = computed.thorax;
      const collarAnchor = {
        x: thorax.x + neckSeparation.x,
        y: thorax.y + neckSeparation.y,
      };
      const thoraxTilt = thorax ? thorax.localAngle : (state.joints.thorax?.localAngle ?? 0);
      bondedLocalAngle = behavior.effectiveLocalAngle + thoraxTilt * 0.35;

      const outwardSplay = NECK_LIFT_MAX_SWIVEL_SPLAY_DEG * easedNeckLift;
      bondedLocalAngle += isRight ? -outwardSplay : outwardSplay;
      const minLim = spec.limits?.[0] ?? (isRight ? -COLLAR_INWARD_NORMAL_LIMIT_DEG : -COLLAR_SWIVEL_MAX_INWARD_DEG);
      const maxLim = spec.limits?.[1] ?? (isRight ? COLLAR_SWIVEL_MAX_INWARD_DEG : COLLAR_INWARD_NORMAL_LIMIT_DEG);
      bondedLocalAngle = Math.max(minLim, Math.min(maxLim, bondedLocalAngle));

      // Enforce strict non-stretching rigid two-piece collarbone reach via circle-circle intersection
      const braceSpec = JOINT_SPEC_MAP[isRight ? 'braceR' : 'braceL'];
      const halfClavLen =
        Math.hypot(
          (braceSpec.restTip?.[0] ?? braceSpec.restPivot[0]) - braceSpec.restPivot[0],
          (braceSpec.restTip?.[1] ?? braceSpec.restPivot[1]) - braceSpec.restPivot[1]
        ) *
        PUPPYT_TO_EVOLVED_SCALE *
        COLLAR_LINK_SCALE;
      const maxReach = halfClavLen * 2;
      const allowedChord = maxReach;

      const swivelRadius =
        Math.hypot(
          (spec.restTip?.[0] ?? spec.restPivot[0]) - spec.restPivot[0],
          (spec.restTip?.[1] ?? spec.restPivot[1]) - spec.restPivot[1]
        ) *
        PUPPYT_TO_EVOLVED_SCALE *
        scaleLen;
      const desiredWorldDeg = restWorldDeg + parent.cumulativeDeltaDeg + bondedLocalAngle;
      const desiredWorldRad = desiredWorldDeg * DEG_TO_RAD;
      const desiredAcroX = originPt.x + swivelRadius * Math.cos(desiredWorldRad);
      const desiredAcroY = originPt.y + swivelRadius * Math.sin(desiredWorldRad);
      const desiredChordDist = Math.hypot(
        desiredAcroX - collarAnchor.x,
        desiredAcroY - collarAnchor.y
      );

      if (desiredChordDist > allowedChord + 1e-4) {
        const sx = collarAnchor.x - originPt.x;
        const sy = collarAnchor.y - originPt.y;
        const dSpine = Math.hypot(sx, sy);
        if (dSpine > 1e-4) {
          const ux = sx / dSpine;
          const uy = sy / dSpine;
          const latX = isRight ? -uy : uy;
          const latY = isRight ? ux : -ux;
          const a = Math.max(
            -swivelRadius,
            Math.min(
              swivelRadius,
              (swivelRadius * swivelRadius - allowedChord * allowedChord + dSpine * dSpine) /
                (2 * dSpine)
            )
          );
          const h = Math.sqrt(Math.max(0, swivelRadius * swivelRadius - a * a));
          const cand1X = originPt.x + a * ux + h * latX;
          const cand1Y = originPt.y + a * uy + h * latY;
          const cand2X = originPt.x + a * ux - h * latX;
          const cand2Y = originPt.y + a * uy - h * latY;
          const dist1 = Math.hypot(cand1X - desiredAcroX, cand1Y - desiredAcroY);
          const dist2 = Math.hypot(cand2X - desiredAcroX, cand2Y - desiredAcroY);
          const clampedAcroX = dist1 <= dist2 ? cand1X : cand2X;
          const clampedAcroY = dist1 <= dist2 ? cand1Y : cand2Y;
          const clampedWorldDeg =
            Math.atan2(clampedAcroY - originPt.y, clampedAcroX - originPt.x) * RAD_TO_DEG;
          bondedLocalAngle = normalizeAngleDeg(
            clampedWorldDeg - restWorldDeg - parent.cumulativeDeltaDeg
          );
        }
      }
    } else if (spec.id === 'hipR' || spec.id === 'hipL') {
      bondedLocalAngle = spec.id === 'hipR' ? resolvedHipR : resolvedHipL;
    } else if (spec.id === 'femurR' || spec.id === 'femurL') {
      const appliedExt = spec.id === 'femurR' ? appliedHipExtR : appliedHipExtL;
      bondedLocalAngle = effLocalAngle - appliedExt;
    } else if (
      spec.id === 'humerusR' ||
      spec.id === 'forearmR' ||
      spec.id === 'handR' ||
      spec.id === 'thumbR' ||
      spec.id === 'fingerR'
    ) {
      armReverseFactor = armFlipCosR;
      effectiveWidMultiplier = armVisualFlipR;
      if (spec.id === 'thumbR' || spec.id === 'fingerR') {
        bondedLocalAngle = effLocalAngle * armFlipCosR;
      }
    } else if (
      spec.id === 'humerusL' ||
      spec.id === 'forearmL' ||
      spec.id === 'handL' ||
      spec.id === 'thumbL' ||
      spec.id === 'fingerL'
    ) {
      armReverseFactor = armFlipCosL;
      effectiveWidMultiplier = armVisualFlipL;
      if (spec.id === 'thumbL' || spec.id === 'fingerL') {
        bondedLocalAngle = effLocalAngle * armFlipCosL;
      }
    }

    const cumulativeDeltaDeg = parent.cumulativeDeltaDeg + bondedLocalAngle;
    const worldAngle = restWorldDeg + cumulativeDeltaDeg;

    const matrix = composeBoneMatrix(
      originPt.x,
      originPt.y,
      worldAngle,
      PUPPYT_TO_EVOLVED_SCALE * scaleLen,
      PUPPYT_TO_EVOLVED_SCALE * scaleWid * effectiveWidMultiplier,
      restWorldDeg,
      spec.restPivot[0],
      spec.restPivot[1]
    );

    const endPt = spec.restTip
      ? applyAffine(matrix, spec.restTip[0], spec.restTip[1])
      : { x: originPt.x, y: originPt.y };

    computed[spec.id] = {
      id: spec.id,
      spec,
      x: originPt.x,
      y: originPt.y,
      endX: endPt.x,
      endY: endPt.y,
      parentX: parent.x,
      parentY: parent.y,
      localAngle: bondedLocalAngle,
      worldAngle,
      restWorldDeg,
      cumulativeDeltaDeg,
      effectiveLengthPx: Math.hypot(endPt.x - originPt.x, endPt.y - originPt.y),
      effectiveWidthScale: scaleWid,
      mode: runtime.mode,
      boneMatrix: matrix,
      ...(collarSnapState !== undefined ? { collarSnapState } : {}),
      ...(collarResistanceRatio !== undefined ? { collarResistanceRatio } : {}),
      ...(spec.id === 'neck' ? { neckCollarPressureRatio } : {}),
      ...(armReverseFactor !== undefined ? { armReverseFactor } : {}),
    };
  }

  return computed;
}

export function getEnvironmentalAnchorPoint(
  computed: Record<JointId, ComputedJoint>,
  anchor: EnvironmentalAnchorId
): Point2D {
  switch (anchor) {
    case 'bottom_of_spine':
      return { x: computed.pelvis.x, y: computed.pelvis.y };
    case 'feet':
      return {
        x: (computed.footR.endX + computed.footL.endX) / 2,
        y: Math.max(computed.footR.endY, computed.footL.endY),
      };
    case 'navel':
      return { x: computed.lumbar.x, y: computed.lumbar.y };
    case 'clavicle':
      return { x: computed.thorax.x, y: computed.thorax.y };
    case 'neck':
      return { x: computed.neck.x, y: computed.neck.y };
    case 'crown':
      return { x: computed.head.endX, y: computed.head.endY };
  }
}

export function computeArmature(
  state: ArmatureState,
  canvasOrigin: Point2D = { x: PAPER_WIDTH / 2, y: PAPER_HEIGHT / 2 }
): Record<JointId, ComputedJoint> {
  const raw = evaluateRawArmature(state);
  if (!state.isAnchorPinned) {
    return raw;
  }

  const anchorPt = getEnvironmentalAnchorPoint(raw, state.environmentalAnchor);
  const shiftX = canvasOrigin.x - anchorPt.x;
  const shiftY = canvasOrigin.y - anchorPt.y;

  const pinned = {} as Record<JointId, ComputedJoint>;
  for (const spec of JOINT_SPECS) {
    const j = raw[spec.id];
    const m = j.boneMatrix;
    pinned[spec.id] = {
      ...j,
      x: j.x + shiftX,
      y: j.y + shiftY,
      endX: j.endX + shiftX,
      endY: j.endY + shiftY,
      parentX: j.parentX !== null ? j.parentX + shiftX : null,
      parentY: j.parentY !== null ? j.parentY + shiftY : null,
      boneMatrix: [m[0], m[1], m[2], m[3], m[4] + shiftX, m[5] + shiftY],
    };
  }
  return pinned;
}

export function computeTorsoCenter(computed: Record<JointId, ComputedJoint>): Point2D {
  const lumbarSvgMatrix = computeSvgBoneMatrix(computed.lumbar.boneMatrix);
  return applyAffine(lumbarSvgMatrix, TORSO_HANDLE_CENTER_SVG[0], TORSO_HANDLE_CENTER_SVG[1]);
}

export interface BodyBalanceCenters {
  upper: Point2D;
  lower: Point2D;
  whole: Point2D;
  upperMass: number;
  lowerMass: number;
}

const UPPER_BALANCE_SEGMENTS: readonly JointId[] = [
  'lumbar',
  'thorax',
  'ribR',
  'ribL',
  'neck',
  'head',
  'humerusR',
  'forearmR',
  'handR',
  'humerusL',
  'forearmL',
  'handL',
];

const LOWER_BALANCE_SEGMENTS: readonly JointId[] = [
  'pelvis',
  'hipR',
  'femurR',
  'shinR',
  'footR',
  'hipL',
  'femurL',
  'shinL',
  'footL',
];

function getWeightedSegmentCenter(
  computed: Record<JointId, ComputedJoint>,
  jointIds: readonly JointId[]
): { center: Point2D; mass: number } {
  let mass = 0;
  let x = 0;
  let y = 0;

  for (const id of jointIds) {
    const joint = computed[id];
    const segmentMass =
      Math.max(0, joint.effectiveLengthPx) * Math.max(0.1, joint.effectiveWidthScale);
    if (segmentMass <= 0) continue;

    mass += segmentMass;
    x += ((joint.x + joint.endX) / 2) * segmentMass;
    y += ((joint.y + joint.endY) / 2) * segmentMass;
  }

  return {
    center: mass > 0 ? { x: x / mass, y: y / mass } : { x: 0, y: 0 },
    mass,
  };
}

export function computeBodyBalanceCenters(
  computed: Record<JointId, ComputedJoint>
): BodyBalanceCenters {
  const upper = getWeightedSegmentCenter(computed, UPPER_BALANCE_SEGMENTS);
  const lower = getWeightedSegmentCenter(computed, LOWER_BALANCE_SEGMENTS);
  const totalMass = upper.mass + lower.mass;
  return {
    upper: upper.center,
    lower: lower.center,
    whole:
      totalMass > 0
        ? {
            x: (upper.center.x * upper.mass + lower.center.x * lower.mass) / totalMass,
            y: (upper.center.y * upper.mass + lower.center.y * lower.mass) / totalMass,
          }
        : { x: 0, y: 0 },
    upperMass: upper.mass,
    lowerMass: lower.mass,
  };
}

export function solveTwoBoneIK(
  state: ArmatureState,
  endId: JointId,
  targetX: number,
  targetY: number,
  canvasOrigin: Point2D = { x: PAPER_WIDTH / 2, y: PAPER_HEIGHT / 2 },
  options?: { softDamp?: boolean; constrainLimits?: boolean }
): ArmatureState {
  const endSpec = JOINT_SPEC_MAP[endId];
  if (!endSpec.ikChain || typeof endSpec.ikChain === 'string') return state;
  const chainArray: readonly JointId[] = endSpec.ikChain;
  if (chainArray.length < 2) return state;

  const rootId: JointId = chainArray[0];
  const midId: JointId = chainArray.length >= 3 ? chainArray[1] : chainArray[0];

  const runPass = (curState: ArmatureState): ArmatureState => {
    const computed = computeArmature(curState, canvasOrigin);
    const rootJoint = computed[rootId];
    const midJoint = computed[midId];
    const endJoint = computed[endId];

    const L1 = Math.max(1, Math.hypot(midJoint.x - rootJoint.x, midJoint.y - rootJoint.y));
    const L2 = Math.max(1, Math.hypot(endJoint.x - midJoint.x, endJoint.y - midJoint.y));

    const dx = targetX - rootJoint.x;
    const dy = targetY - rootJoint.y;
    const rawDist = Math.hypot(dx, dy);

    // Soft-IK reach damping to avoid abrupt joint popping at maximum limb extension
    const totalLen = L1 + L2;
    const softness = totalLen * 0.12;
    const softStart = totalLen - softness;
    let d = Math.max(Math.abs(L1 - L2) + 2, rawDist);

    if ((options?.softDamp ?? true) && rawDist > softStart) {
      const u = Math.min(1.4, (rawDist - softStart) / softness);
      d = softStart + softness * (1 - Math.exp(-u));
    }
    d = Math.min(totalLen - 0.5, d);

    const baseAngle = Math.atan2(dy, dx);
    // Procedural singularity epsilon to keep Law-of-Cosines strictly bounded
    const safeCosMax = 0.99985;
    const cosAlpha = Math.max(-safeCosMax, Math.min(safeCosMax, (L1 * L1 + d * d - L2 * L2) / (2 * L1 * d)));
    const alpha = Math.acos(cosAlpha);
    const cosBeta = Math.max(-safeCosMax, Math.min(safeCosMax, (L1 * L1 + L2 * L2 - d * d) / (2 * L1 * L2)));
    const beta = Math.acos(cosBeta);

    const armFlipDeg =
      rootId === 'humerusR'
        ? (curState.armSwivelR ?? 0)
        : rootId === 'humerusL'
        ? (curState.armSwivelL ?? 0)
        : 0;
    const armFlipSign = Math.cos(armFlipDeg * DEG_TO_RAD) >= 0 ? 1 : -1;
    const bendDir = ((endSpec.bendDirection ?? 1) * armFlipSign) as 1 | -1;
    const desiredWorld1 = ((baseAngle + bendDir * alpha) * 180) / Math.PI;
    const desiredWorld2 =
      ((baseAngle + bendDir * alpha - bendDir * (Math.PI - beta)) * 180) / Math.PI;

    const multRoot = getPhysicsMultiplier(curState.joints[rootId]);
    const multMid = getPhysicsMultiplier(curState.joints[midId]);

    const rawDeltaRoot = shortestAngleDelta(rootJoint.worldAngle, desiredWorld1);
    let nextRootLocal = applyJointAngleConstraint(
      curState.joints[rootId].localAngle + rawDeltaRoot * multRoot,
      curState.continuousSpin
    );
    if (options?.constrainLimits && JOINT_SPEC_MAP[rootId]?.limits) {
      const lim = JOINT_SPEC_MAP[rootId].limits!;
      nextRootLocal = Math.max(lim[0], Math.min(lim[1], nextRootLocal));
    }

    const appliedDeltaRoot = nextRootLocal - curState.joints[rootId].localAngle;
    const midWorldAfterRoot = midJoint.worldAngle + appliedDeltaRoot;
    const rawDeltaMid = shortestAngleDelta(midWorldAfterRoot, desiredWorld2);
    let nextMidLocal = applyJointAngleConstraint(
      curState.joints[midId].localAngle + rawDeltaMid * multMid,
      curState.continuousSpin
    );
    if (options?.constrainLimits && JOINT_SPEC_MAP[midId]?.limits) {
      const rawLim = JOINT_SPEC_MAP[midId].limits!;
      const lim =
        armFlipSign < 0 && (midId === 'forearmR' || midId === 'forearmL')
          ? ([-rawLim[1], -rawLim[0]] as const)
          : rawLim;
      nextMidLocal = Math.max(lim[0], Math.min(lim[1], nextMidLocal));
    }

    return {
      ...curState,
      joints: {
        ...curState.joints,
        [rootId]: { ...curState.joints[rootId], localAngle: nextRootLocal },
        [midId]: { ...curState.joints[midId], localAngle: nextMidLocal },
      },
    };
  };

  let solved = runPass(state);
  if (rootId === 'femurR' || rootId === 'femurL') {
    solved = runPass(solved);
    solved = runPass(solved);
  }
  return solved;
}

export function solveSingleBoneAimIK(
  state: ArmatureState,
  jointId: JointId,
  targetX: number,
  targetY: number,
  canvasOrigin: Point2D = { x: PAPER_WIDTH / 2, y: PAPER_HEIGHT / 2 },
  options?: { constrainLimits?: boolean }
): ArmatureState {
  const spec = JOINT_SPEC_MAP[jointId];
  if (!spec) return state;
  const parentId = spec.parentId;
  if (!parentId) return state;
  const parentSpec = JOINT_SPEC_MAP[parentId];
  if (!parentSpec || parentSpec.locked) return state;

  const computed = computeArmature(state, canvasOrigin);
  const parentJoint = computed[parentId];
  const currentJoint = computed[jointId];

  const dx = targetX - parentJoint.x;
  const dy = targetY - parentJoint.y;
  const dist = Math.hypot(dx, dy);
  if (dist < 1e-3) return state;

  const targetWorldDeg = (Math.atan2(dy, dx) * 180) / Math.PI;
  const mult = getPhysicsMultiplier(state.joints[parentId]);
  if (mult === 0) return state;

  const rawDelta = shortestAngleDelta(currentJoint.worldAngle, targetWorldDeg);
  let nextLocal = applyJointAngleConstraint(
    state.joints[parentId].localAngle + rawDelta * mult,
    state.continuousSpin
  );

  if (options?.constrainLimits && parentSpec.limits) {
    nextLocal = Math.max(parentSpec.limits[0], Math.min(parentSpec.limits[1], nextLocal));
  }

  return {
    ...state,
    joints: {
      ...state.joints,
      [parentId]: { ...state.joints[parentId], localAngle: nextLocal },
    },
  };
}

export function solveSpineCCDIK(
  state: ArmatureState,
  targetX: number,
  targetY: number,
  canvasOrigin: Point2D = { x: PAPER_WIDTH / 2, y: PAPER_HEIGHT / 2 },
  targetJointId: JointId = 'head'
): ArmatureState {
  const fullChain: JointId[] = ['lumbar', 'thorax', 'neck', 'head'];
  const chain: JointId[] =
    targetJointId === 'thorax'
      ? ['lumbar', 'thorax']
      : targetJointId === 'neck'
      ? ['lumbar', 'thorax', 'neck']
      : fullChain;

  let current: ArmatureState = {
    ...state,
    joints: { ...state.joints },
  };

  const endJointId = chain[chain.length - 1];

  for (let iter = 0; iter < 10; iter++) {
    for (let i = chain.length - 1; i >= 0; i--) {
      const jointId = chain[i];
      const runtime = current.joints[jointId];
      if (runtime.mode === 'PIN' || runtime.physicsMode === 'rigid') continue;

      const computed = computeArmature(current, canvasOrigin);
      const pivot = computed[jointId];
      const effector = computed[endJointId];
      const effectorX = effector.endX ?? effector.x;
      const effectorY = effector.endY ?? effector.y;

      const aEff = Math.atan2(effectorY - pivot.y, effectorX - pivot.x);
      const aTgt = Math.atan2(targetY - pivot.y, targetX - pivot.x);
      let deltaDeg = ((aTgt - aEff) * 180) / Math.PI;
      deltaDeg = normalizeAngleDeg(deltaDeg);
      deltaDeg = Math.max(-18, Math.min(18, deltaDeg * 0.65)) * getPhysicsMultiplier(runtime);

      const nextAngle = applyJointAngleConstraint(
        runtime.localAngle + deltaDeg,
        current.continuousSpin
      );

      current = {
        ...current,
        joints: {
          ...current.joints,
          [jointId]: {
            ...runtime,
            localAngle: nextAngle,
          },
        },
      };
    }
  }

  return current;
}

const FACEPLATE_LOOK_CHAIN = [
  { id: 'head', weight: 0.42, maxDelta: 30 },
  { id: 'neck', weight: 0.28, maxDelta: 22 },
  { id: 'thorax', weight: 0.18, maxDelta: 15 },
  { id: 'lumbar', weight: 0.12, maxDelta: 10 },
] as const satisfies readonly { id: JointId; weight: number; maxDelta: number }[];

const FACEPLATE_SUPPORT_JOINTS = [
  { id: 'braceR', weight: 0.015, maxDelta: 2 },
  { id: 'braceL', weight: 0.015, maxDelta: 2 },
  { id: 'clavOuterR', weight: 0.015, maxDelta: 2 },
  { id: 'clavOuterL', weight: 0.015, maxDelta: 2 },
  { id: 'humerusR', weight: 0.01, maxDelta: 2 },
  { id: 'humerusL', weight: 0.01, maxDelta: 2 },
  { id: 'hipR', weight: 0.01, maxDelta: 1.5 },
  { id: 'hipL', weight: 0.01, maxDelta: 1.5 },
] as const satisfies readonly { id: JointId; weight: number; maxDelta: number }[];

export type FaceplateLookCollarHold = Partial<
  Record<'braceR' | 'braceL' | 'clavOuterR' | 'clavOuterL', number>
>;

function stabilizeLookCollars(
  baseline: ArmatureState,
  solved: ArmatureState,
  hold?: FaceplateLookCollarHold
): ArmatureState {
  if (!hold) return solved;
  let current = solved;
  for (const id of ['braceR', 'braceL', 'clavOuterR', 'clavOuterL'] as const) {
    const heldAngle = hold[id];
    const runtime = current.joints[id];
    if (
      heldAngle === undefined ||
      runtime.mode === 'PIN' ||
      runtime.physicsMode === 'rigid'
    ) {
      continue;
    }

    const baselineAngle = baseline.joints[id].localAngle;
    const heldDelta = shortestAngleDelta(baselineAngle, heldAngle);
    const candidateDelta = shortestAngleDelta(baselineAngle, runtime.localAngle);
    const retreating =
      heldDelta * candidateDelta >= 0 && Math.abs(candidateDelta) < Math.abs(heldDelta);
    const difference = shortestAngleDelta(heldAngle, runtime.localAngle);
    const response = retreating
      ? 0.16 + 0.16 * Math.tanh(Math.abs(difference) / 8)
      : 0.52 + 0.38 * Math.tanh(Math.abs(difference) / 8);
    let localAngle = heldAngle + difference * response;

    const [minAngle, maxAngle] = JOINT_SPEC_MAP[id].limits ?? [-180, 180];
    localAngle = Math.max(minAngle, Math.min(maxAngle, localAngle));
    current = {
      ...current,
      joints: {
        ...current.joints,
        [id]: { ...runtime, localAngle },
      },
    };
  }
  return current;
}

export function setFaceplateOffset(
  state: ArmatureState,
  offsetX: number,
  offsetY: number
): ArmatureState {
  return {
    ...state,
    faceplateOffsetX: Math.max(-FACEPLATE_SLIDE_LIMIT_SVG, Math.min(FACEPLATE_SLIDE_LIMIT_SVG, offsetX)),
    faceplateOffsetY: Math.max(-FACEPLATE_SLIDE_LIMIT_SVG, Math.min(FACEPLATE_SLIDE_LIMIT_SVG, offsetY)),
  };
}

export function solveFaceplateLookIK(
  state: ArmatureState,
  targetX: number,
  targetY: number,
  canvasOrigin: Point2D = { x: PAPER_WIDTH / 2, y: PAPER_HEIGHT / 2 },
  collarHold?: FaceplateLookCollarHold
): ArmatureState {
  const initial = computeArmature(state, canvasOrigin);
  const initialFaceplate = initial.lip;
  const headMatrix = initial.head.boneMatrix;
  const determinant = headMatrix[0] * headMatrix[3] - headMatrix[1] * headMatrix[2];
  const worldDeltaX = targetX - initialFaceplate.x;
  const worldDeltaY = targetY - initialFaceplate.y;
  const localDeltaX =
    Math.abs(determinant) > 1e-8
      ? (headMatrix[3] * worldDeltaX - headMatrix[2] * worldDeltaY) / determinant
      : 0;
  const localDeltaY =
    Math.abs(determinant) > 1e-8
      ? (-headMatrix[1] * worldDeltaX + headMatrix[0] * worldDeltaY) / determinant
      : 0;
  const scaleFactor = SVG_MODEL_CONSTANTS.puppytScaleFactor;
  let current = setFaceplateOffset(
    state,
    (state.faceplateOffsetX ?? 0) + localDeltaX / scaleFactor,
    (state.faceplateOffsetY ?? 0) + localDeltaY / scaleFactor
  );
  const slidFaceplate = computeArmature(current, canvasOrigin).lip;
  const thoraxMatrix = initial.thorax.boneMatrix;
  const thoraxDeterminant =
    thoraxMatrix[0] * thoraxMatrix[3] - thoraxMatrix[1] * thoraxMatrix[2];
  const remainingX = targetX - slidFaceplate.x;
  const remainingY = targetY - slidFaceplate.y;
  const remainingLocalY =
    Math.abs(thoraxDeterminant) > 1e-8
      ? (-thoraxMatrix[1] * remainingX + thoraxMatrix[0] * remainingY) / thoraxDeterminant
      : 0;
  const neckSpec = JOINT_SPEC_MAP.neck;
  const neckLengthSvg = neckSpec.restTip
    ? Math.hypot(
        neckSpec.restTip[0] - neckSpec.restPivot[0],
        neckSpec.restTip[1] - neckSpec.restPivot[1]
      ) / scaleFactor
    : 0;
  current = {
    ...current,
    neckLift: Math.max(
      0,
      Math.min(
        neckLengthSvg / 2,
        (state.neckLift ?? 0) - (remainingLocalY / scaleFactor) * 0.35
      )
    ),
  };
  current = setJointMode(current, 'lip', 'IK');

  for (let iteration = 0; iteration < 12; iteration++) {
    for (const { id, weight, maxDelta } of FACEPLATE_LOOK_CHAIN) {
      const runtime = current.joints[id];
      const baseline = state.joints[id];
      if (runtime.mode === 'PIN' || runtime.physicsMode === 'rigid') continue;

      const computed = computeArmature(current, canvasOrigin);
      const pivot = computed[id];
      const faceplate = computed.lip;
      const currentAngle =
        (Math.atan2(faceplate.y - pivot.y, faceplate.x - pivot.x) * 180) / Math.PI;
      const targetAngle = (Math.atan2(targetY - pivot.y, targetX - pivot.x) * 180) / Math.PI;
      const delta = shortestAngleDelta(currentAngle, targetAngle);
      const step = Math.max(
        -7,
        Math.min(7, delta * weight * getPhysicsMultiplier(runtime))
      );
      if (Math.abs(step) < 0.02) continue;

      const specLimits = JOINT_SPEC_MAP[id]?.limits ?? [-180, 180];
      const minAngle = Math.max(specLimits[0], baseline.localAngle - maxDelta);
      const maxAngle = Math.min(specLimits[1], baseline.localAngle + maxDelta);
      if (minAngle > maxAngle) continue;

      const localAngle = Math.max(
        minAngle,
        Math.min(
          maxAngle,
          applyJointAngleConstraint(runtime.localAngle + step, current.continuousSpin)
        )
      );
      current = {
        ...current,
        joints: {
          ...current.joints,
          [id]: { ...runtime, localAngle },
        },
      };
    }

    const faceplate = computeArmature(current, canvasOrigin).lip;
    if (Math.hypot(targetX - faceplate.x, targetY - faceplate.y) < 1.5) break;
  }

  const spineMotion = FACEPLATE_LOOK_CHAIN.reduce(
    (total, { id }) =>
      total + Math.abs(shortestAngleDelta(state.joints[id].localAngle, current.joints[id].localAngle)),
    0
  );
  const signedMotion = FACEPLATE_LOOK_CHAIN.reduce(
    (total, { id, weight }) =>
      total + shortestAngleDelta(state.joints[id].localAngle, current.joints[id].localAngle) * weight,
    0
  );
  if (spineMotion < 1e-4 || Math.abs(signedMotion) < 1e-4) {
    return stabilizeLookCollars(state, current, collarHold);
  }

  const supportDirection = Math.sign(signedMotion);
  for (const { id, weight, maxDelta } of FACEPLATE_SUPPORT_JOINTS) {
    const runtime = current.joints[id];
    const baseline = state.joints[id];
    if (runtime.mode === 'PIN' || runtime.physicsMode === 'rigid') continue;

    const specLimits = JOINT_SPEC_MAP[id]?.limits ?? [-180, 180];
    const response = Math.max(
      -maxDelta,
      Math.min(maxDelta, supportDirection * spineMotion * weight * getPhysicsMultiplier(runtime))
    );
    const localAngle = Math.max(
      specLimits[0],
      Math.min(specLimits[1], baseline.localAngle + response)
    );
    current = {
      ...current,
      joints: {
        ...current.joints,
        [id]: { ...runtime, localAngle },
      },
    };
  }

  return stabilizeLookCollars(state, current, collarHold);
}

export function solveDigitIK(
  state: ArmatureState,
  tipId: JointId,
  targetX: number,
  targetY: number,
  canvasOrigin: Point2D = { x: PAPER_WIDTH / 2, y: PAPER_HEIGHT / 2 }
): ArmatureState {
  return solveTwoBoneIK(state, tipId, targetX, targetY, canvasOrigin);
}

export function solveFABRIKChainIK(
  state: ArmatureState,
  chainIds: JointId[],
  targetX: number,
  targetY: number,
  canvasOrigin: Point2D = { x: PAPER_WIDTH / 2, y: PAPER_HEIGHT / 2 },
  options?: { iterations?: number; tolerance?: number; constrainLimits?: boolean }
): ArmatureState {
  if (chainIds.length < 2) return state;
  const iters = options?.iterations ?? 8;
  const tol = options?.tolerance ?? 1.5;

  let current = { ...state, joints: { ...state.joints } };
  const targetPt: Point2D = { x: targetX, y: targetY };

  for (let it = 0; it < iters; it++) {
    const computed = computeArmature(current, canvasOrigin);
    const endJoint = computed[chainIds[chainIds.length - 1]];
    const distToEnd = Math.hypot(endJoint.x - targetPt.x, endJoint.y - targetPt.y);
    if (distToEnd < tol) break;

    // Backward step: from end toward root
    for (let i = chainIds.length - 1; i >= 1; i--) {
      const childId = chainIds[i];
      const parentId = chainIds[i - 1];
      const childJ = computed[childId];
      const parentJ = computed[parentId];

      const dx = targetPt.x - parentJ.x;
      const dy = targetPt.y - parentJ.y;
      if (Math.hypot(dx, dy) > 1e-3) {
        const desiredAngle = (Math.atan2(dy, dx) * 180) / Math.PI;
        const delta = shortestAngleDelta(parentJ.worldAngle, desiredAngle);
        const runtime = current.joints[parentId];
        if (runtime && runtime.mode !== 'PIN') {
          const mult = getPhysicsMultiplier(runtime);
          let nextLocal = applyJointAngleConstraint(
            runtime.localAngle + delta * mult * 0.45,
            current.continuousSpin
          );
          if (options?.constrainLimits && JOINT_SPEC_MAP[parentId]?.limits) {
            const lim = JOINT_SPEC_MAP[parentId].limits!;
            nextLocal = Math.max(lim[0], Math.min(lim[1], nextLocal));
          }
          current.joints[parentId] = { ...runtime, localAngle: nextLocal };
        }
      }
    }
  }

  return current;
}

export function solveArmTipIK(
  state: ArmatureState,
  tipId: 'fingerTipR' | 'fingerTipL' | 'fingerR' | 'fingerL',
  targetX: number,
  targetY: number,
  canvasOrigin: Point2D = { x: PAPER_WIDTH / 2, y: PAPER_HEIGHT / 2 },
  options?: {
    solver?: 'analytical' | 'fabrik' | 'ccd';
    softDamp?: boolean;
    constrainLimits?: boolean;
  }
): ArmatureState {
  const isRight = tipId === 'fingerTipR' || tipId === 'fingerR';
  const humerusId: JointId = isRight ? 'humerusR' : 'humerusL';
  const forearmId: JointId = isRight ? 'forearmR' : 'forearmL';
  const handId: JointId = isRight ? 'handR' : 'handL';
  const fingerId: JointId = isRight ? 'fingerR' : 'fingerL';
  const fingerTipId: JointId = isRight ? 'fingerTipR' : 'fingerTipL';

  let current: ArmatureState = {
    ...state,
    joints: { ...state.joints },
  };

  const initialComp = computeArmature(current, canvasOrigin);
  const hand0 = initialComp[handId];
  const tip0 = initialComp[fingerTipId];
  const handSpanPx = Math.max(1, Math.hypot(tip0.x - hand0.x, tip0.y - hand0.y));

  for (let pass = 0; pass < 3; pass++) {
    const comp = computeArmature(current, canvasOrigin);
    const wrist = comp[handId];
    const shoulder = comp[humerusId];

    let dx = targetX - wrist.x;
    let dy = targetY - wrist.y;
    let dist = Math.hypot(dx, dy);
    if (dist < 1e-3) {
      dx = targetX - shoulder.x;
      dy = targetY - shoulder.y;
      dist = Math.hypot(dx, dy);
    }
    const dirX = dist > 1e-4 ? dx / dist : isRight ? -1 : 1;
    const dirY = dist > 1e-4 ? dy / dist : 0;

    const wristTargetX = targetX - dirX * handSpanPx;
    const wristTargetY = targetY - dirY * handSpanPx;

    if (
      options?.solver === 'fabrik' &&
      Array.isArray(JOINT_SPEC_MAP[handId].ikChain)
    ) {
      current = solveFABRIKChainIK(
        current,
        [...(JOINT_SPEC_MAP[handId].ikChain as readonly JointId[])],
        wristTargetX,
        wristTargetY,
        canvasOrigin,
        options
      );
    } else {
      current = solveTwoBoneIK(
        current,
        handId,
        wristTargetX,
        wristTargetY,
        canvasOrigin,
        options
      );
    }

    const afterArmComp = computeArmature(current, canvasOrigin);
    const wristAfter = afterArmComp[handId];
    const tipAfter = afterArmComp[fingerTipId];
    const curTipVecX = tipAfter.x - wristAfter.x;
    const curTipVecY = tipAfter.y - wristAfter.y;
    const targetVecX = targetX - wristAfter.x;
    const targetVecY = targetY - wristAfter.y;

    if (Math.hypot(curTipVecX, curTipVecY) > 1e-3 && Math.hypot(targetVecX, targetVecY) > 1e-3) {
      const curDeg = Math.atan2(curTipVecY, curTipVecX) * RAD_TO_DEG;
      const desiredDeg = Math.atan2(targetVecY, targetVecX) * RAD_TO_DEG;
      const deltaDeg = shortestAngleDelta(curDeg, desiredDeg);

      const handRt = current.joints[handId];
      if (handRt && handRt.mode !== 'PIN' && handRt.physicsMode !== 'rigid') {
        const multHand = getPhysicsMultiplier(handRt);
        let nextHandLocal = applyJointAngleConstraint(
          handRt.localAngle + deltaDeg * multHand,
          current.continuousSpin
        );
        if (options?.constrainLimits && JOINT_SPEC_MAP[handId]?.limits) {
          const lim = JOINT_SPEC_MAP[handId].limits!;
          nextHandLocal = Math.max(lim[0], Math.min(lim[1], nextHandLocal));
        }
        current = {
          ...current,
          joints: {
            ...current.joints,
            [handId]: { ...handRt, localAngle: nextHandLocal },
          },
        };
      }

      // Apply any remaining fingertip precision angle onto finger joint (e.g. if wrist hits limit)
      const afterHandComp = computeArmature(current, canvasOrigin);
      const wristAfterHand = afterHandComp[handId];
      const tipAfterHand = afterHandComp[fingerTipId];
      const remCurDeg =
        Math.atan2(tipAfterHand.y - wristAfterHand.y, tipAfterHand.x - wristAfterHand.x) *
        RAD_TO_DEG;
      const remDesiredDeg =
        Math.atan2(targetY - wristAfterHand.y, targetX - wristAfterHand.x) * RAD_TO_DEG;
      const remDelta = shortestAngleDelta(remCurDeg, remDesiredDeg);
      const fingerRt = current.joints[fingerId];
      if (
        Math.abs(remDelta) > 0.1 &&
        fingerRt &&
        fingerRt.mode !== 'PIN' &&
        fingerRt.physicsMode !== 'rigid'
      ) {
        const multFinger = getPhysicsMultiplier(fingerRt);
        let nextFingerLocal = applyJointAngleConstraint(
          fingerRt.localAngle + remDelta * multFinger,
          current.continuousSpin
        );
        if (options?.constrainLimits && JOINT_SPEC_MAP[fingerId]?.limits) {
          const lim = JOINT_SPEC_MAP[fingerId].limits!;
          nextFingerLocal = Math.max(lim[0], Math.min(lim[1], nextFingerLocal));
        }
        current = {
          ...current,
          joints: {
            ...current.joints,
            [fingerId]: { ...fingerRt, localAngle: nextFingerLocal },
          },
        };
      }
    }

    // Keep humerus/forearm mode active when inspected
    void forearmId;
  }

  return current;
}

export function solvePlanquetteArmReach(
  startState: ArmatureState,
  effectorId: 'handR' | 'handL' | 'fingerTipR' | 'fingerTipL',
  targetX: number,
  targetY: number,
  canvasOrigin: Point2D = { x: PAPER_WIDTH / 2, y: PAPER_HEIGHT / 2 },
  options?: {
    solver?: 'analytical' | 'fabrik' | 'ccd';
    softDamp?: boolean;
    constrainLimits?: boolean;
  }
): ArmatureState {
  const isRight = effectorId.endsWith('R');
  const shoulderId: JointId = isRight ? 'humerusR' : 'humerusL';
  const startComp = computeArmature(startState, canvasOrigin);
  const startShoulder = startComp[shoulderId];
  const startEff = startComp[effectorId];
  const armReach = Math.max(
    180,
    Math.hypot(startEff.x - startShoulder.x, startEff.y - startShoulder.y)
  );
  const dx = targetX - startShoulder.x;
  const distFromStartShoulder = Math.hypot(dx, targetY - startShoulder.y);

  const direct = solveJointIK(startState, effectorId, targetX, targetY, canvasOrigin, {
    ...options,
    softDamp: distFromStartShoulder < armReach * 0.88 ? options?.softDamp : false,
  });
  const directComp = computeArmature(direct, canvasOrigin);
  const directEff = directComp[effectorId];
  const directErr = Math.hypot(targetX - directEff.x, targetY - directEff.y);
  if (directErr <= 2 && distFromStartShoulder <= armReach * 0.92) {
    return direct;
  }

  // Recruit upper spine lean + grounded waist sway smoothly toward the target so both wrists and fingertips reach every arc letter
  const excess = Math.max(0, distFromStartShoulder - armReach * 0.88);
  const leanRatio = Math.min(1, excess / 150);
  const leanSign = Math.abs(dx) < 1e-3 ? 0 : Math.sign(dx);
  const leanDeg = leanSign * Math.min(44, 44 * leanRatio);

  let leaned: ArmatureState = {
    ...startState,
    joints: { ...startState.joints },
  };

  if (leanRatio > 0 && leanSign !== 0) {
    const lumbarRt = leaned.joints.lumbar;
    const thoraxRt = leaned.joints.thorax;
    if (lumbarRt && lumbarRt.mode !== 'PIN' && lumbarRt.physicsMode !== 'rigid') {
      leaned.joints.lumbar = {
        ...lumbarRt,
        localAngle: Math.max(-45, Math.min(45, lumbarRt.localAngle + leanDeg * 0.55)),
      };
    }
    if (thoraxRt && thoraxRt.mode !== 'PIN' && thoraxRt.physicsMode !== 'rigid') {
      leaned.joints.thorax = {
        ...thoraxRt,
        localAngle: Math.max(-45, Math.min(45, thoraxRt.localAngle + leanDeg * 0.45)),
      };
    }
  }

  const afterLeanComp = computeArmature(leaned, canvasOrigin);
  const leanShoulder = afterLeanComp[shoulderId];
  const remDx = targetX - leanShoulder.x;
  const remainingDist = Math.hypot(remDx, targetY - leanShoulder.y);

  if (remainingDist > armReach * 0.9) {
    const pelvis = afterLeanComp.pelvis;
    const swayRatio = Math.min(1, (remainingDist - armReach * 0.9) / 110);
    const clampedShiftX = Math.max(-220, Math.min(220, remDx * 0.9 * swayRatio));
    const shiftY = Math.min(24, Math.abs(clampedShiftX) * 0.1);

    const groundY = canvasOrigin.y + computeProportionalCanons().eightHeadTotalPx / 2;
    leaned = solveWaistBalanceHula(
      leaned,
      pelvis.x + clampedShiftX,
      pelvis.y + shiftY,
      getGroundContactPin(afterLeanComp.footR, afterLeanComp.toeR, groundY, 'R'),
      getGroundContactPin(afterLeanComp.footL, afterLeanComp.toeL, groundY, 'L'),
      canvasOrigin,
      {
        baseRootRotation: leaned.rootRotation,
        baseLumbarAngle: leaned.joints.lumbar.localAngle,
        baseThoraxAngle: leaned.joints.thorax.localAngle,
        baseNeckAngle: leaned.joints.neck.localAngle,
        startPelvisX: pelvis.x,
        startPelvisY: pelvis.y,
        torsoCompensation: 0,
      }
    );
  }

  return solveJointIK(leaned, effectorId, targetX, targetY, canvasOrigin, {
    ...options,
    softDamp: false,
  });
}

export function solveJointIK(
  state: ArmatureState,
  jointId: JointId,
  targetX: number,
  targetY: number,
  canvasOrigin: Point2D = { x: PAPER_WIDTH / 2, y: PAPER_HEIGHT / 2 },
  options?: {
    solver?: 'analytical' | 'fabrik' | 'ccd';
    softDamp?: boolean;
    constrainLimits?: boolean;
  }
): ArmatureState {
  const spec = JOINT_SPEC_MAP[jointId];
  if (!spec) return state;

  if (jointId === 'pelvis') {
    return solveBoneAimRotation(state, 'pelvis', targetX, targetY, canvasOrigin);
  }

  // Whole-arm fingertip IK: dragging finger or fingerTip guides humerus + forearm + hand + finger
  if (
    jointId === 'fingerTipR' ||
    jointId === 'fingerTipL' ||
    jointId === 'fingerR' ||
    jointId === 'fingerL'
  ) {
    return solveArmTipIK(state, jointId, targetX, targetY, canvasOrigin, options);
  }

  // Aim IK for joints configured as single-bone direct tracking
  if (spec.ikSolverType === 'aim' || (Array.isArray(spec.ikChain) && spec.ikChain.length === 2)) {
    return solveSingleBoneAimIK(state, jointId, targetX, targetY, canvasOrigin, options);
  }

  // Spine CCD solver for spinal segments
  if (spec.ikChain === 'spine' || spec.ikChain === 'spine_neck' || spec.ikChain === 'spine_thorax') {
    return solveSpineCCDIK(state, targetX, targetY, canvasOrigin, jointId);
  }

  // Jaw swivel
  if (spec.ikChain === 'jaw') {
    return solveSingleBoneAimIK(state, 'lip', targetX, targetY, canvasOrigin, options);
  }

  // Multi-bone FABRIK solver
  if (options?.solver === 'fabrik' && Array.isArray(spec.ikChain) && spec.ikChain.length >= 3) {
    return solveFABRIKChainIK(state, [...spec.ikChain], targetX, targetY, canvasOrigin, options);
  }

  // 2-bone analytical solver for limbs and digits
  if (Array.isArray(spec.ikChain) && spec.ikChain.length === 3) {
    return solveTwoBoneIK(state, jointId, targetX, targetY, canvasOrigin, options);
  }

  // Fallback to aim rotation
  return solveBoneAimRotation(state, jointId, targetX, targetY, canvasOrigin);
}

export interface PressurePlateSolution {
  armature: ArmatureState;
  pressurePlateY: number;
  atHardStop: boolean;
}

export function solvePressurePlateContact(
  state: ArmatureState,
  targetY: number,
  canvasOrigin: Point2D = { x: PAPER_WIDTH / 2, y: PAPER_HEIGHT / 2 }
): PressurePlateSolution {
  const solveAtY = (y: number) => {
    let next = state;
    for (const side of ['R', 'L'] as const) {
      const footId: JointId = `foot${side}`;
      const toeId: JointId = `toe${side}`;
      const computed = computeArmature(next, canvasOrigin);
      const foot = computed[footId];
      const toe = computed[toeId];
      const length = Math.hypot(toe.x - foot.x, toe.y - foot.y);
      if (length < 1e-4) continue;

      const specLimits = JOINT_SPEC_MAP[footId]?.limits ?? [-180, 180];
      const baseWorldDeg = foot.worldAngle - foot.localAngle;
      const sinTarget = Math.max(-1, Math.min(1, (y - foot.y) / length));
      const angleA = Math.asin(sinTarget) * RAD_TO_DEG;
      const worldCandidates = [angleA, 180 - angleA];
      let bestLocalAngle = foot.localAngle;
      let bestError = Number.POSITIVE_INFINITY;
      let bestMotion = Number.POSITIVE_INFINITY;

      for (const worldAngle of worldCandidates) {
        const rawLocalAngle =
          foot.localAngle + shortestAngleDelta(foot.worldAngle, worldAngle);
        const localAngle = Math.max(
          specLimits[0],
          Math.min(specLimits[1], rawLocalAngle)
        );
        const achievedY =
          foot.y + length * Math.sin((baseWorldDeg + localAngle) * DEG_TO_RAD);
        const error = Math.abs(achievedY - y);
        const motion = Math.abs(localAngle - foot.localAngle);
        if (error < bestError || (Math.abs(error - bestError) < 1e-4 && motion < bestMotion)) {
          bestLocalAngle = localAngle;
          bestError = error;
          bestMotion = motion;
        }
      }

      next = {
        ...next,
        joints: {
          ...next.joints,
          [footId]: {
            ...next.joints[footId],
            localAngle: applyJointAngleConstraint(bestLocalAngle, next.continuousSpin),
          },
        },
      };
    }
    return next;
  };

  const first = solveAtY(targetY);
  const firstJoints = computeArmature(first, canvasOrigin);
  const rightError = Math.abs(firstJoints.toeR.y - targetY);
  const leftError = Math.abs(firstJoints.toeL.y - targetY);
  if (Math.max(rightError, leftError) <= 1.5) {
    return { armature: first, pressurePlateY: targetY, atHardStop: false };
  }

  const initial = computeArmature(state, canvasOrigin);
  const movingDown = targetY >= (initial.toeR.y + initial.toeL.y) / 2;
  const hardStopY = movingDown
    ? Math.min(firstJoints.toeR.y, firstJoints.toeL.y)
    : Math.max(firstJoints.toeR.y, firstJoints.toeL.y);
  const pressurePlateY = movingDown
    ? Math.min(targetY, hardStopY)
    : Math.max(targetY, hardStopY);

  return {
    armature: solveAtY(pressurePlateY),
    pressurePlateY,
    atHardStop: true,
  };
}

/**
 * Drives the 2-bone hip toggle hinge (`P_mid` between `femurR` and `femurL`) toward a target
 * vertical/lateral cursor point:
 * - Near the horizontal hip line (|h| <= snapBand), snaps to 180° Single Line (maximum span).
 * - Above the hip line (+1), folds into A-shape and retracts hipR/hipL symmetrically inward.
 * - Below the hip line (-1), folds into V-shape and retracts hipR/hipL symmetrically inward.
 */
export function solveHipToggleFromTarget(
  state: ArmatureState,
  targetMidX: number,
  targetMidY: number,
  canvasOrigin: Point2D = { x: PAPER_WIDTH / 2, y: PAPER_HEIGHT / 2 }
): ArmatureState {
  const computed = computeArmature(state, canvasOrigin);
  const pLeft = computed.femurR;
  const pRight = computed.femurL;
  const pelvis = computed.pelvis;
  const L = HIP_BRIDGE_LINK_STAGE_PX * (state.joints.pelvis?.scaleWidth ?? 1);

  const midX = (pLeft.x + pRight.x) * 0.5;
  const midY = (pLeft.y + pRight.y) * 0.5;
  const dx = pRight.x - pLeft.x;
  const dy = pRight.y - pLeft.y;
  const rawDist = Math.hypot(dx, dy);
  const ux = rawDist > 1e-4 ? dx / rawDist : 1;
  const uy = rawDist > 1e-4 ? dy / rawDist : 0;

  const n1x = -uy;
  const n1y = ux;
  const refDx = pelvis.x - midX;
  const refDy = pelvis.y - midY;
  const dot = n1x * refDx + n1y * refDy;
  const upNx = dot >= 0 ? n1x : -n1x;
  const upNy = dot >= 0 ? n1y : -n1y;

  const signedH = (targetMidX - midX) * upNx + (targetMidY - midY) * upNy;
  const snapBandPx = 4.5;
  const nextSign: 1 | -1 = signedH >= 0 ? 1 : -1;

  let targetHalfSpan = L;
  if (Math.abs(signedH) > snapBandPx) {
    const clampedH = Math.min(L * 0.96, Math.abs(signedH));
    targetHalfSpan = Math.sqrt(Math.max(0, L * L - clampedH * clampedH));
  }

  const hipRadius = Math.max(
    1,
    Math.hypot(pLeft.x - pelvis.x, pLeft.y - pelvis.y)
  );
  const restOutwardDeg = Math.atan2(37, 145) * RAD_TO_DEG; // ~14.315°
  const desiredOutwardDeg =
    Math.asin(Math.max(-0.95, Math.min(0.95, targetHalfSpan / hipRadius))) * RAD_TO_DEG;
  const rawHipAngle = desiredOutwardDeg - restOutwardDeg;
  const clampedHipR = Math.max(-10, Math.min(21, snapAnglePrecision(rawHipAngle)));

  return {
    ...state,
    hipToggleSign: nextSign,
    joints: {
      ...state.joints,
      hipR: { ...state.joints.hipR, localAngle: clampedHipR },
      hipL: { ...state.joints.hipL, localAngle: -clampedHipR },
    },
  };
}

/**
 * Sets the 2-bone hip toggle directly to one of its three mechanical states:
 * `'extended'` (180° dead-center horizontal line), `'A'` (upward fold), or `'V'` (downward fold).
 */
export function setHipToggleState(
  state: ArmatureState,
  toggleState: ToggleHingeFoldState
): ArmatureState {
  if (toggleState === 'extended') {
    return {
      ...state,
      hipToggleSign: state.hipToggleSign ?? 1,
      joints: {
        ...state.joints,
        hipR: { ...state.joints.hipR, localAngle: 20.8 },
        hipL: { ...state.joints.hipL, localAngle: -20.8 },
      },
    };
  }
  const nextSign: 1 | -1 = toggleState === 'A' ? 1 : -1;
  const needsRetract =
    state.joints.hipR.localAngle > 8 || state.joints.hipL.localAngle < -8;
  return {
    ...state,
    hipToggleSign: nextSign,
    joints: {
      ...state.joints,
      hipR: {
        ...state.joints.hipR,
        localAngle: needsRetract ? 0 : state.joints.hipR.localAngle,
      },
      hipL: {
        ...state.joints.hipL,
        localAngle: needsRetract ? 0 : state.joints.hipL.localAngle,
      },
    },
  };
}

/**
 * Drives a 2-bone collarbone toggle hinge (`clavOuterR` / `clavOuterL` center joint) toward a
 * cursor position: snaps to 180° Single Line near the shoulder line, or folds upward (A) / downward (V)
 * while retracting the shoulder socket inward along the rigid swivel arc.
 */
export function solveCollarToggleFromTarget(
  state: ArmatureState,
  side: 'R' | 'L',
  targetMidX: number,
  targetMidY: number,
  canvasOrigin: Point2D = { x: PAPER_WIDTH / 2, y: PAPER_HEIGHT / 2 }
): ArmatureState {
  const braceId: JointId = side === 'R' ? 'braceR' : 'braceL';
  const acromionId: JointId = side === 'R' ? 'acromionR' : 'acromionL';
  const computed = computeArmature(state, canvasOrigin);
  const thorax = computed.thorax;
  const lumbar = computed.lumbar;
  const acromion = computed[acromionId];
  const halfClavLen = Math.max(1, computed[braceId].effectiveLengthPx);

  const midX = (thorax.x + acromion.x) * 0.5;
  const midY = (thorax.y + acromion.y) * 0.5;
  const dx = acromion.x - thorax.x;
  const dy = acromion.y - thorax.y;
  const rawDist = Math.hypot(dx, dy);
  const ux = rawDist > 1e-4 ? dx / rawDist : side === 'R' ? -1 : 1;
  const uy = rawDist > 1e-4 ? dy / rawDist : 0;

  const n1x = -uy;
  const n1y = ux;
  const spineDx = thorax.x - lumbar.x;
  const spineDy = thorax.y - lumbar.y;
  const dot = n1x * spineDx + n1y * spineDy;
  const upNx = dot >= 0 ? n1x : -n1x;
  const upNy = dot >= 0 ? n1y : -n1y;

  const signedH = (targetMidX - midX) * upNx + (targetMidY - midY) * upNy;
  const snapBandPx = 3.5;

  let nextBraceAngle = 0;
  if (Math.abs(signedH) > snapBandPx) {
    const clampedH = Math.min(halfClavLen * 0.9, Math.abs(signedH));
    const foldDeg = Math.asin(clampedH / halfClavLen) * RAD_TO_DEG;
    const isUp = signedH >= 0;
    const rawAngle = side === 'R' ? (isUp ? foldDeg : -foldDeg) : isUp ? -foldDeg : foldDeg;
    nextBraceAngle = Math.max(-65, Math.min(65, snapAnglePrecision(rawAngle)));
  }

  return {
    ...state,
    joints: {
      ...state.joints,
      [braceId]: {
        ...state.joints[braceId],
        localAngle: nextBraceAngle,
      },
    },
  };
}

/**
 * Sets a collarbone toggle directly to `'extended'` (180° line), `'A'` (upward fold), or `'V'` (downward fold).
 */
export function setCollarToggleState(
  state: ArmatureState,
  side: 'R' | 'L',
  toggleState: ToggleHingeFoldState
): ArmatureState {
  const braceId: JointId = side === 'R' ? 'braceR' : 'braceL';
  const foldMag = 24;
  let angle = 0;
  if (toggleState === 'A') {
    angle = side === 'R' ? foldMag : -foldMag;
  } else if (toggleState === 'V') {
    angle = side === 'R' ? -foldMag : foldMag;
  }
  return {
    ...state,
    joints: {
      ...state.joints,
      [braceId]: {
        ...state.joints[braceId],
        localAngle: angle,
      },
    },
  };
}

/**
 * Closed-chain Waist Balance / Hula / Up-and-Down Squat solver.
 * Moves the waist (`pelvis`) laterally (hula/balance) and vertically (crouch/rise) while
 * keeping BOTH planted feet locked in world space (`footRPin`, `footLPin`).
 * Strictly clamps waist motion inside the intersection of both leg reach envelopes so
 * cardboard-cutout legs and hips NEVER stretch or detach from their sockets.
 */
export const GROUND_CONTACT_TOLERANCE_PX = 12;

export function getGroundContactPin(
  ankle: ComputedJoint,
  toe: ComputedJoint,
  groundY: number,
  side: 'R' | 'L',
  lockToGround: boolean = false
): GroundContactPin {
  const ankleDistance = Math.abs(groundY - ankle.y);
  const toeDistance = Math.abs(groundY - toe.y);
  const ankleIsCloser = ankleDistance <= toeDistance;
  const contactDistance = Math.min(ankleDistance, toeDistance);
  const supportWeight = Math.max(
    0,
    Math.min(1, 1 - contactDistance / GROUND_CONTACT_TOLERANCE_PX)
  );
  const useAnkle = ankleIsCloser || supportWeight <= 0;
  const contact = useAnkle ? ankle : toe;

  return {
    x: contact.x,
    y: lockToGround ? groundY : contact.y,
    jointId: useAnkle ? (side === 'R' ? 'footR' : 'footL') : side === 'R' ? 'toeR' : 'toeL',
    supportWeight: lockToGround ? 1 : supportWeight,
    worldAngle: useAnkle ? ankle.worldAngle : undefined,
  };
}

export type WaistBalanceOptions = {
  baseRootRotation?: number;
  baseLumbarAngle?: number;
  baseThoraxAngle?: number;
  baseNeckAngle?: number;
  startPelvisX?: number;
  startPelvisY?: number;
  torsoCompensation?: number;
  clampToVitruvian?: boolean;
  startFootRPin?: GroundContactPin;
  startFootLPin?: GroundContactPin;
};

/**
 * Solves grounded full-body reaction when the body is translated in X/Y.
 * In static ground mode, the ground plane stays fixed at groundY, foot contacts remain planted,
 * and the legs and spine flex dynamically using analytical IK.
 * Light X and Y movements are clamped to stay within the Vitruvian square and dual leg reach limits.
 */
export function solveGroundedBodyMotion(
  state: ArmatureState,
  targetPelvisX: number,
  targetPelvisY: number,
  groundY: number,
  canvasOrigin: Point2D = { x: PAPER_WIDTH / 2, y: PAPER_HEIGHT / 2 },
  options?: WaistBalanceOptions
): ArmatureState {
  const baseline = computeArmature(state, canvasOrigin);
  const footRPin = options?.startFootRPin ?? getGroundContactPin(baseline.footR, baseline.toeR, groundY, 'R', true);
  const footLPin = options?.startFootLPin ?? getGroundContactPin(baseline.footL, baseline.toeL, groundY, 'L', true);

  return solveWaistBalanceHula(
    state,
    targetPelvisX,
    targetPelvisY,
    footRPin,
    footLPin,
    canvasOrigin,
    {
      ...options,
      clampToVitruvian: true,
      torsoCompensation: options?.torsoCompensation ?? 0.85,
    }
  );
}

export function solveWaistBalanceHula(
  state: ArmatureState,
  targetPelvisX: number,
  targetPelvisY: number,
  footRPin: GroundContactPin,
  footLPin: GroundContactPin,
  canvasOrigin: Point2D = { x: PAPER_WIDTH / 2, y: PAPER_HEIGHT / 2 },
  options?: WaistBalanceOptions
): ArmatureState {
  const baseline = computeArmature(state, canvasOrigin);
  const rightContactId = footRPin.jointId === 'toeR' ? 'toeR' : 'footR';
  const leftContactId = footLPin.jointId === 'toeL' ? 'toeL' : 'footL';
  const rightEndpoint = baseline[rightContactId];
  const leftEndpoint = baseline[leftContactId];
  const L1_R = Math.max(
    1,
    Math.hypot(baseline.shinR.x - baseline.femurR.x, baseline.shinR.y - baseline.femurR.y)
  );
  const L2_R = Math.max(
    1,
    Math.hypot(rightEndpoint.x - baseline.shinR.x, rightEndpoint.y - baseline.shinR.y)
  );
  const L1_L = Math.max(
    1,
    Math.hypot(baseline.shinL.x - baseline.femurL.x, baseline.shinL.y - baseline.femurL.y)
  );
  const L2_L = Math.max(
    1,
    Math.hypot(leftEndpoint.x - baseline.shinL.x, leftEndpoint.y - baseline.shinL.y)
  );

  const maxReachR = (L1_R + L2_R) * 0.992;
  const minReachR = Math.abs(L1_R - L2_R) + 8;
  const maxReachL = (L1_L + L2_L) * 0.992;
  const minReachL = Math.abs(L1_L - L2_L) + 8;

  const baseRootRot = options?.baseRootRotation ?? state.rootRotation;
  const baseLumbar = options?.baseLumbarAngle ?? state.joints.lumbar.localAngle;
  const baseThorax = options?.baseThoraxAngle ?? state.joints.thorax.localAngle;
  const baseNeck = options?.baseNeckAngle ?? state.joints.neck.localAngle;
  const refX = options?.startPelvisX ?? baseline.pelvis.x;
  const refY = options?.startPelvisY ?? baseline.pelvis.y;
  const compensationStrength = options?.torsoCompensation ?? 1;
  const rightSupportWeight = Math.max(0, footRPin.supportWeight ?? 1);
  const leftSupportWeight = Math.max(0, footLPin.supportWeight ?? 1);
  const supportWeightTotal = rightSupportWeight + leftSupportWeight;
  const hasGroundSupport = supportWeightTotal > 1e-4;
  const rightGeometryWeight = hasGroundSupport ? rightSupportWeight : 1;
  const leftGeometryWeight = hasGroundSupport ? leftSupportWeight : 1;
  const geometryWeightTotal = rightGeometryWeight + leftGeometryWeight;
  const supportCenterX =
    (footRPin.x * rightGeometryWeight + footLPin.x * leftGeometryWeight) /
    geometryWeightTotal;
  const supportCenterY =
    (footRPin.y * rightGeometryWeight + footLPin.y * leftGeometryWeight) /
    geometryWeightTotal;

  // Natural hula hip tilt proportional to lateral sway + balancing torso counter-lean
  const rawLateral =
    targetPelvisX - refX + (supportCenterX - baseline.pelvis.x) * 0.12;
  const hulaTilt = softSaturate(rawLateral * 0.075, 10);
  const nextRootRot = applyJointAngleConstraint(baseRootRot + hulaTilt, state.continuousSpin);
  const verticalLean = softSaturate(
    (targetPelvisY - refY) * 0.055 * compensationStrength,
    14
  );
  const nextLumbarAngle = applyJointAngleConstraint(
    Math.max(-45, Math.min(45, baseLumbar - hulaTilt * 0.85 - verticalLean * 0.22)),
    state.continuousSpin
  );
  const nextThoraxAngle = applyJointAngleConstraint(
    Math.max(-55, Math.min(55, baseThorax + hulaTilt * 0.3 + verticalLean * 0.65)),
    state.continuousSpin
  );
  const nextNeckAngle = applyJointAngleConstraint(
    Math.max(-55, Math.min(55, baseNeck + hulaTilt * 0.12 - verticalLean * 0.35)),
    state.continuousSpin
  );

  // Measure pelvis-to-hip socket vectors under the tilted pelvis frame
  const probeRaw = evaluateRawArmature({
    ...state,
    rootX: targetPelvisX,
    rootY: targetPelvisY,
    rootRotation: nextRootRot,
    isAnchorPinned: false,
  });
  const hipOffR = {
    x: probeRaw.femurR.x - probeRaw.pelvis.x,
    y: probeRaw.femurR.y - probeRaw.pelvis.y,
  };
  const hipOffL = {
    x: probeRaw.femurL.x - probeRaw.pelvis.x,
    y: probeRaw.femurL.y - probeRaw.pelvis.y,
  };

  const centerR = { x: footRPin.x - hipOffR.x, y: footRPin.y - hipOffR.y };
  const centerL = { x: footLPin.x - hipOffL.x, y: footLPin.y - hipOffL.y };

  // Vitruvian Working Frame Clamping to prevent breaking the viewing window
  const canons = computeProportionalCanons();
  const wsHalf = canons.workingSquareHalfPx;
  const frameMargin = 40;
  const minBoxX = canvasOrigin.x - wsHalf + frameMargin;
  const maxBoxX = canvasOrigin.x + wsHalf - frameMargin;
  const minBoxY = canvasOrigin.y - wsHalf + frameMargin;

  // Project candidate waist position into the intersection of both rigid leg reach circles
  let px =
    options?.clampToVitruvian !== false
      ? Math.max(minBoxX, Math.min(maxBoxX, targetPelvisX))
      : targetPelvisX;
  let py = Math.min(
    options?.clampToVitruvian !== false ? Math.max(minBoxY, targetPelvisY) : targetPelvisY,
    supportCenterY - 52
  );

  for (let pass = 0; pass < 5; pass++) {
    const dxR = px - centerR.x;
    const dyR = py - centerR.y;
    const dR = Math.hypot(dxR, dyR);
    if (dR > maxReachR && dR > 1e-4) {
      px = centerR.x + (dxR / dR) * maxReachR;
      py = centerR.y + (dyR / dR) * maxReachR;
    } else if (dR < minReachR && dR > 1e-4) {
      px = centerR.x + (dxR / dR) * minReachR;
      py = centerR.y + (dyR / dR) * minReachR;
    }

    const dxL = px - centerL.x;
    const dyL = py - centerL.y;
    const dL = Math.hypot(dxL, dyL);
    if (dL > maxReachL && dL > 1e-4) {
      px = centerL.x + (dxL / dL) * maxReachL;
      py = centerL.y + (dyL / dL) * maxReachL;
    } else if (dL < minReachL && dL > 1e-4) {
      px = centerL.x + (dxL / dL) * minReachL;
      py = centerL.y + (dyL / dL) * minReachL;
    }

    if (options?.clampToVitruvian !== false) {
      px = Math.max(minBoxX, Math.min(maxBoxX, px));
      py = Math.max(minBoxY, py);
    }
    py = Math.min(py, supportCenterY - 52);
  }

  let next: ArmatureState = {
    ...state,
    rootX: px,
    rootY: py,
    rootRotation: nextRootRot,
    isAnchorPinned: false,
    joints: {
      ...state.joints,
      lumbar: {
        ...state.joints.lumbar,
        localAngle: nextLumbarAngle,
      },
      thorax: {
        ...state.joints.thorax,
        localAngle: nextThoraxAngle,
      },
      neck: {
        ...state.joints.neck,
        localAngle: nextNeckAngle,
      },
    },
  };

  if (hasGroundSupport) {
    for (let pass = 0; pass < 2; pass++) {
      const centers = computeBodyBalanceCenters(computeArmature(next, canvasOrigin));
      const upperMassShare =
        centers.upperMass / Math.max(1e-4, centers.upperMass + centers.lowerMass);
      const upperLowerOffsetX = centers.upper.x - centers.lower.x;
      const balanceErrorX = centers.whole.x - supportCenterX;
      const balanceTilt = softSaturate(
        balanceErrorX * 0.035 * upperMassShare + upperLowerOffsetX * 0.012,
        8
      );
      if (Math.abs(balanceTilt) < 0.02) break;

      const clampSpineAngle = (id: JointId, angle: number) => {
        const limits = JOINT_SPEC_MAP[id]?.limits ?? [-180, 180];
        return Math.max(
          limits[0],
          Math.min(limits[1], applyJointAngleConstraint(angle, next.continuousSpin))
        );
      };
      next = {
        ...next,
        joints: {
          ...next.joints,
          lumbar: {
            ...next.joints.lumbar,
            localAngle: clampSpineAngle(
              'lumbar',
              next.joints.lumbar.localAngle - balanceTilt * 0.65
            ),
          },
          thorax: {
            ...next.joints.thorax,
            localAngle: clampSpineAngle(
              'thorax',
              next.joints.thorax.localAngle - balanceTilt * 0.25
            ),
          },
          neck: {
            ...next.joints.neck,
            localAngle: clampSpineAngle(
              'neck',
              next.joints.neck.localAngle - balanceTilt * 0.1
            ),
          },
        },
      };
    }
  }

  // Solve both legs analytically with softDamp disabled so feet stay dead-locked to ground pins
  for (let iter = 0; iter < 2; iter++) {
    next = solveJointIK(next, rightContactId, footRPin.x, footRPin.y, canvasOrigin, {
      solver: 'analytical',
      softDamp: false,
      constrainLimits: false,
    });
    next = solveJointIK(next, leftContactId, footLPin.x, footLPin.y, canvasOrigin, {
      solver: 'analytical',
      softDamp: false,
      constrainLimits: false,
    });
  }

  // Keep feet flat at their planted world angles on the ground
  const afterLegs = computeArmature(next, canvasOrigin);
  if (rightContactId === 'footR' && footRPin.worldAngle !== undefined) {
    const deltaFootR = shortestAngleDelta(afterLegs.footR.worldAngle, footRPin.worldAngle);
    next = {
      ...next,
      joints: {
        ...next.joints,
        footR: {
          ...next.joints.footR,
          localAngle: applyJointAngleConstraint(
            next.joints.footR.localAngle + deltaFootR,
            next.continuousSpin
          ),
        },
      },
    };
  }
  if (leftContactId === 'footL' && footLPin.worldAngle !== undefined) {
    const deltaFootL = shortestAngleDelta(afterLegs.footL.worldAngle, footLPin.worldAngle);
    next = {
      ...next,
      joints: {
        ...next.joints,
        footL: {
          ...next.joints.footL,
          localAngle: applyJointAngleConstraint(
            next.joints.footL.localAngle + deltaFootL,
            next.continuousSpin
          ),
        },
      },
    };
  }

  const refineContactPin = (
    contactId: JointId,
    rootId: JointId,
    midId: JointId,
    pin: GroundContactPin
  ) => {
    let refined = next;
    for (let iteration = 0; iteration < 3; iteration++) {
      const current = computeArmature(refined, canvasOrigin)[contactId];
      const errorX = pin.x - current.x;
      const errorY = pin.y - current.y;
      if (Math.hypot(errorX, errorY) < 0.05) break;

      const epsilon = 0.1;
      const rootProbe = {
        ...refined,
        joints: {
          ...refined.joints,
          [rootId]: {
            ...refined.joints[rootId],
            localAngle: refined.joints[rootId].localAngle + epsilon,
          },
        },
      };
      const midProbe = {
        ...refined,
        joints: {
          ...refined.joints,
          [midId]: {
            ...refined.joints[midId],
            localAngle: refined.joints[midId].localAngle + epsilon,
          },
        },
      };
      const rootPoint = computeArmature(rootProbe, canvasOrigin)[contactId];
      const midPoint = computeArmature(midProbe, canvasOrigin)[contactId];
      const j11 = (rootPoint.x - current.x) / epsilon;
      const j21 = (rootPoint.y - current.y) / epsilon;
      const j12 = (midPoint.x - current.x) / epsilon;
      const j22 = (midPoint.y - current.y) / epsilon;
      const determinant = j11 * j22 - j12 * j21;
      if (Math.abs(determinant) < 1e-5) break;

      const rootDelta = Math.max(
        -4,
        Math.min(4, (errorX * j22 - errorY * j12) / determinant)
      );
      const midDelta = Math.max(
        -4,
        Math.min(4, (j11 * errorY - j21 * errorX) / determinant)
      );
      const candidate = {
        ...refined,
        joints: {
          ...refined.joints,
          [rootId]: {
            ...refined.joints[rootId],
            localAngle: refined.joints[rootId].localAngle + rootDelta,
          },
          [midId]: {
            ...refined.joints[midId],
            localAngle: refined.joints[midId].localAngle + midDelta,
          },
        },
      };
      const candidatePoint = computeArmature(candidate, canvasOrigin)[contactId];
      if (
        Math.hypot(pin.x - candidatePoint.x, pin.y - candidatePoint.y) >=
        Math.hypot(errorX, errorY)
      ) {
        break;
      }
      refined = candidate;
    }
    return refined;
  };
  next = refineContactPin(rightContactId, 'femurR', 'shinR', footRPin);
  next = refineContactPin(leftContactId, 'femurL', 'shinL', footLPin);

  return next;
}

export function solveTorsoBalance(
  state: ArmatureState,
  targetTorsoX: number,
  targetTorsoY: number,
  footRPin: Point2D & { worldAngle?: number },
  footLPin: Point2D & { worldAngle?: number },
  canvasOrigin: Point2D = { x: PAPER_WIDTH / 2, y: PAPER_HEIGHT / 2 },
  options?: Omit<WaistBalanceOptions, 'torsoCompensation'>
): ArmatureState {
  const starting = computeArmature(state, canvasOrigin);
  const balanceOptions: WaistBalanceOptions = {
    baseRootRotation: options?.baseRootRotation ?? state.rootRotation,
    baseLumbarAngle: options?.baseLumbarAngle ?? state.joints.lumbar.localAngle,
    baseThoraxAngle: options?.baseThoraxAngle ?? state.joints.thorax.localAngle,
    baseNeckAngle: options?.baseNeckAngle ?? state.joints.neck.localAngle,
    startPelvisX: options?.startPelvisX ?? starting.pelvis.x,
    startPelvisY: options?.startPelvisY ?? starting.pelvis.y,
    torsoCompensation: 1.8,
  };
  let next = state;

  for (let pass = 0; pass < 4; pass++) {
    const computed = computeArmature(next, canvasOrigin);
    const torsoCenter = computeTorsoCenter(computed);
    const errorX = targetTorsoX - torsoCenter.x;
    const errorY = targetTorsoY - torsoCenter.y;
    if (Math.hypot(errorX, errorY) < 0.5) break;

    next = solveWaistBalanceHula(
      next,
      computed.pelvis.x + errorX,
      computed.pelvis.y + errorY,
      footRPin,
      footLPin,
      canvasOrigin,
      balanceOptions
    );
  }

  return next;
}

export type TransitionEasing = 'smooth' | 'linear' | 'elastic';

export function evaluateTransitionEasing(t: number, easing: TransitionEasing = 'smooth'): number {
  if (t <= 0) return 0;
  if (t >= 1) return 1;
  const clamped = t;
  if (easing === 'linear') return clamped;
  if (easing === 'elastic') {
    // Elasti-hose physics: spring and rubber-band damped snap back
    const p = 0.38;
    return Math.pow(2, -8 * clamped) * Math.sin(((clamped - p / 4) * (2 * Math.PI)) / p) + 1;
  }
  return clamped < 0.5
    ? 4 * clamped * clamped * clamped
    : 1 - Math.pow(-2 * clamped + 2, 3) / 2;
}

export const DEFAULT_POSE_TRANSITION_MS = 320;
export const DEFAULT_TOGGLE_TRANSITION_MS = 220;
export const ELASTI_HOSE_TRANSITION_MS = 420;

export function hasArmaturePoseDelta(a: ArmatureState, b: ArmatureState, eps = 1e-3): boolean {
  if (
    Math.abs(a.rootX - b.rootX) > eps ||
    Math.abs(a.rootY - b.rootY) > eps ||
    Math.abs(shortestAngleDelta(a.rootRotation, b.rootRotation)) > eps ||
    Math.abs(shortestAngleDelta(a.armSwivelR ?? 0, b.armSwivelR ?? 0)) > eps ||
    Math.abs(shortestAngleDelta(a.armSwivelL ?? 0, b.armSwivelL ?? 0)) > eps ||
    Math.abs((a.faceplateOffsetX ?? 0) - (b.faceplateOffsetX ?? 0)) > eps ||
    Math.abs((a.faceplateOffsetY ?? 0) - (b.faceplateOffsetY ?? 0)) > eps ||
    Math.abs((a.neckLift ?? 0) - (b.neckLift ?? 0)) > eps ||
    Math.abs((a.neckCollarSeparation?.x ?? 0) - (b.neckCollarSeparation?.x ?? 0)) > eps ||
    Math.abs((a.neckCollarSeparation?.y ?? 0) - (b.neckCollarSeparation?.y ?? 0)) > eps ||
    (a.hipToggleSign ?? 1) !== (b.hipToggleSign ?? 1) ||
    a.isAnchorPinned !== b.isAnchorPinned ||
    a.environmentalAnchor !== b.environmentalAnchor ||
    a.internalKeystone !== b.internalKeystone
  ) {
    return true;
  }
  for (const key of Object.keys(b.joints) as JointId[]) {
    const jA = a.joints[key];
    const jB = b.joints[key];
    if (!jA || !jB) return true;
    if (
      Math.abs(shortestAngleDelta(jA.localAngle, jB.localAngle)) > eps ||
      Math.abs(jA.scaleLength - jB.scaleLength) > eps ||
      Math.abs(jA.scaleWidth - jB.scaleWidth) > eps
    ) {
      return true;
    }
  }
  return false;
}

export function interpolateArmatures(
  a: ArmatureState,
  b: ArmatureState,
  t: number,
  easing: TransitionEasing = 'smooth'
): ArmatureState {
  const clampedT = Math.max(0, Math.min(1, t));
  if (clampedT <= 0) return a;
  if (clampedT >= 1) return b;

  const easedT = evaluateTransitionEasing(clampedT, easing);
  const nextJoints = { ...a.joints };

  for (const key of Object.keys(a.joints) as JointId[]) {
    const jA = a.joints[key];
    const jB = b.joints[key] ?? jA;
    const delta = shortestAngleDelta(jA.localAngle, jB.localAngle);
    const scaleLen = jA.scaleLength + (jB.scaleLength - jA.scaleLength) * easedT;
    const scaleWid = jA.scaleWidth + (jB.scaleWidth - jA.scaleWidth) * easedT;
    nextJoints[key] = {
      ...(easedT >= 0.5 ? jB : jA),
      localAngle: jA.localAngle + delta * easedT,
      scaleLength: scaleLen,
      scaleWidth: scaleWid,
    };
  }

  // Smoothly extend the hip bridge toward its 180° apex when switching toggle signs.
  const hipSignA = a.hipToggleSign ?? 1;
  const hipSignB = b.hipToggleSign ?? 1;
  if (hipSignA !== hipSignB && nextJoints.hipR && nextJoints.hipL) {
    const passOutward = Math.sin(Math.PI * easedT);
    const curR = nextJoints.hipR.localAngle;
    const curL = nextJoints.hipL.localAngle;
    nextJoints.hipR = {
      ...nextJoints.hipR,
      localAngle: curR + (20.8 - curR) * passOutward,
    };
    nextJoints.hipL = {
      ...nextJoints.hipL,
      localAngle: curL + (-20.8 - curL) * passOutward,
    };
  }

  const rotDelta = shortestAngleDelta(a.rootRotation, b.rootRotation);
  const swivelRA = a.armSwivelR ?? 0;
  const swivelRB = b.armSwivelR ?? 0;
  const swivelLA = a.armSwivelL ?? 0;
  const swivelLB = b.armSwivelL ?? 0;
  return {
    ...(easedT >= 0.5 ? b : a),
    rootX: a.rootX + (b.rootX - a.rootX) * easedT,
    rootY: a.rootY + (b.rootY - a.rootY) * easedT,
    rootRotation: a.rootRotation + rotDelta * easedT,
    hipToggleSign: easedT < 0.5 ? hipSignA : hipSignB,
    armSwivelR: normalizeAngleDeg(swivelRA + shortestAngleDelta(swivelRA, swivelRB) * easedT),
    armSwivelL: normalizeAngleDeg(swivelLA + shortestAngleDelta(swivelLA, swivelLB) * easedT),
    faceplateOffsetX:
      (a.faceplateOffsetX ?? 0) +
      ((b.faceplateOffsetX ?? 0) - (a.faceplateOffsetX ?? 0)) * easedT,
    faceplateOffsetY:
      (a.faceplateOffsetY ?? 0) +
      ((b.faceplateOffsetY ?? 0) - (a.faceplateOffsetY ?? 0)) * easedT,
    neckLift: (a.neckLift ?? 0) + ((b.neckLift ?? 0) - (a.neckLift ?? 0)) * easedT,
    neckCollarSeparation: {
      x:
        (a.neckCollarSeparation?.x ?? 0) +
        ((b.neckCollarSeparation?.x ?? 0) - (a.neckCollarSeparation?.x ?? 0)) * easedT,
      y:
        (a.neckCollarSeparation?.y ?? 0) +
        ((b.neckCollarSeparation?.y ?? 0) - (a.neckCollarSeparation?.y ?? 0)) * easedT,
    },
    isAnchorPinned: a.isAnchorPinned && b.isAnchorPinned,
    joints: nextJoints,
  };
}

export function computePoseTransitionFrame(
  from: ArmatureState,
  to: ArmatureState,
  elapsedMs: number,
  durationMs: number = DEFAULT_POSE_TRANSITION_MS,
  easing: TransitionEasing = 'smooth'
): { armature: ArmatureState; progress: number; done: boolean } {
  const safeDuration = Math.max(0, durationMs);
  const progress =
    safeDuration <= 0 ? 1 : Math.max(0, Math.min(1, elapsedMs / safeDuration));
  return {
    armature: interpolateArmatures(from, to, progress, easing),
    progress,
    done: progress >= 1,
  };
}

export function solveBoneAimRotation(
  state: ArmatureState,
  pivotJointId: JointId,
  targetX: number,
  targetY: number,
  canvasOrigin: Point2D = { x: PAPER_WIDTH / 2, y: PAPER_HEIGHT / 2 }
): ArmatureState {
  const computed = computeArmature(state, canvasOrigin);
  const pivot = computed[pivotJointId];
  const dx = targetX - pivot.x;
  const dy = targetY - pivot.y;
  if (Math.hypot(dx, dy) < 1e-3) return state;

  const targetWorldDeg = (Math.atan2(dy, dx) * 180) / Math.PI;

  if (pivotJointId === 'pelvis') {
    const mult = getPhysicsMultiplier(state.joints.pelvis);
    if (mult === 0) return state;
    const rawDelta = shortestAngleDelta(pivot.worldAngle, targetWorldDeg);
    const nextRootRot = applyJointAngleConstraint(
      state.rootRotation + rawDelta * mult,
      state.continuousSpin
    );
    return { ...state, rootRotation: nextRootRot };
  }

  const spec = JOINT_SPEC_MAP[pivotJointId];
  if (spec.locked) return state;
  const runtime = state.joints[pivotJointId];
  const mult = getPhysicsMultiplier(runtime);
  if (mult === 0) return state;

  const rawDelta = shortestAngleDelta(pivot.worldAngle, targetWorldDeg);
  const nextLocal = applyJointAngleConstraint(
    runtime.localAngle + rawDelta * mult,
    state.continuousSpin
  );

  return {
    ...state,
    joints: {
      ...state.joints,
      [pivotJointId]: {
        ...runtime,
        localAngle: nextLocal,
      },
    },
  };
}

export function mirrorPose(state: ArmatureState): ArmatureState {
  const nextJoints = { ...state.joints };
  const bilateralPairs: [JointId, JointId][] = [
    ['ribR', 'ribL'],
    ['swivelR', 'swivelL'],
    ['swivelTipR', 'swivelTipL'],
    ['acromionR', 'acromionL'],
    ['braceR', 'braceL'],
    ['clavOuterR', 'clavOuterL'],
    ['humerusR', 'humerusL'],
    ['forearmR', 'forearmL'],
    ['handR', 'handL'],
    ['thumbR', 'thumbL'],
    ['thumbTipR', 'thumbTipL'],
    ['fingerR', 'fingerL'],
    ['fingerTipR', 'fingerTipL'],
    ['hipR', 'hipL'],
    ['femurR', 'femurL'],
    ['shinR', 'shinL'],
    ['footR', 'footL'],
    ['toeR', 'toeL'],
  ];

  for (const [rId, lId] of bilateralPairs) {
    const r = state.joints[rId];
    const l = state.joints[lId];
    nextJoints[rId] = {
      ...r,
      localAngle: applyJointAngleConstraint(-l.localAngle, state.continuousSpin),
    };
    nextJoints[lId] = {
      ...l,
      localAngle: applyJointAngleConstraint(-r.localAngle, state.continuousSpin),
    };
  }

  const spineIds: JointId[] = ['lumbar', 'thorax', 'neck', 'head', 'lip', 'chinTip'];
  for (const id of spineIds) {
    const rt = state.joints[id];
    nextJoints[id] = {
      ...rt,
      localAngle: applyJointAngleConstraint(-rt.localAngle, state.continuousSpin),
    };
  }

  return {
    ...state,
    rootRotation: applyJointAngleConstraint(-state.rootRotation, state.continuousSpin),
    armSwivelR: state.armSwivelL ?? 0,
    armSwivelL: state.armSwivelR ?? 0,
    faceplateOffsetX: -(state.faceplateOffsetX ?? 0),
    faceplateOffsetY: state.faceplateOffsetY ?? 0,
    neckLift: state.neckLift ?? 0,
    neckCollarSeparation: {
      x: -(state.neckCollarSeparation?.x ?? 0),
      y: state.neckCollarSeparation?.y ?? 0,
    },
    joints: nextJoints,
  };
}

export function setJointMode(
  state: ArmatureState,
  jointId: JointId,
  mode: JointMode
): ArmatureState {
  return {
    ...state,
    joints: {
      ...state.joints,
      [jointId]: { ...state.joints[jointId], mode },
    },
  };
}

export function setBalancedSpineLean(
  state: ArmatureState,
  jointId: 'lumbar' | 'thorax' | 'neck',
  localAngle: number
): ArmatureState {
  const spineIds = ['lumbar', 'thorax', 'neck'] as const;
  const toSliderAngle = (angle: number) =>
    Math.round(Math.max(-55, Math.min(55, angle)) * 2) / 2;
  const nextAngle = toSliderAngle(localAngle);
  const joints = { ...state.joints };
  const otherIds = spineIds.filter((id) => id !== jointId);

  for (const id of spineIds) {
    joints[id] = { ...state.joints[id], localAngle: toSliderAngle(state.joints[id].localAngle) };
  }
  const delta = nextAngle - joints[jointId].localAngle;
  joints[jointId] = { ...state.joints[jointId], localAngle: nextAngle };
  let remainingTicks = Math.round(-delta * 2);
  let adjustableIds = otherIds;
  for (
    let iteration = 0;
    iteration < otherIds.length && adjustableIds.length > 0 && remainingTicks !== 0;
    iteration++
  ) {
    const ticksPerJoint = Math.trunc(remainingTicks / adjustableIds.length);
    let remainderTicks = remainingTicks - ticksPerJoint * adjustableIds.length;
    const nextAdjustableIds: Array<(typeof otherIds)[number]> = [];
    for (const id of adjustableIds) {
      const runtime = joints[id];
      const extraTick = remainderTicks > 0 ? 1 : remainderTicks < 0 ? -1 : 0;
      remainderTicks -= extraTick;
      const angle = toSliderAngle(
        runtime.localAngle + (ticksPerJoint + extraTick) / 2
      );
      remainingTicks -= Math.round((angle - runtime.localAngle) * 2);
      joints[id] = { ...runtime, localAngle: angle };
      if (angle > -55 && angle < 55) nextAdjustableIds.push(id);
    }
    adjustableIds = nextAdjustableIds;
  }

  return { ...state, joints };
}

// ============================================================================
// Pure Procedural Cave-Art Geometry Builder (Line-to-Line Angular Shapes)
// ============================================================================

function ptDist(a: Point2D, b: Point2D): number {
  return Math.hypot(b.x - a.x, b.y - a.y);
}

function lerpPt(a: Point2D, b: Point2D, t: number): Point2D {
  return {
    x: a.x + (b.x - a.x) * t,
    y: a.y + (b.y - a.y) * t,
  };
}

function unitAxis(a: Point2D, b: Point2D): { ux: number; uy: number; len: number } {
  const dx = b.x - a.x;
  const dy = b.y - a.y;
  const len = Math.hypot(dx, dy);
  if (len < 1e-4) return { ux: 0, uy: -1, len: 0 };
  return { ux: dx / len, uy: dy / len, len };
}

function offsetPt(p: Point2D, nx: number, ny: number, dist: number): Point2D {
  return { x: p.x + nx * dist, y: p.y + ny * dist };
}

function makeTrapezoid(
  a: Point2D,
  b: Point2D,
  halfWidthA: number,
  halfWidthB: number
): Point2D[] {
  const { ux, uy } = unitAxis(a, b);
  const nx = -uy;
  const ny = ux;
  return [
    offsetPt(a, nx, ny, -halfWidthA),
    offsetPt(a, nx, ny, halfWidthA),
    offsetPt(b, nx, ny, halfWidthB),
    offsetPt(b, nx, ny, -halfWidthB),
  ];
}

function makeDiamondSquare(center: Point2D, nx: number, ny: number, halfSpan: number): Point2D[] {
  const tx = -ny;
  const ty = nx;
  return [
    offsetPt(offsetPt(center, nx, ny, -halfSpan), tx, ty, -halfSpan),
    offsetPt(offsetPt(center, nx, ny, halfSpan), tx, ty, -halfSpan),
    offsetPt(offsetPt(center, nx, ny, halfSpan), tx, ty, halfSpan),
    offsetPt(offsetPt(center, nx, ny, -halfSpan), tx, ty, halfSpan),
  ];
}

function makeTriangle(baseCenter: Point2D, apex: Point2D, baseHalfWidth: number): Point2D[] {
  const { ux, uy } = unitAxis(baseCenter, apex);
  const nx = -uy;
  const ny = ux;
  return [
    offsetPt(baseCenter, nx, ny, -baseHalfWidth),
    offsetPt(baseCenter, nx, ny, baseHalfWidth),
    apex,
  ];
}

/**
 * Derives procedural mannequin bounding geometry & metrics from the 41-joint rig
 * (no Prometheus wings).
 */
export function buildCaveArtGeometry(
  computed: Record<JointId, ComputedJoint>
): CaveArtRigGeometry {
  const pelvis = computed.pelvis;
  const lumbar = computed.lumbar;
  const thorax = computed.thorax;
  const ribR = computed.ribR;
  const ribL = computed.ribL;
  const neck = computed.neck;
  const head = computed.head;
  const lip = computed.lip;
  const chin = computed.chinTip;

  const swivelR = computed.swivelR;
  const swivelTipR = computed.swivelTipR;
  const acromionR = computed.acromionR;
  const braceR = computed.braceR;
  const clavOuterR = computed.clavOuterR;
  const humerusR = computed.humerusR;
  const forearmR = computed.forearmR;
  const handR = computed.handR;
  const thumbR = computed.thumbR;
  const thumbTipR = computed.thumbTipR;
  const fingerR = computed.fingerR;
  const fingerTipR = computed.fingerTipR;

  const swivelL = computed.swivelL;
  const swivelTipL = computed.swivelTipL;
  const acromionL = computed.acromionL;
  const braceL = computed.braceL;
  const clavOuterL = computed.clavOuterL;
  const humerusL = computed.humerusL;
  const forearmL = computed.forearmL;
  const handL = computed.handL;
  const thumbL = computed.thumbL;
  const thumbTipL = computed.thumbTipL;
  const fingerL = computed.fingerL;
  const fingerTipL = computed.fingerTipL;

  const hipR = computed.hipR;
  const femurR = computed.femurR;
  const shinR = computed.shinR;
  const footR = computed.footR;
  const toeR = computed.toeR;

  const hipL = computed.hipL;
  const femurL = computed.femurL;
  const shinL = computed.shinL;
  const footL = computed.footL;
  const toeL = computed.toeL;

  const templeR = applyAffine(
    head.boneMatrix,
    PUPPYT_LANDMARKS.skullTempleR[0],
    PUPPYT_LANDMARKS.skullTempleR[1]
  );
  const templeL = applyAffine(
    head.boneMatrix,
    PUPPYT_LANDMARKS.skullTempleL[0],
    PUPPYT_LANDMARKS.skullTempleL[1]
  );
  const crestR = applyAffine(
    hipR.boneMatrix,
    PUPPYT_LANDMARKS.hipRTip[0],
    PUPPYT_LANDMARKS.hipRTip[1]
  );
  const crestL = applyAffine(
    hipL.boneMatrix,
    PUPPYT_LANDMARKS.hipLTip[0],
    PUPPYT_LANDMARKS.hipLTip[1]
  );

  // Model-derived hip base width from iliac crest landmarks (120.96 px at rest)
  const hipWidthPx = Math.max(24, ptDist(crestR, crestL));
  // Strict 1.25x torso-to-hip ratio built directly upon the model's hip baseline
  const torsoWidthPx = hipWidthPx * 1.25;

  // Upper spine frame: thorax (upper/lower spine meeting point) -> neck
  const thoraxPt: Point2D = { x: thorax.x, y: thorax.y };
  const neckPt: Point2D = { x: neck.x, y: neck.y };
  const { ux: uThoraxX, uy: uThoraxY } = unitAxis(thoraxPt, neckPt);
  // Lateral Left (+X when upright) and Right (-X when upright) perpendiculars
  const latLX = -uThoraxY;
  const latLY = uThoraxX;
  const latRX = -latLX;
  const latRY = -latLY;

  const upperTopCenter = lerpPt(thoraxPt, neckPt, 0.88);
  const upperMidCenter = lerpPt(thoraxPt, neckPt, 0.44);

  const ribChordPx = ptDist({ x: ribR.endX, y: ribR.endY }, { x: ribL.endX, y: ribL.endY });
  const midTorsoWidthPx = Math.max(
    hipWidthPx * 0.68,
    Math.min(torsoWidthPx * 0.86, ribChordPx)
  );

  const torsoTopR = offsetPt(upperTopCenter, latRX, latRY, torsoWidthPx / 2);
  const torsoTopL = offsetPt(upperTopCenter, latLX, latLY, torsoWidthPx / 2);
  const torsoMidR = offsetPt(upperMidCenter, latRX, latRY, midTorsoWidthPx / 2);
  const torsoMidL = offsetPt(upperMidCenter, latLX, latLY, midTorsoWidthPx / 2);

  // Lower spine thinner triangle from upper/lower spine meeting point (thorax) down to brought-in pelvic top
  const pelvisPt: Point2D = { x: pelvis.x, y: pelvis.y };
  const pelvisTopCenter = lerpPt(thoraxPt, pelvisPt, 0.86);
  const femurSpanPx = ptDist({ x: femurR.x, y: femurR.y }, { x: femurL.x, y: femurL.y });
  const lowerSpineBaseWidthPx = Math.min(femurSpanPx * 0.64, hipWidthPx * 0.4);
  const { ux: uLumX, uy: uLumY } = unitAxis(thoraxPt, pelvisPt);
  const lumLatLX = uLumY;
  const lumLatLY = -uLumX;
  const lumLatRX = -lumLatLX;
  const lumLatRY = -lumLatLY;
  const lowerSpineBaseR = offsetPt(pelvisTopCenter, lumLatRX, lumLatRY, lowerSpineBaseWidthPx / 2);
  const lowerSpineBaseL = offsetPt(pelvisTopCenter, lumLatLX, lumLatLY, lowerSpineBaseWidthPx / 2);

  // Cranium trapezoid above temple chord -> crown
  const crownPt: Point2D = { x: head.endX, y: head.endY };
  const templeMid = lerpPt(templeR, templeL, 0.5);
  const { ux: uHeadX, uy: uHeadY } = unitAxis(templeMid, crownPt);
  const headLatLX = -uHeadY;
  const headLatLY = uHeadX;
  const crownHalfW = ptDist(templeR, templeL) * 0.32;
  const crownTopR = offsetPt(crownPt, -headLatLX, -headLatLY, crownHalfW);
  const crownTopL = offsetPt(crownPt, headLatLX, headLatLY, crownHalfW);

  // Neck trapezoid
  const headPt: Point2D = { x: head.x, y: head.y };
  const neckTrapezoid = makeTrapezoid(
    neckPt,
    headPt,
    torsoWidthPx * 0.145,
    ptDist(templeR, templeL) * 0.24
  );

  const svgPelvisM = computeSvgBoneMatrix(pelvis.boneMatrix);
  const pelvisTopCenterR = applyAffine(svgPelvisM, 824, 953);
  const pelvisTopR = applyAffine(svgPelvisM, 735, 1032);
  const pelvisBotR = applyAffine(svgPelvisM, 735, 1085);
  const pelvisBotCenterR = applyAffine(svgPelvisM, 824, 1140);

  const pelvisTopCenterL = applyAffine(svgPelvisM, 824, 953);
  const pelvisTopL = applyAffine(svgPelvisM, 918, 1032);
  const pelvisBotL = applyAffine(svgPelvisM, 918, 1085);
  const pelvisBotCenterL = applyAffine(svgPelvisM, 824, 1140);

  // Bilateral Arm & Hand shapes
  const humerusRPoly = makeTrapezoid(humerusR, forearmR, 14, 9.5);
  const forearmRPoly = makeTrapezoid(forearmR, handR, 9.5, 6);
  const elbowRVerts = (() => {
    const { ux, uy } = unitAxis(humerusR, forearmR);
    return makeDiamondSquare(forearmR, -uy, ux, 7.5);
  })();
  const thumbRPoly = makeTriangle(thumbR, thumbTipR, 4.8);
  const fingerRPoly = makeTriangle(fingerR, fingerTipR, 5.8);

  const humerusLPoly = makeTrapezoid(humerusL, forearmL, 14, 9.5);
  const forearmLPoly = makeTrapezoid(forearmL, handL, 9.5, 6);
  const elbowLVerts = (() => {
    const { ux, uy } = unitAxis(humerusL, forearmL);
    return makeDiamondSquare(forearmL, -uy, ux, 7.5);
  })();
  const thumbLPoly = makeTriangle(thumbL, thumbTipL, 4.8);
  const fingerLPoly = makeTriangle(fingerL, fingerTipL, 5.8);

  // Clean Lower Body & Leg shapes
  const femurRPoly = makeTrapezoid(femurR, shinR, 16.5, 10.5);
  const kneeRVerts = (() => {
    const { ux, uy } = unitAxis(femurR, shinR);
    return makeDiamondSquare(shinR, -uy, ux, 9.5);
  })();
  const shinRPoly = makeTrapezoid(shinR, footR, 10.5, 6.2);
  const footMidR = lerpPt(footR, toeR, 0.34);
  const heelRPoly = makeTrapezoid(footR, footMidR, 8.2, 6.8);
  const footRPoly = makeTriangle(footMidR, toeR, 6.8);

  const femurLPoly = makeTrapezoid(femurL, shinL, 16.5, 10.5);
  const kneeLVerts = (() => {
    const { ux, uy } = unitAxis(femurL, shinL);
    return makeDiamondSquare(shinL, -uy, ux, 9.5);
  })();
  const shinLPoly = makeTrapezoid(shinL, footL, 10.5, 6.2);
  const footMidL = lerpPt(footL, toeL, 0.34);
  const heelLPoly = makeTrapezoid(footL, footMidL, 8.2, 6.8);
  const footLPoly = makeTriangle(footMidL, toeL, 6.8);

  const polygons: CaveArtPolygon[] = [
    // Middle Pelvic Trapezoid (no Prometheus wings)
    {
      id: 'pelvis-hip-r',
      jointId: 'hipR',
      shapeKind: 'trapezoid',
      points: [pelvisTopCenterR, pelvisTopR, pelvisBotR, pelvisBotCenterR],
      layer: 'pelvis',
      fillOpacity: 0.9,
    },
    {
      id: 'pelvis-hip-l',
      jointId: 'hipL',
      shapeKind: 'trapezoid',
      points: [pelvisTopCenterL, pelvisTopL, pelvisBotL, pelvisBotCenterL],
      layer: 'pelvis',
      fillOpacity: 0.9,
    },
    // Lower spine thinner triangle from upper/lower spine meeting point (thorax) down to pelvic top
    {
      id: 'lower-spine-triangle',
      jointId: 'lumbar',
      shapeKind: 'triangle',
      points: [thoraxPt, lowerSpineBaseL, lowerSpineBaseR],
      layer: 'torso',
      fillOpacity: 0.88,
    },
    // Upper torso: triangle converging to thorax + trapezoid on top (1.25x hip width)
    {
      id: 'upper-torso-triangle',
      jointId: 'thorax',
      shapeKind: 'triangle',
      points: [torsoMidR, torsoMidL, thoraxPt],
      layer: 'torso',
      fillOpacity: 0.9,
    },
    {
      id: 'upper-torso-trapezoid',
      jointId: 'thorax',
      shapeKind: 'trapezoid',
      points: [torsoTopR, torsoTopL, torsoMidL, torsoMidR],
      layer: 'torso',
      fillOpacity: 0.92,
    },
    // Neck & Head
    {
      id: 'neck-trapezoid',
      jointId: 'neck',
      shapeKind: 'trapezoid',
      points: neckTrapezoid,
      layer: 'head',
      fillOpacity: 0.86,
    },
    {
      id: 'jaw-triangle',
      jointId: 'lip',
      shapeKind: 'triangle',
      points: [templeR, templeL, { x: chin.x, y: chin.y }],
      layer: 'head',
      fillOpacity: 0.88,
    },
    {
      id: 'cranium-trapezoid',
      jointId: 'head',
      shapeKind: 'trapezoid',
      points: [crownTopR, crownTopL, templeL, templeR],
      layer: 'head',
      fillOpacity: 0.92,
    },
    // Clean Lower Body & Legs
    {
      id: 'femur-r',
      jointId: 'femurR',
      shapeKind: 'trapezoid',
      points: femurRPoly,
      layer: 'limb',
      fillOpacity: 0.88,
    },
    {
      id: 'shin-r',
      jointId: 'shinR',
      shapeKind: 'trapezoid',
      points: shinRPoly,
      layer: 'limb',
      fillOpacity: 0.88,
    },
    {
      id: 'knee-r',
      jointId: 'shinR',
      shapeKind: 'square',
      points: kneeRVerts,
      layer: 'cap',
      fillOpacity: 0.94,
    },
    {
      id: 'heel-r',
      jointId: 'footR',
      shapeKind: 'trapezoid',
      points: heelRPoly,
      layer: 'limb',
      fillOpacity: 0.9,
    },
    {
      id: 'foot-r',
      jointId: 'footR',
      shapeKind: 'triangle',
      points: footRPoly,
      layer: 'limb',
      fillOpacity: 0.88,
    },
    {
      id: 'femur-l',
      jointId: 'femurL',
      shapeKind: 'trapezoid',
      points: femurLPoly,
      layer: 'limb',
      fillOpacity: 0.88,
    },
    {
      id: 'shin-l',
      jointId: 'shinL',
      shapeKind: 'trapezoid',
      points: shinLPoly,
      layer: 'limb',
      fillOpacity: 0.88,
    },
    {
      id: 'knee-l',
      jointId: 'shinL',
      shapeKind: 'square',
      points: kneeLVerts,
      layer: 'cap',
      fillOpacity: 0.94,
    },
    {
      id: 'heel-l',
      jointId: 'footL',
      shapeKind: 'trapezoid',
      points: heelLPoly,
      layer: 'limb',
      fillOpacity: 0.9,
    },
    {
      id: 'foot-l',
      jointId: 'footL',
      shapeKind: 'triangle',
      points: footLPoly,
      layer: 'limb',
      fillOpacity: 0.88,
    },
    // Arms & Hands
    {
      id: 'humerus-r',
      jointId: 'humerusR',
      shapeKind: 'trapezoid',
      points: humerusRPoly,
      layer: 'limb',
      fillOpacity: 0.88,
    },
    {
      id: 'forearm-r',
      jointId: 'forearmR',
      shapeKind: 'trapezoid',
      points: forearmRPoly,
      layer: 'limb',
      fillOpacity: 0.88,
    },
    {
      id: 'elbow-r',
      jointId: 'forearmR',
      shapeKind: 'square',
      points: elbowRVerts,
      layer: 'cap',
      fillOpacity: 0.94,
    },
    {
      id: 'thumb-r',
      jointId: 'thumbR',
      shapeKind: 'triangle',
      points: thumbRPoly,
      layer: 'limb',
      fillOpacity: 0.86,
    },
    {
      id: 'finger-r',
      jointId: 'fingerR',
      shapeKind: 'triangle',
      points: fingerRPoly,
      layer: 'limb',
      fillOpacity: 0.86,
    },
    {
      id: 'humerus-l',
      jointId: 'humerusL',
      shapeKind: 'trapezoid',
      points: humerusLPoly,
      layer: 'limb',
      fillOpacity: 0.88,
    },
    {
      id: 'forearm-l',
      jointId: 'forearmL',
      shapeKind: 'trapezoid',
      points: forearmLPoly,
      layer: 'limb',
      fillOpacity: 0.88,
    },
    {
      id: 'elbow-l',
      jointId: 'forearmL',
      shapeKind: 'square',
      points: elbowLVerts,
      layer: 'cap',
      fillOpacity: 0.94,
    },
    {
      id: 'thumb-l',
      jointId: 'thumbL',
      shapeKind: 'triangle',
      points: thumbLPoly,
      layer: 'limb',
      fillOpacity: 0.86,
    },
    {
      id: 'finger-l',
      jointId: 'fingerL',
      shapeKind: 'triangle',
      points: fingerLPoly,
      layer: 'limb',
      fillOpacity: 0.86,
    },
  ];

  // Functional line-to-line mechanical connections across all 43 joints
  const functionalLines: CaveArtLine[] = [
    // Axial spine axis
    { id: 'ln-lumbar', jointId: 'lumbar', x1: lumbar.x, y1: lumbar.y, x2: thorax.x, y2: thorax.y, role: 'axis', overlayLight: true },
    { id: 'ln-thorax', jointId: 'thorax', x1: thorax.x, y1: thorax.y, x2: neck.x, y2: neck.y, role: 'axis', overlayLight: true },
    { id: 'ln-neck', jointId: 'neck', x1: neck.x, y1: neck.y, x2: head.x, y2: head.y, role: 'axis', overlayLight: true },
    { id: 'ln-head-crown', jointId: 'head', x1: head.x, y1: head.y, x2: head.endX, y2: head.endY, role: 'axis', overlayLight: true },
    { id: 'ln-head-lip', jointId: 'lip', x1: head.x, y1: head.y, x2: lip.x, y2: lip.y, role: 'skull', overlayLight: true },
    { id: 'ln-lip-chin', jointId: 'lip', x1: lip.x, y1: lip.y, x2: chin.x, y2: chin.y, role: 'skull', overlayLight: true },
    { id: 'ln-brow', jointId: 'head', x1: templeR.x, y1: templeR.y, x2: templeL.x, y2: templeL.y, role: 'skull', overlayLight: true },

    // Torso shoulder-top structural tie-lines to acromions & ribs
    { id: 'ln-torso-shoulder-r', jointId: 'acromionR', x1: torsoTopR.x, y1: torsoTopR.y, x2: acromionR.x, y2: acromionR.y, role: 'girdle' },
    { id: 'ln-torso-shoulder-l', jointId: 'acromionL', x1: torsoTopL.x, y1: torsoTopL.y, x2: acromionL.x, y2: acromionL.y, role: 'girdle' },
    { id: 'ln-rib-r', jointId: 'ribR', x1: ribR.x, y1: ribR.y, x2: ribR.endX, y2: ribR.endY, role: 'rib', overlayLight: true },
    { id: 'ln-rib-l', jointId: 'ribL', x1: ribL.x, y1: ribL.y, x2: ribL.endX, y2: ribL.endY, role: 'rib', overlayLight: true },
    { id: 'ln-rib-chord', jointId: 'thorax', x1: torsoMidR.x, y1: torsoMidR.y, x2: torsoMidL.x, y2: torsoMidL.y, role: 'rib', overlayLight: true },

    // Closed-chain Shoulder Girdle (swivel -> acromion -> humerus, collarbone brace)
    { id: 'ln-swivel-r', jointId: 'swivelR', x1: swivelR.x, y1: swivelR.y, x2: acromionR.x, y2: acromionR.y, role: 'girdle', dashed: true },
    { id: 'ln-swivel-l', jointId: 'swivelL', x1: swivelL.x, y1: swivelL.y, x2: acromionL.x, y2: acromionL.y, role: 'girdle', dashed: true },
    { id: 'ln-swiveltip-r', jointId: 'swivelR', x1: acromionR.x, y1: acromionR.y, x2: swivelTipR.x, y2: swivelTipR.y, role: 'girdle' },
    { id: 'ln-swiveltip-l', jointId: 'swivelL', x1: acromionL.x, y1: acromionL.y, x2: swivelTipL.x, y2: swivelTipL.y, role: 'girdle' },
    { id: 'ln-brace-r', jointId: 'braceR', x1: braceR.x, y1: braceR.y, x2: braceR.endX, y2: braceR.endY, role: 'girdle' },
    { id: 'ln-clav-outer-r', jointId: 'clavOuterR', x1: clavOuterR.x, y1: clavOuterR.y, x2: clavOuterR.endX, y2: clavOuterR.endY, role: 'girdle' },
    { id: 'ln-brace-l', jointId: 'braceL', x1: braceL.x, y1: braceL.y, x2: braceL.endX, y2: braceL.endY, role: 'girdle' },
    { id: 'ln-clav-outer-l', jointId: 'clavOuterL', x1: clavOuterL.x, y1: clavOuterL.y, x2: clavOuterL.endX, y2: clavOuterL.endY, role: 'girdle' },

    // Clean Pelvic structural lines inside trapezoid (2-link rigid hip toggle bridge)
    ...(() : CaveArtLine[] => {
      const hipToggle = solveToggleHinge2Link(
        femurR,
        femurL,
        HIP_BRIDGE_LINK_STAGE_PX * (pelvis.effectiveWidthScale ?? 1),
        1,
        pelvis
      );
      return [
        { id: 'ln-hip-bridge-left', jointId: 'pelvis', x1: hipToggle.pLeft.x, y1: hipToggle.pLeft.y, x2: hipToggle.pMid.x, y2: hipToggle.pMid.y, role: 'pelvis', overlayLight: true },
        { id: 'ln-hip-bridge-right', jointId: 'pelvis', x1: hipToggle.pMid.x, y1: hipToggle.pMid.y, x2: hipToggle.pRight.x, y2: hipToggle.pRight.y, role: 'pelvis', overlayLight: true },
      ];
    })(),
    { id: 'ln-hip-r-strut', jointId: 'hipR', x1: pelvis.x, y1: pelvis.y, x2: femurR.x, y2: femurR.y, role: 'pelvis', overlayLight: true },
    { id: 'ln-hip-l-strut', jointId: 'hipL', x1: pelvis.x, y1: pelvis.y, x2: femurL.x, y2: femurL.y, role: 'pelvis', overlayLight: true },
    { id: 'ln-pelvis-axis', jointId: 'pelvis', x1: pelvisTopCenterR.x, y1: pelvisTopCenterR.y, x2: pelvisBotCenterR.x, y2: pelvisBotCenterR.y, role: 'pelvis', overlayLight: true },

    // Arms & Digits functional axes
    { id: 'ln-r-humerus', jointId: 'humerusR', x1: humerusR.x, y1: humerusR.y, x2: forearmR.x, y2: forearmR.y, role: 'limb', overlayLight: true },
    { id: 'ln-r-forearm', jointId: 'forearmR', x1: forearmR.x, y1: forearmR.y, x2: handR.x, y2: handR.y, role: 'limb', overlayLight: true },
    { id: 'ln-r-thumb', jointId: 'thumbR', x1: handR.x, y1: handR.y, x2: thumbTipR.x, y2: thumbTipR.y, role: 'digit' },
    { id: 'ln-r-finger', jointId: 'fingerR', x1: handR.x, y1: handR.y, x2: fingerTipR.x, y2: fingerTipR.y, role: 'digit', overlayLight: true },

    { id: 'ln-l-humerus', jointId: 'humerusL', x1: humerusL.x, y1: humerusL.y, x2: forearmL.x, y2: forearmL.y, role: 'limb', overlayLight: true },
    { id: 'ln-l-forearm', jointId: 'forearmL', x1: forearmL.x, y1: forearmL.y, x2: handL.x, y2: handL.y, role: 'limb', overlayLight: true },
    { id: 'ln-l-thumb', jointId: 'thumbL', x1: handL.x, y1: handL.y, x2: thumbTipL.x, y2: thumbTipL.y, role: 'digit' },
    { id: 'ln-l-finger', jointId: 'fingerL', x1: handL.x, y1: handL.y, x2: fingerTipL.x, y2: fingerTipL.y, role: 'digit', overlayLight: true },

    // Clean Legs & Feet functional axes (matching arm line clarity)
    { id: 'ln-r-femur', jointId: 'femurR', x1: femurR.x, y1: femurR.y, x2: shinR.x, y2: shinR.y, role: 'limb', overlayLight: true },
    { id: 'ln-r-shin', jointId: 'shinR', x1: shinR.x, y1: shinR.y, x2: footR.x, y2: footR.y, role: 'limb', overlayLight: true },
    { id: 'ln-r-foot', jointId: 'footR', x1: footR.x, y1: footR.y, x2: toeR.x, y2: toeR.y, role: 'limb', overlayLight: true },

    { id: 'ln-l-femur', jointId: 'femurL', x1: femurL.x, y1: femurL.y, x2: shinL.x, y2: shinL.y, role: 'limb', overlayLight: true },
    { id: 'ln-l-shin', jointId: 'shinL', x1: shinL.x, y1: shinL.y, x2: footL.x, y2: footL.y, role: 'limb', overlayLight: true },
    { id: 'ln-l-foot', jointId: 'footL', x1: footL.x, y1: footL.y, x2: toeL.x, y2: toeL.y, role: 'limb', overlayLight: true },
  ];

  return {
    polygons,
    functionalLines,
    metrics: {
      hipWidthPx,
      torsoWidthPx,
      torsoToHipRatio: torsoWidthPx / hipWidthPx,
      lowerSpineBaseWidthPx,
      upperLowerSpineMeetingPoint: thoraxPt,
    },
  };
}
