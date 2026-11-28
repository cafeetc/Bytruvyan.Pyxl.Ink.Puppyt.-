import { JOINT_SPEC_MAP, JOINT_SPECS } from '../data/digitalRig';
import {
  ArmatureState,
  ComputedJoint,
  GroundContactPin,
  JointId,
  JointMode,
  Point2D,
  PosePresetId,
  PoseState,
  RigSystemsConfig,
  ToggleHinge2LinkResult,
  ToggleHingeFoldState,
} from '../types/rig';
import {
  COLLAR_INWARD_NORMAL_LIMIT_DEG,
  COLLAR_SWIVEL_MAX_INWARD_DEG,
  FaceplateLookCollarHold,
  HIP_BRIDGE_LINK_STAGE_PX,
  WaistBalanceOptions,
  applyCollarSwivelDragInput,
  applyJointAngleConstraint,
  computeArmature,
  computeBodyBalanceCenters,
  createPosePreset,
  getPhysicsMultiplier,
  hasArmaturePoseDelta,
  mirrorPose,
  resolveCollarInwardBehavior,
  resolveCollarToggleFold,
  setArmReverseSwivel,
  setBalancedSpineLean,
  setCollarToggleState,
  setFaceplateOffset,
  setHipToggleState,
  setJointMode,
  setNeckCollarSeparation,
  solveCollarToggleFromTarget,
  solveFaceplateLookIK,
  solveHipToggleFromTarget,
  solveJointIK,
  solvePressurePlateContact,
  solveToggleHinge2Link,
  solveTorsoBalance,
  solveWaistBalanceHula,
  toggleArmReverseSwivel,
} from '../utils/kinematics';
import { PAPER_HEIGHT, PAPER_WIDTH, computeProportionalCanons } from '../utils/proportions';

export const DEFAULT_CANVAS_ORIGIN: Point2D = {
  x: PAPER_WIDTH / 2,
  y: PAPER_HEIGHT / 2,
};

export const GROUND_CONTACT_TOLERANCE_PX = 12;

export interface GroundContactSummary {
  groundY: number;
  pressurePlateY: number;
  footR: GroundContactPin;
  footL: GroundContactPin;
  groundedFootCount: number;
  groundedBodyContact: boolean;
  pressureLevel: number;
}

export interface JointConstraintStatus {
  jointId: JointId;
  localAngle: number;
  worldAngle: number;
  mode: JointMode;
  limits: readonly [number, number] | null;
  atLimit: boolean;
  isStrained: boolean;
  tensionRatio: number;
  collarSnapState?: 'normal' | 'resisting' | 'snapped';
  collarResistanceRatio?: number;
  neckCollarPressureRatio?: number;
}

export interface CollarStatus {
  side: 'R' | 'L';
  innerAngle: number;
  outerAngle: number;
  swivelAngle: number;
  foldState: ToggleHingeFoldState;
  snapState?: 'normal' | 'resisting' | 'snapped';
  resistanceRatio?: number;
}

export interface PoseEngineSnapshot {
  pose: PoseState;
  computedJoints: Record<JointId, ComputedJoint>;
  constraints: RigSystemsConfig;
  balanceCenters: ReturnType<typeof computeBodyBalanceCenters>;
  groundContact: GroundContactSummary;
}

export interface PoseEngineOptions {
  canvasOrigin?: Point2D;
  initialArmature?: ArmatureState;
  initialConstraints?: Partial<RigSystemsConfig>;
}

/**
 * Clean controller boundary for the Bitruvius 43-joint Vitruvian Base Posing Model.
 * Future gamification layers (challenges, target tolerances, timers, scoring) can wrap
 * this interface without touching internal kinematics or proportion solvers.
 */
export interface RigController {
  // --- State Read ---
  getState(): PoseState;
  getArmature(): ArmatureState;
  getComputedJoints(): Record<JointId, ComputedJoint>;
  getJoint(jointId: JointId): ComputedJoint;
  getJointAngle(jointId: JointId): number;
  getAllJointAngles(): Record<JointId, number>;
  getConstraints(): RigSystemsConfig;
  getCanvasOrigin(): Point2D;
  getSnapshot(): PoseEngineSnapshot;
  hasPoseDeltaFrom(other: ArmatureState, eps?: number): boolean;

  // --- Joint & Effector Updates ---
  setArmature(armature: ArmatureState, preset?: PosePresetId | null): PoseState;
  setJointAngle(jointId: JointId, angleDeg: number): PoseState;
  nudgeJointAngle(jointId: JointId, deltaDeg: number): PoseState;
  setBalancedSpineAngle(
    jointId: 'lumbar' | 'thorax' | 'neck',
    localAngle: number
  ): PoseState;
  setJointMode(jointId: JointId, mode: JointMode): PoseState;
  solveEffectorIK(
    jointId: JointId,
    target: Point2D,
    options?: {
      solver?: RigSystemsConfig['solver'];
      softDamp?: boolean;
      constrainLimits?: boolean;
    }
  ): PoseState;
  solveFaceplateLook(
    target: Point2D,
    collarHold?: FaceplateLookCollarHold
  ): PoseState;
  solveWaistBalance(
    targetPelvis: Point2D,
    footRPin: GroundContactPin,
    footLPin: GroundContactPin,
    options?: WaistBalanceOptions
  ): PoseState;
  solveTorsoBalance(
    targetTorso: Point2D,
    footRPin: GroundContactPin,
    footLPin: GroundContactPin,
    options?: Omit<WaistBalanceOptions, 'torsoCompensation'>
  ): PoseState;
  setHipToggle(state: ToggleHingeFoldState): PoseState;
  solveHipToggleTarget(target: Point2D): PoseState;
  setCollarToggle(side: 'R' | 'L', state: ToggleHingeFoldState): PoseState;
  solveCollarToggleTarget(side: 'R' | 'L', target: Point2D): PoseState;
  setArmReverseSwivel(side: 'R' | 'L', swivelDeg: number): PoseState;
  toggleArmSwivel(side: 'R' | 'L'): PoseState;
  setNeckCollarSeparation(separation: Point2D): PoseState;
  setFaceplateOffset(offset: Point2D): PoseState;
  setGroundPressurePlate(targetY: number): {
    pose: PoseState;
    pressurePlateY: number;
    atHardStop: boolean;
  };
  mirror(): PoseState;
  setConstraints(patch: Partial<RigSystemsConfig>): RigSystemsConfig;

  // --- Reset ---
  resetToTPose(options?: { preserveConstraints?: boolean }): PoseState;

  // --- Constraint & Contact Queries ---
  getBalanceCenters(): ReturnType<typeof computeBodyBalanceCenters>;
  getGroundContactState(): GroundContactSummary;
  getJointConstraintStatus(jointId: JointId): JointConstraintStatus;
  getCollarState(side: 'R' | 'L'): CollarStatus;
  getHipToggleState(): ToggleHinge2LinkResult;
  getArmReverseFactor(side: 'R' | 'L'): number;

  // --- Subscription ---
  subscribe(listener: (snapshot: PoseEngineSnapshot) => void): () => void;
}

export function createDefaultConstraints(
  canvasOrigin: Point2D = DEFAULT_CANVAS_ORIGIN
): RigSystemsConfig {
  const canons = computeProportionalCanons();
  return {
    softLimits: false,
    floorSnap: false,
    groundY: canvasOrigin.y + canons.eightHeadTotalPx / 2,
    solver: 'analytical',
  };
}

export function getGroundContactPin(
  ankle: ComputedJoint,
  toe: ComputedJoint,
  groundY: number,
  side: 'R' | 'L'
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
    y: contact.y,
    jointId: useAnkle
      ? side === 'R'
        ? 'footR'
        : 'footL'
      : side === 'R'
      ? 'toeR'
      : 'toeL',
    supportWeight,
    worldAngle: useAnkle ? ankle.worldAngle : undefined,
  };
}

const BODY_CONTACT_JOINT_IDS: readonly JointId[] = [
  'pelvis',
  'hipR',
  'hipL',
  'lumbar',
  'ribR',
  'ribL',
  'thorax',
  'head',
  'humerusR',
  'forearmR',
  'handR',
  'fingerTipR',
  'humerusL',
  'forearmL',
  'handL',
  'fingerTipL',
];

export class PoseEngine implements RigController {
  private readonly canvasOrigin: Point2D;
  private pose: PoseState;
  private constraints: RigSystemsConfig;
  private computedJoints: Record<JointId, ComputedJoint>;
  private listeners = new Set<(snapshot: PoseEngineSnapshot) => void>();

  constructor(options: PoseEngineOptions = {}) {
    this.canvasOrigin = options.canvasOrigin
      ? { ...options.canvasOrigin }
      : { ...DEFAULT_CANVAS_ORIGIN };
    this.constraints = {
      ...createDefaultConstraints(this.canvasOrigin),
      ...(options.initialConstraints ?? {}),
    };
    const defaultTPose = createPosePreset('tpose');
    const initialArmature = options.initialArmature
      ? structuredClone(options.initialArmature)
      : {
          ...defaultTPose,
          rootX: this.canvasOrigin.x,
          rootY: this.canvasOrigin.y,
          isAnchorPinned: true,
          hipToggleSign: 1 as const,
        };
    this.pose = {
      preset: options.initialArmature ? null : 'tpose',
      armature: initialArmature,
    };
    this.computedJoints = computeArmature(this.pose.armature, this.canvasOrigin);
  }

  private applyFloorSnapIfNeeded(candidate: ArmatureState): ArmatureState {
    if (!this.constraints.floorSnap || candidate.isAnchorPinned) {
      return candidate;
    }
    const joints = computeArmature(candidate, this.canvasOrigin);
    const groundY = this.constraints.pressurePlateY ?? this.constraints.groundY;
    const maxFootY = Math.max(
      joints.footR.endY,
      joints.footL.endY,
      joints.toeR.endY,
      joints.toeL.endY
    );
    if (maxFootY > groundY) {
      return {
        ...candidate,
        rootY: candidate.rootY - (maxFootY - groundY),
      };
    }
    return candidate;
  }

  private commitPose(
    nextArmature: ArmatureState,
    preset: PosePresetId | null = null,
    skipFloorSnap = false
  ): PoseState {
    const resolved = skipFloorSnap
      ? nextArmature
      : this.applyFloorSnapIfNeeded(nextArmature);
    this.pose = {
      preset,
      armature: resolved,
    };
    this.computedJoints = computeArmature(this.pose.armature, this.canvasOrigin);
    this.notify();
    return this.pose;
  }

  private notify(): void {
    if (this.listeners.size === 0) return;
    const snapshot = this.getSnapshot();
    for (const listener of this.listeners) {
      listener(snapshot);
    }
  }

  getState(): PoseState {
    return this.pose;
  }

  getArmature(): ArmatureState {
    return this.pose.armature;
  }

  getComputedJoints(): Record<JointId, ComputedJoint> {
    return this.computedJoints;
  }

  getJoint(jointId: JointId): ComputedJoint {
    return this.computedJoints[jointId];
  }

  getJointAngle(jointId: JointId): number {
    if (jointId === 'pelvis') {
      return this.pose.armature.rootRotation;
    }
    return this.pose.armature.joints[jointId].localAngle;
  }

  getAllJointAngles(): Record<JointId, number> {
    const angles = {} as Record<JointId, number>;
    for (const spec of JOINT_SPECS) {
      angles[spec.id] = this.getJointAngle(spec.id);
    }
    return angles;
  }

  getConstraints(): RigSystemsConfig {
    return { ...this.constraints };
  }

  getCanvasOrigin(): Point2D {
    return { ...this.canvasOrigin };
  }

  getSnapshot(): PoseEngineSnapshot {
    return {
      pose: this.pose,
      computedJoints: this.computedJoints,
      constraints: this.getConstraints(),
      balanceCenters: this.getBalanceCenters(),
      groundContact: this.getGroundContactState(),
    };
  }

  hasPoseDeltaFrom(other: ArmatureState, eps = 1e-3): boolean {
    return hasArmaturePoseDelta(this.pose.armature, other, eps);
  }

  setArmature(armature: ArmatureState, preset: PosePresetId | null = null): PoseState {
    return this.commitPose(armature, preset);
  }

  setJointAngle(jointId: JointId, angleDeg: number): PoseState {
    const targetId: JointId =
      jointId === 'clavOuterR'
        ? 'braceR'
        : jointId === 'clavOuterL'
        ? 'braceL'
        : jointId;
    const spec = JOINT_SPEC_MAP[targetId];
    const base = this.pose.armature;
    const runtime = base.joints[targetId];
    if (!spec || spec.locked || !runtime || runtime.mode === 'PIN') {
      return this.pose;
    }

    if (targetId === 'pelvis') {
      return this.commitPose({
        ...base,
        rootRotation: applyJointAngleConstraint(angleDeg, base.continuousSpin),
      });
    }

    if (targetId === 'swivelR' || targetId === 'swivelL') {
      const side: 'R' | 'L' = targetId === 'swivelR' ? 'R' : 'L';
      const otherId: JointId = side === 'R' ? 'swivelL' : 'swivelR';
      const constrained = applyJointAngleConstraint(angleDeg, base.continuousSpin);
      const clampedRaw =
        side === 'R'
          ? Math.max(
              -COLLAR_INWARD_NORMAL_LIMIT_DEG,
              Math.min(COLLAR_SWIVEL_MAX_INWARD_DEG, constrained)
            )
          : Math.max(
              -COLLAR_SWIVEL_MAX_INWARD_DEG,
              Math.min(COLLAR_INWARD_NORMAL_LIMIT_DEG, constrained)
            );
      const resolved = resolveCollarInwardBehavior(
        side,
        clampedRaw,
        base.joints[otherId].localAngle
      ).effectiveLocalAngle;
      return this.commitPose({
        ...base,
        joints: {
          ...base.joints,
          [targetId]: { ...runtime, localAngle: resolved },
        },
      });
    }

    const isClosedTriangleSwivel =
      targetId === 'hipR' ||
      targetId === 'hipL' ||
      targetId === 'braceR' ||
      targetId === 'braceL';
    let nextAngle = applyJointAngleConstraint(angleDeg, base.continuousSpin);
    if ((isClosedTriangleSwivel || this.constraints.softLimits) && spec.limits) {
      nextAngle = Math.max(spec.limits[0], Math.min(spec.limits[1], nextAngle));
    }

    return this.commitPose({
      ...base,
      joints: {
        ...base.joints,
        [targetId]: { ...runtime, localAngle: nextAngle },
      },
    });
  }

  nudgeJointAngle(jointId: JointId, deltaDeg: number): PoseState {
    const targetId: JointId =
      jointId === 'clavOuterR'
        ? 'braceR'
        : jointId === 'clavOuterL'
        ? 'braceL'
        : jointId;
    const base = this.pose.armature;
    const runtime = base.joints[targetId];
    if (!runtime || runtime.mode === 'PIN') return this.pose;
    const mult = getPhysicsMultiplier(runtime);
    if (mult === 0) return this.pose;

    if (targetId === 'swivelR' || targetId === 'swivelL') {
      const side: 'R' | 'L' = targetId === 'swivelR' ? 'R' : 'L';
      const prevAngle = runtime.localAngle;
      const rawNext = applyJointAngleConstraint(
        prevAngle + deltaDeg * mult,
        base.continuousSpin
      );
      const prevInward = side === 'R' ? prevAngle : -prevAngle;
      const nextInward = side === 'R' ? rawNext : -rawNext;
      const wasSnapped = prevInward > COLLAR_INWARD_NORMAL_LIMIT_DEG + 0.5;
      const nudgedOutward = nextInward < prevInward - 1e-4;
      if (wasSnapped && nudgedOutward) {
        return this.setJointAngle(
          targetId,
          side === 'R'
            ? COLLAR_INWARD_NORMAL_LIMIT_DEG
            : -COLLAR_INWARD_NORMAL_LIMIT_DEG
        );
      }
      const dragged = applyCollarSwivelDragInput(base, side, prevAngle + deltaDeg * mult);
      return this.commitPose({
        ...base,
        joints: {
          ...base.joints,
          [targetId]: { ...runtime, localAngle: dragged.nextAngle },
        },
      });
    }

    const prev = this.getJointAngle(targetId);
    return this.setJointAngle(targetId, prev + deltaDeg * mult);
  }

  setBalancedSpineAngle(
    jointId: 'lumbar' | 'thorax' | 'neck',
    localAngle: number
  ): PoseState {
    return this.commitPose(
      setBalancedSpineLean(this.pose.armature, jointId, localAngle)
    );
  }

  setJointMode(jointId: JointId, mode: JointMode): PoseState {
    return this.commitPose(
      setJointMode(this.pose.armature, jointId, mode),
      this.pose.preset
    );
  }

  solveEffectorIK(
    jointId: JointId,
    target: Point2D,
    options?: {
      solver?: RigSystemsConfig['solver'];
      softDamp?: boolean;
      constrainLimits?: boolean;
    }
  ): PoseState {
    const nextArmature = solveJointIK(
      this.pose.armature,
      jointId,
      target.x,
      target.y,
      this.canvasOrigin,
      {
        solver: options?.solver ?? this.constraints.solver,
        softDamp: options?.softDamp ?? true,
        constrainLimits: options?.constrainLimits ?? this.constraints.softLimits,
      }
    );
    return this.commitPose(nextArmature);
  }

  solveFaceplateLook(
    target: Point2D,
    collarHold?: FaceplateLookCollarHold
  ): PoseState {
    const nextArmature = solveFaceplateLookIK(
      this.pose.armature,
      target.x,
      target.y,
      this.canvasOrigin,
      collarHold
    );
    return this.commitPose(nextArmature);
  }

  solveWaistBalance(
    targetPelvis: Point2D,
    footRPin: GroundContactPin,
    footLPin: GroundContactPin,
    options?: WaistBalanceOptions
  ): PoseState {
    const nextArmature = solveWaistBalanceHula(
      this.pose.armature,
      targetPelvis.x,
      targetPelvis.y,
      footRPin,
      footLPin,
      this.canvasOrigin,
      options
    );
    return this.commitPose(nextArmature);
  }

  solveTorsoBalance(
    targetTorso: Point2D,
    footRPin: GroundContactPin,
    footLPin: GroundContactPin,
    options?: Omit<WaistBalanceOptions, 'torsoCompensation'>
  ): PoseState {
    const nextArmature = solveTorsoBalance(
      this.pose.armature,
      targetTorso.x,
      targetTorso.y,
      footRPin,
      footLPin,
      this.canvasOrigin,
      options
    );
    return this.commitPose(nextArmature);
  }

  setHipToggle(state: ToggleHingeFoldState): PoseState {
    return this.commitPose(setHipToggleState(this.pose.armature, state));
  }

  solveHipToggleTarget(target: Point2D): PoseState {
    return this.commitPose(
      solveHipToggleFromTarget(
        this.pose.armature,
        target.x,
        target.y,
        this.canvasOrigin
      )
    );
  }

  setCollarToggle(side: 'R' | 'L', state: ToggleHingeFoldState): PoseState {
    return this.commitPose(setCollarToggleState(this.pose.armature, side, state));
  }

  solveCollarToggleTarget(side: 'R' | 'L', target: Point2D): PoseState {
    return this.commitPose(
      solveCollarToggleFromTarget(
        this.pose.armature,
        side,
        target.x,
        target.y,
        this.canvasOrigin
      )
    );
  }

  setArmReverseSwivel(side: 'R' | 'L', swivelDeg: number): PoseState {
    return this.commitPose(
      setArmReverseSwivel(this.pose.armature, side, swivelDeg)
    );
  }

  toggleArmSwivel(side: 'R' | 'L'): PoseState {
    return this.commitPose(toggleArmReverseSwivel(this.pose.armature, side));
  }

  setNeckCollarSeparation(separation: Point2D): PoseState {
    return this.commitPose(
      setNeckCollarSeparation(this.pose.armature, separation)
    );
  }

  setFaceplateOffset(offset: Point2D): PoseState {
    return this.commitPose(
      setFaceplateOffset(this.pose.armature, offset.x, offset.y)
    );
  }

  setGroundPressurePlate(targetY: number): {
    pose: PoseState;
    pressurePlateY: number;
    atHardStop: boolean;
  } {
    const solution = solvePressurePlateContact(
      this.pose.armature,
      targetY,
      this.canvasOrigin
    );
    this.constraints = {
      ...this.constraints,
      pressurePlateY: solution.pressurePlateY,
    };
    const pose = this.commitPose(solution.armature, null, true);
    return {
      pose,
      pressurePlateY: solution.pressurePlateY,
      atHardStop: solution.atHardStop,
    };
  }

  mirror(): PoseState {
    return this.commitPose(mirrorPose(this.pose.armature));
  }

  setConstraints(patch: Partial<RigSystemsConfig>): RigSystemsConfig {
    this.constraints = {
      ...this.constraints,
      ...patch,
    };
    if (patch.floorSnap && !this.pose.armature.isAnchorPinned) {
      const snapped = this.applyFloorSnapIfNeeded(this.pose.armature);
      if (snapped !== this.pose.armature) {
        this.pose = { preset: this.pose.preset, armature: snapped };
        this.computedJoints = computeArmature(this.pose.armature, this.canvasOrigin);
      }
    }
    this.notify();
    return this.getConstraints();
  }

  resetToTPose(options?: { preserveConstraints?: boolean }): PoseState {
    const preserveConstraints = options?.preserveConstraints ?? true;
    if (!preserveConstraints) {
      this.constraints = createDefaultConstraints(this.canvasOrigin);
    } else if (this.constraints.pressurePlateY !== undefined) {
      const { pressurePlateY: _omit, ...rest } = this.constraints;
      this.constraints = rest;
    }
    const nextArmature = createPosePreset('tpose', {
      ...this.pose.armature,
      rootX: this.canvasOrigin.x,
      rootY: this.canvasOrigin.y,
      isAnchorPinned: true,
      hipToggleSign: 1,
    });
    return this.commitPose(nextArmature, 'tpose', true);
  }

  getBalanceCenters(): ReturnType<typeof computeBodyBalanceCenters> {
    return computeBodyBalanceCenters(this.computedJoints);
  }

  getGroundContactState(): GroundContactSummary {
    const joints = this.computedJoints;
    const staticGroundY = this.constraints.groundY;
    const fallbackPlateY = Math.max(
      joints.footR.y,
      joints.toeR.endY,
      joints.footL.y,
      joints.toeL.endY
    );
    const activePressurePlateY =
      this.constraints.pressurePlateY ?? fallbackPlateY;

    const isPointGrounded = (y: number) =>
      Math.abs(y - activePressurePlateY) <= GROUND_CONTACT_TOLERANCE_PX;

    const groundedFootCount = [
      [joints.footR.y, joints.toeR.endY],
      [joints.footL.y, joints.toeL.endY],
    ].filter((points) => points.some(isPointGrounded)).length;

    const groundedBodyContact = BODY_CONTACT_JOINT_IDS.some((id) => {
      const joint = joints[id];
      return (
        Math.max(joint.y, joint.endY) >=
        activePressurePlateY - GROUND_CONTACT_TOLERANCE_PX
      );
    });

    const pressureLevel = Math.max(
      0,
      Math.min(
        1,
        0.5 +
          (staticGroundY - activePressurePlateY) / 160 +
          (groundedBodyContact ? 0.12 : 0)
      )
    );

    return {
      groundY: staticGroundY,
      pressurePlateY: activePressurePlateY,
      footR: getGroundContactPin(joints.footR, joints.toeR, activePressurePlateY, 'R'),
      footL: getGroundContactPin(joints.footL, joints.toeL, activePressurePlateY, 'L'),
      groundedFootCount,
      groundedBodyContact,
      pressureLevel,
    };
  }

  getJointConstraintStatus(jointId: JointId): JointConstraintStatus {
    const spec = JOINT_SPEC_MAP[jointId];
    const computed = this.computedJoints[jointId];
    const limits = spec.limits ?? null;
    const localAngle = this.getJointAngle(jointId);
    const atLimit = limits
      ? localAngle <= limits[0] + 1e-3 || localAngle >= limits[1] - 1e-3
      : false;

    return {
      jointId,
      localAngle,
      worldAngle: computed.worldAngle,
      mode: computed.mode,
      limits,
      atLimit,
      isStrained: Boolean(computed.isStrained),
      tensionRatio: computed.tensionRatio ?? 0,
      collarSnapState: computed.collarSnapState,
      collarResistanceRatio: computed.collarResistanceRatio,
      neckCollarPressureRatio: computed.neckCollarPressureRatio,
    };
  }

  getCollarState(side: 'R' | 'L'): CollarStatus {
    const braceId: JointId = side === 'R' ? 'braceR' : 'braceL';
    const outerId: JointId = side === 'R' ? 'clavOuterR' : 'clavOuterL';
    const swivelId: JointId = side === 'R' ? 'swivelR' : 'swivelL';
    const humerusId: JointId = side === 'R' ? 'humerusR' : 'humerusL';
    const brace = this.computedJoints[braceId];
    const outer = this.computedJoints[outerId];
    const swivel = this.computedJoints[swivelId];
    const humerus = this.computedJoints[humerusId];
    const fold = resolveCollarToggleFold(
      side,
      brace.localAngle,
      humerus.localAngle,
      swivel.localAngle
    );
    const foldState: ToggleHingeFoldState =
      fold.effectiveFoldMag <= 0.35
        ? 'extended'
        : fold.foldSign >= 0
        ? 'A'
        : 'V';

    return {
      side,
      innerAngle: brace.localAngle,
      outerAngle: outer.localAngle,
      swivelAngle: swivel.localAngle,
      foldState,
      snapState: swivel.collarSnapState,
      resistanceRatio: swivel.collarResistanceRatio,
    };
  }

  getHipToggleState(): ToggleHinge2LinkResult {
    const joints = this.computedJoints;
    return solveToggleHinge2Link(
      joints.femurR,
      joints.femurL,
      HIP_BRIDGE_LINK_STAGE_PX * (this.pose.armature.joints.pelvis?.scaleWidth ?? 1),
      this.pose.armature.hipToggleSign ?? 1,
      joints.pelvis
    );
  }

  getArmReverseFactor(side: 'R' | 'L'): number {
    const humerusId: JointId = side === 'R' ? 'humerusR' : 'humerusL';
    return this.computedJoints[humerusId].armReverseFactor ?? 1;
  }

  subscribe(listener: (snapshot: PoseEngineSnapshot) => void): () => void {
    this.listeners.add(listener);
    return () => {
      this.listeners.delete(listener);
    };
  }
}

export function createPoseEngine(options?: PoseEngineOptions): PoseEngine {
  return new PoseEngine(options);
}
