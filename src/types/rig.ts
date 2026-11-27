export type JointMode = 'IK' | 'FK' | 'PIN' | 'ROOT';

/**
 * Canonical 43-joint articulated digital puppet IDs (split folding collarbones, no Prometheus wings).
 */
export type JointId =
  | 'pelvis'
  | 'lumbar'
  | 'thorax'
  | 'ribR'
  | 'ribL'
  | 'neck'
  | 'head'
  | 'lip'
  | 'chinTip'
  | 'swivelR'
  | 'swivelTipR'
  | 'acromionR'
  | 'braceR'
  | 'clavOuterR'
  | 'humerusR'
  | 'forearmR'
  | 'handR'
  | 'thumbR'
  | 'thumbTipR'
  | 'fingerR'
  | 'fingerTipR'
  | 'swivelL'
  | 'swivelTipL'
  | 'acromionL'
  | 'braceL'
  | 'clavOuterL'
  | 'humerusL'
  | 'forearmL'
  | 'handL'
  | 'thumbL'
  | 'thumbTipL'
  | 'fingerL'
  | 'fingerTipL'
  | 'hipR'
  | 'femurR'
  | 'shinR'
  | 'footR'
  | 'toeR'
  | 'hipL'
  | 'femurL'
  | 'shinL'
  | 'footL'
  | 'toeL';

export type PuppytJointRole =
  | 'root'
  | 'spine'
  | 'ribcage'
  | 'head_facial'
  | 'shoulder_girdle'
  | 'accessory_rig'
  | 'limb_arm'
  | 'hand_digit'
  | 'limb_leg'
  | 'foot_digit';

export type BodyPartCategory =
  | 'pelvis'
  | 'thighs'
  | 'calves'
  | 'feet'
  | 'spine'
  | 'ribs'
  | 'girdle'
  | 'upper_arms'
  | 'forearms'
  | 'hands'
  | 'digits'
  | 'neck'
  | 'head';

export type KeystoneOrigin = 'navel' | 'crown';

export type EnvironmentalAnchorId =
  | 'bottom_of_spine'
  | 'feet'
  | 'navel'
  | 'clavicle'
  | 'neck'
  | 'crown';

export type PhysicsModeId =
  | 'twins'
  | 'elastic_rubber'
  | 'damp'
  | 'rigid'
  | 'mode_e';

/**
 * POLICY (Canonical Poses):
 * - Keep only calibrated canonical poses here (currently solely `'tpose'`).
 * - NO LLM may ever invent or add default poses without explicit user prompt or approval.
 * - Any pose preset added or updated MUST be calibrated directly against current rig updates.
 */
export type PosePresetId = 'tpose';

export interface AimConstraintSpec {
  to: JointId;
  minRatio: number;
  maxRatio: number;
}

export type IKChainDef =
  | readonly [JointId, JointId, JointId]
  | readonly [JointId, JointId]
  | readonly [JointId, JointId, JointId, JointId]
  | 'spine'
  | 'spine_neck'
  | 'spine_thorax'
  | 'jaw'
  | 'shoulder';

export interface JointSpec {
  id: JointId;
  label: string;
  shortCode: string;
  subtitle: string;
  parentId: JointId | null;
  role: PuppytJointRole;
  side: 'center' | 'right' | 'left';
  restPivot: readonly [number, number];
  restTip?: readonly [number, number];
  lengthPx: number;
  endExtensionPx?: number;
  bodyPartCategory: BodyPartCategory;
  isSpine?: boolean;
  isTip?: boolean;
  locked?: boolean;
  isGhost?: boolean;
  aimTarget?: AimConstraintSpec;
  limits?: readonly [number, number];
  defaultLocalAngle: number;
  defaultMode: JointMode;
  ikChain?: IKChainDef;
  bendDirection?: 1 | -1;
  ikSolverType?: 'analytical' | 'aim' | 'fabrik' | 'ccd';
}

export interface JointRuntimeState {
  localAngle: number;
  mode: JointMode;
  physicsMode: PhysicsModeId;
  modeERatio: number;
  scaleLength: number;
  scaleWidth: number;
}

export interface ComputedJoint {
  id: JointId;
  spec: JointSpec;
  x: number;
  y: number;
  endX: number;
  endY: number;
  parentX: number | null;
  parentY: number | null;
  localAngle: number;
  worldAngle: number;
  restWorldDeg: number;
  cumulativeDeltaDeg: number;
  effectiveLengthPx: number;
  effectiveWidthScale: number;
  mode: JointMode;
  boneMatrix: readonly [number, number, number, number, number, number];
  isStrained?: boolean;
  tensionRatio?: number;
  collarSnapState?: 'normal' | 'resisting' | 'snapped';
  collarResistanceRatio?: number;
  neckCollarPressureRatio?: number;
  armReverseFactor?: number;
}

export interface RigHierarchyAuditReport {
  valid: boolean;
  totalJoints: number;
  orphanNodes: string[];
  cycles: string[];
  topologicalViolations: string[];
  aimOrderViolations: string[];
  parentMismatches: Array<{ id: string; expected: string | null; actual: string | null }>;
}

export type ToggleHingeFoldState = 'extended' | 'A' | 'V';

export interface ToggleHinge2LinkResult {
  pLeft: Point2D;
  pMid: Point2D;
  pRight: Point2D;
  linkLength: number;
  span: number;
  state: ToggleHingeFoldState;
  foldSign: 1 | -1;
  interiorAngleDeg: number;
  leftAngleDeg: number;
  rightAngleDeg: number;
}

export interface ArmatureState {
  rootX: number;
  rootY: number;
  rootRotation: number;
  hipToggleSign?: 1 | -1;
  armSwivelR?: number;
  armSwivelL?: number;
  faceplateOffsetX?: number;
  faceplateOffsetY?: number;
  neckLift?: number;
  neckCollarSeparation?: Point2D;
  continuousSpin: boolean;
  internalKeystone: KeystoneOrigin;
  environmentalAnchor: EnvironmentalAnchorId;
  isAnchorPinned: boolean;
  joints: Record<JointId, JointRuntimeState>;
}

export interface Point2D {
  x: number;
  y: number;
}

export interface GroundContactPin extends Point2D {
  jointId?: 'footR' | 'toeR' | 'footL' | 'toeL';
  supportWeight?: number;
  worldAngle?: number;
}

export type CaveArtShapeKind = 'triangle' | 'trapezoid' | 'square';

export type CaveArtPartId =
  | 'cranium-trapezoid'
  | 'jaw-triangle'
  | 'neck-trapezoid'
  | 'upper-torso-trapezoid'
  | 'upper-torso-triangle'
  | 'lower-spine-triangle'
  | 'pelvis-hip-r'
  | 'pelvis-hip-l'
  | 'humerus-r'
  | 'forearm-r'
  | 'elbow-r'
  | 'palm-r'
  | 'thumb-r'
  | 'finger-r'
  | 'humerus-l'
  | 'forearm-l'
  | 'elbow-l'
  | 'palm-l'
  | 'thumb-l'
  | 'finger-l'
  | 'femur-r'
  | 'knee-r'
  | 'shin-r'
  | 'heel-r'
  | 'foot-r'
  | 'femur-l'
  | 'knee-l'
  | 'shin-l'
  | 'heel-l'
  | 'foot-l';

export interface CaveArtPolygon {
  id: CaveArtPartId;
  jointId: JointId;
  shapeKind: CaveArtShapeKind;
  points: readonly Point2D[];
  layer: 'back' | 'torso' | 'pelvis' | 'limb' | 'cap' | 'head';
  fillOpacity: number;
}

export interface CaveArtLine {
  id: string;
  jointId: JointId;
  x1: number;
  y1: number;
  x2: number;
  y2: number;
  role: 'axis' | 'girdle' | 'rib' | 'pelvis' | 'limb' | 'digit' | 'skull' | 'strained';
  dashed?: boolean;
  overlayLight?: boolean;
}

export interface CaveArtRigGeometry {
  polygons: CaveArtPolygon[];
  functionalLines: CaveArtLine[];
  metrics: {
    hipWidthPx: number;
    torsoWidthPx: number;
    torsoToHipRatio: number;
    lowerSpineBaseWidthPx: number;
    upperLowerSpineMeetingPoint: Point2D;
  };
}

export const ANGLE_PRECISION_STEP = 0.25;
export const BONE_SCALE_MIN = 0.5;
export const BONE_SCALE_MAX = 2.0;

// Playground Studio Extensions
export type PlaygroundTab = 'poses' | 'timeline' | 'ik_systems' | 'digits' | 'perspective' | 'scene';

export type KeyframeSlot = 'A' | 'B' | 'C';

export type MotionCurve =
  | 'flow'
  | 'snap'
  | 'elastic'
  | 'mechanical'
  | 'clockwork'
  | 'procedural';

export interface RigSystemsConfig {
  softLimits: boolean;
  floorSnap: boolean;
  elasticPin: boolean;
  antiPhallic: boolean;
  groundY: number;
  pressurePlateY?: number;
  solver: 'analytical' | 'fabrik' | 'ccd';
}

export type HandGesturePreset = 'open' | 'fist' | 'point' | 'pinch' | 'relaxed' | 'grasp';

export interface UserSavedPose {
  id: string;
  name: string;
  timestamp: number;
  armature: ArmatureState;
}
