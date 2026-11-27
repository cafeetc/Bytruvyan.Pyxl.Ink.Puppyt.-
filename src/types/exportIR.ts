import {
  JointId,
  JointMode,
  KeyframeSlot,
  MotionCurve,
  PhysicsModeId,
  PosePresetId,
  PuppytJointRole,
} from './rig';

export const EXPORT_IR_SCHEMA = 'bitruvius.export.ir' as const;
export const EXPORT_IR_VERSION = '1.0.0' as const;

export type ExportIRFormat = 'ir_json' | 'layered_svg' | 'spine_json' | 'dragonbones_json';

export interface ExportIRPointPYX {
  x: number;
  y: number;
}

export interface ExportIRTransformPair {
  local: {
    angleDeg: number;
    rotationDeltaDeg: number;
  };
  world: {
    positionPyx: ExportIRPointPYX;
    endPositionPyx: ExportIRPointPYX;
    angleDeg: number;
    restAngleDeg: number;
    matrixPyx: readonly [number, number, number, number, number, number];
  };
}

export interface ExportIRBoneConstraints {
  limitsDeg: readonly [number, number] | null;
  collarSnapState?: 'normal' | 'resisting' | 'snapped';
  collarResistanceRatio?: number;
  armReverseFactor?: number;
  isStrained?: boolean;
  tensionRatio?: number;
}

export interface ExportIRBoneNode {
  id: JointId;
  label: string;
  shortCode: string;
  parentId: JointId | null;
  children: JointId[];
  depth: number;
  role: PuppytJointRole;
  side: 'center' | 'right' | 'left';
  mode: JointMode;
  physicsMode: PhysicsModeId;
  lengthPyx: number;
  restLengthPyx: number;
  scaleLength: number;
  scaleWidth: number;
  isTip: boolean;
  constraints: ExportIRBoneConstraints;
  transform: ExportIRTransformPair;
}

export interface ExportIRRootPose {
  positionPyx: ExportIRPointPYX;
  rotationDeg: number;
  hipToggleSign: 1 | -1;
  armSwivelR: number;
  armSwivelL: number;
}

export interface ExportIRKeyframeJoint extends ExportIRTransformPair {
  lengthPyx: number;
}

export interface ExportIRKeyframe {
  index: number;
  t: number;
  timeSec: number;
  sourceSlot: KeyframeSlot | null;
  root: ExportIRRootPose;
  joints: Record<JointId, ExportIRKeyframeJoint>;
}

export interface ExportIRTimeline {
  durationSec: number;
  fps: number;
  motionCurve: MotionCurve;
  activeSlots: KeyframeSlot[];
  sampleCount: number;
  keyframes: ExportIRKeyframe[];
}

export interface BitruviusExportIR {
  schema: typeof EXPORT_IR_SCHEMA;
  version: typeof EXPORT_IR_VERSION;
  exportedAt: string;
  sceneId: string;
  sceneName: string;
  preset: PosePresetId | 'custom';
  units: {
    pyxPerPx: number;
    pxPerPyx: number;
    canvasSizePyx: { width: number; height: number };
    originPyx: ExportIRPointPYX;
    positionSpace: 'origin-centered-pyx';
    coordinateSystem: 'y-down-2d';
  };
  hierarchy: {
    rootBoneId: JointId;
    boneOrder: JointId[];
    bones: ExportIRBoneNode[];
  };
  pose: {
    root: ExportIRRootPose;
  };
  timeline: ExportIRTimeline;
}
