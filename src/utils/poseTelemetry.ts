import { ArmatureState, ComputedJoint, JointId } from '../types/rig';

export interface PoseTelemetryJoint {
  id: JointId;
  label: string;
  parentId: JointId | null;
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
  mode: string;
  boneMatrix: readonly [number, number, number, number, number, number];
  isStrained: boolean;
  tensionRatio: number | null;
  collarSnapState: string | null;
  collarResistanceRatio: number | null;
  armReverseFactor: number | null;
  state: ArmatureState['joints'][JointId];
}

export interface PoseTelemetrySnapshot {
  format: 'bitruvius-live-pose';
  version: 1;
  workspaceId: string;
  capturedAt: string;
  scene: {
    id: string;
    name: string;
    timelineT: number;
    durationSec: number;
    playbackSpeed: number;
  };
  armature: ArmatureState;
  joints: Record<JointId, PoseTelemetryJoint>;
}

interface BuildPoseTelemetryInput {
  workspaceId: string;
  capturedAt?: string;
  scene: PoseTelemetrySnapshot['scene'];
  armature: ArmatureState;
  computedJoints: Record<JointId, ComputedJoint>;
}

export function buildPoseTelemetrySnapshot({
  workspaceId,
  capturedAt = new Date().toISOString(),
  scene,
  armature,
  computedJoints,
}: BuildPoseTelemetryInput): PoseTelemetrySnapshot {
  const joints = {} as Record<JointId, PoseTelemetryJoint>;
  for (const id of Object.keys(computedJoints) as JointId[]) {
    const joint = computedJoints[id];
    joints[id] = {
      id,
      label: joint.spec.label,
      parentId: joint.spec.parentId,
      x: joint.x,
      y: joint.y,
      endX: joint.endX,
      endY: joint.endY,
      parentX: joint.parentX,
      parentY: joint.parentY,
      localAngle: joint.localAngle,
      worldAngle: joint.worldAngle,
      restWorldDeg: joint.restWorldDeg,
      cumulativeDeltaDeg: joint.cumulativeDeltaDeg,
      effectiveLengthPx: joint.effectiveLengthPx,
      effectiveWidthScale: joint.effectiveWidthScale,
      mode: joint.mode,
      boneMatrix: joint.boneMatrix,
      isStrained: Boolean(joint.isStrained),
      tensionRatio: joint.tensionRatio ?? null,
      collarSnapState: joint.collarSnapState ?? null,
      collarResistanceRatio: joint.collarResistanceRatio ?? null,
      armReverseFactor: joint.armReverseFactor ?? null,
      state: armature.joints[id],
    };
  }

  return {
    format: 'bitruvius-live-pose',
    version: 1,
    workspaceId,
    capturedAt,
    scene,
    armature,
    joints,
  };
}
