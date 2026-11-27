import { JointId } from '../types/rig';
import { PoseTelemetrySnapshot } from './poseTelemetry';

export interface PoseAngleRange {
  sampleCount: number;
  localAngleDeg: { min: number; max: number };
  worldAngleDeg: { min: number; max: number };
}

export interface PoseUsageSummary {
  label: string;
  sampleCount: number;
  firstCapturedAt: string;
  lastCapturedAt: string;
  joints: Partial<Record<JointId, PoseAngleRange>>;
}

export interface PoseUsageLog {
  format: 'bitruvius.pose-usage-log';
  version: 1;
  updatedAt: string;
  sampleCount: number;
  poses: PoseUsageSummary[];
}

function isRecord(value: unknown): value is Record<string, unknown> {
  return typeof value === 'object' && value !== null && !Array.isArray(value);
}

function isFiniteNumber(value: unknown): value is number {
  return typeof value === 'number' && Number.isFinite(value);
}

function isNonNegativeInteger(value: unknown): value is number {
  return Number.isInteger(value) && typeof value === 'number' && value >= 0;
}

function isPositiveInteger(value: unknown): value is number {
  return isNonNegativeInteger(value) && value > 0;
}

function isAngleRange(value: unknown): value is { min: number; max: number } {
  return (
    isRecord(value) &&
    isFiniteNumber(value.min) &&
    isFiniteNumber(value.max) &&
    value.min <= value.max
  );
}

function isPoseUsageSummary(value: unknown): value is PoseUsageSummary {
  if (
    !isRecord(value) ||
    typeof value.label !== 'string' ||
    !isPositiveInteger(value.sampleCount) ||
    typeof value.firstCapturedAt !== 'string' ||
    typeof value.lastCapturedAt !== 'string' ||
    !isRecord(value.joints)
  ) {
    return false;
  }

  return Object.values(value.joints).every(
    (joint) =>
      isRecord(joint) &&
      isPositiveInteger(joint.sampleCount) &&
      isAngleRange(joint.localAngleDeg) &&
      isAngleRange(joint.worldAngleDeg)
  );
}

function isPoseUsageLog(value: unknown): value is PoseUsageLog {
  return (
    isRecord(value) &&
    value.format === 'bitruvius.pose-usage-log' &&
    value.version === 1 &&
    typeof value.updatedAt === 'string' &&
    isNonNegativeInteger(value.sampleCount) &&
    Array.isArray(value.poses) &&
    value.poses.every(isPoseUsageSummary)
  );
}

export function createEmptyPoseUsageLog(now = new Date()): PoseUsageLog {
  return {
    format: 'bitruvius.pose-usage-log',
    version: 1,
    updatedAt: now.toISOString(),
    sampleCount: 0,
    poses: [],
  };
}

export function parsePoseUsageLog(raw: string): PoseUsageLog {
  const parsed: unknown = JSON.parse(raw);
  if (!isPoseUsageLog(parsed)) {
    throw new Error('The selected document is not a valid Bitruvius pose usage log.');
  }

  return parsed;
}

function isPoseTelemetrySnapshot(value: unknown): value is PoseTelemetrySnapshot {
  if (
    !isRecord(value) ||
    value.format !== 'bitruvius-live-pose' ||
    value.version !== 1 ||
    typeof value.workspaceId !== 'string' ||
    typeof value.capturedAt !== 'string' ||
    !Number.isFinite(Date.parse(value.capturedAt)) ||
    !isRecord(value.scene) ||
    typeof value.scene.id !== 'string' ||
    typeof value.scene.name !== 'string' ||
    !isRecord(value.armature) ||
    !isRecord(value.armature.joints) ||
    !isRecord(value.joints)
  ) {
    return false;
  }

  const armatureJointIds = new Set(Object.keys(value.armature.joints));
  const joints = Object.entries(value.joints);
  if (
    joints.length === 0 ||
    joints.some(
      ([id, joint]) =>
        !armatureJointIds.has(id) ||
        !isRecord(joint) ||
        !isFiniteNumber(joint.localAngle) ||
        !isFiniteNumber(joint.worldAngle)
    )
  ) {
    return false;
  }

  return true;
}

export function parsePoseTelemetrySnapshot(raw: string): PoseTelemetrySnapshot {
  const parsed: unknown = JSON.parse(raw);
  if (!isPoseTelemetrySnapshot(parsed)) {
    throw new Error('The selected file is not a valid Bitruvius pose telemetry snapshot or has invalid joint angles.');
  }

  return parsed;
}

function roundedAngle(value: number): number {
  const scaled = value * 100;
  return Math.trunc(scaled + Math.sign(scaled) * 0.5) / 100;
}

function extendRange(
  range: { min: number; max: number },
  value: number
): { min: number; max: number } {
  const angle = roundedAngle(value);
  return { min: Math.min(range.min, angle), max: Math.max(range.max, angle) };
}

export function addPoseTelemetrySample(
  log: PoseUsageLog,
  snapshot: PoseTelemetrySnapshot,
  label: string,
  now = new Date()
): PoseUsageLog {
  const normalizedLabel = label.trim() || snapshot.scene.name.trim() || 'Unlabeled pose';
  const capturedAtMs = Date.parse(snapshot.capturedAt);
  const capturedAt = new Date(capturedAtMs).toISOString();
  const poseIndex = log.poses.findIndex((pose) => pose.label === normalizedLabel);
  const existingPose = poseIndex >= 0 ? log.poses[poseIndex] : null;
  const joints = { ...(existingPose?.joints ?? {}) };

  for (const id of Object.keys(snapshot.joints) as JointId[]) {
    const joint = snapshot.joints[id];
    const existing = joints[id];
    joints[id] = existing
      ? {
          sampleCount: existing.sampleCount + 1,
          localAngleDeg: extendRange(existing.localAngleDeg, joint.localAngle),
          worldAngleDeg: extendRange(existing.worldAngleDeg, joint.worldAngle),
        }
      : {
          sampleCount: 1,
          localAngleDeg: {
            min: roundedAngle(joint.localAngle),
            max: roundedAngle(joint.localAngle),
          },
          worldAngleDeg: {
            min: roundedAngle(joint.worldAngle),
            max: roundedAngle(joint.worldAngle),
          },
        };
  }

  const pose: PoseUsageSummary = {
    label: normalizedLabel,
    sampleCount: (existingPose?.sampleCount ?? 0) + 1,
    firstCapturedAt:
      existingPose && Date.parse(existingPose.firstCapturedAt) < capturedAtMs
        ? existingPose.firstCapturedAt
        : capturedAt,
    lastCapturedAt:
      existingPose && Date.parse(existingPose.lastCapturedAt) > capturedAtMs
        ? existingPose.lastCapturedAt
        : capturedAt,
    joints,
  };
  const poses = [...log.poses];
  if (poseIndex >= 0) poses[poseIndex] = pose;
  else poses.push(pose);

  return {
    format: 'bitruvius.pose-usage-log',
    version: 1,
    updatedAt: now.toISOString(),
    sampleCount: log.sampleCount + 1,
    poses,
  };
}
