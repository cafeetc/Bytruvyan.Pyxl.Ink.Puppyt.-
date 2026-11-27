import assert from 'node:assert/strict';
import test from 'node:test';
import {
  addPoseTelemetrySample,
  createEmptyPoseUsageLog,
  parsePoseTelemetrySnapshot,
  parsePoseUsageLog,
} from './poseUsageLog';

function createSnapshot(capturedAt: string, localAngle: number, worldAngle: number): string {
  return JSON.stringify({
    format: 'bitruvius-live-pose',
    version: 1,
    workspaceId: 'test',
    capturedAt,
    scene: { id: 'scene-1', name: 'Standing' },
    armature: { joints: { pelvis: {} } },
    joints: { pelvis: { localAngle, worldAngle } },
  });
}

test('pose usage ingestion stores only compact counts and angle ranges per label', () => {
  const first = parsePoseTelemetrySnapshot(createSnapshot('2026-01-02T00:00:00.000Z', -12.345, 3));
  const second = parsePoseTelemetrySnapshot(createSnapshot('2026-01-01T00:00:00.000Z', 18, 9.876));
  const initial = createEmptyPoseUsageLog(new Date('2026-01-03T00:00:00.000Z'));

  const afterFirst = addPoseTelemetrySample(initial, first, 'Standing');
  const merged = addPoseTelemetrySample(afterFirst, second, 'Standing');

  assert.equal(merged.sampleCount, 2);
  assert.equal(merged.poses.length, 1);
  assert.equal(merged.poses[0].sampleCount, 2);
  assert.equal(merged.poses[0].firstCapturedAt, '2026-01-01T00:00:00.000Z');
  assert.equal(merged.poses[0].lastCapturedAt, '2026-01-02T00:00:00.000Z');
  assert.deepEqual(merged.poses[0].joints.pelvis, {
    sampleCount: 2,
    localAngleDeg: { min: -12.35, max: 18 },
    worldAngleDeg: { min: 3, max: 9.88 },
  });

  const parsed = parsePoseUsageLog(JSON.stringify(merged));
  assert.deepEqual(parsed, merged);
  assert.equal(JSON.stringify(parsed).includes('workspaceId'), false);
});

test('pose telemetry ingestion rejects unrelated or incomplete JSON', () => {
  assert.throws(() => parsePoseTelemetrySnapshot('{"format":"other"}'), /pose telemetry snapshot/);
  assert.throws(
    () => parsePoseTelemetrySnapshot(createSnapshot('invalid-date', 0, 0)),
    /pose telemetry snapshot/
  );
});

test('pose usage document parsing rejects an invalid schema', () => {
  assert.throws(() => parsePoseUsageLog('{"format":"other"}'), /valid Bitruvius pose usage log/);
});
