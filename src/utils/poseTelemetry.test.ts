import assert from 'node:assert/strict';
import test from 'node:test';
import { createDefaultScene } from './scene';
import { computeArmature, createPosePreset } from './kinematics';
import { buildPoseTelemetrySnapshot } from './poseTelemetry';
import { PAPER_HEIGHT, PAPER_WIDTH } from './proportions';

test('pose telemetry captures the complete 43-joint runtime and computed pose', () => {
  const scene = createDefaultScene(0, { width: 1440, height: 780 });
  const armature = createPosePreset('tpose', scene.armature);
  const computedJoints = computeArmature(armature, { x: PAPER_WIDTH / 2, y: PAPER_HEIGHT / 2 });
  const snapshot = buildPoseTelemetrySnapshot({
    workspaceId: 'bitruvius-workspace',
    capturedAt: '2026-11-27T00:00:00.000Z',
    scene: {
      id: scene.id,
      name: scene.name,
      timelineT: scene.timelineT,
      durationSec: scene.durationSec,
      playbackSpeed: scene.playbackSpeed,
    },
    armature,
    computedJoints,
  });

  assert.equal(Object.keys(snapshot.joints).length, 43);
  assert.deepEqual(Object.keys(snapshot.joints).sort(), Object.keys(armature.joints).sort());
  assert.equal(snapshot.joints.head.id, 'head');
  assert.equal(snapshot.joints.head.state, armature.joints.head);
  assert.equal(snapshot.joints.head.boneMatrix.length, 6);
  assert.equal(snapshot.joints.head.endX, computedJoints.head.endX);
  assert.equal(snapshot.workspaceId, 'bitruvius-workspace');
});
