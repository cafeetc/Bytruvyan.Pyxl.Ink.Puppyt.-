import test from 'node:test';
import assert from 'node:assert/strict';
import {
  DEFAULT_COMPOSITE_FILTER,
  DEFAULT_STAGE_TRANSFORM,
  SAMPLE_PLATES,
  advanceSequencePlayback,
  buildStageSvgTransform,
  cloneSceneForNewShot,
  computeMediaPlateBox,
  createDefaultScene,
  createSamplePlateMedia,
  createWorldPinForJoint,
  mediaToWorld,
  resolveVideoTimeForTimeline,
  solveWorldPins,
  worldToMedia,
} from './scene';
import { computeArmature } from './kinematics';
import { PAPER_HEIGHT, PAPER_WIDTH } from './proportions';

test('default Scene[0] preserves 1901 Manual Cover backdrop, neutral StageTransform, and ink filter defaults', () => {
  const scene = createDefaultScene(0, { width: 1440, height: 780 });
  assert.equal(scene.name, 'Shot 1');
  assert.equal(scene.media.kind, 'paper');
  assert.equal(scene.media.paperVariant, 'cover');
  assert.deepEqual(scene.stageTransform, DEFAULT_STAGE_TRANSFORM);
  assert.deepEqual(scene.compositeFilter, DEFAULT_COMPOSITE_FILTER);
  assert.deepEqual(scene.worldPins, []);
  assert.equal(scene.activePreset, 'tpose');
  assert.ok(scene.camera.zoom > 0.1 && scene.camera.zoom <= 2.4);
});

test('mediaToWorld and worldToMedia are exact inverses across offset, scale, and horizon rotation', () => {
  const origin = { x: PAPER_WIDTH / 2, y: PAPER_HEIGHT / 2 };
  const transform = {
    originX: 120,
    originY: -85,
    pyxScale: 0.72,
    rotation: 14.5,
  };
  const pt = { x: origin.x + 210, y: origin.y - 140 };
  const world = mediaToWorld(pt, transform, origin);
  const roundTrip = worldToMedia(world, transform, origin);

  assert.ok(Math.abs(roundTrip.x - pt.x) < 1e-6);
  assert.ok(Math.abs(roundTrip.y - pt.y) < 1e-6);

  const svgStr = buildStageSvgTransform(transform, origin);
  assert.ok(svgStr.includes('rotate(14.5)'));
  assert.ok(svgStr.includes('scale(0.72)'));
});

test('WorldPin locks effector to background plate coordinates across StageTransform changes and respects frameRange', () => {
  const scene = createDefaultScene(0);
  const plateMedia = createSamplePlateMedia('stone_archway');
  const box = computeMediaPlateBox(plateMedia);
  assert.ok(box.width > box.height);
  assert.ok(SAMPLE_PLATES.stone_archway.src.startsWith('data:image/svg+xml'));

  // Pin right foot in a bent-knee test pose at its current plate position
  const bentPose = createDefaultScene(0).armature;
  bentPose.joints.femurR.localAngle = -16;
  bentPose.joints.shinR.localAngle = 24;
  bentPose.joints.footR.localAngle = -8;
  const initialTransform = { originX: 0, originY: 0, pyxScale: 1, rotation: 0 };
  const pin = createWorldPinForJoint(bentPose, 'footR', initialTransform, undefined, [0.2, 0.8]);
  const initialWorld = computeArmature(bentPose).footR;
  const initialPlatePt = mediaToWorld({ x: initialWorld.x, y: initialWorld.y }, initialTransform);
  assert.ok(Math.hypot(pin.mediaSpaceXY.x - initialPlatePt.x, pin.mediaSpaceXY.y - initialPlatePt.y) < 1e-4);

  // Move StageTransform while keeping the planted foot within limb reach
  const shiftedTransform = { originX: 16, originY: 18, pyxScale: 1.04, rotation: 2.5 };
  const solvedArmature = solveWorldPins(bentPose, [pin], 0.5, shiftedTransform);
  const solvedRig = computeArmature(solvedArmature);
  const solvedPlatePt = mediaToWorld({ x: solvedRig.footR.x, y: solvedRig.footR.y }, shiftedTransform);
  const errInside = Math.hypot(solvedPlatePt.x - pin.mediaSpaceXY.x, solvedPlatePt.y - pin.mediaSpaceXY.y);
  assert.ok(errInside < 3, `WorldPin kept footR anchored on plate within 3px (got ${errInside})`);

  // Outside frameRange -> pin is inactive
  const outsideArmature = solveWorldPins(bentPose, [pin], 0.05, shiftedTransform);
  assert.strictEqual(outsideArmature, bentPose);
});

test('video timeline sync quantizes cleanly to cutout FPS and sequence playback transitions across shots', () => {
  const t12 = resolveVideoTimeForTimeline(0.5, 4.0, 4.0, 12, true, 0);
  assert.equal(t12, 2);
  const tStep = resolveVideoTimeForTimeline(0.11, 4.0, 4.0, 12, true, 0);
  // 0.11 * 4.0 = 0.44s -> floor(0.44 * 12) / 12 = 5 / 12 = 0.4167
  assert.ok(Math.abs(tStep - 5 / 12) < 1e-3);

  const s1 = createDefaultScene(0);
  const s2 = cloneSceneForNewShot(s1, 1);
  assert.equal(s2.name, 'Shot 2');
  assert.notEqual(s2.id, s1.id);

  // Scene loop mode wraps inside Shot 1
  const stayScene = advanceSequencePlayback([s1, s2], s1.id, 0.95, 0.4, 1, 'scene');
  assert.equal(stayScene.nextSceneId, s1.id);
  assert.ok(stayScene.nextT < 0.2);

  // Sequence mode advances from Shot 1 -> Shot 2 when crossing t >= 1
  const nextShot = advanceSequencePlayback([s1, s2], s1.id, 0.95, 0.4, 1, 'sequence');
  assert.equal(nextShot.nextSceneId, s2.id);
  assert.ok(nextShot.nextT >= 0 && nextShot.nextT < 0.2);
});
