import test from 'node:test';
import assert from 'node:assert/strict';
import { JOINT_SPECS } from '../data/digitalRig';
import { EXPORT_IR_SCHEMA, EXPORT_IR_VERSION } from '../types/exportIR';
import {
  adaptIRToCanonicalJson,
  adaptIRToCanonicalJsonBlob,
  adaptIRToDragonBonesJson,
  adaptIRToDragonBonesJsonBlob,
  adaptIRToLayeredSvg,
  adaptIRToLayeredSvgBlob,
  adaptIRToSpineJson,
  adaptIRToSpineJsonBlob,
  buildExportIR,
} from './exportAdapters';
import { computeArmature, createPosePreset, setCollarToggleState } from './kinematics';
import { PYXL_PX } from './proportions';

function makeFixturePose(armAngle = -45, legAngle = 14) {
  const pose = createPosePreset('tpose');
  pose.joints.humerusR.localAngle = armAngle;
  pose.joints.forearmR.localAngle = 24;
  pose.joints.femurR.localAngle = -legAngle;
  pose.joints.shinR.localAngle = legAngle;
  return pose;
}

function makeFixtureIR() {
  const poseA = createPosePreset('tpose');
  const poseB = makeFixturePose(-45, 14);
  const poseC = setCollarToggleState(makeFixturePose(-68, 22), 'R', 'A');
  const current = makeFixturePose(-45, 14);
  current.armSwivelR = 180;

  return buildExportIR({
    armature: current,
    preset: 'tpose',
    sceneId: 'scene-test',
    sceneName: 'Test Scene',
    slots: { A: poseA, B: poseB, C: poseC },
    motionCurve: 'flow',
    durationSec: 2.0,
    fps: 24,
    sampleCount: 5,
  });
}

test('buildExportIR captures versioned schema, 43-joint topological hierarchy, PYX rigid lengths, and transforms', () => {
  const ir = makeFixtureIR();

  assert.equal(ir.schema, EXPORT_IR_SCHEMA);
  assert.equal(ir.version, EXPORT_IR_VERSION);
  assert.equal(ir.units.pxPerPyx, PYXL_PX);
  assert.equal(ir.units.positionSpace, 'origin-centered-pyx');
  assert.equal(ir.hierarchy.rootBoneId, 'pelvis');
  assert.equal(ir.hierarchy.boneOrder.length, 43);
  assert.equal(ir.hierarchy.bones.length, 43);

  // Parent always precedes child in topological hierarchy
  const indexById = new Map(ir.hierarchy.boneOrder.map((id, idx) => [id, idx]));
  for (const bone of ir.hierarchy.bones) {
    if (bone.parentId) {
      const parentIdx = indexById.get(bone.parentId);
      const childIdx = indexById.get(bone.id);
      assert.ok(parentIdx !== undefined && childIdx !== undefined && parentIdx < childIdx);
    }
  }

  // PYX rigid lengths match computed armature px / PYXL_PX
  const poseCurrent = makeFixturePose(-45, 14);
  poseCurrent.armSwivelR = 180;
  const computed = computeArmature(poseCurrent);
  for (const bone of ir.hierarchy.bones) {
    const expectedLenPyx = computed[bone.id].effectiveLengthPx / PYXL_PX;
    assert.ok(Math.abs(bone.lengthPyx - expectedLenPyx) < 1e-3);
  }
  assert.equal(ir.pose.root.armSwivelR, 180);
  assert.equal(ir.hierarchy.bones.find((b) => b.id === 'lumbar')?.transform.world.positionPyx.x, 0);
  assert.equal(ir.hierarchy.bones.find((b) => b.id === 'lumbar')?.transform.world.positionPyx.y, 0);
});

test('buildExportIR samples interpolated timeline keyframes cleanly across A -> B -> C', () => {
  const ir = makeFixtureIR();

  assert.equal(ir.timeline.sampleCount, 5);
  assert.equal(ir.timeline.keyframes.length, 5);
  assert.deepEqual(ir.timeline.activeSlots, ['A', 'B', 'C']);

  assert.equal(ir.timeline.keyframes[0].t, 0);
  assert.equal(ir.timeline.keyframes[0].sourceSlot, 'A');
  assert.equal(ir.timeline.keyframes[2].t, 0.5);
  assert.equal(ir.timeline.keyframes[2].sourceSlot, 'B');
  assert.equal(ir.timeline.keyframes[4].t, 1);
  assert.equal(ir.timeline.keyframes[4].sourceSlot, 'C');

  // Interpolated midpoint keyframes (t=0.25 and t=0.75) stay finite across all 43 joints
  for (const kf of ir.timeline.keyframes) {
    for (const spec of JOINT_SPECS) {
      const j = kf.joints[spec.id];
      assert.ok(j, `keyframe ${kf.index} has joint ${spec.id}`);
      assert.ok(Number.isFinite(j.local.angleDeg));
      assert.ok(Number.isFinite(j.world.positionPyx.x) && Number.isFinite(j.world.positionPyx.y));
    }
  }
});

test('all pure adapters are byte-for-byte deterministic and emit valid Blobs and target schemas', async () => {
  const ir = makeFixtureIR();

  // Canonical JSON determinism & sorted keys
  const jsonA = adaptIRToCanonicalJson(ir);
  const jsonB = adaptIRToCanonicalJson(ir);
  assert.equal(jsonA, jsonB);
  const jsonBlob = adaptIRToCanonicalJsonBlob(ir);
  assert.equal(jsonBlob.type, 'application/json;charset=utf-8');
  assert.equal(await jsonBlob.text(), jsonA);
  const parsedIR = JSON.parse(jsonA);
  assert.equal(parsedIR.schema, EXPORT_IR_SCHEMA);
  assert.equal(parsedIR.version, EXPORT_IR_VERSION);

  // Layered SVG determinism & semantic groups
  const svgA = adaptIRToLayeredSvg(ir);
  const svgB = adaptIRToLayeredSvg(ir);
  assert.equal(svgA, svgB);
  assert.ok(svgA.includes('<svg ') && svgA.includes('</svg>'));
  assert.ok(svgA.includes('id="layer-guides-pyx"'));
  assert.ok(svgA.includes('id="layer-bones"'));
  assert.ok(svgA.includes('id="layer-joints"'));
  assert.ok(svgA.includes(`data-ir-version="${EXPORT_IR_VERSION}"`));
  const svgBlob = adaptIRToLayeredSvgBlob(ir);
  assert.equal(svgBlob.type, 'image/svg+xml;charset=utf-8');
  assert.equal(await svgBlob.text(), svgA);

  // Spine JSON determinism & schema invariants
  const spineA = adaptIRToSpineJson(ir);
  const spineB = adaptIRToSpineJson(ir);
  assert.equal(spineA, spineB);
  const spineParsed = JSON.parse(spineA);
  assert.equal(spineParsed.skeleton.spine, '4.1.24');
  assert.equal(spineParsed.bones.length, 43);
  assert.equal(spineParsed.bones[0].name, 'pelvis');
  assert.equal(spineParsed.animations.bitruvius_clip.bones.humerusR.rotate.length, 5);
  const spineBlob = adaptIRToSpineJsonBlob(ir);
  assert.equal(await spineBlob.text(), spineA);

  // DragonBones JSON determinism & schema invariants
  const dbA = adaptIRToDragonBonesJson(ir);
  const dbB = adaptIRToDragonBonesJson(ir);
  assert.equal(dbA, dbB);
  const dbParsed = JSON.parse(dbA);
  assert.equal(dbParsed.version, '5.5');
  assert.equal(dbParsed.armature[0].bone.length, 43);
  assert.equal(dbParsed.armature[0].animation[0].name, 'bitruvius_clip');
  assert.equal(dbParsed.armature[0].animation[0].bone[0].rotateFrame.length, 5);
  const dbBlob = adaptIRToDragonBonesJsonBlob(ir);
  assert.equal(await dbBlob.text(), dbA);
});
