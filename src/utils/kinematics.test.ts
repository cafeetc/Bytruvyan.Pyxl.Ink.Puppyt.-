import test from 'node:test';
import assert from 'node:assert/strict';
import {
  COLLAR_LINK_SCALE,
  JOINT_SPECS,
  REPO_GREEN_GLOW_DARK,
  REPO_GREEN_GLOW_LIGHT,
} from '../data/digitalRig';
import {
  DEFAULT_SCREEN_FPS,
  FPS_OPTIONS,
  getFrameIntervalMs,
  getNextFpsOption,
  remainingScreenFrameWaitMs,
  shouldEmitScreenFrame,
} from './frameThrottle';
import {
  buildSavedPosePayload,
  computeUncutExportViewBox,
} from './poseExport';
import {
  COLLAR_INWARD_BREAKAWAY_DEG,
  COLLAR_INWARD_NORMAL_LIMIT_DEG,
  COLLAR_INWARD_RESISTANCE_START_DEG,
  HIP_BRIDGE_LINK_STAGE_PX,
  HIP_BRIDGE_LINK_SVG,
  applyAffine,
  applyCollarSwivelDragInput,
  auditRigHierarchy,
  buildCaveArtGeometry,
  computeArmature,
  computeHipThighExtension,
  computePoseTransitionFrame,
  computeSvgBoneMatrix,
  createPosePreset,
  hasArmaturePoseDelta,
  mirrorPose,
  resolveCollarInwardBehavior,
  resolveCollarToggleFold,
  setArmReverseSwivel,
  setCollarToggleState,
  setHipToggleState,
  shortestAngleDelta,
  solveBoneAimRotation,
  solveCollarToggleFromTarget,
  solveHipToggleFromTarget,
  solveToggleHinge2Link,
  solveTwoBoneIK,
  solveSingleBoneAimIK,
  solveJointIK,
  solveWaistBalanceHula,
  interpolateArmatures,
  synthesizeProceduralBreakdown,
  toggleArmReverseSwivel,
} from './kinematics';

test('43-joint rig hierarchy passes strict topological & closed-chain audit without Prometheus wings', () => {
  const report = auditRigHierarchy(JOINT_SPECS);
  assert.equal(report.totalJoints, 43);
  assert.equal(report.valid, true);
  assert.deepEqual(report.orphanNodes, []);
  assert.deepEqual(report.cycles, []);
  assert.deepEqual(report.topologicalViolations, []);
  assert.deepEqual(report.aimOrderViolations, []);
  assert.deepEqual(report.parentMismatches, []);

  const ids = new Set(JOINT_SPECS.map((s) => s.id as string));
  assert.equal(ids.has('wingR'), false, 'wingR removed');
  assert.equal(ids.has('wingL'), false, 'wingL removed');
});

test('canonical T-pose evaluates all 43 joints cleanly without NaN and pins waist/navel to canvas origin', () => {
  const origin = { x: 681.5, y: 900 };
  const state = createPosePreset('tpose');
  const computed = computeArmature(state, origin);

  for (const spec of JOINT_SPECS) {
    const j = computed[spec.id];
    assert.ok(Number.isFinite(j.x) && Number.isFinite(j.y), `tpose.${spec.id} x/y finite`);
    assert.ok(Number.isFinite(j.endX) && Number.isFinite(j.endY), `tpose.${spec.id} end finite`);
  }

  assert.ok(Math.abs(computed.lumbar.x - origin.x) < 1e-4, 'tpose lumbar pinned x');
  assert.ok(Math.abs(computed.lumbar.y - origin.y) < 1e-4, 'tpose lumbar pinned y');
});

test('default pose matches uploaded SVG model (horizontal arms, wide A-stance legs, outward feet, no wings)', () => {
  const state = createPosePreset('tpose');
  const computed = computeArmature(state);
  const geom = buildCaveArtGeometry(computed);

  // Horizontal arms straight out left and right
  assert.ok(Math.abs(computed.humerusR.y - computed.forearmR.y) < 1e-4);
  assert.ok(Math.abs(computed.forearmR.y - computed.handR.y) < 1e-4);
  assert.ok(Math.abs(computed.humerusL.y - computed.forearmL.y) < 1e-4);
  assert.ok(Math.abs(computed.forearmL.y - computed.handL.y) < 1e-4);

  // Wide symmetrical A-stance legs and outward horizontal feet
  assert.ok(computed.pelvis.x - computed.femurR.x > 25);
  assert.ok(computed.femurR.x - computed.shinR.x > 90);
  assert.ok(computed.shinR.x - computed.footR.x > 80);
  assert.ok(computed.footR.x - computed.toeR.x > 130);
  assert.ok(Math.abs((computed.footR.x - computed.pelvis.x) + (computed.footL.x - computed.pelvis.x)) < 1e-4);

  // Zero Prometheus wing polygons or lines
  assert.equal(geom.polygons.some((p) => (p.id as string).includes('wing')), false);
  assert.equal(geom.functionalLines.some((l) => l.id.includes('wing')), false);
});

test('pink joints remain dead-center over every circular joint radius in rest and bent poses', () => {
  const state = createPosePreset('tpose');
  state.joints.humerusR.localAngle = -42;
  state.joints.forearmR.localAngle = 28;
  state.joints.femurR.localAngle = -14;
  state.joints.shinR.localAngle = 20;
  const computed = computeArmature(state);

  const checks: Array<{ id: keyof typeof computed; mirrorX: boolean; svgCx: number; svgCy: number }> = [
    { id: 'lip', mirrorX: false, svgCx: 824, svgCy: 540 },
    { id: 'humerusR', mirrorX: true, svgCx: 923, svgCy: 688 },
    { id: 'humerusL', mirrorX: false, svgCx: 923, svgCy: 688 },
    { id: 'forearmR', mirrorX: true, svgCx: 1113, svgCy: 688 },
    { id: 'forearmL', mirrorX: false, svgCx: 1113, svgCy: 688 },
    { id: 'handR', mirrorX: true, svgCx: 1283, svgCy: 688 },
    { id: 'handL', mirrorX: false, svgCx: 1283, svgCy: 688 },
    { id: 'femurR', mirrorX: false, svgCx: 787, svgCy: 1100 },
    { id: 'femurL', mirrorX: true, svgCx: 787, svgCy: 1100 },
    { id: 'shinR', mirrorX: false, svgCx: 661, svgCy: 1352 },
    { id: 'shinL', mirrorX: true, svgCx: 661, svgCy: 1352 },
    { id: 'footR', mirrorX: false, svgCx: 548, svgCy: 1650 },
    { id: 'footL', mirrorX: true, svgCx: 548, svgCy: 1650 },
  ];

  for (const c of checks) {
    const j = computed[c.id];
    const m = computeSvgBoneMatrix(j.boneMatrix, c.mirrorX);
    const radiusCenter = applyAffine(m, c.svgCx, c.svgCy);
    const err = Math.hypot(radiusCenter.x - j.x, radiusCenter.y - j.y);
    assert.ok(err < 1e-4, `${String(c.id)} pink joint off-center from radius by ${err}`);
  }
});

test('two-bone IK, FK bone aim rotation, mirror symmetry, and shortest-path wrap work predictably', () => {
  assert.equal(shortestAngleDelta(170, -170), 20);
  assert.equal(shortestAngleDelta(-170, 170), -20);

  const base = createPosePreset('tpose');
  const before = computeArmature(base);
  const targetX = before.handR.x + 28;
  const targetY = before.handR.y - 65;

  const afterIK = solveTwoBoneIK(base, 'handR', targetX, targetY);
  const solved = computeArmature(afterIK);
  const distBefore = Math.hypot(before.handR.x - targetX, before.handR.y - targetY);
  const distAfter = Math.hypot(solved.handR.x - targetX, solved.handR.y - targetY);
  assert.ok(distAfter < distBefore, `IK moved handR closer (${distBefore} -> ${distAfter})`);

  const rotated = solveBoneAimRotation(base, 'humerusL', before.humerusL.x + 80, before.humerusL.y - 80);
  assert.notEqual(rotated.joints.humerusL.localAngle, base.joints.humerusL.localAngle);

  const mirrored = mirrorPose(afterIK);
  assert.equal(mirrored.joints.humerusL.localAngle, -afterIK.joints.humerusR.localAngle);
  assert.equal(mirrored.joints.forearmL.localAngle, -afterIK.joints.forearmR.localAngle);
});

test('split two-piece collarbones fold up/down rotationally with zero stretch and half-width segments', () => {
  const rest = computeArmature(createPosePreset('tpose'));
  const fullSpanR = Math.hypot(rest.acromionR.x - rest.thorax.x, rest.acromionR.y - rest.thorax.y);
  const fullSpanL = Math.hypot(rest.acromionL.x - rest.thorax.x, rest.acromionL.y - rest.thorax.y);

  // Lengthened rigid half-segments (fullSpan / 2 * COLLAR_LINK_SCALE) cover full range with zero stretch
  assert.ok(Math.abs(rest.braceR.effectiveLengthPx - (fullSpanR / 2) * COLLAR_LINK_SCALE) < 1e-4);
  assert.ok(Math.abs(rest.clavOuterR.effectiveLengthPx - (fullSpanR / 2) * COLLAR_LINK_SCALE) < 1e-4);
  assert.ok(rest.braceR.effectiveLengthPx + rest.clavOuterR.effectiveLengthPx > fullSpanR);
  assert.ok(Math.abs(rest.braceL.effectiveLengthPx - (fullSpanL / 2) * COLLAR_LINK_SCALE) < 1e-4);
  assert.ok(Math.abs(rest.clavOuterL.effectiveLengthPx - (fullSpanL / 2) * COLLAR_LINK_SCALE) < 1e-4);
  assert.ok(rest.braceL.effectiveLengthPx + rest.clavOuterL.effectiveLengthPx > fullSpanL);

  // Fold UP on right (braceR = +28) and DOWN on left (braceL = +28)
  const folded = createPosePreset('tpose');
  folded.joints.lumbar.localAngle = -18;
  folded.joints.thorax.localAngle = 22;
  folded.joints.braceR.localAngle = 28;
  folded.joints.braceL.localAngle = 28;
  folded.joints.swivelR.localAngle = 6;
  folded.joints.swivelL.localAngle = -14;

  const computed = computeArmature(folded);

  // Exact zero stretch on every segment in folded pose
  assert.ok(Math.abs(Math.hypot(computed.braceR.endX - computed.braceR.x, computed.braceR.endY - computed.braceR.y) - rest.braceR.effectiveLengthPx) < 1e-4);
  assert.ok(Math.abs(Math.hypot(computed.clavOuterR.endX - computed.clavOuterR.x, computed.clavOuterR.endY - computed.clavOuterR.y) - rest.clavOuterR.effectiveLengthPx) < 1e-4);
  assert.ok(Math.abs(Math.hypot(computed.braceL.endX - computed.braceL.x, computed.braceL.endY - computed.braceL.y) - rest.braceL.effectiveLengthPx) < 1e-4);
  assert.ok(Math.abs(Math.hypot(computed.clavOuterL.endX - computed.clavOuterL.x, computed.clavOuterL.endY - computed.clavOuterL.y) - rest.clavOuterL.effectiveLengthPx) < 1e-4);

  // Exact continuous chain connections (thorax -> inner -> outer -> acromion) & rigid SVG matrix tips
  const braceRSvgTip = applyAffine(computeSvgBoneMatrix(computed.braceR.boneMatrix, true), 873.5, 685.5);
  const clavOuterRSvgTip = applyAffine(computeSvgBoneMatrix(computed.clavOuterR.boneMatrix, true), 923, 688);
  const braceLSvgTip = applyAffine(computeSvgBoneMatrix(computed.braceL.boneMatrix, false), 873.5, 685.5);
  const clavOuterLSvgTip = applyAffine(computeSvgBoneMatrix(computed.clavOuterL.boneMatrix, false), 923, 688);

  assert.ok(Math.hypot(computed.braceR.x - computed.thorax.x, computed.braceR.y - computed.thorax.y) < 1e-4);
  assert.ok(Math.hypot(braceRSvgTip.x - computed.clavOuterR.x, braceRSvgTip.y - computed.clavOuterR.y) < 1e-4);
  assert.ok(Math.hypot(clavOuterRSvgTip.x - computed.acromionR.x, clavOuterRSvgTip.y - computed.acromionR.y) < 1e-4);
  assert.ok(Math.hypot(computed.braceL.x - computed.thorax.x, computed.braceL.y - computed.thorax.y) < 1e-4);
  assert.ok(Math.hypot(braceLSvgTip.x - computed.clavOuterL.x, braceLSvgTip.y - computed.clavOuterL.y) < 1e-4);
  assert.ok(Math.hypot(clavOuterLSvgTip.x - computed.acromionL.x, clavOuterLSvgTip.y - computed.acromionL.y) < 1e-4);

  // Fold UP check vs Fold DOWN check at neutral spine
  const foldUpDown = createPosePreset('tpose');
  foldUpDown.joints.braceR.localAngle = 25; // UP
  foldUpDown.joints.braceL.localAngle = 25; // DOWN
  const cUpDown = computeArmature(foldUpDown);
  assert.ok(cUpDown.clavOuterR.y < cUpDown.thorax.y - 8, 'braceR +25 folds UP above thorax');
  assert.ok(cUpDown.clavOuterL.y > cUpDown.thorax.y + 8, 'braceL +25 folds DOWN below thorax');
});

test('spineless hip swivel engine clamps hipR/hipL bounds, keeps inter-hip bridge intact, and stays independent of spine', () => {
  const extreme = createPosePreset('tpose');
  extreme.joints.lumbar.localAngle = -28;
  extreme.joints.thorax.localAngle = 36;
  extreme.joints.hipR.localAngle = 65;
  extreme.joints.hipL.localAngle = -65;

  const computedExtreme = computeArmature(extreme);
  assert.ok(computedExtreme.hipR.localAngle <= 21 && computedExtreme.hipR.localAngle >= 18);
  assert.ok(computedExtreme.hipL.localAngle >= -21 && computedExtreme.hipL.localAngle <= -18);
  const extremeHipSpan = Math.hypot(
    computedExtreme.femurL.x - computedExtreme.femurR.x,
    computedExtreme.femurL.y - computedExtreme.femurR.y
  );
  assert.ok(Math.abs(extremeHipSpan - 2 * HIP_BRIDGE_LINK_STAGE_PX) < 1e-2, 'extreme spread locks rigidly at 2x link span');
  assert.ok(computedExtreme.femurL.x - computedExtreme.femurR.x > 40, 'hips never invert or collapse');

  // Spineless independence: bending lumbar/thorax does not alter hip swivel angles or hip world coords
  const upright = createPosePreset('tpose');
  upright.joints.hipR.localAngle = 10;
  upright.joints.hipL.localAngle = -8;
  const bentSpine = createPosePreset('tpose');
  bentSpine.joints.lumbar.localAngle = -25;
  bentSpine.joints.thorax.localAngle = 30;
  bentSpine.joints.hipR.localAngle = 10;
  bentSpine.joints.hipL.localAngle = -8;

  const cUpright = computeArmature(upright);
  const cBent = computeArmature(bentSpine);
  assert.equal(cBent.hipR.localAngle, cUpright.hipR.localAngle);
  assert.equal(cBent.hipL.localAngle, cUpright.hipL.localAngle);
  assert.ok(Math.hypot(cBent.femurR.x - cUpright.femurR.x, cBent.femurR.y - cUpright.femurR.y) < 1e-4);
  assert.ok(Math.hypot(cBent.femurL.x - cUpright.femurL.x, cBent.femurL.y - cUpright.femurL.y) < 1e-4);

  // Legs track hipR/hipL swivel rotation cleanly
  const rest = computeArmature(createPosePreset('tpose'));
  assert.ok(Math.hypot(cUpright.footR.x - rest.footR.x, cUpright.footR.y - rest.footR.y) > 40);
  assert.ok(Math.hypot(cUpright.footL.x - rest.footL.x, cUpright.footL.y - rest.footL.y) > 40);
});

test('rotating waist (pelvis) acts as root carrying both legs and torso, while torso tilts independently of hips', () => {
  const restState = createPosePreset('tpose');
  const rest = computeArmature(restState);

  const waistTurned = createPosePreset('tpose');
  waistTurned.rootRotation = 28;
  const turned = computeArmature(waistTurned);

  // Waist (pelvis) rotates bottom half (legs) AND torso/upper body together per body_pose_full_b3b6bf24.svg
  assert.ok(Math.hypot(turned.femurR.x - rest.femurR.x, turned.femurR.y - rest.femurR.y) > 40);
  assert.ok(Math.hypot(turned.femurL.x - rest.femurL.x, turned.femurL.y - rest.femurL.y) > 40);
  assert.ok(Math.hypot(turned.footR.x - rest.footR.x, turned.footR.y - rest.footR.y) > 200);
  assert.ok(Math.hypot(turned.footL.x - rest.footL.x, turned.footL.y - rest.footL.y) > 200);
  assert.ok(Math.abs(turned.lumbar.worldAngle - rest.lumbar.worldAngle - 28) < 1e-4);
  assert.ok(Math.hypot(turned.head.x - rest.head.x, turned.head.y - rest.head.y) > 100);
  assert.ok(Math.hypot(turned.handR.x - rest.handR.x, turned.handR.y - rest.handR.y) > 200);

  // Bending torso (lumbar) moves upper body without moving hips or legs
  const torsoTilted = createPosePreset('tpose');
  torsoTilted.joints.lumbar.localAngle = 22;
  const cTorso = computeArmature(torsoTilted);
  assert.ok(Math.hypot(cTorso.head.x - rest.head.x, cTorso.head.y - rest.head.y) > 80);
  for (const lowerId of ['hipR', 'hipL', 'femurR', 'femurL', 'footR', 'footL'] as const) {
    const err = Math.hypot(cTorso[lowerId].x - rest[lowerId].x, cTorso[lowerId].y - rest[lowerId].y);
    assert.ok(err < 1e-4, `lower body joint ${lowerId} moved by ${err} when torso rotated`);
  }
});

function hexSaturation(hex: string): number {
  const clean = hex.replace('#', '');
  const r = parseInt(clean.slice(0, 2), 16) / 255;
  const g = parseInt(clean.slice(2, 4), 16) / 255;
  const b = parseInt(clean.slice(4, 6), 16) / 255;
  const max = Math.max(r, g, b);
  const min = Math.min(r, g, b);
  const l = (max + min) / 2;
  if (max === min) return 0;
  const d = max - min;
  return l > 0.5 ? d / (2 - max - min) : d / (max + min);
}

test('thumbs and fingers pivot at the wrist, articulate independently, selection glow is greatly desaturated, and FPS cap defaults to 24 with 12/15/24/30 cycle', () => {
  const pose = createPosePreset('tpose');
  pose.joints.thumbL.localAngle = -25;
  pose.joints.fingerR.localAngle = 35;
  const computed = computeArmature(pose, { x: 681.5, y: 900 });
  assert.ok(Math.abs(computed.thumbL.localAngle - -25) < 1e-4);
  assert.ok(Math.abs(computed.fingerR.localAngle - 35) < 1e-4);
  assert.ok(Math.abs(computed.handL.localAngle) < 1e-4);
  assert.ok(Math.abs(computed.handR.localAngle) < 1e-4);

  // Both finger and thumb originate squarely at the wrist (no elevated origin or palm buffer)
  assert.ok(Math.hypot(computed.thumbR.x - computed.handR.x, computed.thumbR.y - computed.handR.y) < 1e-4);
  assert.ok(Math.hypot(computed.fingerR.x - computed.handR.x, computed.fingerR.y - computed.handR.y) < 1e-4);
  assert.ok(Math.hypot(computed.thumbL.x - computed.handL.x, computed.thumbL.y - computed.handL.y) < 1e-4);
  assert.ok(Math.hypot(computed.fingerL.x - computed.handL.x, computed.fingerL.y - computed.handL.y) < 1e-4);
  assert.ok(computed.thumbTipR.x < computed.handR.x && computed.fingerTipR.x < computed.thumbTipR.x);
  assert.ok(computed.thumbTipL.x > computed.handL.x && computed.fingerTipL.x > computed.thumbTipL.x);

  const geom = buildCaveArtGeometry(computed);
  assert.equal(geom.polygons.some((p) => p.id === 'palm-r' || p.id === 'palm-l'), false);
  for (const l of geom.functionalLines) {
    assert.ok(Math.hypot(l.x2 - l.x1, l.y2 - l.y1) > 1e-2, `${l.id} must be non-degenerate`);
  }

  assert.ok(hexSaturation(REPO_GREEN_GLOW_LIGHT) <= 0.12);
  assert.ok(hexSaturation(REPO_GREEN_GLOW_DARK) <= 0.12);

  assert.equal(DEFAULT_SCREEN_FPS, 24);
  assert.deepEqual(FPS_OPTIONS, [12, 15, 24, 30]);
  assert.equal(getNextFpsOption(24), 30);
  assert.equal(getNextFpsOption(30), 12);
  assert.equal(getNextFpsOption(12), 15);
  assert.equal(getNextFpsOption(15), 24);

  assert.ok(Math.abs(getFrameIntervalMs(24) - 1000 / 24) < 1e-6);
  assert.equal(shouldEmitScreenFrame(-1, 100, 24), true);
  assert.equal(shouldEmitScreenFrame(100, 120, 24), false);
  assert.equal(shouldEmitScreenFrame(100, 142, 24), true);
  assert.equal(shouldEmitScreenFrame(100, 142, 12), false);
  assert.equal(shouldEmitScreenFrame(100, 184, 12), true);
  assert.equal(shouldEmitScreenFrame(100, 134, 30), true);
  assert.ok(remainingScreenFrameWaitMs(100, 120, 24) > 20);
  assert.equal(remainingScreenFrameWaitMs(100, 145, 24), 0);
});

test('double-click pose export payload and double-right-click model export viewBox stay uncut and unskewed', () => {
  const pose = createPosePreset('tpose');
  pose.joints.humerusR.localAngle = -35;
  const computed = computeArmature(pose, { x: 681.5, y: 900 });
  const fixedDate = new Date('2026-11-26T09:05:00.000Z');
  const payload = buildSavedPosePayload(pose, computed, 'tpose', fixedDate);

  assert.equal(payload.savedAt, '2026-11-26T09:05:00.000Z');
  assert.equal(payload.savedDate, '2026-11-26');
  assert.equal(payload.preset, 'tpose');
  assert.ok(payload.joints.pelvis && Number.isFinite(payload.joints.pelvis.x));

  const vb = computeUncutExportViewBox({ x: 250, y: 310, width: 920, height: 1120 }, computed, 48);
  assert.ok(vb.x <= 202 && vb.y <= 262);
  assert.ok(vb.x + vb.width >= 1218 && vb.y + vb.height >= 1478);
  const aspectViewBox = vb.width / vb.height;
  const aspectExport = vb.exportWidth / vb.exportHeight;
  assert.ok(Math.abs(aspectViewBox - aspectExport) < 0.01, 'export dimensions must preserve aspect ratio without skew');
});

test('multi-joint IK solvers for elbows, knees, digits, and keyframe interpolation work smoothly', () => {
  const base = createPosePreset('tpose');
  const computed = computeArmature(base);

  // Test elbow IK aim solver
  const elbowTargetX = computed.forearmR.x - 25;
  const elbowTargetY = computed.forearmR.y + 40;
  const solvedElbow = solveSingleBoneAimIK(base, 'forearmR', elbowTargetX, elbowTargetY);
  assert.notEqual(solvedElbow.joints.humerusR.localAngle, base.joints.humerusR.localAngle);

  // Test knee IK aim solver
  const kneeTargetX = computed.shinR.x + 20;
  const kneeTargetY = computed.shinR.y - 30;
  const solvedKnee = solveSingleBoneAimIK(base, 'shinR', kneeTargetX, kneeTargetY);
  assert.notEqual(solvedKnee.joints.femurR.localAngle, base.joints.femurR.localAngle);

  // Test universal solveJointIK dispatcher
  const solvedUniversalElbow = solveJointIK(base, 'forearmR', elbowTargetX, elbowTargetY);
  assert.notEqual(solvedUniversalElbow.joints.humerusR.localAngle, base.joints.humerusR.localAngle);

  const solvedUniversalFoot = solveJointIK(base, 'footR', computed.footR.x + 30, computed.footR.y - 20);
  assert.notEqual(solvedUniversalFoot.joints.femurR.localAngle, base.joints.femurR.localAngle);

  // Test interpolation and procedural breakdown between calibrated T-pose and posed frame
  const poseA = createPosePreset('tpose');
  const poseB = createPosePreset('tpose');
  poseB.rootRotation = 4;
  poseB.joints.humerusR.localAngle = -68;
  poseB.joints.forearmR.localAngle = 36;
  const half = interpolateArmatures(poseA, poseB, 0.5, 'flow');
  assert.ok(Number.isFinite(half.rootRotation));
  assert.ok(Math.abs(half.joints.humerusR.localAngle - (poseA.joints.humerusR.localAngle + poseB.joints.humerusR.localAngle) / 2) < 15);

  const breakdown = synthesizeProceduralBreakdown(poseA, poseB);
  assert.ok(Number.isFinite(breakdown.rootY));
});

test('hip bones act as anatomical thigh extensions during leg swings without breaking static clamps', () => {
  assert.equal(computeHipThighExtension('hipR', 0), 0);
  assert.equal(computeHipThighExtension('hipL', 0), 0);

  // Outward swing on right leg (+60°) recruits upward/outward hipR extension (~36%, up to 24°)
  const extRight = computeHipThighExtension('hipR', 60);
  assert.ok(extRight > 15 && extRight <= 24, `expected positive outward hipR extension, got ${extRight}`);

  // Mirrored outward swing on left leg (-60°) recruits symmetric negative hipL extension
  const extLeft = computeHipThighExtension('hipL', -60);
  assert.ok(Math.abs(extLeft + extRight) < 1e-6);

  // Evaluate full action pose (e.g. Striding / Discobolus): hip follows thigh cleanly & preserves cumulative leg aim
  const pose = createPosePreset('tpose');
  pose.joints.femurR.localAngle = 60;
  const computed = computeArmature(pose);
  assert.ok(computed.hipR.localAngle > 15, `hipR localAngle (${computed.hipR.localAngle}) should engage during +60 femurR swing`);
  assert.ok(
    Math.abs((computed.hipR.localAngle + computed.femurR.localAngle) - 60) < 1e-4,
    'hipR + femurR cumulative delta preserves commanded thigh orientation'
  );

  // Symmetric opposite leg swing engages left hip extension
  const poseLeft = createPosePreset('tpose');
  poseLeft.joints.femurL.localAngle = -38;
  const compLeft = computeArmature(poseLeft);
  assert.ok(Math.abs(compLeft.hipL.localAngle) > 1, 'femurL swing engages hipL thigh extension');
});

test('2-bone toggle-hinge solves Single Line 180°, A-shape, and V-shape rigidly across hip bridge and collarbones', () => {
  const pLeft = { x: 100, y: 200 };
  const pRightExt = { x: 200, y: 200 };
  const ext = solveToggleHinge2Link(pLeft, pRightExt, 50, 1);
  assert.equal(ext.state, 'extended');
  assert.equal(ext.interiorAngleDeg, 180);
  assert.ok(Math.abs(ext.pMid.x - 150) < 1e-6 && Math.abs(ext.pMid.y - 200) < 1e-6);

  const pRightFold = { x: 160, y: 200 };
  const aShape = solveToggleHinge2Link(pLeft, pRightFold, 50, 1);
  const vShape = solveToggleHinge2Link(pLeft, pRightFold, 50, -1);
  assert.equal(aShape.state, 'A');
  assert.equal(vShape.state, 'V');
  assert.ok(aShape.pMid.y < 200, 'A-shape bends upward (-Y)');
  assert.ok(vShape.pMid.y > 200, 'V-shape bends downward (+Y)');
  assert.ok(Math.abs(Math.hypot(aShape.pMid.x - pLeft.x, aShape.pMid.y - pLeft.y) - 50) < 1e-4);
  assert.ok(Math.abs(Math.hypot(pRightFold.x - aShape.pMid.x, pRightFold.y - aShape.pMid.y) - 50) < 1e-4);
  assert.ok(Math.abs(Math.hypot(vShape.pMid.x - pLeft.x, vShape.pMid.y - pLeft.y) - 50) < 1e-4);

  // Hip bridge 3-state transitions preserve rigid link length & re-span cleanly
  let pose = createPosePreset('tpose');
  pose = setHipToggleState(pose, 'extended');
  let c = computeArmature(pose);
  const hipExt = solveToggleHinge2Link(c.femurR, c.femurL, HIP_BRIDGE_LINK_STAGE_PX, pose.hipToggleSign ?? 1, c.pelvis);
  assert.equal(hipExt.state, 'extended');

  pose = setHipToggleState(pose, 'V');
  c = computeArmature(pose);
  const hipV = solveToggleHinge2Link(c.femurR, c.femurL, HIP_BRIDGE_LINK_STAGE_PX, pose.hipToggleSign ?? 1, c.pelvis);
  assert.equal(hipV.state, 'V');
  assert.ok(hipV.pMid.y > (c.femurR.y + c.femurL.y) / 2);

  pose = solveHipToggleFromTarget(pose, c.pelvis.x, c.femurR.y - 35);
  assert.equal(pose.hipToggleSign, 1);
});

test('collarbones favor downward rotation by default and smoothly ease upward when impossible', () => {
  // Neutral / downward arm elevation favors downward (-1) fold when compressed
  const rest = resolveCollarToggleFold('R', 0, 0, -5);
  assert.equal(rest.foldSign, -1);

  const pose = createPosePreset('tpose');
  pose.joints.swivelR.localAngle = 10;
  pose.joints.swivelL.localAngle = -10;
  let c = computeArmature(pose);
  assert.ok(c.clavOuterR.y > c.thorax.y + 4, 'neutral compressed braceR favors downward fold');
  assert.ok(c.clavOuterL.y > c.thorax.y + 4, 'neutral compressed braceL favors downward fold');

  // Raising right arm overhead (+72°) makes downward impossible -> eases braceR upward while L stays downward
  pose.joints.humerusR.localAngle = 72;
  c = computeArmature(pose);
  assert.ok(c.clavOuterR.y < c.thorax.y - 4, 'elevated overhead arm eases braceR upward');
  assert.ok(c.clavOuterL.y > c.thorax.y + 4, 'lowered left side remains downward');

  // Explicit downward braceR = -24 smoothly eases through 180° up to +24 as humerusR lifts overhead
  const easeLow = resolveCollarToggleFold('R', -24, 10, 0);
  const easeMid = resolveCollarToggleFold('R', -24, 37, 0);
  const easeHigh = resolveCollarToggleFold('R', -24, 65, 0);
  assert.equal(easeLow.foldSign, -1);
  assert.ok(Math.abs(easeMid.effectiveFoldMag) < 1e-4, 'passes cleanly through 180° straight line at midpoint');
  assert.equal(easeHigh.foldSign, 1);

  // Lowered arm pose uses downward collarbone; elevated overhead arm eases upward
  const loweredArm = createPosePreset('tpose');
  loweredArm.joints.braceR.localAngle = -14;
  loweredArm.joints.humerusR.localAngle = -68;
  assert.ok(computeArmature(loweredArm).braceR.localAngle < 0);

  const raisedArm = createPosePreset('tpose');
  raisedArm.joints.braceR.localAngle = 26;
  raisedArm.joints.humerusR.localAngle = 38;
  assert.ok(computeArmature(raisedArm).braceR.localAngle > 0);

  // Target drag and quick toggle helper stay rigid
  const collarToggled = solveCollarToggleFromTarget(
    setCollarToggleState(createPosePreset('tpose'), 'L', 'V'),
    'R',
    c.thorax.x - 42,
    c.thorax.y + 18
  );
  assert.ok(collarToggled.joints.braceR.localAngle < 0);
});

test('waist balance/hula moves pelvis laterally and vertically with planted feet fixed and rigid leg lengths', () => {
  const base = createPosePreset('tpose');
  const start = computeArmature(base);
  const L1R = Math.hypot(start.shinR.x - start.femurR.x, start.shinR.y - start.femurR.y);
  const L2R = Math.hypot(start.footR.x - start.shinR.x, start.footR.y - start.shinR.y);
  const L1L = Math.hypot(start.shinL.x - start.femurL.x, start.shinL.y - start.femurL.y);
  const L2L = Math.hypot(start.footL.x - start.shinL.x, start.footL.y - start.shinL.y);

  const footRPin = { x: start.footR.x, y: start.footR.y, worldAngle: start.footR.worldAngle };
  const footLPin = { x: start.footL.x, y: start.footL.y, worldAngle: start.footL.worldAngle };

  // Hula sway right + squat crouch downward
  const hulaSquat = solveWaistBalanceHula(
    base,
    start.pelvis.x + 36,
    start.pelvis.y + 58,
    footRPin,
    footLPin,
    undefined,
    {
      baseRootRotation: base.rootRotation,
      baseLumbarAngle: base.joints.lumbar.localAngle,
      startPelvisX: start.pelvis.x,
    }
  );
  const after = computeArmature(hulaSquat);

  assert.ok(Math.abs(after.pelvis.x - start.pelvis.x) > 20, 'waist swayed laterally');
  assert.ok(after.pelvis.y > start.pelvis.y + 35, 'waist crouched downward');
  assert.ok(Math.hypot(after.footR.x - footRPin.x, after.footR.y - footRPin.y) < 0.75, 'footR remained planted on floor');
  assert.ok(Math.hypot(after.footL.x - footLPin.x, after.footL.y - footLPin.y) < 0.75, 'footL remained planted on floor');
  assert.ok(Math.abs(hulaSquat.rootRotation) > 1 && Math.abs(after.pelvis.localAngle) > 1, 'pelvic hula tilt engaged');

  // Extreme upward pull never stretches or detaches cardboard-cutout leg bones
  const overPull = solveWaistBalanceHula(base, start.pelvis.x + 220, start.pelvis.y - 450, footRPin, footLPin);
  const afterOver = computeArmature(overPull);
  const curL1R = Math.hypot(afterOver.shinR.x - afterOver.femurR.x, afterOver.shinR.y - afterOver.femurR.y);
  const curL2R = Math.hypot(afterOver.footR.x - afterOver.shinR.x, afterOver.footR.y - afterOver.shinR.y);
  const curL1L = Math.hypot(afterOver.shinL.x - afterOver.femurL.x, afterOver.shinL.y - afterOver.femurL.y);
  const curL2L = Math.hypot(afterOver.footL.x - afterOver.shinL.x, afterOver.footL.y - afterOver.shinL.y);
  assert.ok(Math.abs(curL1R - L1R) < 1e-4 && Math.abs(curL2R - L2R) < 1e-4, 'right leg rigid');
  assert.ok(Math.abs(curL1L - L1L) < 1e-4 && Math.abs(curL2L - L2L) < 1e-4, 'left leg rigid');
  assert.ok(Math.hypot(afterOver.footR.x - afterOver.femurR.x, afterOver.footR.y - afterOver.femurR.y) <= L1R + L2R + 1e-3);
  assert.ok(Math.hypot(afterOver.footL.x - afterOver.femurL.x, afterOver.footL.y - afterOver.femurL.y) <= L1L + L2L + 1e-3);
});

test('collar and hip bones remain strictly non-elastic (zero stretch) across lengthened full-range poses', () => {
  assert.ok(HIP_BRIDGE_LINK_SVG > 80);

  const rest = computeArmature(createPosePreset('tpose'));
  const halfClavLen = rest.braceR.effectiveLengthPx;

  // Test extreme outward collar swivel + extreme thorax tilt -> zero collarbone stretch
  const stretchedAttempt = createPosePreset('tpose');
  stretchedAttempt.joints.thorax.localAngle = 36;
  stretchedAttempt.joints.swivelR.localAngle = -16;
  stretchedAttempt.joints.swivelL.localAngle = 16;
  const cAttempt = computeArmature(stretchedAttempt);

  const spanR = Math.hypot(cAttempt.acromionR.x - cAttempt.thorax.x, cAttempt.acromionR.y - cAttempt.thorax.y);
  const spanL = Math.hypot(cAttempt.acromionL.x - cAttempt.thorax.x, cAttempt.acromionL.y - cAttempt.thorax.y);
  assert.ok(spanR <= 2 * halfClavLen + 1e-3, `collarbone R never stretches (${spanR} <= ${2 * halfClavLen})`);
  assert.ok(spanL <= 2 * halfClavLen + 1e-3, `collarbone L never stretches (${spanL} <= ${2 * halfClavLen})`);
  assert.ok(Math.abs(Math.hypot(cAttempt.braceR.endX - cAttempt.braceR.x, cAttempt.braceR.endY - cAttempt.braceR.y) - halfClavLen) < 1e-3);
  assert.ok(Math.abs(Math.hypot(cAttempt.clavOuterR.endX - cAttempt.clavOuterR.x, cAttempt.clavOuterR.endY - cAttempt.clavOuterR.y) - halfClavLen) < 1e-3);

  // Test extreme hip spread + thigh leg swing -> 2-bone hip bridge never stretches beyond 2 * L
  const hipSplit = createPosePreset('tpose');
  hipSplit.joints.hipR.localAngle = 21;
  hipSplit.joints.hipL.localAngle = -21;
  hipSplit.joints.femurR.localAngle = 60;
  hipSplit.joints.femurL.localAngle = -60;
  const cSplit = computeArmature(hipSplit);
  const hipDist = Math.hypot(cSplit.femurL.x - cSplit.femurR.x, cSplit.femurL.y - cSplit.femurR.y);
  assert.ok(hipDist <= 2 * HIP_BRIDGE_LINK_STAGE_PX + 1e-3, `hip bridge never stretches (${hipDist} <= ${2 * HIP_BRIDGE_LINK_STAGE_PX})`);

  // Even solveToggleHinge2Link given an over-span distance clamps pLeft/pRight rigidly to linkLength
  const clampedBridge = solveToggleHinge2Link({ x: 0, y: 0 }, { x: 260, y: 0 }, 80, 1);
  assert.equal(clampedBridge.state, 'extended');
  assert.ok(Math.abs(Math.hypot(clampedBridge.pMid.x - clampedBridge.pLeft.x, clampedBridge.pMid.y - clampedBridge.pLeft.y) - 80) < 1e-4);
  assert.ok(Math.abs(Math.hypot(clampedBridge.pRight.x - clampedBridge.pMid.x, clampedBridge.pRight.y - clampedBridge.pMid.y) - 80) < 1e-4);
});

test('collar swivel enters resistance near inward limit then snaps cleanly onto opposite collar in side poses', () => {
  const normal = resolveCollarInwardBehavior('R', 8, -2);
  assert.equal(normal.snapState, 'normal');
  assert.equal(normal.resistanceRatio, 0);

  const resisting = resolveCollarInwardBehavior('R', COLLAR_INWARD_RESISTANCE_START_DEG + 2.25, 0);
  assert.equal(resisting.snapState, 'resisting');
  assert.ok(resisting.resistanceRatio > 0.4 && resisting.resistanceRatio < 0.6);

  // Dragging within resistance band (11.5..21.5) damps angle within [11.5, 16]
  let pose = createPosePreset('tpose');
  const dragResist = applyCollarSwivelDragInput(pose, 'R', 18.5);
  assert.equal(dragResist.behavior.snapState, 'resisting');
  assert.ok(dragResist.nextAngle >= COLLAR_INWARD_RESISTANCE_START_DEG && dragResist.nextAngle <= COLLAR_INWARD_NORMAL_LIMIT_DEG);

  // Breaking past inward threshold (> 21.5) snaps R onto opposite L collar
  const dragSnap = applyCollarSwivelDragInput(pose, 'R', COLLAR_INWARD_BREAKAWAY_DEG + 1.5);
  assert.equal(dragSnap.behavior.snapState, 'snapped');
  pose.joints.swivelR.localAngle = dragSnap.nextAngle;
  const snappedComp = computeArmature(pose);
  assert.equal(snappedComp.swivelR.collarSnapState, 'snapped');
  assert.ok(
    Math.hypot(snappedComp.acromionR.x - snappedComp.acromionL.x, snappedComp.acromionR.y - snappedComp.acromionL.y) < 1e-2,
    'snapped right collar lands squarely on left collar socket in side pose'
  );
  // Zero collarbone stretch even in cross-body side snap
  const snapClavSpanR = Math.hypot(snappedComp.acromionR.x - snappedComp.thorax.x, snappedComp.acromionR.y - snappedComp.thorax.y);
  assert.ok(snapClavSpanR <= snappedComp.braceR.effectiveLengthPx + snappedComp.clavOuterR.effectiveLengthPx + 1e-3);

  // Pulling back out below hysteresis (< 18°) cleanly unsnaps
  const dragUnsnap = applyCollarSwivelDragInput(pose, 'R', 15);
  assert.notEqual(dragUnsnap.behavior.snapState, 'snapped');
});

test('per-arm reverse swivel flips arm orientation smoothly, reverses IK elbow bend, and mirrors/interpolates cleanly', () => {
  const base = createPosePreset('tpose');
  base.joints.thumbR.localAngle = -20;
  base.joints.fingerR.localAngle = 30;
  const normalComp = computeArmature(base);
  assert.equal(normalComp.humerusR.armReverseFactor, 1);

  const reversed = toggleArmReverseSwivel(base, 'R');
  assert.equal(reversed.armSwivelR, 180);
  assert.equal(reversed.armSwivelL, 0);

  const revComp = computeArmature(reversed);
  assert.equal(revComp.humerusR.armReverseFactor, -1);
  assert.equal(revComp.humerusL.armReverseFactor, 1);
  // Wrist and elbow stay attached at exact bone endpoints, while thumb/finger curl flips cleanly
  assert.ok(Math.hypot(revComp.forearmR.x - normalComp.forearmR.x, revComp.forearmR.y - normalComp.forearmR.y) < 1e-4);
  assert.ok(Math.hypot(revComp.handR.x - normalComp.handR.x, revComp.handR.y - normalComp.handR.y) < 1e-4);
  assert.ok(Math.abs(revComp.thumbR.localAngle - 20) < 1e-4);
  assert.ok(Math.abs(revComp.fingerR.localAngle - -30) < 1e-4);

  // Midpoint 90° foreshortens smoothly with armReverseFactor ~ 0
  const midSwivel = setArmReverseSwivel(base, 'R', 90);
  const midComp = computeArmature(midSwivel);
  assert.ok(Math.abs(midComp.humerusR.armReverseFactor ?? 1) < 1e-4);
  const tween = interpolateArmatures(base, reversed, 0.5, 'mechanical');
  assert.ok(Math.abs(Math.abs(tween.armSwivelR ?? 0) - 90) < 1e-4);
  assert.ok(Math.abs(computeArmature(tween).humerusR.armReverseFactor ?? 1) < 1e-4);

  // Mirroring swaps armSwivelR <-> armSwivelL
  const mirrored = mirrorPose(reversed);
  assert.equal(mirrored.armSwivelR, 0);
  assert.equal(Math.abs(mirrored.armSwivelL ?? 0), 180);

  // Two-bone IK elbow bend reverses cleanly when armSwivelR = 180
  const ikNormal = computeArmature(solveTwoBoneIK(base, 'handR', normalComp.handR.x + 30, normalComp.handR.y - 40));
  const ikReversed = computeArmature(solveTwoBoneIK(reversed, 'handR', normalComp.handR.x + 30, normalComp.handR.y - 40));
  assert.ok(
    Math.sign(ikNormal.forearmR.localAngle) !== Math.sign(ikReversed.forearmR.localAngle),
    'reversed arm bends elbow on opposite side cleanly'
  );
});

test('pose & toggle transition interpolation detects deltas, blends scales, and swings hip A<->V cleanly through 180deg', () => {
  const poseA = createPosePreset('tpose');
  const poseB = createPosePreset('tpose');
  poseB.joints.humerusR.localAngle = -54;
  poseB.joints.humerusR.scaleLength = 1.4;
  poseB.joints.humerusR.scaleWidth = 1.2;

  assert.equal(hasArmaturePoseDelta(poseA, createPosePreset('tpose')), false);
  assert.equal(hasArmaturePoseDelta(poseA, poseB), true);

  const startFrame = computePoseTransitionFrame(poseA, poseB, 0, 320, 'flow');
  assert.equal(startFrame.done, false);
  assert.equal(startFrame.progress, 0);
  assert.equal(startFrame.armature.joints.humerusR.scaleLength, 1);

  const midFrame = computePoseTransitionFrame(poseA, poseB, 160, 320, 'mechanical');
  assert.equal(midFrame.done, false);
  assert.equal(midFrame.progress, 0.5);
  assert.ok(Math.abs(midFrame.armature.joints.humerusR.scaleLength - 1.2) < 1e-4);
  assert.ok(Math.abs(midFrame.armature.joints.humerusR.scaleWidth - 1.1) < 1e-4);

  const endFrame = computePoseTransitionFrame(poseA, poseB, 400, 320, 'flow');
  assert.equal(endFrame.done, true);
  assert.equal(endFrame.progress, 1);
  assert.equal(endFrame.armature, poseB);

  // Hip A <-> V toggle interpolation passes smoothly through extended ~20.8 / -20.8 outward apex
  const hipA = setHipToggleState(createPosePreset('tpose'), 'A');
  const hipV = setHipToggleState(createPosePreset('tpose'), 'V');
  assert.equal(hasArmaturePoseDelta(hipA, hipV), true);
  const hipMid = interpolateArmatures(hipA, hipV, 0.5, 'mechanical');
  assert.ok(Math.abs(hipMid.joints.hipR.localAngle - 20.8) < 1e-3);
  assert.ok(Math.abs(hipMid.joints.hipL.localAngle - -20.8) < 1e-3);
});
