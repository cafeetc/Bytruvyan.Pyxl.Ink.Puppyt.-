import test from 'node:test';
import assert from 'node:assert/strict';
import { JOINT_SPECS, REPO_STRAIN_CRIMSON } from '../data/digitalRig';
import { createPoseEngine } from '../engine/PoseEngine';
import {
  DOUBLE_LINE_TOP_WORLD_Y,
  INTRO_FACEPLATE_THRESHOLD_PX,
  INTRO_GREETING_TEXT,
  INTRO_DRAG_PROMPT,
  INTRO_MARGIN_Y,
  INTRO_N_LABEL,
  INTRO_TARGET_OFFSET_PX,
  INTRO_Y_LABEL,
  PLANQUETTE_ARC_RADIUS,
  PLANQUETTE_CHARS,
  PLANQUETTE_TITLE,
  PLANQUETTE_WARNING,
  Y_EXCLAMATIONS,
  computeIntroCamera,
  computePlanquetteArc,
  computePlanquetteCamera,
  evaluateFaceplateChoice,
  getRandomYExclamation,
  hitTestPlanquetteArc,
  projectFaceplateToPlanquetteArc,
  resolveFaceplateTargetChoice,
} from './introFraming';
import {
  computeArmature,
  evaluateTransitionEasing,
  solveJointIK,
  solvePlanquetteArmReach,
} from './kinematics';

test('PoseEngine initializes to canonical T-pose with finite 43-joint state and notifies subscribers cleanly', () => {
  const engine = createPoseEngine();
  let notifications = 0;
  const unsubscribe = engine.subscribe(() => {
    notifications += 1;
  });

  const initial = engine.getState();
  assert.equal(initial.preset, 'tpose');
  assert.equal(Object.keys(engine.getAllJointAngles()).length, 43);

  for (const spec of JOINT_SPECS) {
    const j = engine.getJoint(spec.id);
    assert.ok(Number.isFinite(j.x) && Number.isFinite(j.y));
    assert.ok(Number.isFinite(j.endX) && Number.isFinite(j.endY));
  }

  engine.setJointAngle('humerusR', -45);
  assert.equal(engine.getState().preset, null);
  assert.equal(engine.getJointAngle('humerusR'), -45);
  assert.equal(notifications, 1);

  engine.nudgeJointAngle('humerusR', 10);
  assert.equal(engine.getJointAngle('humerusR'), -35);

  engine.resetToTPose();
  assert.equal(engine.getState().preset, 'tpose');
  assert.equal(engine.getJointAngle('humerusR'), 0);

  unsubscribe();
  engine.setJointAngle('humerusL', 30);
  assert.equal(notifications, 3);
});

test('PoseEngine supports effector IK, balance queries, ground contact queries, collar/hip toggles, and arm swivel', () => {
  const engine = createPoseEngine();
  const handBefore = engine.getJoint('handR');
  const target = { x: handBefore.x + 24, y: handBefore.y - 60 };

  engine.solveEffectorIK('handR', target);
  const handAfter = engine.getJoint('handR');
  assert.ok(
    Math.hypot(handAfter.x - target.x, handAfter.y - target.y) <
      Math.hypot(handBefore.x - target.x, handBefore.y - target.y)
  );

  // Balance & ground contact queries
  const balance = engine.getBalanceCenters();
  assert.ok(Number.isFinite(balance.whole.x) && Number.isFinite(balance.whole.y));

  const contactBefore = engine.getGroundContactState();
  assert.ok(contactBefore.groundedFootCount >= 0);

  const plateResult = engine.setGroundPressurePlate(contactBefore.pressurePlateY - 20);
  assert.equal(plateResult.atHardStop, false);
  const contactAfter = engine.getGroundContactState();
  assert.equal(contactAfter.groundedFootCount, 2);

  // Hip & collar toggle hinges + per-arm reverse swivel
  engine.setHipToggle('V');
  assert.equal(engine.getHipToggleState().state, 'V');

  engine.setCollarToggle('R', 'A');
  assert.equal(engine.getCollarState('R').foldState, 'A');

  engine.toggleArmSwivel('R');
  assert.equal(engine.getArmReverseFactor('R'), -1);

  // Joint constraint status query
  engine.setConstraints({ softLimits: true, solver: 'fabrik' });
  const humerusStatus = engine.getJointConstraintStatus('humerusR');
  assert.equal(humerusStatus.jointId, 'humerusR');
  assert.ok(Number.isFinite(humerusStatus.worldAngle));
});

test('First-screen intro centers faceplate at 100% zoom and detects Y (left) vs N (right) faceplate drags', () => {
  assert.equal(INTRO_GREETING_TEXT, 'Greetings, pyrson. Have you no name?');
  assert.equal(INTRO_Y_LABEL, 'Y · Yes, I have no name');
  assert.equal(INTRO_N_LABEL, 'N · No, I have a name');
  assert.equal(
    INTRO_DRAG_PROMPT,
    'Drag face plate to Y or N and hover or place to choose your identity'
  );

  const engine = createPoseEngine();
  const restLip = engine.getJoint('lip');
  const vw = 1280;
  const vh = 720;
  const introCam = computeIntroCamera(vw, vh, restLip);

  assert.equal(introCam.zoom, 1.0);
  const screenX = introCam.pan.x + restLip.x * introCam.zoom;
  const screenY = introCam.pan.y + restLip.y * introCam.zoom;
  assert.ok(Math.abs(screenX - vw / 2) < 1e-6);
  assert.ok(Math.abs(screenY - vh / 2) < 1e-6);

  assert.equal(evaluateFaceplateChoice(restLip.x, restLip.x).choice, null);

  // Drag faceplate left -> Y
  engine.solveFaceplateLook({ x: restLip.x - 80, y: restLip.y });
  const lipLeft = engine.getJoint('lip');
  assert.ok(lipLeft.x <= restLip.x - INTRO_FACEPLATE_THRESHOLD_PX);
  assert.equal(evaluateFaceplateChoice(lipLeft.x, restLip.x).choice, 'Y');

  // Reset and drag faceplate right -> N
  engine.resetToTPose();
  engine.solveFaceplateLook({ x: restLip.x + 80, y: restLip.y });
  const lipRight = engine.getJoint('lip');
  assert.ok(lipRight.x >= restLip.x + INTRO_FACEPLATE_THRESHOLD_PX);
  assert.equal(evaluateFaceplateChoice(lipRight.x, restLip.x).choice, 'N');
});

test('Elasti-hose physics, exclamations, and intro margin conform to specifications', () => {
  // Elastic spring and rubber-band easing: bounded at endpoints, overshoots 1.0, settles cleanly
  assert.equal(evaluateTransitionEasing(0, 'elastic'), 0);
  assert.equal(evaluateTransitionEasing(1, 'elastic'), 1);

  // Peak spring overshoot occurs around t = 0.15..0.20
  const peak = evaluateTransitionEasing(0.15, 'elastic');
  assert.ok(peak > 1.0, 'Elastic easing overshoots 1.0 like a spring');

  // Strictly validated exclamations (one exclamation at a time; Genius removed)
  assert.deepEqual(Y_EXCLAMATIONS, [
    'Congratulations!',
    'Zounds!',
    'Huzzah!',
    'Brilliant!',
    'Excellent!',
  ]);
  const randomExcl = getRandomYExclamation();
  assert.ok(Y_EXCLAMATIONS.includes(randomExcl as (typeof Y_EXCLAMATIONS)[number]));
  assert.ok(!Y_EXCLAMATIONS.includes('Genius!' as any));

  // Margin coordinate is ~1 head unit above the head crown (486.05 - 122.08 ≈ 364)
  assert.equal(INTRO_MARGIN_Y, 364);

  // Placed on top detection (requires faceplate clicked down for target resolution)
  const targetY = { x: 681.5 - INTRO_TARGET_OFFSET_PX, y: 547.25 };
  const targetN = { x: 681.5 + INTRO_TARGET_OFFSET_PX, y: 547.25 };
  const rest = { x: 681.5, y: 547.25 };

  assert.equal(resolveFaceplateTargetChoice(false, rest, [targetY]), null);
  assert.equal(resolveFaceplateTargetChoice(false, rest, [targetN]), null);
  assert.equal(resolveFaceplateTargetChoice(true, rest, [targetY]), 'Y');
  assert.equal(resolveFaceplateTargetChoice(true, rest, [targetN]), 'N');

  const onTopY = evaluateFaceplateChoice(targetY.x, rest.x, 36, targetY.y, rest.y);
  assert.equal(onTopY.choice, 'Y');
  assert.equal(onTopY.placedOnTop, true);

  const onTopN = evaluateFaceplateChoice(targetN.x, rest.x, 36, targetN.y, rest.y);
  assert.equal(onTopN.choice, 'N');
  assert.equal(onTopN.placedOnTop, true);

  // Cooler crimson for active neck tension
  assert.equal(REPO_STRAIN_CRIMSON, '#E11D48');
});

test('Arm IK activates whole arm for wrist (hand) and fingertip (finger/fingerTip), plus wide Planquette reach', () => {
  const engine = createPoseEngine();
  const baseArmature = engine.getArmature();
  const origin = { x: 681.5, y: 900 };
  const startComp = computeArmature(baseArmature, origin);

  // 1. Fingertip / finger IK recruits humerusR + forearmR + handR so fingerTipR tracks target
  const tipBefore = startComp.fingerTipR;
  const tipTarget = { x: tipBefore.x + 40, y: tipBefore.y - 180 };
  const afterTipIK = solveJointIK(baseArmature, 'fingerTipR', tipTarget.x, tipTarget.y, origin, {
    solver: 'analytical',
    softDamp: true,
    constrainLimits: true,
  });
  const afterTipComp = computeArmature(afterTipIK, origin);
  assert.ok(
    Math.hypot(afterTipComp.fingerTipR.x - tipTarget.x, afterTipComp.fingerTipR.y - tipTarget.y) <
      Math.hypot(tipBefore.x - tipTarget.x, tipBefore.y - tipTarget.y) * 0.35
  );
  assert.ok(
    Math.abs(afterTipIK.joints.humerusR.localAngle) > 5 &&
      Math.abs(afterTipIK.joints.forearmR.localAngle) > 1
  );

  // Grabbing fingerR also routes through fingertip whole-arm IK
  const afterFingerIK = solveJointIK(baseArmature, 'fingerR', tipTarget.x, tipTarget.y, origin, {
    solver: 'analytical',
    softDamp: true,
    constrainLimits: true,
  });
  const afterFingerComp = computeArmature(afterFingerIK, origin);
  assert.ok(
    Math.hypot(afterFingerComp.fingerTipR.x - tipTarget.x, afterFingerComp.fingerTipR.y - tipTarget.y) <
      Math.hypot(tipBefore.x - tipTarget.x, tipBefore.y - tipTarget.y) * 0.35
  );

  // 2. Hand wrist IK guides the whole arm via handR
  const handBefore = startComp.handR;
  const wristTarget = { x: handBefore.x + 30, y: handBefore.y - 150 };
  const afterWristIK = solveJointIK(baseArmature, 'handR', wristTarget.x, wristTarget.y, origin, {
    solver: 'analytical',
    softDamp: true,
    constrainLimits: true,
  });
  const afterWristComp = computeArmature(afterWristIK, origin);
  assert.ok(
    Math.hypot(afterWristComp.handR.x - wristTarget.x, afterWristComp.handR.y - wristTarget.y) <
      Math.hypot(handBefore.x - wristTarget.x, handBefore.y - wristTarget.y) * 0.35
  );

  // 3. Planquette wide arm reach reaches toward high left/right bar coordinates above y = 370
  const highRight = solvePlanquetteArmReach(baseArmature, 'fingerTipL', 940, 340, origin, {
    solver: 'analytical',
    softDamp: true,
    constrainLimits: true,
  });
  const highRightComp = computeArmature(highRight, origin);
  assert.ok(highRightComp.fingerTipL.y < 420);
  assert.ok(highRightComp.fingerTipL.x > 840);
});

test('Ouija-arc Planquette places space markers before A and after Z on the canvas, wholly reachable by both hands at furthest span', () => {
  assert.equal(PLANQUETTE_TITLE, 'Spell your name');
  assert.equal(PLANQUETTE_WARNING, 'WARNING RESULTS ARE FINAL');
  assert.equal(PLANQUETTE_CHARS.length, 28);
  assert.equal(PLANQUETTE_CHARS[0], ' ');
  assert.equal(PLANQUETTE_CHARS[1], 'A');
  assert.equal(PLANQUETTE_CHARS[26], 'Z');
  assert.equal(PLANQUETTE_CHARS[27], ' ');
  assert.equal(DOUBLE_LINE_TOP_WORLD_Y, 370);

  const origin = { x: 681.5, y: 900 };
  const cam = computePlanquetteCamera(1440, 780, origin.x);
  assert.ok(cam.zoom >= 0.68 && cam.zoom <= 1.18);

  const arc = computePlanquetteArc(origin.x, origin.y);
  assert.equal(arc.letters.length, 28);
  assert.equal(arc.radius, PLANQUETTE_ARC_RADIUS);
  assert.ok(arc.apexY >= DOUBLE_LINE_TOP_WORLD_Y, 'Arc apex sits cleanly below the double line top');

  // Verify every single letter along the Ouija arc is wholly reachable by BOTH handR and handL (and both fingertips)
  const engine = createPoseEngine();
  const baseArmature = engine.getArmature();
  for (const letter of arc.letters) {
    for (const eff of ['handR', 'handL', 'fingerTipR', 'fingerTipL'] as const) {
      const solved = solvePlanquetteArmReach(baseArmature, eff, letter.x, letter.y, origin, {
        solver: 'analytical',
        softDamp: true,
        constrainLimits: true,
      });
      const comp = computeArmature(solved, origin);
      const err = Math.hypot(comp[eff].x - letter.x, comp[eff].y - letter.y);
      assert.ok(
        err <= 2.5,
        `${eff} must reach letter '${letter.label}' (idx ${letter.index}) cleanly, got err=${err.toFixed(2)}`
      );
    }
  }

  // Hit testing leading space, A, Z, trailing space, and off-arc center
  const firstSpace = arc.letters[0];
  const letterA = arc.letters[1];
  const letterZ = arc.letters[26];
  const lastSpace = arc.letters[27];

  assert.equal(hitTestPlanquetteArc(firstSpace.x, firstSpace.y, arc)?.char, ' ');
  assert.equal(hitTestPlanquetteArc(letterA.x, letterA.y, arc)?.char, 'A');
  assert.equal(hitTestPlanquetteArc(letterZ.x, letterZ.y, arc)?.char, 'Z');
  assert.equal(hitTestPlanquetteArc(lastSpace.x, lastSpace.y, arc)?.char, ' ');
  assert.equal(hitTestPlanquetteArc(origin.x, 547.25, arc), null);

  // Faceplate upward sightline projection onto the Ouija arc
  const projected = projectFaceplateToPlanquetteArc(
    { x: origin.x, y: 616.95 },
    { x: origin.x, y: 547.25 },
    { x: origin.x + 24, y: 520 },
    arc
  );
  assert.ok(hitTestPlanquetteArc(projected.x, projected.y, arc) !== null);
});

test('Default Vitruvian T-pose angles both thumbs down 22.5 degrees symmetrically for a hand-like silhouette', () => {
  const engine = createPoseEngine();
  const handR = engine.getJoint('handR');
  const thumbR = engine.getJoint('thumbR');
  const thumbTipR = engine.getJoint('thumbTipR');
  const handL = engine.getJoint('handL');
  const thumbL = engine.getJoint('thumbL');
  const thumbTipL = engine.getJoint('thumbTipL');

  assert.equal(thumbR.localAngle, -22.5);
  assert.equal(thumbL.localAngle, 22.5);

  // Both thumb tips sit cleanly below the horizontal wrist axis (+Y down) at 22.5 deg
  assert.ok(thumbTipR.y > handR.y && thumbTipR.x < handR.x);
  assert.ok(thumbTipL.y > handL.y && thumbTipL.x > handL.x);

  const angleDownR = Math.atan2(thumbTipR.y - handR.y, handR.x - thumbTipR.x) * (180 / Math.PI);
  const angleDownL = Math.atan2(thumbTipL.y - handL.y, thumbTipL.x - handL.x) * (180 / Math.PI);
  assert.ok(Math.abs(angleDownR - 22.5) < 1e-4);
  assert.ok(Math.abs(angleDownL - 22.5) < 1e-4);
});



