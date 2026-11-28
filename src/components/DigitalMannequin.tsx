import React, { useMemo, useState } from 'react';
import {
  COLLAR_LINK_SCALE,
  REPO_FACE_PURPLE,
  REPO_GREEN_GLOW_LIGHT,
  REPO_GREEN_JOINT_LIGHT,
  REPO_SVG_PURPLE,
  REPO_STRAIN_CRIMSON,
  TORSO_HANDLE_CENTER_SVG,
} from '../data/digitalRig';
import { ArmatureState, ComputedJoint, JointId, Point2D } from '../types/rig';
import { CompositeFilter } from '../types/scene';
import {
  applyAffine,
  computeSvgBoneMatrix,
  HIP_BRIDGE_LINK_STAGE_PX,
  solveToggleHinge2Link,
} from '../utils/kinematics';

interface DigitalMannequinProps {
  armature: ArmatureState;
  computedJoints: Record<JointId, ComputedJoint>;
  selectedJointId: JointId;
  zoom: number;
  darkMode: boolean;
  isPanning: boolean;
  isMotionActive?: boolean;
  compositeFilter?: CompositeFilter;
  onPointerDownJoint: (e: React.PointerEvent<SVGGElement>, jointId: JointId) => void;
  onPointerDownBone: (e: React.PointerEvent<SVGElement>, jointId: JointId) => void;
  onPointerDownHipToggle?: (e: React.PointerEvent<SVGElement>) => void;
  onPointerDownArmSwivel?: (e: React.PointerEvent<SVGGElement>, side: 'R' | 'L') => void;
  onPointerDownNeckCollarMagnet: (e: React.PointerEvent<SVGGElement>) => void;
  onResetNeckCollarMagnet: () => void;
  onPointerDownTorso: (e: React.PointerEvent<SVGGElement>, center: Point2D) => void;
}

const DARK_MODE_BONE_GREEN = '#34D399';
const DARK_MODE_FLESH_GREEN = '#6EE7B7';
const DARK_MODE_HIGHLIGHT_PURPLE = '#D8B4FE';

function toMatrixAttr(m: readonly [number, number, number, number, number, number]): string {
  return `matrix(${m[0].toFixed(5)} ${m[1].toFixed(5)} ${m[2].toFixed(5)} ${m[3].toFixed(5)} ${m[4].toFixed(2)} ${m[5].toFixed(2)})`;
}

function makeBridgePoints(
  ax: number,
  ay: number,
  bx: number,
  by: number,
  halfWidth: number
): string {
  const dx = bx - ax;
  const dy = by - ay;
  const len = Math.hypot(dx, dy);
  if (len < 1e-4) {
    return `${ax - halfWidth},${ay - halfWidth} ${ax + halfWidth},${ay - halfWidth} ${ax + halfWidth},${ay + halfWidth} ${ax - halfWidth},${ay + halfWidth}`;
  }
  const nx = (-dy / len) * halfWidth;
  const ny = (dx / len) * halfWidth;
  return `${(ax + nx).toFixed(2)},${(ay + ny).toFixed(2)} ${(ax - nx).toFixed(2)},${(ay - ny).toFixed(2)} ${(bx - nx).toFixed(2)},${(by - ny).toFixed(2)} ${(bx + nx).toFixed(2)},${(by + ny).toFixed(2)}`;
}

export const DigitalMannequin: React.FC<DigitalMannequinProps> = React.memo(({
  armature,
  computedJoints,
  selectedJointId,
  zoom,
  darkMode,
  isPanning,
  isMotionActive = false,
  compositeFilter,
  onPointerDownJoint,
  onPointerDownBone,
  onPointerDownHipToggle,
  onPointerDownArmSwivel,
  onPointerDownNeckCollarMagnet,
  onResetNeckCollarMagnet,
  onPointerDownTorso,
}) => {
  const [hoveredJointId, setHoveredJointId] = useState<JointId | null>(null);
  const selectedJoint = computedJoints[selectedJointId];
  const blendMode = darkMode ? 'normal' : (compositeFilter?.blendMode ?? 'multiply');
  const inkOpacity = compositeFilter?.inkOpacity ?? 0.94;
  const edgeThreshold = compositeFilter?.edgeThreshold ?? 38;
  const paperGrain = compositeFilter?.paperGrain ?? 0.35;
  const bleedDispScale = Number((1.0 + paperGrain * 3.4).toFixed(2));
  const haloStdDev = Number((Math.max(0.4, 2.4 - (edgeThreshold / 100) * 1.8)).toFixed(2));
  const coreStdDev = Number((Math.max(0.15, 0.75 - (edgeThreshold / 100) * 0.55)).toFixed(2));
  const alphaSlope = Number((1.0 + (edgeThreshold / 100) * 0.65).toFixed(2));
  const bodyFill = darkMode ? DARK_MODE_BONE_GREEN : 'url(#bitruvius-ink-body-grad)';
  const faceFill = darkMode ? '#D8B4FE' : 'url(#bitruvius-face-grad)';
  const greenJoint = darkMode ? DARK_MODE_BONE_GREEN : REPO_GREEN_JOINT_LIGHT;
  const greenGlow = darkMode ? DARK_MODE_HIGHLIGHT_PURPLE : REPO_GREEN_GLOW_LIGHT;
  const hoverAccent = greenGlow;
  const activeAccent = greenGlow;
  const darkGreenFill = darkMode ? '#064E3B' : '#143828';
  const getSwivelFill = (id: JointId) => (hoveredJointId === id ? faceFill : darkGreenFill);
  const getSpineFill = () => (hoveredJointId === 'lumbar' || hoveredJointId === 'thorax' ? faceFill : darkGreenFill);
  const getHipFill = (id: JointId) => (hoveredJointId === id ? faceFill : darkGreenFill);

  const svgMatrices = useMemo(() => {
    const m = (id: JointId, mirrorX = false) =>
      computeSvgBoneMatrix(computedJoints[id].boneMatrix, mirrorX);

    return {
      pelvis: m('pelvis'),
      lumbar: m('lumbar'),
      thorax: m('thorax'),
      neck: m('neck'),
      head: m('head'),
      lip: m('lip'),
      swivelR: m('swivelR', true),
      swivelL: m('swivelL', false),
      braceR: m('braceR', true),
      clavOuterR: m('clavOuterR', true),
      braceL: m('braceL', false),
      clavOuterL: m('clavOuterL', false),
      humerusR: m('humerusR', true),
      humerusL: m('humerusL', false),
      forearmR: m('forearmR', true),
      forearmL: m('forearmL', false),
      handR: m('handR', true),
      fingerR: m('fingerR', true),
      thumbR: m('thumbR', true),
      handL: m('handL', false),
      fingerL: m('fingerL', false),
      thumbL: m('thumbL', false),
      hipR: m('hipR', false),
      hipL: m('hipL', true),
      femurR: m('femurR', false),
      femurL: m('femurL', true),
      shinR: m('shinR', false),
      shinL: m('shinL', true),
      footR: m('footR', false),
      footL: m('footL', true),
    };
  }, [computedJoints]);
  const torsoCenter = useMemo(
    () =>
      applyAffine(
        svgMatrices.lumbar,
        TORSO_HANDLE_CENTER_SVG[0],
        TORSO_HANDLE_CENTER_SVG[1]
      ),
    [svgMatrices.lumbar]
  );

  const hipToggle = useMemo(
    () =>
      solveToggleHinge2Link(
        computedJoints.femurR,
        computedJoints.femurL,
        HIP_BRIDGE_LINK_STAGE_PX * (armature.joints.pelvis?.scaleWidth ?? 1),
        armature.hipToggleSign ?? 1,
        computedJoints.pelvis
      ),
    [
      armature.hipToggleSign,
      armature.joints.pelvis?.scaleWidth,
      computedJoints.femurL,
      computedJoints.femurR,
      computedJoints.pelvis,
    ]
  );

  // 25 interactive green joints centered squarely over each joint radius, swivels & body centers
  const jointHandles = useMemo(() => {
    const pelvisCenter = applyAffine(svgMatrices.pelvis, 824, 996);
    const spineSwivelCenter = applyAffine(svgMatrices.lumbar, 824, 860);
    const shoulderSwivelRCenter = applyAffine(svgMatrices.swivelR, 873.5, 821.5);
    const shoulderSwivelLCenter = applyAffine(svgMatrices.swivelL, 873.5, 821.5);
    const clavMidRCenter = applyAffine(svgMatrices.clavOuterR, 873.5, 685.5);
    const clavMidLCenter = applyAffine(svgMatrices.clavOuterL, 873.5, 685.5);
    const hipSwivelRCenter = applyAffine(svgMatrices.hipR, 808.5, 1015.9);
    const hipSwivelLCenter = applyAffine(svgMatrices.hipL, 808.5, 1015.9);

    const list: Array<{ id: JointId; x: number; y: number; radiusName: string }> = [
      { id: 'lip', x: computedJoints.lip.x, y: computedJoints.lip.y, radiusName: 'face' },
      { id: 'neck', x: computedJoints.neck.x, y: computedJoints.neck.y, radiusName: 'neck-collar' },
      { id: 'thorax', x: computedJoints.thorax.x, y: computedJoints.thorax.y, radiusName: 'clavicle' },
      { id: 'lumbar', x: spineSwivelCenter.x, y: spineSwivelCenter.y, radiusName: 'spine-swivel' },
      { id: 'swivelR', x: shoulderSwivelRCenter.x, y: shoulderSwivelRCenter.y, radiusName: 'shoulder-swivel-r' },
      { id: 'swivelL', x: shoulderSwivelLCenter.x, y: shoulderSwivelLCenter.y, radiusName: 'shoulder-swivel-l' },
      { id: 'clavOuterR', x: clavMidRCenter.x, y: clavMidRCenter.y, radiusName: 'clavicle-mid-r' },
      { id: 'clavOuterL', x: clavMidLCenter.x, y: clavMidLCenter.y, radiusName: 'clavicle-mid-l' },
      { id: 'pelvis', x: pelvisCenter.x, y: pelvisCenter.y, radiusName: 'waist' },
      { id: 'hipR', x: hipSwivelRCenter.x, y: hipSwivelRCenter.y, radiusName: 'hip-swivel-r' },
      { id: 'hipL', x: hipSwivelLCenter.x, y: hipSwivelLCenter.y, radiusName: 'hip-swivel-l' },
      { id: 'humerusR', x: computedJoints.humerusR.x, y: computedJoints.humerusR.y, radiusName: 'shoulder-r' },
      { id: 'forearmR', x: computedJoints.forearmR.x, y: computedJoints.forearmR.y, radiusName: 'elbow-r' },
      { id: 'handR', x: computedJoints.handR.x, y: computedJoints.handR.y, radiusName: 'wrist-r' },
      { id: 'thumbTipR', x: computedJoints.thumbTipR.x, y: computedJoints.thumbTipR.y, radiusName: 'thumb-tip-r' },
      { id: 'fingerTipR', x: computedJoints.fingerTipR.x, y: computedJoints.fingerTipR.y, radiusName: 'finger-tip-r' },
      { id: 'humerusL', x: computedJoints.humerusL.x, y: computedJoints.humerusL.y, radiusName: 'shoulder-l' },
      { id: 'forearmL', x: computedJoints.forearmL.x, y: computedJoints.forearmL.y, radiusName: 'elbow-l' },
      { id: 'handL', x: computedJoints.handL.x, y: computedJoints.handL.y, radiusName: 'wrist-l' },
      { id: 'thumbTipL', x: computedJoints.thumbTipL.x, y: computedJoints.thumbTipL.y, radiusName: 'thumb-tip-l' },
      { id: 'fingerTipL', x: computedJoints.fingerTipL.x, y: computedJoints.fingerTipL.y, radiusName: 'finger-tip-l' },
      { id: 'femurR', x: computedJoints.femurR.x, y: computedJoints.femurR.y, radiusName: 'hip-r' },
      { id: 'shinR', x: computedJoints.shinR.x, y: computedJoints.shinR.y, radiusName: 'knee-r' },
      { id: 'footR', x: computedJoints.footR.x, y: computedJoints.footR.y, radiusName: 'ankle-r' },
      { id: 'toeR', x: computedJoints.toeR.x, y: computedJoints.toeR.y, radiusName: 'toe-r' },
      { id: 'femurL', x: computedJoints.femurL.x, y: computedJoints.femurL.y, radiusName: 'hip-l' },
      { id: 'shinL', x: computedJoints.shinL.x, y: computedJoints.shinL.y, radiusName: 'knee-l' },
      { id: 'footL', x: computedJoints.footL.x, y: computedJoints.footL.y, radiusName: 'ankle-l' },
      { id: 'toeL', x: computedJoints.toeL.x, y: computedJoints.toeL.y, radiusName: 'toe-l' },
      { id: 'head', x: computedJoints.head.x, y: computedJoints.head.y, radiusName: 'crown' },
    ];
    return list;
  }, [
    computedJoints,
    svgMatrices.clavOuterL,
    svgMatrices.clavOuterR,
    svgMatrices.hipL,
    svgMatrices.hipR,
    svgMatrices.lumbar,
    svgMatrices.pelvis,
    svgMatrices.swivelL,
    svgMatrices.swivelR,
  ]);

  const armSwivelHandles = useMemo(() => {
    const midR = applyAffine(svgMatrices.humerusR, 1018, 688);
    const midL = applyAffine(svgMatrices.humerusL, 1018, 688);
    return [
      {
        side: 'R' as const,
        x: midR.x,
        y: midR.y,
        deg: armature.armSwivelR ?? 0,
        reverseFactor: computedJoints.humerusR.armReverseFactor ?? 1,
      },
      {
        side: 'L' as const,
        x: midL.x,
        y: midL.y,
        deg: armature.armSwivelL ?? 0,
        reverseFactor: computedJoints.humerusL.armReverseFactor ?? 1,
      },
    ];
  }, [
    armature.armSwivelL,
    armature.armSwivelR,
    computedJoints.humerusL.armReverseFactor,
    computedJoints.humerusR.armReverseFactor,
    svgMatrices.humerusL,
    svgMatrices.humerusR,
  ]);

  const segStroke = (jointId: JointId) => {
    if (selectedJointId === jointId) return activeAccent;
    if (hoveredJointId === jointId) return hoverAccent;
    return 'none';
  };

  const segStrokeWidth = (jointId: JointId) => {
    if (selectedJointId === jointId) return 1.8;
    if (hoveredJointId === jointId) return 1.5;
    return 0;
  };

  const segStrokeOpacity = (jointId: JointId) => {
    if (selectedJointId === jointId) return darkMode ? 0.62 : 0.56;
    if (hoveredJointId === jointId) return darkMode ? 0.45 : 0.4;
    return 0;
  };

  const bindSegment = (jointId: JointId) => ({
    'data-joint-id': jointId,
    strokeOpacity: segStrokeOpacity(jointId),
    style: { cursor: isPanning ? 'grabbing' : 'pointer' } as React.CSSProperties,
    onPointerEnter: () => setHoveredJointId(jointId),
    onPointerLeave: () => setHoveredJointId(null),
    onPointerDown: (e: React.PointerEvent<SVGElement>) => onPointerDownBone(e, jointId),
  });

  return (
    <g
      data-digital-bones="true"
      data-model-style="svg-pose-full"
      data-model-color={bodyFill}
      data-selection-glow={greenGlow}
      data-prometheus-wings="removed"
      style={
        darkMode && !isMotionActive
          ? {
              filter: 'drop-shadow(0 0 1.4px rgba(216, 206, 224, 0.5))',
            }
          : undefined
      }
    >
      <defs>
        {/* Natural iron-gall plum & mulberry ink gradients */}
        <linearGradient id="bitruvius-ink-body-grad" x1="15%" y1="0%" x2="85%" y2="100%">
          <stop offset="0%" stopColor={darkMode ? '#4A1B5E' : '#3C0E4E'} stopOpacity={darkMode ? 0.94 : 0.87} />
          <stop offset="52%" stopColor={darkMode ? '#39124A' : '#2E083E'} stopOpacity={darkMode ? 0.96 : 0.90} />
          <stop offset="100%" stopColor={darkMode ? '#2B0B38' : '#22042E'} stopOpacity={darkMode ? 0.95 : 0.89} />
        </linearGradient>

        <radialGradient id="bitruvius-face-grad" cx="42%" cy="38%" r="65%">
          <stop offset="0%" stopColor={darkMode ? '#D8B4FE' : '#C084FC'} stopOpacity={darkMode ? 0.95 : 0.90} />
          <stop offset="65%" stopColor={darkMode ? '#C084FC' : '#A855F7'} stopOpacity={darkMode ? 0.96 : 0.92} />
          <stop offset="100%" stopColor={darkMode ? '#A855F7' : '#9333EA'} stopOpacity={darkMode ? 0.98 : 0.94} />
        </radialGradient>

        {/* Subtle Leonardo pen cross-hatching texture */}
        <pattern
          id="bitruvius-hatch-pattern"
          width="8"
          height="8"
          patternTransform="rotate(32)"
          patternUnits="userSpaceOnUse"
        >
          <line
            x1="0"
            y1="0"
            x2="0"
            y2="8"
            stroke={darkMode ? DARK_MODE_HIGHLIGHT_PURPLE : '#180222'}
            strokeWidth="0.65"
            strokeOpacity={darkMode ? 0.09 : 0.14}
          />
        </pattern>

        <filter
          id="bitruvius-ink-bleed"
          x="-6%"
          y="-6%"
          width="112%"
          height="112%"
          colorInterpolationFilters="sRGB"
        >
          {/* Organic capillary edge feathering & plate grain driven by CompositeFilter */}
          <feTurbulence
            type="fractalNoise"
            baseFrequency={0.05 + paperGrain * 0.04}
            numOctaves="1"
            seed="7"
            result="paperNoise"
          />
          <feDisplacementMap
            in="SourceGraphic"
            in2="paperNoise"
            scale={bleedDispScale}
            xChannelSelector="R"
            yChannelSelector="G"
            result="bleedDisplaced"
          />
          <feGaussianBlur
            in="bleedDisplaced"
            stdDeviation={haloStdDev}
            result="bleedHalo"
          />
          <feComponentTransfer in="bleedHalo" result="bleedSoft">
            <feFuncA type="linear" slope={darkMode ? 0.38 : Math.max(0.12, 0.32 - edgeThreshold * 0.002)} />
          </feComponentTransfer>
          <feGaussianBlur
            in="bleedDisplaced"
            stdDeviation={coreStdDev}
            result="featheredCore"
          />
          <feComponentTransfer in="featheredCore" result="thresholdCore">
            <feFuncA type="linear" slope={alphaSlope} intercept={-(edgeThreshold * 0.0018)} />
          </feComponentTransfer>
          <feMerge>
            <feMergeNode in="bleedSoft" />
            <feMergeNode in="thresholdCore" />
          </feMerge>
        </filter>
      </defs>

      {/* Selected joint parent reference axis ray */}
      {selectedJoint && selectedJoint.parentX !== null && selectedJoint.parentY !== null && (
        <line
          x1={selectedJoint.parentX}
          y1={selectedJoint.parentY}
          x2={
            selectedJoint.parentX +
            Math.cos(((selectedJoint.worldAngle - selectedJoint.localAngle) * Math.PI) / 180) * 68
          }
          y2={
            selectedJoint.parentY +
            Math.sin(((selectedJoint.worldAngle - selectedJoint.localAngle) * Math.PI) / 180) * 68
          }
          stroke={greenGlow}
          strokeOpacity={0.48}
          strokeWidth={1.1}
          strokeDasharray="4 3"
          vectorEffect="non-scaling-stroke"
          pointerEvents="none"
        />
      )}

      {/* Body silhouette with natural ink-on-paper multiply wash & subtle capillary edge feathering (live filter bypassed during active transitions/drags) */}
      <g
        data-ink-bleed-body="true"
        filter={isMotionActive ? undefined : 'url(#bitruvius-ink-bleed)'}
        opacity={inkOpacity}
        style={{ mixBlendMode: blendMode }}
      >
      {/* Pelvis / Waist Core matching body_pose_full_b3b6bf24.svg silhouette */}
      <g
        transform={toMatrixAttr(svgMatrices.pelvis)}
        fill={bodyFill}
        stroke={segStroke('pelvis')}
        strokeWidth={segStrokeWidth('pelvis')}
        {...bindSegment('pelvis')}
      >
        <path
          data-shape-id="waist"
          d="M824,953 C808,958 790,968 770,985 C745,1008 735,1032 735,1058 C736,1090 752,1118 780,1132 C800,1140 812,1140 825,1140 C842,1140 862,1138 880,1126 C905,1110 918,1085 918,1058 C918,1030 905,1005 880,980 C862,964 842,956 824,953 Z"
          vectorEffect="non-scaling-stroke"
        />
      </g>

      {/* Right Leg (Anatomical R -> screen left) */}
      <g>
        <g
          transform={toMatrixAttr(svgMatrices.femurR)}
          fill={bodyFill}
          stroke={segStroke('femurR')}
          strokeWidth={segStrokeWidth('femurR')}
          {...bindSegment('femurR')}
        >
          {/* Shifted upper thigh ball from body_pose_full_b3b6bf24.svg, joint pivot at (787, 1100) */}
          <circle cx="780" cy="1100" r="38" vectorEffect="non-scaling-stroke" />
          <polygon
            data-shape-id="femur-r"
            points="814.4,1116.2 671.8,1357.1 650.2,1346.9 745.6,1083.8"
            vectorEffect="non-scaling-stroke"
          />
          <circle cx="661" cy="1352" r="12" vectorEffect="non-scaling-stroke" />
        </g>
        <g
          transform={toMatrixAttr(svgMatrices.shinR)}
          fill={bodyFill}
          stroke={segStroke('shinR')}
          strokeWidth={segStrokeWidth('shinR')}
          {...bindSegment('shinR')}
        >
          {/* Knee circle radius centered squarely on shinR pivot (661, 1352) */}
          <circle cx="661" cy="1352" r="24" vectorEffect="non-scaling-stroke" />
          <polygon
            data-shape-id="shin-r"
            points="683.4,1360.5 557.4,1651.2 540.6,1644.8 638.6,1343.5"
            vectorEffect="non-scaling-stroke"
          />
        </g>
        <g
          transform={toMatrixAttr(svgMatrices.footR)}
          fill={bodyFill}
          stroke={segStroke('footR')}
          strokeWidth={segStrokeWidth('footR')}
          {...bindSegment('footR')}
        >
          {/* Ankle ball radius centered squarely on footR pivot (548, 1650) */}
          <circle cx="548" cy="1650" r="22" vectorEffect="non-scaling-stroke" />
          <polygon
            data-shape-id="foot-r"
            points="367.9,1629.9 545.0,1671.8 551.0,1628.2 370.1,1614.1"
            vectorEffect="non-scaling-stroke"
          />
          <circle cx="369" cy="1622" r="8" vectorEffect="non-scaling-stroke" />
        </g>
      </g>

      {/* Left Leg (Anatomical L -> screen right, mirrored around x = 824) */}
      <g>
        <g
          transform={toMatrixAttr(svgMatrices.femurL)}
          fill={bodyFill}
          stroke={segStroke('femurL')}
          strokeWidth={segStrokeWidth('femurL')}
          {...bindSegment('femurL')}
        >
          <circle cx="780" cy="1100" r="38" vectorEffect="non-scaling-stroke" />
          <polygon
            data-shape-id="femur-l"
            points="814.4,1116.2 671.8,1357.1 650.2,1346.9 745.6,1083.8"
            vectorEffect="non-scaling-stroke"
          />
          <circle cx="661" cy="1352" r="12" vectorEffect="non-scaling-stroke" />
        </g>
        <g
          transform={toMatrixAttr(svgMatrices.shinL)}
          fill={bodyFill}
          stroke={segStroke('shinL')}
          strokeWidth={segStrokeWidth('shinL')}
          {...bindSegment('shinL')}
        >
          <circle cx="661" cy="1352" r="24" vectorEffect="non-scaling-stroke" />
          <polygon
            data-shape-id="shin-l"
            points="683.4,1360.5 557.4,1651.2 540.6,1644.8 638.6,1343.5"
            vectorEffect="non-scaling-stroke"
          />
        </g>
        <g
          transform={toMatrixAttr(svgMatrices.footL)}
          fill={bodyFill}
          stroke={segStroke('footL')}
          strokeWidth={segStrokeWidth('footL')}
          {...bindSegment('footL')}
        >
          <circle cx="548" cy="1650" r="22" vectorEffect="non-scaling-stroke" />
          <polygon
            data-shape-id="foot-l"
            points="367.9,1629.9 545.0,1671.8 551.0,1628.2 370.1,1614.1"
            vectorEffect="non-scaling-stroke"
          />
          <circle cx="369" cy="1622" r="8" vectorEffect="non-scaling-stroke" />
        </g>
      </g>

      {/* Active Hip-Thigh Extension Silhouettes (bodyFill upper-thigh flare + faceFill inner swivel) */}
      <g
        transform={toMatrixAttr(svgMatrices.hipR)}
        fill={bodyFill}
        stroke={segStroke('hipR')}
        strokeWidth={segStrokeWidth('hipR')}
        {...bindSegment('hipR')}
      >
        <path
          data-shape-id="hip-thigh-flare-r"
          d="M824,953 C803,959 778,982 762,1014 C746,1045 741,1078 743,1102 C746,1124 762,1138 782,1138 C802,1138 818,1120 822,1096 C825,1056 826,994 824,953 Z"
          vectorEffect="non-scaling-stroke"
        />
      </g>
      <g
        transform={toMatrixAttr(svgMatrices.hipL)}
        fill={bodyFill}
        stroke={segStroke('hipL')}
        strokeWidth={segStrokeWidth('hipL')}
        {...bindSegment('hipL')}
      >
        <path
          data-shape-id="hip-thigh-flare-l"
          d="M824,953 C803,959 778,982 762,1014 C746,1045 741,1078 743,1102 C746,1124 762,1138 782,1138 C802,1138 818,1120 822,1096 C825,1056 826,994 824,953 Z"
          vectorEffect="non-scaling-stroke"
        />
      </g>
      {/* Two-Bone Rigid Toggle-Hinge Hip Bridge (Left Anchor -> Center Knee P_mid -> Right Anchor) */}
      <g
        fill={faceFill}
        stroke={segStroke('pelvis')}
        strokeWidth={segStrokeWidth('pelvis')}
        data-hip-toggle-state={hipToggle.state}
        style={{ cursor: isPanning ? 'grabbing' : 'ns-resize' }}
        onPointerDown={(e) => {
          if (onPointerDownHipToggle) {
            onPointerDownHipToggle(e);
          } else {
            onPointerDownBone(e, 'pelvis');
          }
        }}
      >
        <polygon
          data-shape-id="hip-bridge-left"
          points={makeBridgePoints(
            hipToggle.pLeft.x,
            hipToggle.pLeft.y,
            hipToggle.pMid.x,
            hipToggle.pMid.y,
            5.0
          )}
          vectorEffect="non-scaling-stroke"
        />
        <polygon
          data-shape-id="hip-bridge-right"
          points={makeBridgePoints(
            hipToggle.pMid.x,
            hipToggle.pMid.y,
            hipToggle.pRight.x,
            hipToggle.pRight.y,
            5.0
          )}
          vectorEffect="non-scaling-stroke"
        />
        <circle
          data-shape-id="hip-toggle-left-pin"
          cx={hipToggle.pLeft.x}
          cy={hipToggle.pLeft.y}
          r={6.5}
          vectorEffect="non-scaling-stroke"
        />
        <circle
          data-shape-id="hip-toggle-knee"
          cx={hipToggle.pMid.x}
          cy={hipToggle.pMid.y}
          r={7.5}
          vectorEffect="non-scaling-stroke"
        />
        <circle
          data-shape-id="hip-toggle-right-pin"
          cx={hipToggle.pRight.x}
          cy={hipToggle.pRight.y}
          r={6.5}
          vectorEffect="non-scaling-stroke"
        />
      </g>
      <g
        transform={toMatrixAttr(svgMatrices.hipR)}
        fill={faceFill}
        stroke={segStroke('hipR')}
        strokeWidth={segStrokeWidth('hipR')}
        {...bindSegment('hipR')}
      >
        <circle cx="824" cy="955" r="10" vectorEffect="non-scaling-stroke" />
        <polygon
          data-shape-id="hip-r"
          points="833.7,957.3 795.7,1102.3 778.3,1097.7 816.3,952.7"
          vectorEffect="non-scaling-stroke"
        />
        <circle cx="787" cy="1100" r="14" vectorEffect="non-scaling-stroke" />
      </g>
      <g
        transform={toMatrixAttr(svgMatrices.hipL)}
        fill={faceFill}
        stroke={segStroke('hipL')}
        strokeWidth={segStrokeWidth('hipL')}
        {...bindSegment('hipL')}
      >
        <circle cx="824" cy="955" r="10" vectorEffect="non-scaling-stroke" />
        <polygon
          data-shape-id="hip-l"
          points="833.7,957.3 795.7,1102.3 778.3,1097.7 816.3,952.7"
          vectorEffect="non-scaling-stroke"
        />
        <circle cx="787" cy="1100" r="14" vectorEffect="non-scaling-stroke" />
      </g>

      {/* Torso Shield (solid bodyFill matching other bones, layered behind active swivels) */}
      <g
        transform={toMatrixAttr(svgMatrices.lumbar)}
        fill={bodyFill}
        stroke={segStroke('lumbar')}
        strokeWidth={segStrokeWidth('lumbar')}
        {...bindSegment('lumbar')}
      >
        <path
          data-shape-id="torso"
          d="M824,652 L844,654 C880,664 912,700 916,760 C919,800 905,860 880,910 C858,945 840,968 824,968 C808,968 790,945 768,910 C743,860 729,800 732,760 C736,700 768,664 804,654 Z"
          vectorEffect="non-scaling-stroke"
        />
      </g>

      {/* Active Spine Swivel Column & Clavicle Hub (faceFill accent) */}
      <g
        transform={toMatrixAttr(svgMatrices.lumbar)}
        fill={faceFill}
        stroke={segStroke('lumbar')}
        strokeWidth={segStrokeWidth('lumbar')}
        {...bindSegment('lumbar')}
      >
        <circle cx="824" cy="955" r="14" vectorEffect="non-scaling-stroke" />
        <rect
          data-shape-id="spine-swivel"
          x="816"
          y="668"
          width="16"
          height="290"
          rx="8"
          vectorEffect="non-scaling-stroke"
        />
      </g>
      <g
        transform={toMatrixAttr(svgMatrices.thorax)}
        fill={faceFill}
        stroke={segStroke('thorax')}
        strokeWidth={segStrokeWidth('thorax')}
        {...bindSegment('thorax')}
      >
        <circle data-shape-id="clavicle-hub" cx="824" cy="683" r="13" vectorEffect="non-scaling-stroke" />
      </g>

      {/* Active Shoulder Swivel Braces (faceFill accent layered on top of solid torso) */}
      <g
        transform={toMatrixAttr(svgMatrices.swivelR)}
        fill={faceFill}
        stroke={segStroke('swivelR')}
        strokeWidth={segStrokeWidth('swivelR')}
        {...bindSegment('swivelR')}
      >
        <circle cx="824" cy="955" r="11" vectorEffect="non-scaling-stroke" />
        <polygon
          data-shape-id="swivel-r"
          points="815.5,953.2 830.5,958.8 930.5,690.8 915.5,685.2"
          vectorEffect="non-scaling-stroke"
        />
        <circle cx="923" cy="688" r="14" vectorEffect="non-scaling-stroke" />
      </g>
      <g
        transform={toMatrixAttr(svgMatrices.swivelL)}
        fill={faceFill}
        stroke={segStroke('swivelL')}
        strokeWidth={segStrokeWidth('swivelL')}
        {...bindSegment('swivelL')}
      >
        <circle cx="824" cy="955" r="11" vectorEffect="non-scaling-stroke" />
        <polygon
          data-shape-id="swivel-l"
          points="815.5,953.2 830.5,958.8 930.5,690.8 915.5,685.2"
          vectorEffect="non-scaling-stroke"
        />
        <circle cx="923" cy="688" r="14" vectorEffect="non-scaling-stroke" />
      </g>

      {/* Right Arm (Anatomical R -> screen left, mirrored from SVG armR) */}
      <g>
        <g
          transform={toMatrixAttr(svgMatrices.braceR)}
          fill={bodyFill}
          stroke={segStroke('braceR')}
          strokeWidth={segStrokeWidth('braceR')}
          {...bindSegment('braceR')}
        >
          <ellipse cx="824" cy="683" rx={(11.5 / COLLAR_LINK_SCALE).toFixed(2)} ry="11.5" vectorEffect="non-scaling-stroke" />
          <polygon
            data-shape-id="clav-inner-r"
            points={makeBridgePoints(824, 683, 873.5, 685.5, 11.5)}
            vectorEffect="non-scaling-stroke"
          />
          <ellipse cx="873.5" cy="685.5" rx={(11.5 / COLLAR_LINK_SCALE).toFixed(2)} ry="11.5" vectorEffect="non-scaling-stroke" />
        </g>
        <g
          transform={toMatrixAttr(svgMatrices.clavOuterR)}
          fill={bodyFill}
          stroke={segStroke('clavOuterR')}
          strokeWidth={segStrokeWidth('clavOuterR')}
          {...bindSegment('clavOuterR')}
        >
          <ellipse cx="873.5" cy="685.5" rx={(11.5 / COLLAR_LINK_SCALE).toFixed(2)} ry="11.5" vectorEffect="non-scaling-stroke" />
          <polygon
            data-shape-id="clav-outer-r"
            points={makeBridgePoints(873.5, 685.5, 923, 688, 11.5)}
            vectorEffect="non-scaling-stroke"
          />
          <ellipse cx="923" cy="688" rx={(12 / COLLAR_LINK_SCALE).toFixed(2)} ry="12" vectorEffect="non-scaling-stroke" />
        </g>
        <g
          transform={toMatrixAttr(svgMatrices.humerusR)}
          fill={bodyFill}
          stroke={segStroke('humerusR')}
          strokeWidth={segStrokeWidth('humerusR')}
          {...bindSegment('humerusR')}
        >
          <circle cx="923" cy="688" r="24" vectorEffect="non-scaling-stroke" />
          <path
            data-shape-id="upper-r"
            d="M916,686 C940,664 1000,655 1060,658 C1090,660 1110,672 1118,688 C1110,704 1090,716 1060,718 C1000,721 940,712 916,686 Z"
            vectorEffect="non-scaling-stroke"
          />
        </g>
        <g
          transform={toMatrixAttr(svgMatrices.forearmR)}
          fill={bodyFill}
          stroke={segStroke('forearmR')}
          strokeWidth={segStrokeWidth('forearmR')}
          {...bindSegment('forearmR')}
        >
          <circle cx="1113" cy="688" r="14" vectorEffect="non-scaling-stroke" />
          <path
            data-shape-id="fore-r"
            d="M1100,688 C1104,678 1112,672 1126,670 L1262,676 C1278,680 1288,684 1290,688 C1288,692 1278,696 1262,700 L1126,706 C1112,704 1104,698 1100,688 Z"
            vectorEffect="non-scaling-stroke"
          />
        </g>
        <g
          transform={toMatrixAttr(svgMatrices.fingerR)}
          fill={bodyFill}
          stroke={segStroke('fingerR')}
          strokeWidth={segStrokeWidth('fingerR')}
          {...bindSegment('fingerR')}
        >
          <path
            data-shape-id="hand-r-fingers"
            d="M1280,688 C1284,680 1292,676 1305,677 L1392,681 L1405,688 L1392,695 L1305,699 C1292,700 1284,696 1280,688 Z"
            vectorEffect="non-scaling-stroke"
          />
        </g>
        <g
          transform={toMatrixAttr(svgMatrices.thumbR)}
          fill={darkMode ? DARK_MODE_BONE_GREEN : '#4A2259'}
          stroke={segStroke('thumbR')}
          strokeWidth={segStrokeWidth('thumbR')}
          {...bindSegment('thumbR')}
        >
          <path
            data-shape-id="hand-r-thumb"
            d="M1280,688 C1284,680 1292,676 1305,677 L1355,682 L1368,688 L1355,694 L1305,699 C1292,700 1284,696 1280,688 Z"
            vectorEffect="non-scaling-stroke"
          />
        </g>
        <g
          transform={toMatrixAttr(svgMatrices.handR)}
          fill={bodyFill}
          stroke={segStroke('handR')}
          strokeWidth={segStrokeWidth('handR')}
          {...bindSegment('handR')}
        >
          <circle data-shape-id="wrist-r" cx="1283" cy="688" r="10" vectorEffect="non-scaling-stroke" />
        </g>
      </g>

      {/* Left Arm (Anatomical L -> screen right) */}
      <g>
        <g
          transform={toMatrixAttr(svgMatrices.braceL)}
          fill={bodyFill}
          stroke={segStroke('braceL')}
          strokeWidth={segStrokeWidth('braceL')}
          {...bindSegment('braceL')}
        >
          <ellipse cx="824" cy="683" rx={(11.5 / COLLAR_LINK_SCALE).toFixed(2)} ry="11.5" vectorEffect="non-scaling-stroke" />
          <polygon
            data-shape-id="clav-inner-l"
            points={makeBridgePoints(824, 683, 873.5, 685.5, 11.5)}
            vectorEffect="non-scaling-stroke"
          />
          <ellipse cx="873.5" cy="685.5" rx={(11.5 / COLLAR_LINK_SCALE).toFixed(2)} ry="11.5" vectorEffect="non-scaling-stroke" />
        </g>
        <g
          transform={toMatrixAttr(svgMatrices.clavOuterL)}
          fill={bodyFill}
          stroke={segStroke('clavOuterL')}
          strokeWidth={segStrokeWidth('clavOuterL')}
          {...bindSegment('clavOuterL')}
        >
          <ellipse cx="873.5" cy="685.5" rx={(11.5 / COLLAR_LINK_SCALE).toFixed(2)} ry="11.5" vectorEffect="non-scaling-stroke" />
          <polygon
            data-shape-id="clav-outer-l"
            points={makeBridgePoints(873.5, 685.5, 923, 688, 11.5)}
            vectorEffect="non-scaling-stroke"
          />
          <ellipse cx="923" cy="688" rx={(12 / COLLAR_LINK_SCALE).toFixed(2)} ry="12" vectorEffect="non-scaling-stroke" />
        </g>
        <g
          transform={toMatrixAttr(svgMatrices.humerusL)}
          fill={bodyFill}
          stroke={segStroke('humerusL')}
          strokeWidth={segStrokeWidth('humerusL')}
          {...bindSegment('humerusL')}
        >
          <circle cx="923" cy="688" r="24" vectorEffect="non-scaling-stroke" />
          <path
            data-shape-id="upper-l"
            d="M916,686 C940,664 1000,655 1060,658 C1090,660 1110,672 1118,688 C1110,704 1090,716 1060,718 C1000,721 940,712 916,686 Z"
            vectorEffect="non-scaling-stroke"
          />
        </g>
        <g
          transform={toMatrixAttr(svgMatrices.forearmL)}
          fill={bodyFill}
          stroke={segStroke('forearmL')}
          strokeWidth={segStrokeWidth('forearmL')}
          {...bindSegment('forearmL')}
        >
          <circle cx="1113" cy="688" r="14" vectorEffect="non-scaling-stroke" />
          <path
            data-shape-id="fore-l"
            d="M1100,688 C1104,678 1112,672 1126,670 L1262,676 C1278,680 1288,684 1290,688 C1288,692 1278,696 1262,700 L1126,706 C1112,704 1104,698 1100,688 Z"
            vectorEffect="non-scaling-stroke"
          />
        </g>
        <g
          transform={toMatrixAttr(svgMatrices.fingerL)}
          fill={bodyFill}
          stroke={segStroke('fingerL')}
          strokeWidth={segStrokeWidth('fingerL')}
          {...bindSegment('fingerL')}
        >
          <path
            data-shape-id="hand-l-fingers"
            d="M1280,688 C1284,680 1292,676 1305,677 L1392,681 L1405,688 L1392,695 L1305,699 C1292,700 1284,696 1280,688 Z"
            vectorEffect="non-scaling-stroke"
          />
        </g>
        <g
          transform={toMatrixAttr(svgMatrices.thumbL)}
          fill={darkMode ? DARK_MODE_BONE_GREEN : '#4A2259'}
          stroke={segStroke('thumbL')}
          strokeWidth={segStrokeWidth('thumbL')}
          {...bindSegment('thumbL')}
        >
          <path
            data-shape-id="hand-l-thumb"
            d="M1280,688 C1284,680 1292,676 1305,677 L1355,682 L1368,688 L1355,694 L1305,699 C1292,700 1284,696 1280,688 Z"
            vectorEffect="non-scaling-stroke"
          />
        </g>
        <g
          transform={toMatrixAttr(svgMatrices.handL)}
          fill={bodyFill}
          stroke={segStroke('handL')}
          strokeWidth={segStrokeWidth('handL')}
          {...bindSegment('handL')}
        >
          <circle data-shape-id="wrist-l" cx="1283" cy="688" r="10" vectorEffect="non-scaling-stroke" />
        </g>
      </g>

      {/* Neck & Head Stack */}
      <g>
        <g
          transform={toMatrixAttr(svgMatrices.neck)}
          fill={bodyFill}
          stroke={segStroke('neck')}
          strokeWidth={segStrokeWidth('neck')}
          {...bindSegment('neck')}
        >
          <rect x="800" y="618" width="48" height="62" rx="20" vectorEffect="non-scaling-stroke" />
        </g>
        <g
          transform={toMatrixAttr(svgMatrices.head)}
          fill={bodyFill}
          stroke={segStroke('head')}
          strokeWidth={segStrokeWidth('head')}
          {...bindSegment('head')}
        >
          <rect x="789" y="565" width="70" height="45" rx="22" vectorEffect="non-scaling-stroke" />
          <path
            data-shape-id="skull"
            d="M824,468 C870,468 878,520 878,545 C878,590 866,612 850,620 C840,625 832,626 824,626 C816,626 808,625 798,620 C782,612 770,590 770,545 C770,520 778,468 824,468 Z"
            vectorEffect="non-scaling-stroke"
          />
          <rect x="818" y="540" width="12" height="82" rx="6" vectorEffect="non-scaling-stroke" />
        </g>
        <g
          transform={toMatrixAttr(svgMatrices.lip)}
          fill={faceFill}
          stroke={segStroke('lip')}
          strokeWidth={segStrokeWidth('lip')}
          {...bindSegment('lip')}
        >
          <circle data-shape-id="face-medallion" cx="824" cy="540" r="30" vectorEffect="non-scaling-stroke" />
        </g>
      </g>
      </g>

      {/* Interactive Translucent Verdigris/Green Joint Dots Centered Squarely Over Each Radius */}
      <g data-joint-handles="true">
        {jointHandles.map((handle) => {
          const joint = computedJoints[handle.id];
          const runtime = armature.joints[handle.id];
          const isSelected = selectedJointId === handle.id;
          const isHovered = hoveredJointId === handle.id;
          const isRigid = runtime?.physicsMode === 'rigid';
          const isExplicitPin = joint.mode === 'PIN' && !joint.spec.locked;
          const isIK = joint.mode === 'IK';
          const canDrag =
            !isExplicitPin &&
            !isRigid &&
            (!joint.spec.locked ||
              (joint.spec.isTip &&
                Boolean(joint.spec.parentId) &&
                armature.joints[joint.spec.parentId!]?.mode !== 'PIN' &&
                armature.joints[joint.spec.parentId!]?.physicsMode !== 'rigid'));

          const collarSnapState = joint.collarSnapState;
          const collarResistanceRatio = joint.collarResistanceRatio;
          const neckCollarPressureRatio = joint.neckCollarPressureRatio ?? 0;
          const isCollarResisting = collarSnapState === 'resisting';
          const isCollarSnapped = collarSnapState === 'snapped';

          const ringStroke = isExplicitPin
            ? '#D97706'
            : isCollarResisting
            ? '#D97706'
            : isCollarSnapped
            ? darkMode
              ? DARK_MODE_HIGHLIGHT_PURPLE
              : '#0D9488'
            : isIK
            ? darkMode
              ? DARK_MODE_HIGHLIGHT_PURPLE
              : '#0D9488'
            : greenGlow;

          return (
            <g
              key={`joint-dot-${handle.id}`}
              data-joint-dot={handle.id}
              data-joint-mode={joint.mode}
              data-radius-center={handle.radiusName}
              data-collar-snap-state={collarSnapState}
              data-collar-resistance={
                collarResistanceRatio !== undefined
                  ? collarResistanceRatio.toFixed(2)
                  : undefined
              }
              data-neck-collar-pressure={neckCollarPressureRatio.toFixed(2)}
              transform={`translate(${handle.x}, ${handle.y}) scale(${1 / zoom})`}
              onPointerEnter={() => setHoveredJointId(handle.id)}
              onPointerLeave={() => setHoveredJointId(null)}
              onPointerDown={(e) => onPointerDownJoint(e, handle.id)}
              style={{
                cursor: isPanning
                  ? 'grabbing'
                  : !canDrag
                  ? 'default'
                  : 'pointer',
              }}
            >
              <circle r={6.5} fill="transparent" />
              {handle.id === 'neck' && neckCollarPressureRatio > 0 && (
                <circle
                  r={5.5 + neckCollarPressureRatio * 2.5}
                  fill="none"
                  stroke={REPO_STRAIN_CRIMSON}
                  strokeOpacity={0.25 + neckCollarPressureRatio * 0.65}
                  strokeWidth={1 + neckCollarPressureRatio}
                  style={{
                    filter: `drop-shadow(0 0 ${1 + neckCollarPressureRatio * 3}px ${REPO_STRAIN_CRIMSON})`,
                  }}
                />
              )}
              {/* Subtle translucent verdigris halo on IK/selected/hovered/collar-snap joints */}
              {(isSelected ||
                isHovered ||
                isIK ||
                isExplicitPin ||
                isCollarResisting ||
                isCollarSnapped) && (
                <circle
                  r={
                    isSelected
                      ? 5.2
                      : isCollarSnapped
                      ? 4.9
                      : isCollarResisting
                      ? 4.7
                      : isHovered
                      ? 4.6
                      : 3.7
                  }
                  fill="none"
                  stroke={ringStroke}
                  strokeOpacity={
                    isSelected
                      ? 0.82
                      : isCollarSnapped
                      ? 0.86
                      : isCollarResisting
                      ? 0.55 + (collarResistanceRatio ?? 0) * 0.35
                      : isHovered
                      ? 0.68
                      : 0.48
                  }
                  strokeWidth={isSelected || isCollarSnapped ? 1.35 : isHovered ? 1.15 : 0.9}
                  strokeDasharray={
                    isExplicitPin || isCollarResisting ? '1.8 1.5' : undefined
                  }
                />
              )}
              <circle
                r={isSelected ? 2.4 : 2.05}
                fill={
                  handle.id === 'neck' && neckCollarPressureRatio > 0
                    ? REPO_STRAIN_CRIMSON
                    : isExplicitPin
                    ? '#F59E0B'
                    : isCollarSnapped
                    ? darkMode
                      ? DARK_MODE_HIGHLIGHT_PURPLE
                      : '#0D9488'
                    : isCollarResisting
                    ? '#F59E0B'
                    : greenJoint
                }
                fillOpacity={
                  isSelected || isHovered || isCollarResisting || isCollarSnapped ? 0.82 : 0.54
                }
              />
            </g>
          );
        })}

        {(() => {
          const thorax = computedJoints.thorax;
          const neck = computedJoints.neck;
          const pressure = neck.neckCollarPressureRatio ?? 0;
          const x = (thorax.x + neck.x) / 2;
          const y = (thorax.y + neck.y) / 2;
          return (
            <g
              data-neck-collar-magnet="true"
              data-neck-collar-pressure={pressure.toFixed(2)}
              transform={`translate(${x}, ${y}) scale(${1 / zoom})`}
              onPointerDown={onPointerDownNeckCollarMagnet}
              onDoubleClick={(event) => {
                event.stopPropagation();
                onResetNeckCollarMagnet();
              }}
              style={{ cursor: isPanning ? 'grabbing' : 'move' }}
            >
              <title>Drag to separate the neck and collar group; double-click to reconnect</title>
              <circle r={8} fill="transparent" />
              {pressure > 0.01 && (
                <circle
                  r={5.6 + pressure * 2.2}
                  fill="none"
                  stroke={REPO_STRAIN_CRIMSON}
                  strokeOpacity={0.28 + pressure * 0.62}
                  strokeWidth={1 + pressure}
                  style={{
                    filter: `drop-shadow(0 0 ${1 + pressure * 3}px ${REPO_STRAIN_CRIMSON})`,
                  }}
                />
              )}
              <circle
                r={3.5}
                fill="none"
                stroke={pressure > 0.01 ? REPO_STRAIN_CRIMSON : greenGlow}
                strokeOpacity={pressure > 0.01 ? 0.92 : 0.38}
                strokeWidth={1.15}
              />
              <path
                d="M-1.6,-1.7 V1.7 M1.6,-1.7 V1.7"
                fill="none"
                stroke={pressure > 0.01 ? REPO_STRAIN_CRIMSON : greenJoint}
                strokeWidth={1.15}
                strokeLinecap="round"
              />
            </g>
          );
        })()}

        {/* Interactive Center Toggle Hinge ("knee") for the 2-bone hip bridge */}
        <g
          data-hip-toggle-dot="true"
          data-hip-toggle-state={hipToggle.state}
          transform={`translate(${hipToggle.pMid.x}, ${hipToggle.pMid.y}) scale(${1 / zoom})`}
          onPointerDown={(e) => onPointerDownHipToggle?.(e)}
          style={{ cursor: isPanning ? 'grabbing' : 'ns-resize' }}
        >
          <circle r={6.5} fill="transparent" />
          <circle
            r={3.9}
            fill="none"
            stroke={greenGlow}
            strokeOpacity={0.56}
            strokeWidth={0.95}
          />
          <circle
            r={2.1}
            fill={greenJoint}
            fillOpacity={0.76}
          />
        </g>

        {/* Per-Arm Axial Reverse Swivel Handles (click to flip 180°, drag to swivel smoothly) */}
        {armSwivelHandles.map((sw) => {
          const isReversed = sw.reverseFactor < 0;
          const accent = isReversed
            ? darkMode
              ? DARK_MODE_HIGHLIGHT_PURPLE
              : '#0D9488'
            : greenGlow;
          return (
            <g
              key={`arm-swivel-dot-${sw.side}`}
              data-arm-swivel-dot={sw.side}
              data-arm-swivel-deg={sw.deg.toFixed(1)}
              data-arm-reversed={isReversed ? 'true' : 'false'}
              transform={`translate(${sw.x}, ${sw.y}) scale(${1 / zoom})`}
              onPointerDown={(e) => onPointerDownArmSwivel?.(e, sw.side)}
              style={{ cursor: isPanning ? 'grabbing' : 'ns-resize' }}
            >
              <circle r={6.5} fill="transparent" />
              <circle
                r={3.9}
                fill="none"
                stroke={accent}
                strokeOpacity={isReversed ? 0.84 : 0.52}
                strokeWidth={isReversed ? 1.25 : 0.9}
                strokeDasharray={isReversed ? undefined : '2.1 1.4'}
              />
              <circle
                r={1.85}
                fill={isReversed ? accent : greenJoint}
                fillOpacity={isReversed ? 0.86 : 0.62}
              />
            </g>
          );
        })}
        <g
          data-torso-ik-handle="true"
          transform={`translate(${torsoCenter.x}, ${torsoCenter.y}) scale(${1 / zoom})`}
          onPointerEnter={() => setHoveredJointId('lumbar')}
          onPointerLeave={() => setHoveredJointId(null)}
          onPointerDown={(e) => onPointerDownTorso(e, torsoCenter)}
          style={{ cursor: isPanning ? 'grabbing' : 'move' }}
        >
          <title>Drag to balance the torso with the whole figure</title>
          <circle r={8} fill="transparent" />
          <circle
            r={4.5}
            fill="none"
            stroke={activeAccent}
            strokeOpacity={selectedJointId === 'lumbar' ? 0.9 : 0.58}
            strokeWidth={selectedJointId === 'lumbar' ? 1.35 : 1}
          />
          <circle r={1.7} fill={greenJoint} fillOpacity={0.78} />
        </g>
      </g>
    </g>
  );
});
