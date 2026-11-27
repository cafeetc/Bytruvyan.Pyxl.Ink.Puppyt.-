import { JOINT_SPECS } from '../data/digitalRig';
import {
  BitruviusExportIR,
  EXPORT_IR_SCHEMA,
  EXPORT_IR_VERSION,
  ExportIRBoneNode,
  ExportIRKeyframe,
  ExportIRKeyframeJoint,
  ExportIRPointPYX,
  ExportIRRootPose,
  ExportIRTransformPair,
} from '../types/exportIR';
import {
  ArmatureState,
  ComputedJoint,
  JointId,
  KeyframeSlot,
  MotionCurve,
  Point2D,
  PosePresetId,
} from '../types/rig';
import { computeArmature, interpolateArmatures } from './kinematics';
import { PAPER_HEIGHT, PAPER_WIDTH, PYXL_PX } from './proportions';

export interface BuildExportIROptions {
  armature: ArmatureState;
  preset?: PosePresetId | null;
  sceneId?: string;
  sceneName?: string;
  slots?: Record<KeyframeSlot, ArmatureState | null>;
  motionCurve?: MotionCurve;
  durationSec?: number;
  fps?: number;
  sampleCount?: number;
  canvasOriginPx?: Point2D;
  exportedAt?: string;
}

const DEFAULT_CANVAS_ORIGIN: Point2D = {
  x: PAPER_WIDTH / 2,
  y: PAPER_HEIGHT / 2,
};

export const DETERMINISTIC_EXPORT_TIMESTAMP = '2026-01-01T00:00:00.000Z';

function roundIR(value: number, decimals = 4): number {
  if (!Number.isFinite(value)) return 0;
  const factor = Math.pow(10, decimals);
  const rounded = Math.round(value * factor) / factor;
  return Object.is(rounded, -0) ? 0 : rounded;
}

function normalizeSignedDeg(deg: number): number {
  let d = deg % 360;
  if (d > 180) d -= 360;
  if (d <= -180) d += 360;
  return roundIR(d, 4);
}

function pxPointToOriginPyx(xPx: number, yPx: number, originPx: Point2D): ExportIRPointPYX {
  return {
    x: roundIR((xPx - originPx.x) / PYXL_PX, 4),
    y: roundIR((yPx - originPx.y) / PYXL_PX, 4),
  };
}

function buildTransformPair(
  joint: ComputedJoint,
  originPx: Point2D
): ExportIRTransformPair {
  const m = joint.boneMatrix;
  return {
    local: {
      angleDeg: roundIR(joint.localAngle, 4),
      rotationDeltaDeg: roundIR(joint.cumulativeDeltaDeg, 4),
    },
    world: {
      positionPyx: pxPointToOriginPyx(joint.x, joint.y, originPx),
      endPositionPyx: pxPointToOriginPyx(joint.endX, joint.endY, originPx),
      angleDeg: roundIR(joint.worldAngle, 4),
      restAngleDeg: roundIR(joint.restWorldDeg, 4),
      matrixPyx: [
        roundIR(m[0], 6),
        roundIR(m[1], 6),
        roundIR(m[2], 6),
        roundIR(m[3], 6),
        roundIR((m[4] - originPx.x) / PYXL_PX, 4),
        roundIR((m[5] - originPx.y) / PYXL_PX, 4),
      ] as const,
    },
  };
}

function buildRootPose(armature: ArmatureState, originPx: Point2D): ExportIRRootPose {
  return {
    positionPyx: pxPointToOriginPyx(armature.rootX, armature.rootY, originPx),
    rotationDeg: roundIR(armature.rootRotation, 4),
    hipToggleSign: armature.hipToggleSign ?? 1,
    armSwivelR: roundIR(armature.armSwivelR ?? 0, 4),
    armSwivelL: roundIR(armature.armSwivelL ?? 0, 4),
  };
}

function evaluateTimelineArmature(
  slots: Record<KeyframeSlot, ArmatureState | null>,
  t: number,
  curve: MotionCurve,
  fallback: ArmatureState
): ArmatureState {
  const a = slots.A;
  const b = slots.B;
  const c = slots.C;
  if (!a) {
    return b ?? c ?? fallback;
  }
  if (b && c) {
    return t <= 0.5
      ? interpolateArmatures(a, b, t * 2, curve)
      : interpolateArmatures(b, c, (t - 0.5) * 2, curve);
  }
  if (c) return interpolateArmatures(a, c, t, curve);
  if (b) return interpolateArmatures(a, b, t, curve);
  return a;
}

function resolveSlotAtT(
  activeSlots: KeyframeSlot[],
  t: number,
  slots?: Record<KeyframeSlot, ArmatureState | null>
): KeyframeSlot | null {
  if (!slots || activeSlots.length === 0) return null;
  if (Math.abs(t) < 1e-6) {
    if (slots.A) return 'A';
    return activeSlots[0] ?? null;
  }
  if (slots.B && slots.C && Math.abs(t - 0.5) < 1e-6) return 'B';
  if (Math.abs(t - 1) < 1e-6) {
    if (slots.C) return 'C';
    if (slots.B && !slots.C) return 'B';
  }
  return null;
}

export function buildExportIR(options: BuildExportIROptions): BitruviusExportIR {
  const originPx = options.canvasOriginPx ?? DEFAULT_CANVAS_ORIGIN;
  const motionCurve = options.motionCurve ?? 'flow';
  const durationSec = Math.max(0.1, roundIR(options.durationSec ?? 2.0, 4));
  const fps = Math.max(1, Math.round(options.fps ?? 24));
  const currentComputed = computeArmature(options.armature, originPx);

  const childrenMap = new Map<JointId, JointId[]>();
  const depthMap = new Map<JointId, number>();
  for (const spec of JOINT_SPECS) {
    childrenMap.set(spec.id, []);
  }
  for (const spec of JOINT_SPECS) {
    if (spec.parentId) {
      childrenMap.get(spec.parentId)?.push(spec.id);
    }
    const parentDepth = spec.parentId ? (depthMap.get(spec.parentId) ?? 0) : -1;
    depthMap.set(spec.id, parentDepth + 1);
  }

  const boneOrder: JointId[] = JOINT_SPECS.map((s) => s.id);
  const bones: ExportIRBoneNode[] = JOINT_SPECS.map((spec) => {
    const comp = currentComputed[spec.id];
    const rt = options.armature.joints[spec.id];
    return {
      id: spec.id,
      label: spec.label,
      shortCode: spec.shortCode,
      parentId: spec.parentId,
      children: [...(childrenMap.get(spec.id) ?? [])],
      depth: depthMap.get(spec.id) ?? 0,
      role: spec.role,
      side: spec.side,
      mode: comp.mode,
      physicsMode: rt?.physicsMode ?? 'twins',
      lengthPyx: roundIR(comp.effectiveLengthPx / PYXL_PX, 4),
      restLengthPyx: roundIR(spec.lengthPx / PYXL_PX, 4),
      scaleLength: roundIR(rt?.scaleLength ?? 1, 4),
      scaleWidth: roundIR(comp.effectiveWidthScale ?? rt?.scaleWidth ?? 1, 4),
      isTip: Boolean(spec.isTip),
      constraints: {
        limitsDeg: spec.limits ? [spec.limits[0], spec.limits[1]] : null,
        ...(comp.collarSnapState ? { collarSnapState: comp.collarSnapState } : {}),
        ...(comp.collarResistanceRatio !== undefined
          ? { collarResistanceRatio: roundIR(comp.collarResistanceRatio, 4) }
          : {}),
        ...(comp.armReverseFactor !== undefined
          ? { armReverseFactor: roundIR(comp.armReverseFactor, 4) }
          : {}),
        ...(comp.isStrained ? { isStrained: true } : {}),
        ...(comp.tensionRatio !== undefined
          ? { tensionRatio: roundIR(comp.tensionRatio, 4) }
          : {}),
      },
      transform: buildTransformPair(comp, originPx),
    };
  });

  const slots = options.slots;
  const activeSlots: KeyframeSlot[] = slots
    ? (['A', 'B', 'C'] as KeyframeSlot[]).filter((k) => Boolean(slots[k]))
    : [];
  const hasInterpolationRange = Boolean(slots?.A && (slots.B || slots.C));
  const resolvedSampleCount = hasInterpolationRange
    ? Math.max(2, Math.round(options.sampleCount ?? 9))
    : 1;

  const keyframes: ExportIRKeyframe[] = [];
  for (let idx = 0; idx < resolvedSampleCount; idx++) {
    const t = resolvedSampleCount <= 1 ? 0 : roundIR(idx / (resolvedSampleCount - 1), 6);
    const frameArmature =
      hasInterpolationRange && slots
        ? evaluateTimelineArmature(slots, t, motionCurve, options.armature)
        : activeSlots.length === 1 && slots
        ? slots[activeSlots[0]] ?? options.armature
        : options.armature;
    const frameComputed = computeArmature(frameArmature, originPx);
    const frameJoints = {} as Record<JointId, ExportIRKeyframeJoint>;
    for (const id of boneOrder) {
      const jc = frameComputed[id];
      frameJoints[id] = {
        lengthPyx: roundIR(jc.effectiveLengthPx / PYXL_PX, 4),
        ...buildTransformPair(jc, originPx),
      };
    }
    keyframes.push({
      index: idx,
      t,
      timeSec: roundIR(t * durationSec, 4),
      sourceSlot: resolveSlotAtT(activeSlots, t, slots),
      root: buildRootPose(frameArmature, originPx),
      joints: frameJoints,
    });
  }

  return {
    schema: EXPORT_IR_SCHEMA,
    version: EXPORT_IR_VERSION,
    exportedAt: options.exportedAt ?? DETERMINISTIC_EXPORT_TIMESTAMP,
    sceneId: options.sceneId ?? 'scene-1',
    sceneName: options.sceneName ?? 'Scene 1 · Back Cover',
    preset: options.preset ?? 'custom',
    units: {
      pyxPerPx: roundIR(1 / PYXL_PX, 6),
      pxPerPyx: PYXL_PX,
      canvasSizePyx: {
        width: roundIR(PAPER_WIDTH / PYXL_PX, 4),
        height: roundIR(PAPER_HEIGHT / PYXL_PX, 4),
      },
      originPyx: {
        x: roundIR(originPx.x / PYXL_PX, 4),
        y: roundIR(originPx.y / PYXL_PX, 4),
      },
      positionSpace: 'origin-centered-pyx',
      coordinateSystem: 'y-down-2d',
    },
    hierarchy: {
      rootBoneId: 'pelvis',
      boneOrder,
      bones,
    },
    pose: {
      root: buildRootPose(options.armature, originPx),
    },
    timeline: {
      durationSec,
      fps,
      motionCurve,
      activeSlots,
      sampleCount: keyframes.length,
      keyframes,
    },
  };
}

function canonicalizeValue(value: unknown): unknown {
  if (value === null || typeof value !== 'object') {
    if (typeof value === 'number') {
      return Object.is(value, -0) ? 0 : value;
    }
    return value;
  }
  if (Array.isArray(value)) {
    return value.map((item) => canonicalizeValue(item));
  }
  const obj = value as Record<string, unknown>;
  const sortedKeys = Object.keys(obj).sort();
  const result: Record<string, unknown> = {};
  for (const key of sortedKeys) {
    const v = obj[key];
    if (v !== undefined) {
      result[key] = canonicalizeValue(v);
    }
  }
  return result;
}

function escapeXmlAttr(value: string): string {
  return value
    .replace(/&/g, '&amp;')
    .replace(/"/g, '&quot;')
    .replace(/</g, '&lt;')
    .replace(/>/g, '&gt;');
}

export function adaptIRToCanonicalJson(ir: BitruviusExportIR): string {
  return `${JSON.stringify(canonicalizeValue(ir), null, 2)}\n`;
}

export function adaptIRToCanonicalJsonBlob(ir: BitruviusExportIR): Blob {
  return new Blob([adaptIRToCanonicalJson(ir)], {
    type: 'application/json;charset=utf-8',
  });
}

export function adaptIRToLayeredSvg(ir: BitruviusExportIR): string {
  const pxPerPyx = ir.units.pxPerPyx || PYXL_PX;
  const originX = ir.units.originPyx.x * pxPerPyx;
  const originY = ir.units.originPyx.y * pxPerPyx;

  let minX = Number.POSITIVE_INFINITY;
  let minY = Number.POSITIVE_INFINITY;
  let maxX = Number.NEGATIVE_INFINITY;
  let maxY = Number.NEGATIVE_INFINITY;

  for (const bone of ir.hierarchy.bones) {
    const x1 = originX + bone.transform.world.positionPyx.x * pxPerPyx;
    const y1 = originY + bone.transform.world.positionPyx.y * pxPerPyx;
    const x2 = originX + bone.transform.world.endPositionPyx.x * pxPerPyx;
    const y2 = originY + bone.transform.world.endPositionPyx.y * pxPerPyx;
    minX = Math.min(minX, x1, x2);
    minY = Math.min(minY, y1, y2);
    maxX = Math.max(maxX, x1, x2);
    maxY = Math.max(maxY, y1, y2);
  }

  if (!Number.isFinite(minX)) {
    minX = 0;
    minY = 0;
    maxX = PAPER_WIDTH;
    maxY = PAPER_HEIGHT;
  }

  const pad = 64;
  const vbX = roundIR(minX - pad, 2);
  const vbY = roundIR(minY - pad, 2);
  const vbW = Math.max(100, roundIR(maxX - minX + pad * 2, 2));
  const vbH = Math.max(100, roundIR(maxY - minY + pad * 2, 2));

  const boneLines: string[] = [];
  const jointNodes: string[] = [];

  for (const bone of ir.hierarchy.bones) {
    const x1 = roundIR(originX + bone.transform.world.positionPyx.x * pxPerPyx, 3);
    const y1 = roundIR(originY + bone.transform.world.positionPyx.y * pxPerPyx, 3);
    const x2 = roundIR(originX + bone.transform.world.endPositionPyx.x * pxPerPyx, 3);
    const y2 = roundIR(originY + bone.transform.world.endPositionPyx.y * pxPerPyx, 3);
    const parentAttr = bone.parentId ? escapeXmlAttr(bone.parentId) : 'none';

    if (bone.lengthPyx > 0.0001) {
      const strokeW = roundIR(Math.max(1.5, 4.2 * bone.scaleWidth), 2);
      boneLines.push(
        `    <line id="bone-${escapeXmlAttr(bone.id)}" data-bone-id="${escapeXmlAttr(bone.id)}" data-parent-id="${parentAttr}" data-length-pyx="${bone.lengthPyx}" x1="${x1}" y1="${y1}" x2="${x2}" y2="${y2}" stroke="#16001E" stroke-width="${strokeW}" stroke-linecap="round" />`
      );
    }

    const radius = bone.id === ir.hierarchy.rootBoneId ? 6.5 : bone.isTip ? 2.6 : 4.2;
    const fill = bone.isTip ? '#583168' : '#3CA360';
    jointNodes.push(
      `    <circle id="joint-${escapeXmlAttr(bone.id)}" data-joint-id="${escapeXmlAttr(bone.id)}" data-parent-id="${parentAttr}" data-depth="${bone.depth}" data-mode="${escapeXmlAttr(bone.mode)}" data-local-deg="${bone.transform.local.angleDeg}" data-world-deg="${bone.transform.world.angleDeg}" cx="${x1}" cy="${y1}" r="${radius}" fill="${fill}" stroke="#16001E" stroke-width="1.2" />`
    );
  }

  return [
    '<?xml version="1.0" encoding="UTF-8"?>',
    `<svg xmlns="http://www.w3.org/2000/svg" viewBox="${vbX} ${vbY} ${vbW} ${vbH}" width="${Math.round(vbW)}" height="${Math.round(vbH)}" data-ir-schema="${escapeXmlAttr(ir.schema)}" data-ir-version="${escapeXmlAttr(ir.version)}" data-scene-id="${escapeXmlAttr(ir.sceneId)}" data-preset="${escapeXmlAttr(ir.preset)}" data-keyframe-count="${ir.timeline.sampleCount}">`,
    '  <g id="layer-guides-pyx" data-layer="guides">',
    `    <rect x="${vbX + 8}" y="${vbY + 8}" width="${roundIR(vbW - 16, 2)}" height="${roundIR(vbH - 16, 2)}" fill="none" stroke="#16001E" stroke-opacity="0.18" stroke-width="1" />`,
    `    <line x1="${roundIR(originX, 2)}" y1="${vbY}" x2="${roundIR(originX, 2)}" y2="${roundIR(vbY + vbH, 2)}" stroke="#16001E" stroke-opacity="0.16" stroke-dasharray="4 6" stroke-width="1" />`,
    `    <line x1="${vbX}" y1="${roundIR(originY, 2)}" x2="${roundIR(vbX + vbW, 2)}" y2="${roundIR(originY, 2)}" stroke="#16001E" stroke-opacity="0.16" stroke-dasharray="4 6" stroke-width="1" />`,
    '  </g>',
    '  <g id="layer-bones" data-layer="bones">',
    ...boneLines,
    '  </g>',
    '  <g id="layer-joints" data-layer="joints">',
    ...jointNodes,
    '  </g>',
    '</svg>',
    '',
  ].join('\n');
}

export function adaptIRToLayeredSvgBlob(ir: BitruviusExportIR): Blob {
  return new Blob([adaptIRToLayeredSvg(ir)], {
    type: 'image/svg+xml;charset=utf-8',
  });
}

function toParentLocalYUp(
  wxPx: number,
  wyPx: number,
  worldAngleYDownDeg: number,
  parentWxPx: number,
  parentWyPx: number,
  parentWorldAngleYDownDeg: number
): { x: number; y: number; rotation: number } {
  // Spine uses Y-up coordinates and CCW angles
  const wyUp = -wyPx;
  const parentWyUp = -parentWyPx;
  const wDegUp = -worldAngleYDownDeg;
  const parentWDegUp = -parentWorldAngleYDownDeg;

  const dx = wxPx - parentWxPx;
  const dy = wyUp - parentWyUp;
  const pr = (parentWDegUp * Math.PI) / 180;
  const cp = Math.cos(pr);
  const sp = Math.sin(pr);

  return {
    x: roundIR(dx * cp + dy * sp, 3),
    y: roundIR(-dx * sp + dy * cp, 3),
    rotation: normalizeSignedDeg(wDegUp - parentWDegUp),
  };
}

export function adaptIRToSpineJson(ir: BitruviusExportIR): string {
  const pxPerPyx = ir.units.pxPerPyx || PYXL_PX;
  const boneMap = new Map(ir.hierarchy.bones.map((b) => [b.id, b]));

  const setupWorld = new Map<
    JointId,
    { wx: number; wy: number; wDeg: number; localX: number; localY: number; localRot: number }
  >();

  const spineBones = ir.hierarchy.bones.map((b) => {
    const wx = b.transform.world.positionPyx.x * pxPerPyx;
    const wy = b.transform.world.positionPyx.y * pxPerPyx;
    const wDeg = b.transform.world.angleDeg;
    const parentBone = b.parentId ? boneMap.get(b.parentId) : undefined;
    const parentSetup = b.parentId ? setupWorld.get(b.parentId) : undefined;

    const local =
      parentBone && parentSetup
        ? toParentLocalYUp(wx, wy, wDeg, parentSetup.wx, parentSetup.wy, parentSetup.wDeg)
        : {
            x: roundIR(wx, 3),
            y: roundIR(-wy, 3),
            rotation: normalizeSignedDeg(-wDeg),
          };

    setupWorld.set(b.id, {
      wx,
      wy,
      wDeg,
      localX: local.x,
      localY: local.y,
      localRot: local.rotation,
    });

    const entry: Record<string, unknown> = {
      name: b.id,
      ...(b.parentId ? { parent: b.parentId } : {}),
      ...(b.lengthPyx > 0 ? { length: roundIR(b.lengthPyx * pxPerPyx, 3) } : {}),
      x: local.x,
      y: local.y,
      rotation: local.rotation,
      scaleX: b.scaleLength,
      scaleY: b.scaleWidth,
    };
    return entry;
  });

  const nonZeroBones = ir.hierarchy.bones.filter((b) => b.lengthPyx > 0.0001);
  const spineSlots = nonZeroBones.map((b) => ({
    name: b.id,
    bone: b.id,
    attachment: b.id,
  }));

  const defaultAttachments: Record<string, Record<string, unknown>> = {};
  for (const b of nonZeroBones) {
    const lenPx = roundIR(b.lengthPyx * pxPerPyx, 3);
    defaultAttachments[b.id] = {
      [b.id]: {
        type: 'region',
        x: roundIR(lenPx / 2, 3),
        y: 0,
        width: lenPx,
        height: roundIR(Math.max(6, 14 * b.scaleWidth), 2),
      },
    };
  }

  const boneAnimTracks: Record<
    string,
    {
      rotate: Array<{ time: number; value: number }>;
      translate: Array<{ time: number; x: number; y: number }>;
    }
  > = {};

  for (const b of ir.hierarchy.bones) {
    const baseLocal = setupWorld.get(b.id)!;
    const rotate: Array<{ time: number; value: number }> = [];
    const translate: Array<{ time: number; x: number; y: number }> = [];

    for (const kf of ir.timeline.keyframes) {
      const jf = kf.joints[b.id];
      const pf = b.parentId ? kf.joints[b.parentId] : null;
      const wx = jf.world.positionPyx.x * pxPerPyx;
      const wy = jf.world.positionPyx.y * pxPerPyx;
      const wDeg = jf.world.angleDeg;

      const local = pf
        ? toParentLocalYUp(
            wx,
            wy,
            wDeg,
            pf.world.positionPyx.x * pxPerPyx,
            pf.world.positionPyx.y * pxPerPyx,
            pf.world.angleDeg
          )
        : {
            x: roundIR(wx, 3),
            y: roundIR(-wy, 3),
            rotation: normalizeSignedDeg(-wDeg),
          };

      rotate.push({
        time: kf.timeSec,
        value: normalizeSignedDeg(local.rotation - baseLocal.localRot),
      });
      translate.push({
        time: kf.timeSec,
        x: roundIR(local.x - baseLocal.localX, 3),
        y: roundIR(local.y - baseLocal.localY, 3),
      });
    }

    boneAnimTracks[b.id] = { rotate, translate };
  }

  const spinePayload = {
    skeleton: {
      spine: '4.1.24',
      hash: `${ir.schema}:${ir.version}`,
      width: roundIR(ir.units.canvasSizePyx.width * pxPerPyx, 2),
      height: roundIR(ir.units.canvasSizePyx.height * pxPerPyx, 2),
      fps: ir.timeline.fps,
    },
    bones: spineBones,
    slots: spineSlots,
    skins: [
      {
        name: 'default',
        attachments: defaultAttachments,
      },
    ],
    animations: {
      bitruvius_clip: {
        bones: boneAnimTracks,
      },
    },
  };

  return `${JSON.stringify(canonicalizeValue(spinePayload), null, 2)}\n`;
}

export function adaptIRToSpineJsonBlob(ir: BitruviusExportIR): Blob {
  return new Blob([adaptIRToSpineJson(ir)], {
    type: 'application/json;charset=utf-8',
  });
}

function toParentLocalYDown(
  wxPx: number,
  wyPx: number,
  worldAngleDeg: number,
  parentWxPx: number,
  parentWyPx: number,
  parentWorldAngleDeg: number
): { x: number; y: number; sk: number } {
  const dx = wxPx - parentWxPx;
  const dy = wyPx - parentWyPx;
  const pr = (parentWorldAngleDeg * Math.PI) / 180;
  const cp = Math.cos(pr);
  const sp = Math.sin(pr);

  return {
    x: roundIR(dx * cp + dy * sp, 3),
    y: roundIR(-dx * sp + dy * cp, 3),
    sk: normalizeSignedDeg(worldAngleDeg - parentWorldAngleDeg),
  };
}

export function adaptIRToDragonBonesJson(ir: BitruviusExportIR): string {
  const pxPerPyx = ir.units.pxPerPyx || PYXL_PX;
  const fps = ir.timeline.fps;
  const totalFrames = Math.max(1, Math.round(ir.timeline.durationSec * fps));

  const setupLocal = new Map<JointId, { x: number; y: number; sk: number }>();

  const dbBones = ir.hierarchy.bones.map((b) => {
    const wx = b.transform.world.positionPyx.x * pxPerPyx;
    const wy = b.transform.world.positionPyx.y * pxPerPyx;
    const wDeg = b.transform.world.angleDeg;
    const parent = b.parentId
      ? ir.hierarchy.bones.find((candidate) => candidate.id === b.parentId)
      : undefined;

    const local = parent
      ? toParentLocalYDown(
          wx,
          wy,
          wDeg,
          parent.transform.world.positionPyx.x * pxPerPyx,
          parent.transform.world.positionPyx.y * pxPerPyx,
          parent.transform.world.angleDeg
        )
      : {
          x: roundIR(wx, 3),
          y: roundIR(wy, 3),
          sk: normalizeSignedDeg(wDeg),
        };

    setupLocal.set(b.id, local);

    return {
      name: b.id,
      ...(b.parentId ? { parent: b.parentId } : {}),
      ...(b.lengthPyx > 0 ? { length: roundIR(b.lengthPyx * pxPerPyx, 3) } : {}),
      transform: {
        x: local.x,
        y: local.y,
        skX: local.sk,
        skY: local.sk,
        scX: b.scaleLength,
        scY: b.scaleWidth,
      },
    };
  });

  const nonZeroBones = ir.hierarchy.bones.filter((b) => b.lengthPyx > 0.0001);
  const dbSlots = nonZeroBones.map((b) => ({
    name: b.id,
    parent: b.id,
  }));
  const dbSkinSlots = nonZeroBones.map((b) => ({
    name: b.id,
    display: [
      {
        name: b.id,
        type: 'image',
        transform: {
          x: roundIR((b.lengthPyx * pxPerPyx) / 2, 3),
          y: 0,
        },
      },
    ],
  }));

  const framePositions = ir.timeline.keyframes.map((kf) => Math.round(kf.t * totalFrames));

  const dbBoneAnimations = ir.hierarchy.bones.map((b) => {
    const base = setupLocal.get(b.id)!;
    const rotateFrame: Array<{ duration: number; rotate: number; tweenEasing?: number }> = [];
    const translateFrame: Array<{ duration: number; x: number; y: number; tweenEasing?: number }> =
      [];

    ir.timeline.keyframes.forEach((kf, idx) => {
      const jf = kf.joints[b.id];
      const pf = b.parentId ? kf.joints[b.parentId] : null;
      const wx = jf.world.positionPyx.x * pxPerPyx;
      const wy = jf.world.positionPyx.y * pxPerPyx;
      const wDeg = jf.world.angleDeg;

      const local = pf
        ? toParentLocalYDown(
            wx,
            wy,
            wDeg,
            pf.world.positionPyx.x * pxPerPyx,
            pf.world.positionPyx.y * pxPerPyx,
            pf.world.angleDeg
          )
        : {
            x: roundIR(wx, 3),
            y: roundIR(wy, 3),
            sk: normalizeSignedDeg(wDeg),
          };

      const nextFramePos =
        idx < framePositions.length - 1 ? framePositions[idx + 1] : totalFrames;
      const duration = Math.max(0, nextFramePos - framePositions[idx]);
      const isLast = idx === ir.timeline.keyframes.length - 1;

      rotateFrame.push({
        duration,
        rotate: normalizeSignedDeg(local.sk - base.sk),
        ...(isLast ? {} : { tweenEasing: 0 }),
      });
      translateFrame.push({
        duration,
        x: roundIR(local.x - base.x, 3),
        y: roundIR(local.y - base.y, 3),
        ...(isLast ? {} : { tweenEasing: 0 }),
      });
    });

    return {
      name: b.id,
      rotateFrame,
      translateFrame,
    };
  });

  const dbPayload = {
    frameRate: fps,
    name: ir.sceneName,
    version: '5.5',
    compatibleVersion: '5.5',
    isGlobal: 0,
    armature: [
      {
        type: 'Armature',
        frameRate: fps,
        name: ir.sceneName,
        aabb: {
          x: roundIR(-ir.units.originPyx.x * pxPerPyx, 2),
          y: roundIR(-ir.units.originPyx.y * pxPerPyx, 2),
          width: roundIR(ir.units.canvasSizePyx.width * pxPerPyx, 2),
          height: roundIR(ir.units.canvasSizePyx.height * pxPerPyx, 2),
        },
        bone: dbBones,
        slot: dbSlots,
        skin: [
          {
            name: 'default',
            slot: dbSkinSlots,
          },
        ],
        animation: [
          {
            name: 'bitruvius_clip',
            duration: totalFrames,
            playTimes: 0,
            bone: dbBoneAnimations,
          },
        ],
      },
    ],
  };

  return `${JSON.stringify(canonicalizeValue(dbPayload), null, 2)}\n`;
}

export function adaptIRToDragonBonesJsonBlob(ir: BitruviusExportIR): Blob {
  return new Blob([adaptIRToDragonBonesJson(ir)], {
    type: 'application/json;charset=utf-8',
  });
}
