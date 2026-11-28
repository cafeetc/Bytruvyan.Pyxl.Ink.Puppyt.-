import React, { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { JOINT_SPEC_MAP, REPO_STRAIN_CRIMSON } from '../data/digitalRig';
import {
  GROUND_CONTACT_TOLERANCE_PX,
  getGroundContactPin,
} from '../engine/PoseEngine';
import { GridConfig, PageId } from '../types/canvas';
import {
  ArmatureState,
  ComputedJoint,
  GroundContactPin,
  JointId,
  Point2D,
  RigSystemsConfig,
} from '../types/rig';
import {
  DEFAULT_TOGGLE_TRANSITION_MS,
  FaceplateLookCollarHold,
  HIP_BRIDGE_LINK_STAGE_PX,
  applyCollarSwivelDragInput,
  applyJointAngleConstraint,
  computeArmature,
  getPhysicsMultiplier,
  normalizeAngleDeg,
  setArmReverseSwivel,
  setJointMode,
  setNeckCollarSeparation,
  shortestAngleDelta,
  solveCollarToggleFromTarget,
  solveFaceplateLookIK,
  solveHipToggleFromTarget,
  solveJointIK,
  solvePlanquetteArmReach,
  solvePressurePlateContact,
  solveToggleHinge2Link,
  solveTorsoBalance,
  solveWaistBalanceHula,
  toggleArmReverseSwivel,
} from '../utils/kinematics';
import {
  MEGAPYXL_PX,
  PAPER_HEIGHT,
  PAPER_WIDTH,
  PYXL_PX,
  REPOSITORY_ASSET_PATHS,
  SUBPYXL_PX,
  computeProportionalCanons,
  computeSymmetricalFitZoom,
  getPageMetadata,
} from '../utils/proportions';
import { DigitalMannequin } from './DigitalMannequin';
import { ScanBorderLayer } from './ScanBorderLayer';

export interface BookStageCanvasProps {
  pageId: PageId;
  grid: GridConfig;
  darkMode: boolean;
  zoom: number;
  pan: { x: number; y: number };
  armature: ArmatureState;
  computedJoints: Record<JointId, ComputedJoint>;
  selectedJointId: JointId;
  onZoomChange: (nextZoom: number) => void;
  onPanChange: (nextPan: { x: number; y: number }) => void;
  onViewportResize: (size: { width: number; height: number }) => void;
  onCursorMove?: (coords: { x: number; y: number } | null) => void;
  onSelectJoint: (id: JointId) => void;
  onUpdateArmature: (next: ArmatureState) => void;
  onAnimateArmature?: (next: ArmatureState, durationMs?: number) => void;
  isPoseTransitioning?: boolean;
  onCommitPoseHistory: () => void;
  rigSystems: RigSystemsConfig;
  onUpdateRigSystems: (patch: Partial<RigSystemsConfig>) => void;
  worldOverlay?: React.ReactNode;
  planquetteReachActive?: boolean;
  onFaceplateDragChange?: (isDragging: boolean) => void;
  onFaceplateDragMove?: (drag: { pointer: Point2D; target: Point2D }) => void;
  onPlanquetteDragChange?: (info: {
    isDragging: boolean;
    source: 'faceplate' | 'handR' | 'handL' | null;
  }) => void;
  onPlanquetteDragMove?: (info: {
    source: 'faceplate' | 'handR' | 'handL';
    effectorJointId: 'lip' | 'handR' | 'handL' | 'fingerTipR' | 'fingerTipL';
    effectorWorld: Point2D;
    headWorld?: Point2D;
    lipWorld?: Point2D;
    pointerWorld: Point2D;
    pointerScreen: Point2D;
  }) => void;
  onDragEnd?: () => void;
}

const LIGHT_INK_OPACITY = {
  sub: 0.065,
  pyxl: 0.145,
  mega: 0.28,
  axis: 0.36,
  guide: 0.34,
} as const;

const EIGHT_HEAD_INDICES = [0, 1, 2, 3, 4, 5, 6, 7, 8] as const;
const MAX_ZOOM = 2.4;

function getFigureBounds(joints: Record<JointId, ComputedJoint>) {
  const bounds = {
    minX: Number.POSITIVE_INFINITY,
    minY: Number.POSITIVE_INFINITY,
    maxX: Number.NEGATIVE_INFINITY,
    maxY: Number.NEGATIVE_INFINITY,
  };
  for (const joint of Object.values(joints)) {
    bounds.minX = Math.min(bounds.minX, joint.x, joint.endX);
    bounds.minY = Math.min(bounds.minY, joint.y, joint.endY);
    bounds.maxX = Math.max(bounds.maxX, joint.x, joint.endX);
    bounds.maxY = Math.max(bounds.maxY, joint.y, joint.endY);
  }
  return bounds;
}

type ActiveDragState =
  | {
      type: 'pan';
      startClientX: number;
      startClientY: number;
      startPanX: number;
      startPanY: number;
    }
  | {
      type: 'ik';
      jointId: JointId;
      startArmature?: ArmatureState;
      offsetX: number;
      offsetY: number;
    }
  | {
      type: 'faceplate_look';
      startArmature: ArmatureState;
      offsetX: number;
      offsetY: number;
      collarHold?: FaceplateLookCollarHold;
    }
  | {
      type: 'pressure_plate';
      startArmature: ArmatureState;
      offsetY: number;
    }
  | {
      type: 'neck_collar_magnet';
      startPointer: Point2D;
      startSeparation: Point2D;
    }
  | {
      type: 'fk_rotate';
      targetJointId: JointId;
      pivotX: number;
      pivotY: number;
      lastCursorDeg: number | null;
      rawAngle: number;
    }
  | {
      type: 'waist_balance';
      startPelvis: Point2D;
      startRootRotation: number;
      startLumbarAngle: number;
      startThoraxAngle: number;
      startNeckAngle: number;
      footRPin: GroundContactPin;
      footLPin: GroundContactPin;
      offsetX: number;
      offsetY: number;
    }
  | {
      type: 'torso_balance';
      startPelvis: Point2D;
      startRootRotation: number;
      startLumbarAngle: number;
      startThoraxAngle: number;
      startNeckAngle: number;
      footRPin: GroundContactPin;
      footLPin: GroundContactPin;
      offsetX: number;
      offsetY: number;
    }
  | {
      type: 'hip_toggle';
      offsetX: number;
      offsetY: number;
    }
  | {
      type: 'collar_toggle';
      side: 'R' | 'L';
      offsetX: number;
      offsetY: number;
    }
  | {
      type: 'arm_swivel';
      side: 'R' | 'L';
      startClientX: number;
      startClientY: number;
      startSwivelDeg: number;
      hasDragged: boolean;
    };

export const BookStageCanvas: React.FC<BookStageCanvasProps> = React.memo(({
  pageId,
  grid,
  darkMode,
  zoom,
  pan,
  armature,
  computedJoints,
  selectedJointId,
  onZoomChange,
  onPanChange,
  onViewportResize,
  onCursorMove,
  onSelectJoint,
  onUpdateArmature,
  onAnimateArmature,
  isPoseTransitioning = false,
  onCommitPoseHistory,
  rigSystems,
  onUpdateRigSystems,
  worldOverlay,
  planquetteReachActive = false,
  onFaceplateDragChange,
  onFaceplateDragMove,
  onPlanquetteDragChange,
  onPlanquetteDragMove,
  onDragEnd,
}) => {
  const containerRef = useRef<HTMLDivElement | null>(null);
  const svgRef = useRef<SVGSVGElement | null>(null);
  const [viewportSize, setViewportSize] = useState({ width: 1440, height: 820 });
  const [dragState, setDragState] = useState<ActiveDragState | null>(null);
  const dragRef = useRef<ActiveDragState | null>(null);
  const [isGroundLineHovered, setIsGroundLineHovered] = useState(false);
  const [activePlusWorld, setActivePlusWorld] = useState<Point2D | null>(null);

  const armatureRef = useRef(armature);
  armatureRef.current = armature;
  const computedJointsRef = useRef(computedJoints);
  computedJointsRef.current = computedJoints;
  const panRef = useRef(pan);
  panRef.current = pan;
  const zoomRef = useRef(zoom);
  zoomRef.current = zoom;

  const pendingUpdateRef = useRef<{
    hasCursor?: boolean;
    cursor?: { x: number; y: number } | null;
    pan?: { x: number; y: number };
    zoom?: number;
    armature?: ArmatureState;
  }>({});
  const flushFrameRef = useRef<number | null>(null);
  const pendingPointerMoveRef = useRef<{ clientX: number; clientY: number } | null>(null);
  const pointerMoveFrameRef = useRef<number | null>(null);

  const meta = getPageMetadata(pageId);
  const isCover = meta.substrate === 'cover';

  const flushUpdates = useCallback(() => {
    if (flushFrameRef.current !== null) {
      window.cancelAnimationFrame(flushFrameRef.current);
      flushFrameRef.current = null;
    }
    const pending = pendingUpdateRef.current;
    if (
      !pending.hasCursor &&
      pending.pan === undefined &&
      pending.zoom === undefined &&
      pending.armature === undefined
    ) {
      return;
    }
    pendingUpdateRef.current = {};

    if (pending.hasCursor) {
      onCursorMove?.(pending.cursor ?? null);
    }
    if (pending.zoom !== undefined) {
      onZoomChange(pending.zoom);
    }
    if (pending.pan !== undefined) {
      onPanChange(pending.pan);
    }
    if (pending.armature !== undefined) {
      onUpdateArmature(pending.armature);
    }
  }, [onCursorMove, onPanChange, onUpdateArmature, onZoomChange]);

  const enqueueUpdate = useCallback(
    (patch: {
      hasCursor?: boolean;
      cursor?: { x: number; y: number } | null;
      pan?: { x: number; y: number };
      zoom?: number;
      armature?: ArmatureState;
    }) => {
      pendingUpdateRef.current = {
        ...pendingUpdateRef.current,
        ...patch,
      };
      if (flushFrameRef.current !== null) return;
      flushFrameRef.current = window.requestAnimationFrame(() => {
        flushFrameRef.current = null;
        flushUpdates();
      });
    },
    [flushUpdates]
  );

  useEffect(() => {
    return () => {
      if (flushFrameRef.current !== null) {
        window.cancelAnimationFrame(flushFrameRef.current);
        flushFrameRef.current = null;
      }
      if (pointerMoveFrameRef.current !== null) {
        window.cancelAnimationFrame(pointerMoveFrameRef.current);
        pointerMoveFrameRef.current = null;
      }
      pendingPointerMoveRef.current = null;
    };
  }, []);

  useEffect(() => {
    const el = containerRef.current;
    if (!el) return;
    const update = () => {
      const rect = el.getBoundingClientRect();
      if (rect.width > 0 && rect.height > 0) {
        const nextSize = { width: Math.round(rect.width), height: Math.round(rect.height) };
        setViewportSize(nextSize);
        onViewportResize(nextSize);
      }
    };
    update();
    const ro = new ResizeObserver(update);
    ro.observe(el);
    return () => ro.disconnect();
  }, [onViewportResize]);

  const canons = useMemo(() => computeProportionalCanons(), []);
  const originOffset = useMemo(() => ({ x: PAPER_WIDTH / 2, y: PAPER_HEIGHT / 2 }), []);
  const figureBounds = useMemo(() => getFigureBounds(computedJoints), [computedJoints]);

  const endlessBounds = useMemo(() => {
    const overscan = Math.max(2200, 1600 / Math.max(0.1, zoom));
    const minX = -pan.x / zoom - overscan;
    const minY = -pan.y / zoom - overscan;
    const width = viewportSize.width / zoom + overscan * 2;
    const height = viewportSize.height / zoom + overscan * 2;
    return {
      minX,
      minY,
      width,
      height,
      maxX: minX + width,
      maxY: minY + height,
    };
  }, [pan.x, pan.y, zoom, viewportSize.width, viewportSize.height]);

  const subpyxlZoomOpacity = zoom < 0.25 ? 0.22 : zoom < 0.5 ? 0.55 : 1.0;

  const clientToRigWorld = useCallback(
    (clientX: number, clientY: number) => {
      const rect = svgRef.current?.getBoundingClientRect();
      if (!rect) return { x: originOffset.x, y: originOffset.y };
      const sx = clientX - rect.left;
      const sy = clientY - rect.top;
      return {
        x: (sx - panRef.current.x) / zoomRef.current,
        y: (sy - panRef.current.y) / zoomRef.current,
      };
    },
    [originOffset.x, originOffset.y]
  );

  const clientToScreenLocal = useCallback((clientX: number, clientY: number): Point2D => {
    const rect = svgRef.current?.getBoundingClientRect();
    if (!rect) return { x: clientX, y: clientY };
    return { x: clientX - rect.left, y: clientY - rect.top };
  }, []);

  const handleWheel = useCallback(
    (e: WheelEvent) => {
      e.preventDefault();
      const rect = svgRef.current?.getBoundingClientRect();
      if (!rect) return;
      const factor = e.deltaY < 0 ? 1.1 : 0.91;
      const curZoom = zoomRef.current;
      const curPan = panRef.current;
      const bypassResistance = e.metaKey && e.shiftKey;
      const minZoom = bypassResistance
        ? 0.1
        : computeSymmetricalFitZoom(viewportSize.width, viewportSize.height);
      const centerX = viewportSize.width / 2;
      const centerY = viewportSize.height / 2;
      const worldX = (centerX - curPan.x) / curZoom;
      const worldY = (centerY - curPan.y) / curZoom;
      const desiredZoom = Math.max(minZoom, Math.min(MAX_ZOOM, curZoom * factor));
      let nextZoom = desiredZoom;
      if (!bypassResistance && desiredZoom > curZoom) {
        const fitZoom = Math.max(
          minZoom,
          Math.min(
            (viewportSize.width / 2) /
              Math.max(Math.abs(figureBounds.minX - worldX), Math.abs(figureBounds.maxX - worldX)),
            (viewportSize.height / 2) /
              Math.max(Math.abs(figureBounds.minY - worldY), Math.abs(figureBounds.maxY - worldY))
          )
        );
        if (desiredZoom > fitZoom) {
          nextZoom =
            curZoom < fitZoom
              ? fitZoom + (desiredZoom - fitZoom) * 0.18
              : curZoom + (desiredZoom - curZoom) * 0.18;
        }
      }
      nextZoom = Math.max(minZoom, Math.min(MAX_ZOOM, nextZoom));
      const nextPan = {
        x: centerX - worldX * nextZoom,
        y: centerY - worldY * nextZoom,
      };
      zoomRef.current = nextZoom;
      panRef.current = nextPan;
      enqueueUpdate({ zoom: nextZoom, pan: nextPan });
    },
    [enqueueUpdate, figureBounds, viewportSize.height, viewportSize.width]
  );

  useEffect(() => {
    const svg = svgRef.current;
    if (!svg) return;
    svg.addEventListener('wheel', handleWheel, { passive: false });
    return () => svg.removeEventListener('wheel', handleWheel);
  }, [handleWheel]);

  const safeSetPointerCapture = useCallback((pointerId: number) => {
    try {
      svgRef.current?.setPointerCapture(pointerId);
    } catch {
      // Ignore synthetic/stale pointer IDs
    }
  }, []);

  const setActiveDrag = useCallback((next: ActiveDragState | null) => {
    dragRef.current = next;
    setDragState(next);
  }, []);

  const beginPanDrag = useCallback(
    (e: React.PointerEvent) => {
      safeSetPointerCapture(e.pointerId);
      setActiveDrag({
        type: 'pan',
        startClientX: e.clientX,
        startClientY: e.clientY,
        startPanX: panRef.current.x,
        startPanY: panRef.current.y,
      });
    },
    [safeSetPointerCapture, setActiveDrag]
  );

  const startCumulativeFKRotate = useCallback(
    (e: React.PointerEvent, targetJointId: JointId) => {
      const curArmature = armatureRef.current;
      const runtime = curArmature.joints[targetJointId];
      const spec = JOINT_SPEC_MAP[targetJointId];
      if (!runtime || spec?.locked || runtime.mode === 'PIN' || runtime.physicsMode === 'rigid') {
        return;
      }
      const world = clientToRigWorld(e.clientX, e.clientY);
      const pivot = computedJointsRef.current[targetJointId];
      const dist = Math.hypot(world.x - pivot.x, world.y - pivot.y);
      const lastCursorDeg =
        dist >= 4 ? (Math.atan2(world.y - pivot.y, world.x - pivot.x) * 180) / Math.PI : null;
      const rawAngle = targetJointId === 'pelvis' ? curArmature.rootRotation : runtime.localAngle;
      safeSetPointerCapture(e.pointerId);
      setActiveDrag({
        type: 'fk_rotate',
        targetJointId,
        pivotX: pivot.x,
        pivotY: pivot.y,
        lastCursorDeg,
        rawAngle,
      });
    },
    [clientToRigWorld, safeSetPointerCapture, setActiveDrag]
  );

  const beginFaceplateLookDrag = useCallback(
    (e: React.PointerEvent<SVGElement>) => {
      e.stopPropagation();
      onSelectJoint('lip');
      const curArmature = armatureRef.current;
      const runtime = curArmature.joints.lip;
      if (!runtime || runtime.mode === 'PIN' || runtime.physicsMode === 'rigid') return;

      const world = clientToRigWorld(e.clientX, e.clientY);
      const screen = clientToScreenLocal(e.clientX, e.clientY);
      const faceplate = computedJointsRef.current.lip;
      const startArmature = setJointMode(curArmature, 'lip', 'IK');
      armatureRef.current = startArmature;
      safeSetPointerCapture(e.pointerId);
      setActiveDrag({
        type: 'faceplate_look',
        startArmature,
        offsetX: faceplate.x - world.x,
        offsetY: faceplate.y - world.y,
      });
      setActivePlusWorld(world);
      onFaceplateDragChange?.(true);
      onFaceplateDragMove?.({ pointer: world, target: faceplate });
      onPlanquetteDragChange?.({ isDragging: true, source: 'faceplate' });
      const headJoint = computedJointsRef.current.head;
      onPlanquetteDragMove?.({
        source: 'faceplate',
        effectorJointId: 'lip',
        effectorWorld: { x: faceplate.x, y: faceplate.y },
        headWorld: { x: headJoint.x, y: headJoint.y },
        lipWorld: { x: faceplate.x, y: faceplate.y },
        pointerWorld: world,
        pointerScreen: screen,
      });
      enqueueUpdate({ armature: startArmature });
    },
    [
      clientToRigWorld,
      clientToScreenLocal,
      enqueueUpdate,
      onFaceplateDragChange,
      onFaceplateDragMove,
      onPlanquetteDragChange,
      onPlanquetteDragMove,
      onSelectJoint,
      safeSetPointerCapture,
      setActiveDrag,
    ]
  );

  const beginNeckCollarMagnetDrag = useCallback(
    (e: React.PointerEvent<SVGGElement>) => {
      e.stopPropagation();
      onSelectJoint('neck');
      const world = clientToRigWorld(e.clientX, e.clientY);
      const separation = armatureRef.current.neckCollarSeparation ?? { x: 0, y: 0 };
      safeSetPointerCapture(e.pointerId);
      setActiveDrag({
        type: 'neck_collar_magnet',
        startPointer: world,
        startSeparation: separation,
      });
    },
    [clientToRigWorld, onSelectJoint, safeSetPointerCapture, setActiveDrag]
  );

  const resetNeckCollarMagnet = useCallback(() => {
    const reset = setNeckCollarSeparation(armatureRef.current, { x: 0, y: 0 });
    armatureRef.current = reset;
    onUpdateArmature(reset);
    onCommitPoseHistory();
  }, [onCommitPoseHistory, onUpdateArmature]);

  const handlePointerDownStage = useCallback(
    (e: React.PointerEvent<SVGSVGElement>) => {
      if (e.button !== 1) return;
      beginPanDrag(e);
    },
    [beginPanDrag]
  );

  const beginWaistBalanceDrag = useCallback(
    (e: React.PointerEvent) => {
      const curArmature = armatureRef.current;
      const pelvisRt = curArmature.joints.pelvis;
      if (!pelvisRt || pelvisRt.mode === 'PIN' || pelvisRt.physicsMode === 'rigid') return;
      const world = clientToRigWorld(e.clientX, e.clientY);
      const joints = computedJointsRef.current;
      const pelvis = joints.pelvis;
      const groundY = rigSystems.pressurePlateY ?? rigSystems.groundY;
      safeSetPointerCapture(e.pointerId);
      setActiveDrag({
        type: 'waist_balance',
        startPelvis: { x: pelvis.x, y: pelvis.y },
        startRootRotation: curArmature.rootRotation,
        startLumbarAngle: curArmature.joints.lumbar.localAngle,
        startThoraxAngle: curArmature.joints.thorax.localAngle,
        startNeckAngle: curArmature.joints.neck.localAngle,
        footRPin: getGroundContactPin(joints.footR, joints.toeR, groundY, 'R'),
        footLPin: getGroundContactPin(joints.footL, joints.toeL, groundY, 'L'),
        offsetX: pelvis.x - world.x,
        offsetY: pelvis.y - world.y,
      });
    },
    [
      clientToRigWorld,
      rigSystems.groundY,
      rigSystems.pressurePlateY,
      safeSetPointerCapture,
      setActiveDrag,
    ]
  );

  const beginTorsoBalanceDrag = useCallback(
    (e: React.PointerEvent<SVGGElement>, torsoCenter: Point2D) => {
      e.stopPropagation();
      const curArmature = armatureRef.current;
      const lumbarRt = curArmature.joints.lumbar;
      if (!lumbarRt || lumbarRt.mode === 'PIN' || lumbarRt.physicsMode === 'rigid') return;

      onSelectJoint('lumbar');
      const world = clientToRigWorld(e.clientX, e.clientY);
      const joints = computedJointsRef.current;
      const groundY = rigSystems.pressurePlateY ?? rigSystems.groundY;
      safeSetPointerCapture(e.pointerId);
      setActiveDrag({
        type: 'torso_balance',
        startPelvis: { x: joints.pelvis.x, y: joints.pelvis.y },
        startRootRotation: curArmature.rootRotation,
        startLumbarAngle: curArmature.joints.lumbar.localAngle,
        startThoraxAngle: curArmature.joints.thorax.localAngle,
        startNeckAngle: curArmature.joints.neck.localAngle,
        footRPin: getGroundContactPin(joints.footR, joints.toeR, groundY, 'R'),
        footLPin: getGroundContactPin(joints.footL, joints.toeL, groundY, 'L'),
        offsetX: torsoCenter.x - world.x,
        offsetY: torsoCenter.y - world.y,
      });
    },
    [
      clientToRigWorld,
      onSelectJoint,
      rigSystems.groundY,
      rigSystems.pressurePlateY,
      safeSetPointerCapture,
      setActiveDrag,
    ]
  );

  const beginPressurePlateDrag = useCallback(
    (e: React.PointerEvent<SVGPathElement>) => {
      e.stopPropagation();
      const joints = computedJointsRef.current;
      const pressureY =
        rigSystems.pressurePlateY ??
        Math.max(joints.footR.y, joints.toeR.y, joints.footL.y, joints.toeL.y);
      const world = clientToRigWorld(e.clientX, e.clientY);
      safeSetPointerCapture(e.pointerId);
      setActiveDrag({
        type: 'pressure_plate',
        startArmature: armatureRef.current,
        offsetY: pressureY - world.y,
      });
    },
    [clientToRigWorld, rigSystems.pressurePlateY, safeSetPointerCapture, setActiveDrag]
  );

  const handlePointerDownHipToggle = useCallback(
    (e: React.PointerEvent<SVGElement>) => {
      e.stopPropagation();
      onSelectJoint('pelvis');
      const curArmature = armatureRef.current;
      if (
        curArmature.joints.hipR.physicsMode === 'rigid' ||
        curArmature.joints.hipL.physicsMode === 'rigid'
      ) {
        return;
      }
      const world = clientToRigWorld(e.clientX, e.clientY);
      const joints = computedJointsRef.current;
      const toggle = solveToggleHinge2Link(
        joints.femurR,
        joints.femurL,
        HIP_BRIDGE_LINK_STAGE_PX * (curArmature.joints.pelvis?.scaleWidth ?? 1),
        curArmature.hipToggleSign ?? 1,
        joints.pelvis
      );
      safeSetPointerCapture(e.pointerId);
      setActiveDrag({
        type: 'hip_toggle',
        offsetX: toggle.pMid.x - world.x,
        offsetY: toggle.pMid.y - world.y,
      });
    },
    [clientToRigWorld, onSelectJoint, safeSetPointerCapture, setActiveDrag]
  );

  const handlePointerDownArmSwivel = useCallback(
    (e: React.PointerEvent<SVGGElement>, side: 'R' | 'L') => {
      e.stopPropagation();
      const armJointId: JointId = side === 'R' ? 'humerusR' : 'humerusL';
      onSelectJoint(armJointId);
      const curArmature = armatureRef.current;
      if (curArmature.joints[armJointId]?.physicsMode === 'rigid') return;
      safeSetPointerCapture(e.pointerId);
      setActiveDrag({
        type: 'arm_swivel',
        side,
        startClientX: e.clientX,
        startClientY: e.clientY,
        startSwivelDeg:
          side === 'R' ? (curArmature.armSwivelR ?? 0) : (curArmature.armSwivelL ?? 0),
        hasDragged: false,
      });
    },
    [onSelectJoint, safeSetPointerCapture, setActiveDrag]
  );

  const handlePointerDownJoint = useCallback(
    (e: React.PointerEvent<SVGGElement>, jointId: JointId) => {
      if (jointId === 'lip') {
        beginFaceplateLookDrag(e);
        return;
      }
      e.stopPropagation();
      onSelectJoint(jointId);
      const curArmature = armatureRef.current;
      const joint = computedJointsRef.current[jointId];
      const spec = JOINT_SPEC_MAP[jointId];
      const world = clientToRigWorld(e.clientX, e.clientY);
      const screen = clientToScreenLocal(e.clientX, e.clientY);

      if (joint.mode === 'ROOT' || jointId === 'pelvis') {
        beginWaistBalanceDrag(e);
        return;
      }

      if (jointId === 'clavOuterR' || jointId === 'clavOuterL') {
        const side: 'R' | 'L' = jointId === 'clavOuterR' ? 'R' : 'L';
        const braceRt = curArmature.joints[side === 'R' ? 'braceR' : 'braceL'];
        if (!braceRt || braceRt.mode === 'PIN' || braceRt.physicsMode === 'rigid') return;
        safeSetPointerCapture(e.pointerId);
        setActiveDrag({
          type: 'collar_toggle',
          side,
          offsetX: joint.x - world.x,
          offsetY: joint.y - world.y,
        });
        return;
      }

      // Finger or fingertip guides the whole arm via tip IK
      if (
        jointId === 'fingerTipR' ||
        jointId === 'fingerR' ||
        jointId === 'fingerTipL' ||
        jointId === 'fingerL'
      ) {
        const isRight = jointId === 'fingerTipR' || jointId === 'fingerR';
        const tipId: 'fingerTipR' | 'fingerTipL' = isRight ? 'fingerTipR' : 'fingerTipL';
        const handId: JointId = isRight ? 'handR' : 'handL';
        if (curArmature.joints[handId]?.physicsMode === 'rigid') return;
        const tipJoint = computedJointsRef.current[tipId];
        safeSetPointerCapture(e.pointerId);
        setActiveDrag({
          type: 'ik',
          jointId: tipId,
          startArmature: curArmature,
          offsetX: tipJoint.x - world.x,
          offsetY: tipJoint.y - world.y,
        });
        const source = isRight ? 'handR' : 'handL';
        setActivePlusWorld(world);
        onPlanquetteDragChange?.({ isDragging: true, source });
        onPlanquetteDragMove?.({
          source,
          effectorJointId: tipId,
          effectorWorld: { x: tipJoint.x, y: tipJoint.y },
          pointerWorld: world,
          pointerScreen: screen,
        });
        return;
      }

      if (joint.mode === 'IK' && joint.spec.ikChain) {
        if (curArmature.joints[jointId].physicsMode === 'rigid') return;
        safeSetPointerCapture(e.pointerId);
        setActiveDrag({
          type: 'ik',
          jointId,
          startArmature: curArmature,
          offsetX: joint.x - world.x,
          offsetY: joint.y - world.y,
        });
        if (jointId === 'handR' || jointId === 'handL') {
          setActivePlusWorld(world);
          onPlanquetteDragChange?.({ isDragging: true, source: jointId });
          onPlanquetteDragMove?.({
            source: jointId,
            effectorJointId: jointId,
            effectorWorld: { x: joint.x, y: joint.y },
            pointerWorld: world,
            pointerScreen: screen,
          });
        }
        return;
      }

      let targetId: JointId = jointId;
      if (spec.isTip || spec.locked) {
        let cur: JointId | null = spec.parentId;
        while (cur && JOINT_SPEC_MAP[cur]?.locked) {
          cur = JOINT_SPEC_MAP[cur].parentId;
        }
        if (cur) targetId = cur;
      }
      const targetRt = curArmature.joints[targetId];
      if (!targetRt || targetRt.mode === 'PIN' || targetRt.physicsMode === 'rigid') return;
      startCumulativeFKRotate(e, targetId);
    },
    [
      beginFaceplateLookDrag,
      beginWaistBalanceDrag,
      clientToRigWorld,
      clientToScreenLocal,
      onPlanquetteDragChange,
      onPlanquetteDragMove,
      onSelectJoint,
      safeSetPointerCapture,
      setActiveDrag,
      startCumulativeFKRotate,
    ]
  );

  const handlePointerDownBone = useCallback(
    (e: React.PointerEvent<SVGElement>, jointId: JointId) => {
      if (jointId === 'lip') {
        beginFaceplateLookDrag(e);
        return;
      }
      e.stopPropagation();
      onSelectJoint(jointId);
      if (jointId === 'pelvis') {
        beginWaistBalanceDrag(e);
        return;
      }

      const curArmature = armatureRef.current;
      const world = clientToRigWorld(e.clientX, e.clientY);
      const screen = clientToScreenLocal(e.clientX, e.clientY);

      // Grabbing hand wrist activates whole-arm wrist IK
      if (jointId === 'handR' || jointId === 'handL') {
        if (curArmature.joints[jointId]?.physicsMode === 'rigid') return;
        const joint = computedJointsRef.current[jointId];
        safeSetPointerCapture(e.pointerId);
        setActiveDrag({
          type: 'ik',
          jointId,
          startArmature: curArmature,
          offsetX: joint.x - world.x,
          offsetY: joint.y - world.y,
        });
        setActivePlusWorld(world);
        onPlanquetteDragChange?.({ isDragging: true, source: jointId });
        onPlanquetteDragMove?.({
          source: jointId,
          effectorJointId: jointId,
          effectorWorld: { x: joint.x, y: joint.y },
          pointerWorld: world,
          pointerScreen: screen,
        });
        return;
      }

      // Grabbing finger or fingertip activates whole-arm fingertip IK
      if (
        jointId === 'fingerR' ||
        jointId === 'fingerTipR' ||
        jointId === 'fingerL' ||
        jointId === 'fingerTipL'
      ) {
        const isRight = jointId === 'fingerR' || jointId === 'fingerTipR';
        const tipId: 'fingerTipR' | 'fingerTipL' = isRight ? 'fingerTipR' : 'fingerTipL';
        const handId: JointId = isRight ? 'handR' : 'handL';
        if (curArmature.joints[handId]?.physicsMode === 'rigid') return;
        const tipJoint = computedJointsRef.current[tipId];
        safeSetPointerCapture(e.pointerId);
        setActiveDrag({
          type: 'ik',
          jointId: tipId,
          startArmature: curArmature,
          offsetX: tipJoint.x - world.x,
          offsetY: tipJoint.y - world.y,
        });
        const source = isRight ? 'handR' : 'handL';
        setActivePlusWorld(world);
        onPlanquetteDragChange?.({ isDragging: true, source });
        onPlanquetteDragMove?.({
          source,
          effectorJointId: tipId,
          effectorWorld: { x: tipJoint.x, y: tipJoint.y },
          pointerWorld: world,
          pointerScreen: screen,
        });
        return;
      }

      const drivenJointId: JointId =
        jointId === 'clavOuterR' ? 'braceR' : jointId === 'clavOuterL' ? 'braceL' : jointId;
      const runtime = curArmature.joints[drivenJointId];
      if (!runtime || runtime.mode === 'PIN' || runtime.physicsMode === 'rigid') return;

      const boneSpec = JOINT_SPEC_MAP[jointId];
      if (curArmature.joints[jointId]?.mode === 'IK' && boneSpec?.ikChain) {
        const joint = computedJointsRef.current[jointId];
        safeSetPointerCapture(e.pointerId);
        setActiveDrag({
          type: 'ik',
          jointId,
          startArmature: curArmature,
          offsetX: joint.x - world.x,
          offsetY: joint.y - world.y,
        });
        return;
      }

      startCumulativeFKRotate(e, drivenJointId);
    },
    [
      beginFaceplateLookDrag,
      beginWaistBalanceDrag,
      clientToRigWorld,
      clientToScreenLocal,
      onPlanquetteDragChange,
      onPlanquetteDragMove,
      onSelectJoint,
      safeSetPointerCapture,
      setActiveDrag,
      startCumulativeFKRotate,
    ]
  );

  const handlePointerMove = useCallback(
    ({ clientX, clientY }: Pick<React.PointerEvent<SVGSVGElement>, 'clientX' | 'clientY'>) => {
      const world = clientToRigWorld(clientX, clientY);
      const cursorPatch = {
        hasCursor: true,
        cursor: {
          x: world.x - originOffset.x,
          y: world.y - originOffset.y,
        },
      };

      const activeDrag = dragRef.current;
      if (!activeDrag) {
        enqueueUpdate(cursorPatch);
        return;
      }

      if (activeDrag.type === 'pan') {
        const nextPan = {
          x: activeDrag.startPanX + (clientX - activeDrag.startClientX),
          y: activeDrag.startPanY + (clientY - activeDrag.startClientY),
        };
        panRef.current = nextPan;
        enqueueUpdate({ ...cursorPatch, pan: nextPan });
        return;
      }

      if (activeDrag.type === 'pressure_plate') {
        const solution = solvePressurePlateContact(
          activeDrag.startArmature,
          world.y + activeDrag.offsetY,
          originOffset
        );
        onUpdateRigSystems({ pressurePlateY: solution.pressurePlateY });
        armatureRef.current = solution.armature;
        enqueueUpdate({ ...cursorPatch, armature: solution.armature });
        return;
      }

      const curArmature = armatureRef.current;

      if (activeDrag.type === 'neck_collar_magnet') {
        const nextArmature = setNeckCollarSeparation(curArmature, {
          x: activeDrag.startSeparation.x + world.x - activeDrag.startPointer.x,
          y: activeDrag.startSeparation.y + world.y - activeDrag.startPointer.y,
        });
        armatureRef.current = nextArmature;
        enqueueUpdate({ ...cursorPatch, armature: nextArmature });
        return;
      }

      if (activeDrag.type === 'faceplate_look') {
        const screen = clientToScreenLocal(clientX, clientY);
        const target = {
          x: world.x + activeDrag.offsetX,
          y: world.y + activeDrag.offsetY,
        };
        const nextArmature = solveFaceplateLookIK(
          activeDrag.startArmature,
          target.x,
          target.y,
          originOffset,
          activeDrag.collarHold
        );
        dragRef.current = {
          ...activeDrag,
          collarHold: {
            braceR: nextArmature.joints.braceR.localAngle,
            braceL: nextArmature.joints.braceL.localAngle,
            clavOuterR: nextArmature.joints.clavOuterR.localAngle,
            clavOuterL: nextArmature.joints.clavOuterL.localAngle,
          },
        };
        armatureRef.current = nextArmature;
        setActivePlusWorld(world);
        onFaceplateDragMove?.({ pointer: world, target });
        if (onPlanquetteDragMove) {
          const nextComp = computeArmature(nextArmature, originOffset);
          onPlanquetteDragMove({
            source: 'faceplate',
            effectorJointId: 'lip',
            effectorWorld: { x: nextComp.lip.x, y: nextComp.lip.y },
            headWorld: { x: nextComp.head.x, y: nextComp.head.y },
            lipWorld: { x: nextComp.lip.x, y: nextComp.lip.y },
            pointerWorld: world,
            pointerScreen: screen,
          });
        }
        enqueueUpdate({ ...cursorPatch, armature: nextArmature });
        return;
      }

      if (activeDrag.type === 'waist_balance') {
        const targetPelvisX = world.x + activeDrag.offsetX;
        const targetPelvisY = world.y + activeDrag.offsetY;
        const nextArmature = solveWaistBalanceHula(
          curArmature,
          targetPelvisX,
          targetPelvisY,
          activeDrag.footRPin,
          activeDrag.footLPin,
          originOffset,
          {
            baseRootRotation: activeDrag.startRootRotation,
            baseLumbarAngle: activeDrag.startLumbarAngle,
            baseThoraxAngle: activeDrag.startThoraxAngle,
            baseNeckAngle: activeDrag.startNeckAngle,
            startPelvisX: activeDrag.startPelvis.x,
            startPelvisY: activeDrag.startPelvis.y,
          }
        );
        armatureRef.current = nextArmature;
        enqueueUpdate({ ...cursorPatch, armature: nextArmature });
        return;
      }

      if (activeDrag.type === 'torso_balance') {
        const targetTorsoX = world.x + activeDrag.offsetX;
        const targetTorsoY = world.y + activeDrag.offsetY;
        const nextArmature = solveTorsoBalance(
          curArmature,
          targetTorsoX,
          targetTorsoY,
          activeDrag.footRPin,
          activeDrag.footLPin,
          originOffset,
          {
            baseRootRotation: activeDrag.startRootRotation,
            baseLumbarAngle: activeDrag.startLumbarAngle,
            baseThoraxAngle: activeDrag.startThoraxAngle,
            baseNeckAngle: activeDrag.startNeckAngle,
            startPelvisX: activeDrag.startPelvis.x,
            startPelvisY: activeDrag.startPelvis.y,
          }
        );
        armatureRef.current = nextArmature;
        enqueueUpdate({ ...cursorPatch, armature: nextArmature });
        return;
      }

      if (activeDrag.type === 'hip_toggle') {
        const nextArmature = solveHipToggleFromTarget(
          curArmature,
          world.x + activeDrag.offsetX,
          world.y + activeDrag.offsetY,
          originOffset
        );
        armatureRef.current = nextArmature;
        enqueueUpdate({ ...cursorPatch, armature: nextArmature });
        return;
      }

      if (activeDrag.type === 'collar_toggle') {
        const nextArmature = solveCollarToggleFromTarget(
          curArmature,
          activeDrag.side,
          world.x + activeDrag.offsetX,
          world.y + activeDrag.offsetY,
          originOffset
        );
        armatureRef.current = nextArmature;
        enqueueUpdate({ ...cursorPatch, armature: nextArmature });
        return;
      }

      if (activeDrag.type === 'arm_swivel') {
        const moveDist = Math.hypot(
          clientX - activeDrag.startClientX,
          clientY - activeDrag.startClientY
        );
        if (!activeDrag.hasDragged && moveDist < 5) {
          enqueueUpdate(cursorPatch);
          return;
        }
        if (!activeDrag.hasDragged) {
          dragRef.current = { ...activeDrag, hasDragged: true };
        }
        const deltaDeg = (clientY - activeDrag.startClientY) * 1.4;
        const rawNext = normalizeAngleDeg(activeDrag.startSwivelDeg + deltaDeg);
        const snappedNext =
          Math.abs(rawNext) < 14
            ? 0
            : Math.abs(Math.abs(rawNext) - 180) < 14
            ? 180
            : rawNext;
        const nextArmature = setArmReverseSwivel(curArmature, activeDrag.side, snappedNext);
        armatureRef.current = nextArmature;
        enqueueUpdate({ ...cursorPatch, armature: nextArmature });
        return;
      }

      if (activeDrag.type === 'ik') {
        const targetX = world.x + activeDrag.offsetX;
        const targetY = world.y + activeDrag.offsetY;
        const isArmEffector =
          activeDrag.jointId === 'handR' ||
          activeDrag.jointId === 'handL' ||
          activeDrag.jointId === 'fingerTipR' ||
          activeDrag.jointId === 'fingerTipL';
        const nextArmature =
          planquetteReachActive && isArmEffector && activeDrag.startArmature
            ? solvePlanquetteArmReach(
                activeDrag.startArmature,
                activeDrag.jointId as 'handR' | 'handL' | 'fingerTipR' | 'fingerTipL',
                targetX,
                targetY,
                originOffset,
                {
                  solver: rigSystems.solver,
                  softDamp: true,
                  constrainLimits: rigSystems.softLimits,
                }
              )
            : solveJointIK(curArmature, activeDrag.jointId, targetX, targetY, originOffset, {
                solver: rigSystems.solver,
                softDamp: true,
                constrainLimits: rigSystems.softLimits,
              });
        armatureRef.current = nextArmature;
        if (isArmEffector) {
          const screen = clientToScreenLocal(clientX, clientY);
          const source =
            activeDrag.jointId === 'handR' || activeDrag.jointId === 'fingerTipR'
              ? 'handR'
              : 'handL';
          setActivePlusWorld(world);
          if (onPlanquetteDragMove) {
            const nextComp = computeArmature(nextArmature, originOffset);
            const eff = nextComp[activeDrag.jointId];
            onPlanquetteDragMove({
              source,
              effectorJointId: activeDrag.jointId as
                | 'handR'
                | 'handL'
                | 'fingerTipR'
                | 'fingerTipL',
              effectorWorld: { x: eff.x, y: eff.y },
              pointerWorld: world,
              pointerScreen: screen,
            });
          }
        }
        enqueueUpdate({ ...cursorPatch, armature: nextArmature });
        return;
      }

      if (activeDrag.type === 'fk_rotate') {
        const { targetJointId, pivotX, pivotY, lastCursorDeg, rawAngle } = activeDrag;
        const distFromPivot = Math.hypot(world.x - pivotX, world.y - pivotY);
        if (distFromPivot < 4) {
          enqueueUpdate(cursorPatch);
          return;
        }
        const currDeg = (Math.atan2(world.y - pivotY, world.x - pivotX) * 180) / Math.PI;
        if (lastCursorDeg === null) {
          dragRef.current = { ...activeDrag, lastCursorDeg: currDeg };
          enqueueUpdate(cursorPatch);
          return;
        }
        const deltaCursor = shortestAngleDelta(lastCursorDeg, currDeg);
        if (Math.abs(deltaCursor) < 1e-4) {
          enqueueUpdate(cursorPatch);
          return;
        }

        const runtime = curArmature.joints[targetJointId];
        const mult = getPhysicsMultiplier(runtime);
        if (mult === 0) {
          enqueueUpdate(cursorPatch);
          return;
        }

        const nextUnclamped = rawAngle + deltaCursor * mult;
        if (targetJointId === 'swivelR' || targetJointId === 'swivelL') {
          const collarSide: 'R' | 'L' = targetJointId === 'swivelR' ? 'R' : 'L';
          const { nextAngle, nextRawAngle } = applyCollarSwivelDragInput(
            curArmature,
            collarSide,
            nextUnclamped
          );
          dragRef.current = {
            ...activeDrag,
            lastCursorDeg: currDeg,
            rawAngle: nextRawAngle,
          };
          const nextArmature: ArmatureState = {
            ...curArmature,
            joints: {
              ...curArmature.joints,
              [targetJointId]: {
                ...runtime,
                localAngle: nextAngle,
              },
            },
          };
          armatureRef.current = nextArmature;
          enqueueUpdate({ ...cursorPatch, armature: nextArmature });
          return;
        }

        const isClosedTriangleSwivel =
          targetJointId === 'hipR' ||
          targetJointId === 'hipL' ||
          targetJointId === 'braceR' ||
          targetJointId === 'braceL';
        const swivelLimits = JOINT_SPEC_MAP[targetJointId]?.limits;
        const nextRawAngle =
          isClosedTriangleSwivel && swivelLimits
            ? Math.max(swivelLimits[0], Math.min(swivelLimits[1], nextUnclamped))
            : curArmature.continuousSpin
            ? nextUnclamped
            : Math.max(-180, Math.min(180, nextUnclamped));
        dragRef.current = {
          ...activeDrag,
          lastCursorDeg: currDeg,
          rawAngle: nextRawAngle,
        };

        const constrainedAngle = applyJointAngleConstraint(
          nextRawAngle,
          curArmature.continuousSpin
        );
        const nextLocalAngle =
          isClosedTriangleSwivel && swivelLimits
            ? Math.max(swivelLimits[0], Math.min(swivelLimits[1], constrainedAngle))
            : constrainedAngle;
        const nextArmature: ArmatureState =
          targetJointId === 'pelvis'
            ? {
                ...curArmature,
                rootRotation: nextLocalAngle,
              }
            : {
                ...curArmature,
                joints: {
                  ...curArmature.joints,
                  [targetJointId]: {
                    ...runtime,
                    localAngle: nextLocalAngle,
                  },
                },
              };

        armatureRef.current = nextArmature;
        enqueueUpdate({ ...cursorPatch, armature: nextArmature });
      }
    },
    [
      clientToRigWorld,
      clientToScreenLocal,
      enqueueUpdate,
      onFaceplateDragMove,
      onPlanquetteDragMove,
      onUpdateRigSystems,
      originOffset,
      planquetteReachActive,
      rigSystems.softLimits,
      rigSystems.solver,
    ]
  );

  const queuePointerMove = useCallback(
    (clientX: number, clientY: number) => {
      pendingPointerMoveRef.current = { clientX, clientY };
      if (pointerMoveFrameRef.current !== null) return;
      pointerMoveFrameRef.current = window.requestAnimationFrame(() => {
        pointerMoveFrameRef.current = null;
        const pending = pendingPointerMoveRef.current;
        pendingPointerMoveRef.current = null;
        if (pending) handlePointerMove(pending);
      });
    },
    [handlePointerMove]
  );

  const flushPendingPointerMove = useCallback(() => {
    if (pointerMoveFrameRef.current !== null) {
      window.cancelAnimationFrame(pointerMoveFrameRef.current);
      pointerMoveFrameRef.current = null;
    }
    const pending = pendingPointerMoveRef.current;
    pendingPointerMoveRef.current = null;
    if (pending) handlePointerMove(pending);
  }, [handlePointerMove]);

  const handlePointerUp = useCallback(
    (e: React.PointerEvent<SVGSVGElement>) => {
      flushPendingPointerMove();
      if (svgRef.current?.hasPointerCapture(e.pointerId)) {
        svgRef.current.releasePointerCapture(e.pointerId);
      }
      const activeDrag = dragRef.current;
      flushUpdates();
      const isSwivelClickToggle = Boolean(activeDrag?.type === 'arm_swivel' && !activeDrag.hasDragged);
      if (isSwivelClickToggle && activeDrag?.type === 'arm_swivel') {
        onCommitPoseHistory();
        const toggled = toggleArmReverseSwivel(armatureRef.current, activeDrag.side);
        if (onAnimateArmature) {
          onAnimateArmature(toggled, DEFAULT_TOGGLE_TRANSITION_MS);
        } else {
          armatureRef.current = toggled;
          onUpdateArmature(toggled);
        }
      } else if (
        activeDrag &&
        (activeDrag.type === 'fk_rotate' ||
          activeDrag.type === 'ik' ||
          activeDrag.type === 'faceplate_look' ||
          activeDrag.type === 'waist_balance' ||
          activeDrag.type === 'torso_balance' ||
          activeDrag.type === 'pressure_plate' ||
          activeDrag.type === 'neck_collar_magnet' ||
          activeDrag.type === 'hip_toggle' ||
          activeDrag.type === 'collar_toggle' ||
          activeDrag.type === 'arm_swivel')
      ) {
        onCommitPoseHistory();
      }
      setActiveDrag(null);
      setActivePlusWorld(null);
      if (activeDrag?.type === 'faceplate_look') {
        onFaceplateDragChange?.(false);
        onPlanquetteDragChange?.({ isDragging: false, source: null });
      } else if (
        activeDrag?.type === 'ik' &&
        (activeDrag.jointId === 'handR' ||
          activeDrag.jointId === 'handL' ||
          activeDrag.jointId === 'fingerTipR' ||
          activeDrag.jointId === 'fingerTipL')
      ) {
        onPlanquetteDragChange?.({ isDragging: false, source: null });
      }
      if (activeDrag && activeDrag.type !== 'pan') {
        onDragEnd?.();
      }
    },
    [
      flushPendingPointerMove,
      flushUpdates,
      onAnimateArmature,
      onCommitPoseHistory,
      onDragEnd,
      onFaceplateDragChange,
      onPlanquetteDragChange,
      onUpdateArmature,
      setActiveDrag,
    ]
  );

  const inkRgb = darkMode ? '240, 234, 248' : '22, 0, 30';
  const paperBg = darkMode ? '#000000' : isCover ? '#CECAC2' : '#EFECE6';

  const eightHeadSpan = canons.eightHeadTotalPx;
  const eightHeadTopY = originOffset.y - eightHeadSpan / 2;

  const wsHalf = canons.workingSquareHalfPx;
  const wsX = originOffset.x - wsHalf;
  const wsY = originOffset.y - wsHalf;
  const navelCenter = computedJoints.lumbar ?? originOffset;
  const pressurePlateY = Math.max(
    computedJoints.footR.y,
    computedJoints.toeR.endY,
    computedJoints.footL.y,
    computedJoints.toeL.endY
  );
  const staticGroundY = rigSystems.groundY;
  const activePressurePlateY = rigSystems.pressurePlateY ?? pressurePlateY;
  const isPointGrounded = (y: number) =>
    Math.abs(y - activePressurePlateY) <= GROUND_CONTACT_TOLERANCE_PX;
  const groundedFootCount = [
    [computedJoints.footR.y, computedJoints.toeR.endY],
    [computedJoints.footL.y, computedJoints.toeL.endY],
  ].filter((points) => points.some(isPointGrounded)).length;
  const groundedBodyIds: JointId[] = [
    'pelvis',
    'hipR',
    'hipL',
    'lumbar',
    'ribR',
    'ribL',
    'thorax',
    'head',
    'humerusR',
    'forearmR',
    'handR',
    'fingerTipR',
    'humerusL',
    'forearmL',
    'handL',
    'fingerTipL',
  ];
  const groundedBodyContact = groundedBodyIds.some((id) => {
    const joint = computedJoints[id];
    return Math.max(joint.y, joint.endY) >= activePressurePlateY - GROUND_CONTACT_TOLERANCE_PX;
  });
  const pressureLevel = Math.max(
    0,
    Math.min(
      1,
      0.5 +
        (staticGroundY - activePressurePlateY) / 160 +
        (groundedBodyContact ? 0.12 : 0)
    )
  );
  const pressurePlateSlopePx = (1 - pressureLevel) * 5;
  const pressurePlateControlY = activePressurePlateY + pressureLevel * 1.25;
  const pressurePlatePath =
    `M ${endlessBounds.minX} ${activePressurePlateY + pressurePlateSlopePx} ` +
    `Q ${(endlessBounds.minX + endlessBounds.maxX) / 2} ${pressurePlateControlY} ` +
    `${endlessBounds.maxX} ${activePressurePlateY - pressurePlateSlopePx}`;
  const isMotionActive = isPoseTransitioning || dragState !== null;
  const isCursorHiddenForIK =
    dragState?.type === 'faceplate_look' ||
    (dragState?.type === 'ik' &&
      (dragState.jointId === 'handR' ||
        dragState.jointId === 'handL' ||
        dragState.jointId === 'fingerTipR' ||
        dragState.jointId === 'fingerTipL'));

  return (
    <div
      ref={containerRef}
      data-stage-viewport="true"
      data-page-id={pageId}
      data-dark-mode={darkMode ? 'true' : 'false'}
      data-pose-interpolating={isPoseTransitioning ? 'true' : 'false'}
      data-vw={viewportSize.width}
      data-vh={viewportSize.height}
      className={`relative flex-1 w-full h-full overflow-hidden select-none transition-colors duration-300 ${
        isCursorHiddenForIK ? 'cursor-hidden-active' : ''
      }`}
      style={{
        backgroundColor: paperBg,
        cursor: isCursorHiddenForIK
          ? 'none'
          : dragState?.type === 'pan'
          ? 'grabbing'
          : 'crosshair',
      }}
    >
      {/* Procedural Paper / Cover Substrate */}
      <div
        aria-hidden="true"
        data-paper-surface="true"
        data-paper-role="backing-leaf"
        className="pointer-events-none absolute inset-0"
      >
        <img
          src={REPOSITORY_ASSET_PATHS.page21PaperPng}
          alt="Archival backing leaf"
          referrerPolicy="no-referrer"
          className={`absolute inset-0 w-full h-full object-cover transition-opacity duration-300 ${
            darkMode ? 'invert opacity-[0.07]' : 'opacity-85'
          }`}
        />
        <div
          className="absolute inset-0"
          style={{
            backgroundImage: `url(${REPOSITORY_ASSET_PATHS.analogPaperPng})`,
            backgroundSize: 'cover',
            backgroundPosition: 'center',
            backgroundRepeat: 'no-repeat',
            opacity: darkMode ? 0.05 : 0.14,
            filter: darkMode ? 'invert(1)' : undefined,
            mixBlendMode: darkMode ? 'screen' : 'multiply',
          }}
        />
      </div>

      {isCover && (
        <div
          aria-hidden="true"
          data-cover-surface="true"
          className="pointer-events-none absolute inset-0"
          style={{ backgroundColor: darkMode ? '#000000' : '#CECAC2' }}
        >
          <img
            src={REPOSITORY_ASSET_PATHS.page22CoverPng}
            alt="Symmetrical mottled back cover texture traced from 1901 manual page 22"
            referrerPolicy="no-referrer"
            className={`absolute inset-0 w-full h-full object-cover transition-all duration-300 ${
              darkMode
                ? 'invert opacity-[0.09] contrast-125'
                : 'opacity-86 contrast-105'
            }`}
          />
          <div
            className="absolute inset-0"
            style={{
              boxShadow: darkMode
                ? 'inset 0 0 0 1px rgba(255, 255, 255, 0.16), inset 0 0 42px rgba(0, 0, 0, 0.85)'
                : 'inset 0 0 0 2px rgba(22, 0, 30, 0.24), inset 0 0 42px rgba(20, 16, 24, 0.22)',
            }}
          />
        </div>
      )}

      {/* Main Proportional SVG World Plane */}
      <svg
        ref={svgRef}
        data-world-canvas="true"
        className="relative w-full h-full block touch-none"
        onContextMenu={(e) => e.preventDefault()}
        onPointerDownCapture={(e) => {
          if (e.button === 0 && e.metaKey && e.shiftKey) {
            e.preventDefault();
            e.stopPropagation();
            beginPanDrag(e);
          }
        }}
        onPointerDown={handlePointerDownStage}
        onPointerMove={(e) => queuePointerMove(e.clientX, e.clientY)}
        onPointerUp={handlePointerUp}
        onPointerCancel={handlePointerUp}
        onPointerLeave={() => {
          flushPendingPointerMove();
          enqueueUpdate({ hasCursor: true, cursor: null });
          flushUpdates();
        }}
      >
        <defs>
          <pattern
            id="grid-subpyxl"
            width={SUBPYXL_PX}
            height={SUBPYXL_PX}
            patternUnits="userSpaceOnUse"
            x={originOffset.x}
            y={originOffset.y}
          >
            <path
              d={`M ${SUBPYXL_PX} 0 L 0 0 0 ${SUBPYXL_PX}`}
              fill="none"
              stroke={`rgba(${inkRgb}, ${LIGHT_INK_OPACITY.sub})`}
              strokeWidth={0.5}
              vectorEffect="non-scaling-stroke"
            />
          </pattern>
          <pattern
            id="grid-pyxl"
            width={PYXL_PX}
            height={PYXL_PX}
            patternUnits="userSpaceOnUse"
            x={originOffset.x}
            y={originOffset.y}
          >
            <path
              d={`M ${PYXL_PX} 0 L 0 0 0 ${PYXL_PX}`}
              fill="none"
              stroke={`rgba(${inkRgb}, ${LIGHT_INK_OPACITY.pyxl})`}
              strokeWidth={0.9}
              vectorEffect="non-scaling-stroke"
            />
          </pattern>
          <pattern
            id="grid-megapyxl"
            width={MEGAPYXL_PX}
            height={MEGAPYXL_PX}
            patternUnits="userSpaceOnUse"
            x={originOffset.x}
            y={originOffset.y}
          >
            <path
              d={`M ${MEGAPYXL_PX} 0 L 0 0 0 ${MEGAPYXL_PX}`}
              fill="none"
              stroke={`rgba(${inkRgb}, ${LIGHT_INK_OPACITY.mega})`}
              strokeWidth={1.35}
              vectorEffect="non-scaling-stroke"
            />
          </pattern>

          <radialGradient
            id="vitruvian-outer-dim-grad"
            gradientUnits="userSpaceOnUse"
            cx={originOffset.x}
            cy={originOffset.y}
            r={canons.vitruvianCircleRadius * 1.48}
          >
            <stop offset="0%" stopColor={darkMode ? '#000000' : '#140E1C'} stopOpacity={0} />
            <stop offset="62%" stopColor={darkMode ? '#000000' : '#140E1C'} stopOpacity={0} />
            <stop offset="82%" stopColor={darkMode ? '#000000' : '#140E1C'} stopOpacity={darkMode ? 0.34 : 0.16} />
            <stop offset="100%" stopColor={darkMode ? '#000000' : '#140E1C'} stopOpacity={darkMode ? 0.56 : 0.27} />
          </radialGradient>

          <radialGradient id="navel-backlight-outer" cx="50%" cy="50%" r="50%">
            <stop
              offset="0%"
              stopColor={darkMode ? '#FAF5FF' : '#FFFDF6'}
              stopOpacity={darkMode ? 0.25 : 0.94}
            />
            <stop
              offset="18%"
              stopColor={darkMode ? '#F3E8FF' : '#FFF9E8'}
              stopOpacity={darkMode ? 0.18 : 0.8}
            />
            <stop
              offset="38%"
              stopColor={darkMode ? '#E9D5FF' : '#FFF4D6'}
              stopOpacity={darkMode ? 0.11 : 0.54}
            />
            <stop
              offset="60%"
              stopColor={darkMode ? '#C084FC' : '#FDECC2'}
              stopOpacity={darkMode ? 0.055 : 0.28}
            />
            <stop
              offset="80%"
              stopColor={darkMode ? '#A855F7' : '#F8E3B2'}
              stopOpacity={darkMode ? 0.02 : 0.1}
            />
            <stop
              offset="93%"
              stopColor={darkMode ? '#9333EA' : '#F6E0AD'}
              stopOpacity={darkMode ? 0.005 : 0.025}
            />
            <stop
              offset="100%"
              stopColor={darkMode ? '#9333EA' : '#F6E0AD'}
              stopOpacity={0}
            />
          </radialGradient>

          <radialGradient id="navel-backlight-core" cx="50%" cy="50%" r="50%">
            <stop
              offset="0%"
              stopColor="#FFFFFF"
              stopOpacity={darkMode ? 0.26 : 0.9}
            />
            <stop
              offset="28%"
              stopColor={darkMode ? '#F3E8FF' : '#FFFAED'}
              stopOpacity={darkMode ? 0.14 : 0.62}
            />
            <stop
              offset="62%"
              stopColor={darkMode ? '#D8B4FE' : '#FDF1CE'}
              stopOpacity={darkMode ? 0.05 : 0.24}
            />
            <stop
              offset="88%"
              stopColor={darkMode ? '#C084FC' : '#FDECC0'}
              stopOpacity={darkMode ? 0.012 : 0.05}
            />
            <stop
              offset="100%"
              stopColor={darkMode ? '#C084FC' : '#FDECC0'}
              stopOpacity={0}
            />
          </radialGradient>
        </defs>

        <g transform={`translate(${pan.x}, ${pan.y}) scale(${zoom})`}>
          {/* Endless proportional light ink grid layers */}
          {grid.showSubpyxl && (
            <rect
              x={endlessBounds.minX - 2400}
              y={endlessBounds.minY - 2400}
              width={endlessBounds.width + 4800}
              height={endlessBounds.height + 4800}
              fill="url(#grid-subpyxl)"
              opacity={subpyxlZoomOpacity}
              pointerEvents="none"
            />
          )}
          {grid.showPyxl && (
            <rect
              x={endlessBounds.minX - 2400}
              y={endlessBounds.minY - 2400}
              width={endlessBounds.width + 4800}
              height={endlessBounds.height + 4800}
              fill="url(#grid-pyxl)"
              pointerEvents="none"
            />
          )}
          {grid.showMegapyxl && (
            <rect
              x={endlessBounds.minX - 2400}
              y={endlessBounds.minY - 2400}
              width={endlessBounds.width + 4800}
              height={endlessBounds.height + 4800}
              fill="url(#grid-megapyxl)"
              pointerEvents="none"
            />
          )}

          {/* Symmetrical X / Y Datum Axes & Optical Center Reticle */}
          <g pointerEvents="none">
            <line
              x1={originOffset.x}
              y1={endlessBounds.minY - 1200}
              x2={originOffset.x}
              y2={endlessBounds.maxY + 1200}
              stroke={`rgba(${inkRgb}, ${LIGHT_INK_OPACITY.axis})`}
              strokeWidth={1.1}
              vectorEffect="non-scaling-stroke"
            />
            <line
              x1={endlessBounds.minX - 1200}
              y1={originOffset.y}
              x2={endlessBounds.maxX + 1200}
              y2={originOffset.y}
              stroke={`rgba(${inkRgb}, ${LIGHT_INK_OPACITY.axis})`}
              strokeWidth={1.1}
              vectorEffect="non-scaling-stroke"
            />
            <g transform={`translate(${originOffset.x}, ${originOffset.y}) scale(${1 / zoom})`}>
              <circle
                r={7}
                fill="none"
                stroke={`rgba(${inkRgb}, ${LIGHT_INK_OPACITY.axis + 0.18})`}
                strokeWidth={1.1}
              />
              <circle r={1.8} fill={`rgba(${inkRgb}, ${LIGHT_INK_OPACITY.axis + 0.3})`} />
            </g>
          </g>

          {/* Backlit Cover Paper: Navel-Centered Halo + Outer Dim */}
          {isCover && (
            <>
              <g data-navel-halo="true" pointerEvents="none">
                <circle
                  cx={navelCenter.x}
                  cy={navelCenter.y}
                  r={canons.vitruvianCircleRadius * 1.72}
                  fill="url(#navel-backlight-outer)"
                />
                <circle
                  cx={navelCenter.x}
                  cy={navelCenter.y}
                  r={canons.headUnitPx * 3.4}
                  fill="url(#navel-backlight-core)"
                />
              </g>
              <rect
                data-vitruvian-dim="true"
                x={endlessBounds.minX - 2400}
                y={endlessBounds.minY - 2400}
                width={endlessBounds.width + 4800}
                height={endlessBounds.height + 4800}
                fill="url(#vitruvian-outer-dim-grad)"
                pointerEvents="none"
              />
            </>
          )}

          {/* Proportional Working Square */}
          {grid.showWorkingSquare && (
            <g pointerEvents="none">
              <rect
                x={wsX}
                y={wsY}
                width={canons.workingSquareSidePx}
                height={canons.workingSquareSidePx}
                fill="none"
                stroke={`rgba(${inkRgb}, ${LIGHT_INK_OPACITY.guide})`}
                strokeWidth={1.1}
                vectorEffect="non-scaling-stroke"
              />
              <line
                x1={wsX}
                y1={wsY}
                x2={wsX + canons.workingSquareSidePx}
                y2={wsY + canons.workingSquareSidePx}
                stroke={`rgba(${inkRgb}, ${LIGHT_INK_OPACITY.guide * 0.42})`}
                strokeWidth={0.75}
                strokeDasharray="4 6"
                vectorEffect="non-scaling-stroke"
              />
              <line
                x1={wsX + canons.workingSquareSidePx}
                y1={wsY}
                x2={wsX}
                y2={wsY + canons.workingSquareSidePx}
                stroke={`rgba(${inkRgb}, ${LIGHT_INK_OPACITY.guide * 0.42})`}
                strokeWidth={0.75}
                strokeDasharray="4 6"
                vectorEffect="non-scaling-stroke"
              />
            </g>
          )}

          {/* Vitruvian Classical Circle & Square Overlay */}
          {grid.showVitruvianGuide && (
            <g pointerEvents="none">
              <rect
                x={originOffset.x - canons.vitruvianSquareWidth / 2}
                y={originOffset.y - canons.vitruvianSquareHeight / 2}
                width={canons.vitruvianSquareWidth}
                height={canons.vitruvianSquareHeight}
                fill="none"
                stroke={`rgba(${inkRgb}, ${LIGHT_INK_OPACITY.guide})`}
                strokeWidth={1.1}
                vectorEffect="non-scaling-stroke"
              />
              <circle
                cx={originOffset.x}
                cy={originOffset.y}
                r={canons.vitruvianCircleRadius}
                fill="none"
                stroke={`rgba(${inkRgb}, ${LIGHT_INK_OPACITY.guide})`}
                strokeWidth={1.1}
                strokeDasharray="5 5"
                vectorEffect="non-scaling-stroke"
              />
            </g>
          )}

          {/* 8-Head Horizontal Proportional Ruled Bands */}
          {grid.showEightHeadGuide && (
            <g pointerEvents="none">
              {EIGHT_HEAD_INDICES.map((idx) => {
                const y = eightHeadTopY + idx * canons.headUnitPx;
                const halfBand = canons.vitruvianSquareWidth / 2;
                return (
                  <g key={`head-div-${idx}`}>
                    <line
                      x1={originOffset.x - halfBand}
                      y1={y}
                      x2={originOffset.x + halfBand}
                      y2={y}
                      stroke={`rgba(${inkRgb}, ${idx === 0 || idx === 8 ? 0.28 : 0.14})`}
                      strokeWidth={idx === 0 || idx === 8 ? 1.0 : 0.75}
                      strokeDasharray={idx === 4 ? undefined : '4 6'}
                      vectorEffect="non-scaling-stroke"
                    />
                    <text
                      x={originOffset.x - halfBand - 22}
                      y={y + 6}
                      textAnchor="end"
                      fill={`rgba(${inkRgb}, 0.48)`}
                      fontSize={16}
                      fontFamily="Bitstream Vera Sans Mono, monospace"
                    >
                      {idx}H
                    </text>
                  </g>
                );
              })}
            </g>
          )}

          {/* Archival Proportional Spec Caption */}
          {isCover && (
            <g pointerEvents="none">
              <text
                x={originOffset.x}
                y={originOffset.y - canons.vitruvianCircleRadius - 24}
                textAnchor="middle"
                fill={`rgba(${inkRgb}, 0.62)`}
                fontSize={20}
                fontFamily="UMTypewriter, monospace"
                letterSpacing="4"
              >
                BITRUVIUS · VITRUVIAN PROPORTIONAL RIG
              </text>
              <text
                x={originOffset.x}
                y={originOffset.y + canons.vitruvianCircleRadius + 30}
                textAnchor="middle"
                fill={`rgba(${inkRgb}, 0.56)`}
                fontSize={17}
                fontFamily="Bitstream Vera Sans Mono, monospace"
              >
                TORSO 1.25× HIPS · 1.00 PYX = 100 PX · R = 6.35 PYX
              </text>
            </g>
          )}

          {/* Static Horizon & Interactive Pressure Plate Ground Line */}
          <line
            data-ground-reference="true"
            x1={endlessBounds.minX}
            y1={staticGroundY}
            x2={endlessBounds.maxX}
            y2={staticGroundY}
            stroke="#14532D"
            strokeOpacity={darkMode ? 0.42 : 0.3}
            strokeWidth={1.1}
            strokeDasharray="9 6"
            vectorEffect="non-scaling-stroke"
            pointerEvents="none"
          />
          <path
            data-ground-line="true"
            data-pressure-plate="true"
            data-pressure-level={pressureLevel.toFixed(2)}
            data-grounded-foot-count={groundedFootCount}
            data-grounded-body-contact={groundedBodyContact ? 'true' : 'false'}
            d={pressurePlatePath}
            fill="none"
            stroke={
              groundedFootCount === 0
                ? REPO_STRAIN_CRIMSON
                : isGroundLineHovered
                ? '#6EE7B7'
                : '#14532D'
            }
            strokeOpacity={
              isGroundLineHovered ? 0.96 : groundedFootCount === 0 ? 0.9 : darkMode ? 0.8 : 0.72
            }
            strokeWidth={isGroundLineHovered ? 2.2 : 1.6}
            vectorEffect="non-scaling-stroke"
            pointerEvents="stroke"
            style={{ cursor: 'ns-resize' }}
            onPointerEnter={() => setIsGroundLineHovered(true)}
            onPointerLeave={() => setIsGroundLineHovered(false)}
            onPointerDown={beginPressurePlateDrag}
          >
            <title>Drag vertically to adjust ground pressure plate</title>
          </path>

          {/* 43-Joint Vitruvian Ink Mannequin */}
          <DigitalMannequin
            armature={armature}
            computedJoints={computedJoints}
            selectedJointId={selectedJointId}
            zoom={zoom}
            darkMode={darkMode}
            isPanning={dragState?.type === 'pan'}
            isMotionActive={isMotionActive}
            onPointerDownJoint={handlePointerDownJoint}
            onPointerDownBone={handlePointerDownBone}
            onPointerDownHipToggle={handlePointerDownHipToggle}
            onPointerDownArmSwivel={handlePointerDownArmSwivel}
            onPointerDownNeckCollarMagnet={beginNeckCollarMagnetDrag}
            onResetNeckCollarMagnet={resetNeckCollarMagnet}
            onPointerDownTorso={beginTorsoBalanceDrag}
          />
          {worldOverlay}
          {isCursorHiddenForIK && activePlusWorld && (
            <g
              data-ik-green-plus="true"
              transform={`translate(${activePlusWorld.x}, ${activePlusWorld.y}) scale(${1 / zoom})`}
              pointerEvents="none"
            >
              <path
                d="M -7 0 H 7 M 0 -7 V 7"
                fill="none"
                stroke={darkMode ? '#4ADE80' : '#16A34A'}
                strokeWidth={2}
                strokeLinecap="square"
              />
            </g>
          )}
        </g>
      </svg>

      {/* Symmetrical Scan-Edge Stage Border */}
      <ScanBorderLayer
        vw={viewportSize.width}
        vh={viewportSize.height}
        substrate={meta.substrate}
        darkMode={darkMode}
        turnAnim={null}
      />
    </div>
  );
});
