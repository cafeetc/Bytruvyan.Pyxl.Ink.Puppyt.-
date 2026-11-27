import React, { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { JOINT_SPEC_MAP, REPO_DARKEST_PURPLE, REPO_STRAIN_CRIMSON } from '../data/digitalRig';
import { GridConfig, PageId, TurnDirection } from '../types/canvas';
import {
  ArmatureState,
  ComputedJoint,
  GroundContactPin,
  JointId,
  Point2D,
  RigSystemsConfig,
} from '../types/rig';
import {
  CompositeFilter,
  SceneMedia,
  StageTransform,
  WorldPin,
} from '../types/scene';
import {
  applyCollarSwivelDragInput,
  applyJointAngleConstraint,
  DEFAULT_TOGGLE_TRANSITION_MS,
  FaceplateLookCollarHold,
  getPhysicsMultiplier,
  HIP_BRIDGE_LINK_STAGE_PX,
  normalizeAngleDeg,
  setArmReverseSwivel,
  setJointMode,
  shortestAngleDelta,
  solveCollarToggleFromTarget,
  solveHipToggleFromTarget,
  solveJointIK,
  solveFaceplateLookIK,
  solvePressurePlateContact,
  solveToggleHinge2Link,
  solveTorsoBalance,
  solveWaistBalanceHula,
  setNeckCollarSeparation,
  toggleArmReverseSwivel,
} from '../utils/kinematics';
import {
  DEFAULT_COMPOSITE_FILTER,
  DEFAULT_SCENE_DURATION_SEC,
  DEFAULT_STAGE_TRANSFORM,
  buildStageSvgTransform,
  computeMediaPlateBox,
  createDefaultSceneMedia,
  mediaToWorld,
  resolveVideoTimeForTimeline,
  worldToMedia,
} from '../utils/scene';
import {
  MEGAPYXL_PX,
  PAPER_HEIGHT,
  PAPER_WIDTH,
  PYXL_PX,
  REPOSITORY_ASSET_PATHS,
  SUBPYXL_PX,
  computeProportionalCanons,
  computeSymmetricalBorderLayout,
  computeSymmetricalFitZoom,
  computeSymmetricalRulerTicks,
  getPageMetadata,
} from '../utils/proportions';
import {
  ScreenFpsOption,
  remainingScreenFrameWaitMs,
  shouldEmitScreenFrame,
} from '../utils/frameThrottle';
import { saveTransparentModelImage } from '../utils/poseExport';
import { DigitalMannequin } from './DigitalMannequin';
import { ScanBorderLayer } from './ScanBorderLayer';

interface BookStageCanvasProps {
  pageId: PageId;
  grid: GridConfig;
  darkMode: boolean;
  screenFps: ScreenFpsOption;
  zoom: number;
  pan: { x: number; y: number };
  armature: ArmatureState;
  computedJoints: Record<JointId, ComputedJoint>;
  selectedJointId: JointId;
  onZoomChange: (nextZoom: number) => void;
  onPanChange: (nextPan: { x: number; y: number }) => void;
  onViewportResize: (size: { width: number; height: number }) => void;
  onCursorMove: (coords: { x: number; y: number } | null) => void;
  onTurnPage: (target: PageId, direction: TurnDirection) => void;
  onFitRequest: () => void;
  onCopyPoseData: () => Promise<unknown> | void;
  onSavePoseData: () => void;
  onSelectJoint: (id: JointId) => void;
  onUpdateArmature: (next: ArmatureState) => void;
  onAnimateArmature?: (next: ArmatureState, durationMs?: number) => void;
  isPoseTransitioning?: boolean;
  isPlaying?: boolean;
  onCommitPoseHistory: () => void;
  rigSystems?: RigSystemsConfig;
  onUpdateRigSystems?: (patch: Partial<RigSystemsConfig>) => void;
  showPerspective?: boolean;
  vanishingPoint?: { x: number; y: number };
  onVanishingPointChange?: (vp: { x: number; y: number }) => void;
  sceneMedia?: SceneMedia;
  stageTransform?: StageTransform;
  compositeFilter?: CompositeFilter;
  worldPins?: WorldPin[];
  timelineT?: number;
  sceneDurationSec?: number;
  showCalibrationOrigin?: boolean;
  onUpdateStageTransform?: (patch: Partial<StageTransform>) => void;
  onUpdateWorldPin?: (pinId: string, mediaSpaceXY: { x: number; y: number }) => void;
}

const INK_STRENGTH_OPACITY = {
  whisper: { sub: 0.04, pyxl: 0.095, mega: 0.19, axis: 0.26, guide: 0.24 },
  light: { sub: 0.065, pyxl: 0.145, mega: 0.28, axis: 0.36, guide: 0.34 },
  draft: { sub: 0.105, pyxl: 0.22, mega: 0.40, axis: 0.50, guide: 0.46 },
} as const;

const EIGHT_HEAD_INDICES = [0, 1, 2, 3, 4, 5, 6, 7, 8] as const;
const MAX_ZOOM = 2.4;
const PERSPECTIVE_RAY_VECTORS = Array.from({ length: 16 }, (_, idx) => {
  const angle = (idx / 16) * Math.PI * 2;
  return { idx, dx: Math.cos(angle) * 2800, dy: Math.sin(angle) * 2800 };
});

function getFigureBounds(
  joints: Record<JointId, ComputedJoint>,
  transform: StageTransform,
  origin: Point2D
) {
  const bounds = {
    minX: Number.POSITIVE_INFINITY,
    minY: Number.POSITIVE_INFINITY,
    maxX: Number.NEGATIVE_INFINITY,
    maxY: Number.NEGATIVE_INFINITY,
  };
  for (const joint of Object.values(joints)) {
    for (const point of [
      { x: joint.x, y: joint.y },
      { x: joint.endX, y: joint.endY },
    ]) {
      const screenWorldPoint = mediaToWorld(point, transform, origin);
      bounds.minX = Math.min(bounds.minX, screenWorldPoint.x);
      bounds.minY = Math.min(bounds.minY, screenWorldPoint.y);
      bounds.maxX = Math.max(bounds.maxX, screenWorldPoint.x);
      bounds.maxY = Math.max(bounds.maxY, screenWorldPoint.y);
    }
  }
  return bounds;
}

const GROUND_CONTACT_TOLERANCE_PX = 12;

function getGroundContactPin(
  ankle: ComputedJoint,
  toe: ComputedJoint,
  groundY: number,
  side: 'R' | 'L'
): GroundContactPin {
  const ankleDistance = Math.abs(groundY - ankle.y);
  const toeDistance = Math.abs(groundY - toe.y);
  const ankleIsCloser = ankleDistance <= toeDistance;
  const contactDistance = Math.min(ankleDistance, toeDistance);
  const supportWeight = Math.max(
    0,
    Math.min(1, 1 - contactDistance / GROUND_CONTACT_TOLERANCE_PX)
  );
  const useAnkle = ankleIsCloser || supportWeight <= 0;
  const contact = useAnkle ? ankle : toe;

  return {
    x: contact.x,
    y: contact.y,
    jointId: useAnkle ? (side === 'R' ? 'footR' : 'footL') : side === 'R' ? 'toeR' : 'toeL',
    supportWeight,
    worldAngle: useAnkle ? ankle.worldAngle : undefined,
  };
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
      type: 'root_translate';
      offsetX: number;
      offsetY: number;
    }
  | {
      type: 'ik';
      jointId: JointId;
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
      startPelvis?: Point2D;
      startRootRotation?: number;
      startLumbarAngle?: number;
      startThoraxAngle?: number;
      startNeckAngle?: number;
      footRPin?: GroundContactPin;
      footLPin?: GroundContactPin;
    }
  | {
      type: 'vanishing_point';
      offsetX: number;
      offsetY: number;
    }
  | {
      type: 'stage_origin';
      offsetX: number;
      offsetY: number;
    }
  | {
      type: 'world_pin';
      pinId: string;
      offsetX: number;
      offsetY: number;
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
  screenFps,
  zoom,
  pan,
  armature,
  computedJoints,
  selectedJointId,
  onZoomChange,
  onPanChange,
  onViewportResize,
  onCursorMove,
  onTurnPage,
  onFitRequest,
  onCopyPoseData,
  onSavePoseData,
  onSelectJoint,
  onUpdateArmature,
  onAnimateArmature,
  isPoseTransitioning = false,
  isPlaying = false,
  onCommitPoseHistory,
  rigSystems,
  onUpdateRigSystems,
  showPerspective = false,
  vanishingPoint,
  onVanishingPointChange,
  sceneMedia,
  stageTransform,
  compositeFilter,
  worldPins = [],
  timelineT = 0,
  sceneDurationSec = DEFAULT_SCENE_DURATION_SEC,
  showCalibrationOrigin = false,
  onUpdateStageTransform,
  onUpdateWorldPin,
}) => {
  const containerRef = useRef<HTMLDivElement | null>(null);
  const svgRef = useRef<SVGSVGElement | null>(null);
  const videoRef = useRef<HTMLVideoElement | null>(null);
  const [viewportSize, setViewportSize] = useState({ width: 1440, height: 820 });
  const [dragState, setDragState] = useState<ActiveDragState | null>(null);
  const dragRef = useRef<ActiveDragState | null>(null);
  const [isGroundLineHovered, setIsGroundLineHovered] = useState(false);

  const effectiveMedia = useMemo(
    () => sceneMedia ?? createDefaultSceneMedia(),
    [sceneMedia]
  );
  const effectiveTransform = stageTransform ?? DEFAULT_STAGE_TRANSFORM;
  const effectiveFilter = compositeFilter ?? DEFAULT_COMPOSITE_FILTER;
  const stageTransformRef = useRef(effectiveTransform);
  stageTransformRef.current = effectiveTransform;

  const armatureRef = useRef(armature);
  armatureRef.current = armature;
  const computedJointsRef = useRef(computedJoints);
  computedJointsRef.current = computedJoints;
  const panRef = useRef(pan);
  panRef.current = pan;
  const zoomRef = useRef(zoom);
  zoomRef.current = zoom;
  const screenFpsRef = useRef<ScreenFpsOption>(screenFps);
  screenFpsRef.current = screenFps;
  const lastRightClickRef = useRef<{ time: number; x: number; y: number } | null>(null);
  const rightClickTimerRef = useRef<number | null>(null);
  const lastLeftClickRef = useRef<{ time: number; x: number; y: number; count: number } | null>(null);
  const leftClickTimerRef = useRef<number | null>(null);
  const pointerDownPosRef = useRef<{ x: number; y: number } | null>(null);
  const [lastExportKind, setLastExportKind] = useState<'none' | 'clipboard' | 'pose' | 'png'>('none');

  const pendingUpdateRef = useRef<{
    hasCursor?: boolean;
    cursor?: { x: number; y: number } | null;
    pan?: { x: number; y: number };
    zoom?: number;
    armature?: ArmatureState;
  }>({});
  const lastScreenEmitMsRef = useRef<number>(-1);
  const screenTimerRef = useRef<number | null>(null);
  const pendingPointerMoveRef = useRef<{ clientX: number; clientY: number } | null>(null);
  const pointerMoveFrameRef = useRef<number | null>(null);

  const [turnAnim, setTurnAnim] = useState<{
    key: number;
    direction: TurnDirection;
    fromPage: PageId;
    fromSubstrate: 'cover' | 'paper';
    toSubstrate: 'cover' | 'paper';
  } | null>(null);
  const prevPageRef = useRef<PageId>(pageId);
  const lastRequestedDirRef = useRef<TurnDirection>('next');
  const turnCounterRef = useRef(0);

  const meta = getPageMetadata(pageId);
  const isCover = meta.substrate === 'cover';
  // Back Cover (Page 22) hosts all circles, squares, light ink grids, rulers & procedural digital bones;
  // Soft Paper (Page 21) stays kept in the back as a clean backing paper leaf.
  const showCoverWorkspace = isCover;
  void onTurnPage;
  void onFitRequest;

  const flushScreenFrame = useCallback(
    (force = false) => {
      if (screenTimerRef.current !== null) {
        window.clearTimeout(screenTimerRef.current);
        screenTimerRef.current = null;
      }
      const now = performance.now();
      const fps = screenFpsRef.current;
      if (!force && !shouldEmitScreenFrame(lastScreenEmitMsRef.current, now, fps)) {
        const waitMs = remainingScreenFrameWaitMs(lastScreenEmitMsRef.current, now, fps);
        screenTimerRef.current = window.setTimeout(() => {
          screenTimerRef.current = null;
          flushScreenFrame(false);
        }, waitMs);
        return;
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
      lastScreenEmitMsRef.current = now;

      if (pending.hasCursor) {
        onCursorMove(pending.cursor ?? null);
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
    },
    [onCursorMove, onPanChange, onUpdateArmature, onZoomChange]
  );

  const enqueueScreenUpdate = useCallback(
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
      if (screenTimerRef.current !== null) {
        return;
      }
      const now = performance.now();
      const fps = screenFpsRef.current;
      if (shouldEmitScreenFrame(lastScreenEmitMsRef.current, now, fps)) {
        flushScreenFrame(false);
      } else {
        const waitMs = remainingScreenFrameWaitMs(lastScreenEmitMsRef.current, now, fps);
        screenTimerRef.current = window.setTimeout(() => {
          screenTimerRef.current = null;
          flushScreenFrame(false);
        }, waitMs);
      }
    },
    [flushScreenFrame]
  );

  useEffect(() => {
    return () => {
      if (screenTimerRef.current !== null) {
        window.clearTimeout(screenTimerRef.current);
        screenTimerRef.current = null;
      }
      if (leftClickTimerRef.current !== null) {
        window.clearTimeout(leftClickTimerRef.current);
        leftClickTimerRef.current = null;
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

  useEffect(() => {
    const prevPage = prevPageRef.current;
    if (prevPage === pageId) return;
    const prevMeta = getPageMetadata(prevPage);
    const currMeta = getPageMetadata(pageId);

    turnCounterRef.current += 1;
    setTurnAnim({
      key: turnCounterRef.current,
      direction: lastRequestedDirRef.current,
      fromPage: prevPage,
      fromSubstrate: prevMeta.substrate,
      toSubstrate: currMeta.substrate,
    });
    prevPageRef.current = pageId;

    const timer = window.setTimeout(() => {
      setTurnAnim(null);
    }, 480);
    return () => window.clearTimeout(timer);
  }, [pageId]);

  const borderLayout = useMemo(
    () => computeSymmetricalBorderLayout(viewportSize.width, viewportSize.height),
    [viewportSize.width, viewportSize.height]
  );

  const canons = useMemo(() => computeProportionalCanons(), []);
  const originOffset = useMemo(() => ({ x: PAPER_WIDTH / 2, y: PAPER_HEIGHT / 2 }), []);
  const figureBounds = useMemo(
    () => getFigureBounds(computedJoints, effectiveTransform, originOffset),
    [computedJoints, effectiveTransform, originOffset]
  );
  const mediaPlateBox = useMemo(
    () => computeMediaPlateBox(effectiveMedia, originOffset),
    [effectiveMedia, originOffset]
  );

  // Keep <video> backdrop plate synchronized to BottomAnimationDrawer scrub / cutout FPS
  useEffect(() => {
    if (effectiveMedia.kind !== 'video') return;
    const vid = videoRef.current;
    if (!vid) return;
    const targetSec = resolveVideoTimeForTimeline(
      timelineT,
      sceneDurationSec,
      effectiveMedia.duration || sceneDurationSec,
      effectiveMedia.fps,
      effectiveMedia.loop,
      effectiveMedia.playbackOffset
    );
    if (Math.abs((vid.currentTime || 0) - targetSec) > 0.02) {
      try {
        vid.currentTime = targetSec;
      } catch {
        // Video metadata not ready yet
      }
    }
  }, [
    effectiveMedia.duration,
    effectiveMedia.fps,
    effectiveMedia.kind,
    effectiveMedia.loop,
    effectiveMedia.playbackOffset,
    sceneDurationSec,
    timelineT,
  ]);

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

  const rulerTicks = useMemo(() => {
    const edgeMargin = Math.max(46, borderLayout.cornerSize + 18);
    return {
      xTicks: computeSymmetricalRulerTicks(
        viewportSize.width,
        pan.x,
        zoom,
        originOffset.x + effectiveTransform.originX,
        edgeMargin,
        edgeMargin,
        44
      ),
      yTicks: computeSymmetricalRulerTicks(
        viewportSize.height,
        pan.y,
        zoom,
        originOffset.y + effectiveTransform.originY,
        edgeMargin,
        edgeMargin,
        24
      ),
    };
  }, [
    borderLayout.cornerSize,
    effectiveTransform.originX,
    effectiveTransform.originY,
    originOffset.x,
    originOffset.y,
    pan.x,
    pan.y,
    viewportSize.height,
    viewportSize.width,
    zoom,
  ]);

  const inkOpacity = INK_STRENGTH_OPACITY[grid.inkStrength];
  const subpyxlZoomOpacity = zoom < 0.25 ? 0.22 : zoom < 0.5 ? 0.55 : 1.0;

  // Uncalibrated SVG stage/media world coordinates
  const clientToSvgWorld = useCallback(
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

  // Calibrated rig world coordinates (inverse StageTransform so IK/FK dragging matches cursor!)
  const clientToRigWorld = useCallback(
    (clientX: number, clientY: number) => {
      const mediaWorld = clientToSvgWorld(clientX, clientY);
      return worldToMedia(mediaWorld, stageTransformRef.current, originOffset);
    },
    [clientToSvgWorld, originOffset]
  );

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
      enqueueScreenUpdate({ zoom: nextZoom, pan: nextPan });
    },
    [enqueueScreenUpdate, figureBounds, viewportSize.height, viewportSize.width]
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
      const joints = computedJointsRef.current;
      const groundY =
        rigSystems?.pressurePlateY ??
        rigSystems?.groundY ??
        originOffset.y + canons.eightHeadTotalPx / 2;
      const isLoinPivot = targetJointId === 'hipR' || targetJointId === 'hipL';
      safeSetPointerCapture(e.pointerId);
      setActiveDrag({
        type: 'fk_rotate',
        targetJointId,
        pivotX: pivot.x,
        pivotY: pivot.y,
        lastCursorDeg,
        rawAngle,
        ...(isLoinPivot
          ? {
              startPelvis: { x: joints.pelvis.x, y: joints.pelvis.y },
              startRootRotation: curArmature.rootRotation,
              startLumbarAngle: curArmature.joints.lumbar.localAngle,
              startThoraxAngle: curArmature.joints.thorax.localAngle,
              startNeckAngle: curArmature.joints.neck.localAngle,
              footRPin: getGroundContactPin(joints.footR, joints.toeR, groundY, 'R'),
              footLPin: getGroundContactPin(joints.footL, joints.toeL, groundY, 'L'),
            }
          : {}),
      });
    },
    [
      canons.eightHeadTotalPx,
      clientToRigWorld,
      originOffset.y,
      rigSystems?.groundY,
      rigSystems?.pressurePlateY,
      safeSetPointerCapture,
      setActiveDrag,
    ]
  );

  const beginFaceplateLookDrag = useCallback(
    (e: React.PointerEvent<SVGElement>) => {
      e.stopPropagation();
      onSelectJoint('lip');
      const curArmature = armatureRef.current;
      const runtime = curArmature.joints.lip;
      if (!runtime || runtime.mode === 'PIN' || runtime.physicsMode === 'rigid') return;

      const world = clientToRigWorld(e.clientX, e.clientY);
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
      enqueueScreenUpdate({ armature: startArmature });
    },
    [
      clientToRigWorld,
      enqueueScreenUpdate,
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
      const groundY =
        rigSystems?.pressurePlateY ??
        rigSystems?.groundY ??
        originOffset.y + canons.eightHeadTotalPx / 2;
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
      canons.eightHeadTotalPx,
      clientToRigWorld,
      originOffset.y,
      rigSystems?.groundY,
      rigSystems?.pressurePlateY,
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
      const groundY =
        rigSystems?.pressurePlateY ??
        rigSystems?.groundY ??
        originOffset.y + canons.eightHeadTotalPx / 2;
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
    [clientToRigWorld, onSelectJoint, safeSetPointerCapture, setActiveDrag]
  );

  const beginPressurePlateDrag = useCallback(
    (e: React.PointerEvent<SVGPathElement>) => {
      e.stopPropagation();
      const joints = computedJointsRef.current;
      const pressureY =
        rigSystems?.pressurePlateY ??
        Math.max(joints.footR.y, joints.toeR.y, joints.footL.y, joints.toeL.y);
      const world = clientToRigWorld(e.clientX, e.clientY);
      safeSetPointerCapture(e.pointerId);
      setActiveDrag({
        type: 'pressure_plate',
        startArmature: armatureRef.current,
        offsetY: pressureY - world.y,
      });
    },
    [clientToRigWorld, rigSystems?.pressurePlateY, safeSetPointerCapture, setActiveDrag]
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
      if (jointId === 'handR' || jointId === 'handL') {
        startCumulativeFKRotate(e, jointId);
        return;
      }
      const curArmature = armatureRef.current;
      const joint = computedJointsRef.current[jointId];
      const spec = JOINT_SPEC_MAP[jointId];
      const world = clientToRigWorld(e.clientX, e.clientY);

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

      if (joint.mode === 'IK' && joint.spec.ikChain) {
        if (curArmature.joints[jointId].physicsMode === 'rigid') return;
        safeSetPointerCapture(e.pointerId);
        setActiveDrag({
          type: 'ik',
          jointId,
          offsetX: joint.x - world.x,
          offsetY: joint.y - world.y,
        });
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
      canons.eightHeadTotalPx,
      originOffset.y,
      onSelectJoint,
      rigSystems?.groundY,
      rigSystems?.pressurePlateY,
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
      const clawJointId: JointId | null =
        jointId === 'handR' || jointId === 'fingerR'
          ? 'handR'
          : jointId === 'handL' || jointId === 'fingerL'
            ? 'handL'
            : null;
      onSelectJoint(clawJointId ?? jointId);
      if (clawJointId) {
        startCumulativeFKRotate(e, clawJointId);
        return;
      }
      // Dragging the waist bowl activates grounded balance / hula / squat without moving feet
      if (jointId === 'pelvis') {
        beginWaistBalanceDrag(e);
        return;
      }
      // Dragging either half of the split collarbone folds it rotationally up or down
      const drivenJointId: JointId =
        jointId === 'clavOuterR' ? 'braceR' : jointId === 'clavOuterL' ? 'braceL' : jointId;
      const curArmature = armatureRef.current;
      const runtime = curArmature.joints[drivenJointId];
      if (!runtime || runtime.mode === 'PIN' || runtime.physicsMode === 'rigid') return;

      const boneSpec = JOINT_SPEC_MAP[jointId];
      if (curArmature.joints[jointId]?.mode === 'IK' && boneSpec?.ikChain) {
        const world = clientToRigWorld(e.clientX, e.clientY);
        const joint = computedJointsRef.current[jointId];
        safeSetPointerCapture(e.pointerId);
        setActiveDrag({
          type: 'ik',
          jointId,
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
      onSelectJoint,
      safeSetPointerCapture,
      setActiveDrag,
      startCumulativeFKRotate,
    ]
  );

  const handlePointerMove = useCallback(
    ({ clientX, clientY }: Pick<React.PointerEvent<SVGSVGElement>, 'clientX' | 'clientY'>) => {
      const mediaWorld = clientToSvgWorld(clientX, clientY);
      const world = worldToMedia(mediaWorld, stageTransformRef.current, originOffset);
      const cursorPatch = {
        hasCursor: true,
        cursor: {
          x: world.x - originOffset.x,
          y: world.y - originOffset.y,
        },
      };

      const activeDrag = dragRef.current;
      if (!activeDrag) {
        enqueueScreenUpdate(cursorPatch);
        return;
      }

      if (activeDrag.type === 'pan') {
        const nextPan = {
          x: activeDrag.startPanX + (clientX - activeDrag.startClientX),
          y: activeDrag.startPanY + (clientY - activeDrag.startClientY),
        };
        panRef.current = nextPan;
        enqueueScreenUpdate({ ...cursorPatch, pan: nextPan });
        return;
      }

      if (activeDrag.type === 'stage_origin') {
        onUpdateStageTransform?.({
          originX: Number((mediaWorld.x - originOffset.x + activeDrag.offsetX).toFixed(1)),
          originY: Number((mediaWorld.y - originOffset.y + activeDrag.offsetY).toFixed(1)),
        });
        enqueueScreenUpdate(cursorPatch);
        return;
      }

      if (activeDrag.type === 'world_pin') {
        onUpdateWorldPin?.(activeDrag.pinId, {
          x: Number((mediaWorld.x + activeDrag.offsetX).toFixed(1)),
          y: Number((mediaWorld.y + activeDrag.offsetY).toFixed(1)),
        });
        enqueueScreenUpdate(cursorPatch);
        return;
      }

      if (activeDrag.type === 'pressure_plate') {
        const solution = solvePressurePlateContact(
          activeDrag.startArmature,
          world.y + activeDrag.offsetY,
          originOffset
        );
        onUpdateRigSystems?.({ pressurePlateY: solution.pressurePlateY });
        armatureRef.current = solution.armature;
        enqueueScreenUpdate({ ...cursorPatch, armature: solution.armature });
        return;
      }

      const curArmature = armatureRef.current;

      if (activeDrag.type === 'neck_collar_magnet') {
        const nextArmature = setNeckCollarSeparation(curArmature, {
          x: activeDrag.startSeparation.x + world.x - activeDrag.startPointer.x,
          y: activeDrag.startSeparation.y + world.y - activeDrag.startPointer.y,
        });
        armatureRef.current = nextArmature;
        enqueueScreenUpdate({ ...cursorPatch, armature: nextArmature });
        return;
      }

      if (activeDrag.type === 'faceplate_look') {
        const nextArmature = solveFaceplateLookIK(
          activeDrag.startArmature,
          world.x + activeDrag.offsetX,
          world.y + activeDrag.offsetY,
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
        enqueueScreenUpdate({ ...cursorPatch, armature: nextArmature });
        return;
      }

      if (activeDrag.type === 'root_translate') {
        const nextArmature: ArmatureState = {
          ...curArmature,
          rootX: world.x + activeDrag.offsetX,
          rootY: world.y + activeDrag.offsetY,
        };
        armatureRef.current = nextArmature;
        enqueueScreenUpdate({ ...cursorPatch, armature: nextArmature });
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
        enqueueScreenUpdate({ ...cursorPatch, armature: nextArmature });
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
        enqueueScreenUpdate({ ...cursorPatch, armature: nextArmature });
        return;
      }

      if (activeDrag.type === 'hip_toggle') {
        const targetMidX = world.x + activeDrag.offsetX;
        const targetMidY = world.y + activeDrag.offsetY;
        const nextArmature = solveHipToggleFromTarget(
          curArmature,
          targetMidX,
          targetMidY,
          originOffset
        );
        armatureRef.current = nextArmature;
        enqueueScreenUpdate({ ...cursorPatch, armature: nextArmature });
        return;
      }

      if (activeDrag.type === 'collar_toggle') {
        const targetMidX = world.x + activeDrag.offsetX;
        const targetMidY = world.y + activeDrag.offsetY;
        const nextArmature = solveCollarToggleFromTarget(
          curArmature,
          activeDrag.side,
          targetMidX,
          targetMidY,
          originOffset
        );
        armatureRef.current = nextArmature;
        enqueueScreenUpdate({ ...cursorPatch, armature: nextArmature });
        return;
      }

      if (activeDrag.type === 'arm_swivel') {
        const moveDist = Math.hypot(
          clientX - activeDrag.startClientX,
          clientY - activeDrag.startClientY
        );
        if (!activeDrag.hasDragged && moveDist < 5) {
          enqueueScreenUpdate(cursorPatch);
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
        enqueueScreenUpdate({ ...cursorPatch, armature: nextArmature });
        return;
      }

      if (activeDrag.type === 'vanishing_point') {
        onVanishingPointChange?.({
          x: world.x + activeDrag.offsetX,
          y: world.y + activeDrag.offsetY,
        });
        enqueueScreenUpdate(cursorPatch);
        return;
      }

      if (activeDrag.type === 'ik') {
        const targetX = world.x + activeDrag.offsetX;
        const targetY = world.y + activeDrag.offsetY;
        const nextArmature = solveJointIK(
          curArmature,
          activeDrag.jointId,
          targetX,
          targetY,
          originOffset,
          {
            solver: rigSystems?.solver ?? 'analytical',
            softDamp: true,
            constrainLimits: rigSystems?.softLimits ?? false,
          }
        );
        armatureRef.current = nextArmature;
        enqueueScreenUpdate({ ...cursorPatch, armature: nextArmature });
        return;
      }

      if (activeDrag.type === 'fk_rotate') {
        const { targetJointId, pivotX, pivotY, lastCursorDeg, rawAngle } = activeDrag;
        const distFromPivot = Math.hypot(world.x - pivotX, world.y - pivotY);
        if (distFromPivot < 4) {
          enqueueScreenUpdate(cursorPatch);
          return;
        }
        const currDeg = (Math.atan2(world.y - pivotY, world.x - pivotX) * 180) / Math.PI;
        if (lastCursorDeg === null) {
          dragRef.current = { ...activeDrag, lastCursorDeg: currDeg };
          enqueueScreenUpdate(cursorPatch);
          return;
        }
        const deltaCursor = shortestAngleDelta(lastCursorDeg, currDeg);
        if (Math.abs(deltaCursor) < 1e-4) {
          enqueueScreenUpdate(cursorPatch);
          return;
        }

        const runtime = curArmature.joints[targetJointId];
        const mult = getPhysicsMultiplier(runtime);
        if (mult === 0) {
          enqueueScreenUpdate(cursorPatch);
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
          enqueueScreenUpdate({ ...cursorPatch, armature: nextArmature });
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
        let nextArmature: ArmatureState =
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
        if (
          (targetJointId === 'hipR' || targetJointId === 'hipL') &&
          activeDrag.startPelvis &&
          activeDrag.footRPin &&
          activeDrag.footLPin
        ) {
          nextArmature = solveWaistBalanceHula(
            nextArmature,
            activeDrag.startPelvis.x,
            activeDrag.startPelvis.y,
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
        }
        armatureRef.current = nextArmature;
        enqueueScreenUpdate({ ...cursorPatch, armature: nextArmature });
      }
    },
    [
      clientToSvgWorld,
      enqueueScreenUpdate,
      onUpdateRigSystems,
      onUpdateStageTransform,
      onUpdateWorldPin,
      onVanishingPointChange,
      originOffset,
      rigSystems?.softLimits,
      rigSystems?.solver,
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
      flushScreenFrame(true);
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
          activeDrag.type === 'root_translate' ||
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
    },
    [
      flushPendingPointerMove,
      flushScreenFrame,
      onAnimateArmature,
      onCommitPoseHistory,
      onUpdateArmature,
      setActiveDrag,
    ]
  );

  const isPlateMedia = effectiveMedia.kind === 'image' || effectiveMedia.kind === 'video';
  const showPaperCover =
    effectiveMedia.kind === 'paper' && (effectiveMedia.paperVariant ?? 'cover') === 'cover' && isCover;
  const inkRgb = darkMode ? '240, 234, 248' : '22, 0, 30';
  const inkHex = darkMode ? '#F0EAF8' : REPO_DARKEST_PURPLE;
  const paperBg = darkMode
    ? '#000000'
    : isPlateMedia
    ? '#CFCAC0'
    : showPaperCover
    ? '#CECAC2'
    : '#EFECE6';

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
  const staticGroundY =
    rigSystems?.groundY ?? originOffset.y + canons.eightHeadTotalPx / 2;
  const activePressurePlateY = rigSystems?.pressurePlateY ?? pressurePlateY;
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
  const calibratedStageTransform = buildStageSvgTransform(effectiveTransform, originOffset);
  const isMotionActive = isPoseTransitioning || isPlaying || dragState !== null;

  return (
    <div
      ref={containerRef}
      data-stage-viewport="true"
      data-page-id={pageId}
      data-substrate={effectiveMedia.kind === 'paper' ? meta.substrate : effectiveMedia.kind}
      data-media-kind={effectiveMedia.kind}
      data-dark-mode={darkMode ? 'true' : 'false'}
      data-target-fps={screenFps}
      data-pose-interpolating={isPoseTransitioning ? 'true' : 'false'}
      data-last-export={lastExportKind}
      data-vw={viewportSize.width}
      data-vh={viewportSize.height}
      className="relative flex-1 w-full h-full overflow-hidden select-none transition-colors duration-300"
      style={{
        backgroundColor: paperBg,
        cursor: dragState?.type === 'pan' ? 'grabbing' : 'crosshair',
      }}
    >
      {/* 01. Procedural Paper Plate (Backing Leaf + Default 1901 Back Cover) */}
      {effectiveMedia.kind === 'paper' && (
        <>
          <div
            aria-hidden="true"
            data-paper-surface="true"
            data-paper-role="backing-leaf"
            className="pointer-events-none absolute inset-0"
          >
            <img
              src={REPOSITORY_ASSET_PATHS.page21PaperPng}
              alt="Soft print paper leaf kept in the back"
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

          {showPaperCover && (
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
        </>
      )}

      {/* 01b. External Image or Synced Video Backdrop Plate (World-space locked to pan/zoom) */}
      {isPlateMedia && (
        <div
          aria-hidden="true"
          data-media-plate="true"
          data-media-src={effectiveMedia.name}
          className="pointer-events-none absolute inset-0 overflow-hidden"
        >
          <div
            style={{
              position: 'absolute',
              left: 0,
              top: 0,
              transformOrigin: '0 0',
              transform: `translate(${pan.x}px, ${pan.y}px) scale(${zoom})`,
            }}
          >
            <div
              style={{
                position: 'absolute',
                left: mediaPlateBox.x,
                top: mediaPlateBox.y,
                width: mediaPlateBox.width,
                height: mediaPlateBox.height,
                boxShadow: '0 24px 60px rgba(0, 0, 0, 0.52), 0 0 0 2px rgba(235, 224, 245, 0.16)',
                backgroundColor: '#D6D0C5',
              }}
            >
              {effectiveMedia.kind === 'image' ? (
                <img
                  src={effectiveMedia.src}
                  alt={effectiveMedia.name}
                  className={`w-full h-full object-cover ${darkMode ? 'brightness-75 contrast-115' : ''}`}
                />
              ) : (
                <video
                  ref={videoRef}
                  src={effectiveMedia.src}
                  muted
                  playsInline
                  loop={effectiveMedia.loop}
                  className={`w-full h-full object-cover ${darkMode ? 'brightness-75 contrast-115' : ''}`}
                />
              )}
              {effectiveFilter.paperGrain > 0 && (
                <div
                  className="absolute inset-0 pointer-events-none"
                  style={{
                    backgroundImage: `url(${REPOSITORY_ASSET_PATHS.analogPaperPng})`,
                    backgroundSize: 'cover',
                    backgroundPosition: 'center',
                    opacity: Math.min(0.45, effectiveFilter.paperGrain * 0.34),
                    mixBlendMode: 'multiply',
                  }}
                />
              )}
            </div>
          </div>
        </div>
      )}

      {/* Main Proportional SVG World Plane (Mounted onto the Active Scene Backdrop Plate) */}
      <svg
        ref={svgRef}
        data-world-canvas="true"
        className={`relative w-full h-full block touch-none transition-opacity duration-200 ${
          showCoverWorkspace ? 'opacity-100' : 'opacity-0 pointer-events-none'
        }`}
        onContextMenu={(e) => e.preventDefault()}
        onPointerDownCapture={(e) => {
          if (e.button === 0) {
            pointerDownPosRef.current = { x: e.clientX, y: e.clientY };
            if (e.metaKey && e.shiftKey) {
              e.preventDefault();
              e.stopPropagation();
              beginPanDrag(e);
            }
          }
          if (e.button === 2) {
            e.preventDefault();
            e.stopPropagation();
            safeSetPointerCapture(e.pointerId);
            if (rightClickTimerRef.current !== null) {
              window.clearTimeout(rightClickTimerRef.current);
              rightClickTimerRef.current = null;
            }
            const now = performance.now();
            const prev = lastRightClickRef.current;
            if (
              prev &&
              now - prev.time <= 450 &&
              Math.hypot(e.clientX - prev.x, e.clientY - prev.y) <= 16
            ) {
              lastRightClickRef.current = null;
              const svgEl = svgRef.current;
              if (svgEl) {
                void saveTransparentModelImage(svgEl, computedJointsRef.current).then((res) => {
                  if (res) setLastExportKind('png');
                });
              }
            } else {
              lastRightClickRef.current = { time: now, x: e.clientX, y: e.clientY };
              rightClickTimerRef.current = window.setTimeout(() => {
                rightClickTimerRef.current = null;
                lastRightClickRef.current = null;
                void Promise.resolve(onCopyPoseData()).then(() => {
                  setLastExportKind('clipboard');
                });
              }, 600);
            }
          }
        }}
        onPointerDown={handlePointerDownStage}
        onPointerMove={(e) => queuePointerMove(e.clientX, e.clientY)}
        onPointerUpCapture={(e) => {
          if (e.button !== 2 || rightClickTimerRef.current === null) return;
          window.clearTimeout(rightClickTimerRef.current);
          rightClickTimerRef.current = null;
        }}
        onPointerCancelCapture={() => {
          if (rightClickTimerRef.current === null) return;
          window.clearTimeout(rightClickTimerRef.current);
          rightClickTimerRef.current = null;
        }}
        onPointerUp={handlePointerUp}
        onPointerLeave={() => {
          flushPendingPointerMove();
          enqueueScreenUpdate({ hasCursor: true, cursor: null });
          flushScreenFrame(true);
        }}
        onClick={(e) => {
          if (e.button !== 0) return;
          const downPos = pointerDownPosRef.current;
          pointerDownPosRef.current = null;
          if (downPos && Math.hypot(e.clientX - downPos.x, e.clientY - downPos.y) > 6) {
            lastLeftClickRef.current = null;
            return;
          }
          const now = performance.now();
          const prev = lastLeftClickRef.current;
          const withinWindow =
            prev &&
            now - prev.time <= 380 &&
            Math.hypot(e.clientX - prev.x, e.clientY - prev.y) <= 16;
          const nextCount = withinWindow ? prev.count + 1 : 1;
          lastLeftClickRef.current = {
            time: now,
            x: e.clientX,
            y: e.clientY,
            count: nextCount,
          };

          if (leftClickTimerRef.current !== null) {
            window.clearTimeout(leftClickTimerRef.current);
            leftClickTimerRef.current = null;
          }
          if (rightClickTimerRef.current !== null) {
            window.clearTimeout(rightClickTimerRef.current);
            rightClickTimerRef.current = null;
          }

          if (nextCount >= 3) {
            lastLeftClickRef.current = null;
            onSavePoseData();
            setLastExportKind('pose');
          } else if (nextCount === 2) {
            leftClickTimerRef.current = window.setTimeout(() => {
              leftClickTimerRef.current = null;
              lastLeftClickRef.current = null;
              void Promise.resolve(onCopyPoseData()).then(() => {
                setLastExportKind('clipboard');
              });
            }, 250);
          }
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
              stroke={`rgba(${inkRgb}, ${inkOpacity.sub})`}
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
              stroke={`rgba(${inkRgb}, ${inkOpacity.pyxl})`}
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
              stroke={`rgba(${inkRgb}, ${inkOpacity.mega})`}
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
          {/* 02. Calibration Grid & Calibrated Rig Plane (StageTransform) */}
          <g data-stage-calibrated="true" transform={calibratedStageTransform}>
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
              stroke={`rgba(${inkRgb}, ${inkOpacity.axis})`}
              strokeWidth={1.1}
              vectorEffect="non-scaling-stroke"
            />
            <line
              x1={endlessBounds.minX - 1200}
              y1={originOffset.y}
              x2={endlessBounds.maxX + 1200}
              y2={originOffset.y}
              stroke={`rgba(${inkRgb}, ${inkOpacity.axis})`}
              strokeWidth={1.1}
              vectorEffect="non-scaling-stroke"
            />
            <g transform={`translate(${originOffset.x}, ${originOffset.y}) scale(${1 / zoom})`}>
              <circle
                r={7}
                fill="none"
                stroke={`rgba(${inkRgb}, ${inkOpacity.axis + 0.18})`}
                strokeWidth={1.1}
              />
              <circle r={1.8} fill={`rgba(${inkRgb}, ${inkOpacity.axis + 0.3})`} />
            </g>
          </g>

          {/* Backlit Cover Paper: Navel-Centered Halo + Dimming Outside Vitruvian Square & Circle */}
          {showCoverWorkspace && !isPlateMedia && (
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
                stroke={`rgba(${inkRgb}, ${inkOpacity.guide})`}
                strokeWidth={1.1}
                vectorEffect="non-scaling-stroke"
              />
              <line
                x1={wsX}
                y1={wsY}
                x2={wsX + canons.workingSquareSidePx}
                y2={wsY + canons.workingSquareSidePx}
                stroke={`rgba(${inkRgb}, ${inkOpacity.guide * 0.42})`}
                strokeWidth={0.75}
                strokeDasharray="4 6"
                vectorEffect="non-scaling-stroke"
              />
              <line
                x1={wsX + canons.workingSquareSidePx}
                y1={wsY}
                x2={wsX}
                y2={wsY + canons.workingSquareSidePx}
                stroke={`rgba(${inkRgb}, ${inkOpacity.guide * 0.42})`}
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
                stroke={`rgba(${inkRgb}, ${inkOpacity.guide})`}
                strokeWidth={1.1}
                vectorEffect="non-scaling-stroke"
              />
              <circle
                cx={originOffset.x}
                cy={originOffset.y}
                r={canons.vitruvianCircleRadius}
                fill="none"
                stroke={`rgba(${inkRgb}, ${inkOpacity.guide})`}
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

          {/* Cover / Plate Calibration Spec Caption */}
          {showCoverWorkspace && (
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
                {isPlateMedia
                  ? `${effectiveMedia.name.toUpperCase()} · CALIBRATION PLATE`
                  : 'BACK COVER · PROPORTIONAL CANVAS'}
              </text>
              <text
                x={originOffset.x}
                y={originOffset.y + canons.vitruvianCircleRadius + 30}
                textAnchor="middle"
                fill={`rgba(${inkRgb}, 0.56)`}
                fontSize={17}
                fontFamily="Bitstream Vera Sans Mono, monospace"
              >
                {isPlateMedia
                  ? `PYX SCALE ${effectiveTransform.pyxScale.toFixed(2)}× · TILT ${effectiveTransform.rotation.toFixed(1)}° · ORIGIN (${Math.round(effectiveTransform.originX)}, ${Math.round(effectiveTransform.originY)})`
                  : 'TORSO 1.25× HIPS · 1.00 PYX = 100 PX · R = 6.35 PYX'}
              </text>
            </g>
          )}

          {/* 1-Point Perspective Grid & Draggable Vanishing Point */}
          {showPerspective && vanishingPoint && (
            <g data-perspective-overlay="true">
              {/* Horizon Line */}
              <line
                x1={endlessBounds.minX}
                y1={vanishingPoint.y}
                x2={endlessBounds.maxX}
                y2={vanishingPoint.y}
                stroke={darkMode ? 'rgba(216, 180, 254, 0.46)' : 'rgba(95, 0, 128, 0.28)'}
                strokeWidth={1.6}
                strokeDasharray="6 4"
                vectorEffect="non-scaling-stroke"
                pointerEvents="none"
              />
              {/* Radial perspective rays */}
              {PERSPECTIVE_RAY_VECTORS.map((ray) => (
                <line
                  key={`vp-ray-${ray.idx}`}
                  x1={vanishingPoint.x}
                  y1={vanishingPoint.y}
                  x2={vanishingPoint.x + ray.dx}
                  y2={vanishingPoint.y + ray.dy}
                  stroke={darkMode ? 'rgba(192, 132, 252, 0.16)' : 'rgba(95, 0, 128, 0.14)'}
                  strokeWidth={0.85}
                  vectorEffect="non-scaling-stroke"
                  pointerEvents="none"
                />
              ))}
              {/* Draggable Vanishing Point Handle */}
              <g
                transform={`translate(${vanishingPoint.x}, ${vanishingPoint.y}) scale(${1 / zoom})`}
                style={{ cursor: 'move' }}
                onPointerDown={(e) => {
                  e.stopPropagation();
                  const world = clientToSvgWorld(e.clientX, e.clientY);
                  safeSetPointerCapture(e.pointerId);
                  setActiveDrag({
                    type: 'vanishing_point',
                    offsetX: vanishingPoint.x - world.x,
                    offsetY: vanishingPoint.y - world.y,
                  });
                }}
              >
                <circle r={14} fill="transparent" />
                <circle
                  r={7}
                  fill={darkMode ? 'rgba(13, 148, 136, 0.28)' : 'rgba(13, 148, 136, 0.18)'}
                  stroke={darkMode ? '#5EEAD4' : '#0D9488'}
                  strokeWidth={1.4}
                />
                <circle r={2.2} fill={darkMode ? '#5EEAD4' : '#0D9488'} />
                <text
                  x={11}
                  y={4}
                  fill={darkMode ? '#5EEAD4' : '#0D9488'}
                  fontSize={10}
                  fontFamily="Bitstream Vera Sans Mono, monospace"
                  fontWeight={700}
                >
                  VP
                </text>
              </g>
            </g>
          )}

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
            <title>Drag vertically to change ground pressure; toes compensate within joint limits</title>
          </path>

          {/* Pure Data Procedural SVG Digital Bones Puppet */}
          <DigitalMannequin
            armature={armature}
            computedJoints={computedJoints}
            selectedJointId={selectedJointId}
            zoom={zoom}
            darkMode={darkMode}
            isPanning={dragState?.type === 'pan'}
            isMotionActive={isMotionActive}
            compositeFilter={effectiveFilter}
            onPointerDownJoint={handlePointerDownJoint}
            onPointerDownBone={handlePointerDownBone}
            onPointerDownHipToggle={handlePointerDownHipToggle}
            onPointerDownArmSwivel={handlePointerDownArmSwivel}
            onPointerDownNeckCollarMagnet={beginNeckCollarMagnetDrag}
            onResetNeckCollarMagnet={resetNeckCollarMagnet}
            onPointerDownTorso={beginTorsoBalanceDrag}
          />
          </g>

          {/* Draggable Calibration Origin Handle (Shown when Scene Calibration active) */}
          {showCalibrationOrigin && onUpdateStageTransform && (
            <g
              data-calibration-origin-handle="true"
              transform={`translate(${originOffset.x + effectiveTransform.originX}, ${originOffset.y + effectiveTransform.originY}) scale(${1 / zoom})`}
              style={{ cursor: 'move' }}
              onPointerDown={(e) => {
                e.stopPropagation();
                const mediaWorld = clientToSvgWorld(e.clientX, e.clientY);
                safeSetPointerCapture(e.pointerId);
                setActiveDrag({
                  type: 'stage_origin',
                  offsetX: originOffset.x + effectiveTransform.originX - mediaWorld.x,
                  offsetY: originOffset.y + effectiveTransform.originY - mediaWorld.y,
                });
              }}
            >
              <circle r={18} fill="transparent" />
              <circle
                r={10}
                fill="rgba(52, 211, 153, 0.16)"
                stroke="#34D399"
                strokeWidth={1.4}
                strokeDasharray="3 2"
              />
              <line x1={-14} y1={0} x2={14} y2={0} stroke="#34D399" strokeWidth={1.1} />
              <line x1={0} y1={-14} x2={0} y2={14} stroke="#34D399" strokeWidth={1.1} />
              <text
                x={14}
                y={-8}
                fill="#34D399"
                fontSize={10}
                fontFamily="Bitstream Vera Sans Mono, monospace"
                fontWeight={700}
              >
                PYX ORIGIN
              </text>
            </g>
          )}

          {/* 03. Grounding / Pinning: WorldPin Anchor Reticles in Media Space */}
          {worldPins.length > 0 && (
            <g data-world-pins-layer="true">
              {worldPins.map((pin) => {
                const inRange =
                  pin.enabled &&
                  timelineT + 1e-6 >= pin.frameRange[0] &&
                  timelineT - 1e-6 <= pin.frameRange[1];
                const shortCode = JOINT_SPEC_MAP[pin.jointId]?.shortCode ?? pin.jointId;
                return (
                  <g
                    key={pin.id}
                    data-world-pin={pin.jointId}
                    data-pin-active={inRange ? 'true' : 'false'}
                    transform={`translate(${pin.mediaSpaceXY.x}, ${pin.mediaSpaceXY.y}) scale(${1 / zoom})`}
                    style={{ cursor: onUpdateWorldPin ? 'move' : 'default' }}
                    onPointerDown={(e) => {
                      if (!onUpdateWorldPin) return;
                      e.stopPropagation();
                      const mediaWorld = clientToSvgWorld(e.clientX, e.clientY);
                      safeSetPointerCapture(e.pointerId);
                      setActiveDrag({
                        type: 'world_pin',
                        pinId: pin.id,
                        offsetX: pin.mediaSpaceXY.x - mediaWorld.x,
                        offsetY: pin.mediaSpaceXY.y - mediaWorld.y,
                      });
                    }}
                  >
                    <circle r={14} fill="transparent" />
                    <polygon
                      points="0,-7 7,0 0,7 -7,0"
                      fill={inRange ? 'rgba(245, 158, 11, 0.28)' : 'rgba(148, 163, 184, 0.14)'}
                      stroke={inRange ? '#D97706' : '#94A3B8'}
                      strokeWidth={1.5}
                      strokeDasharray={inRange ? undefined : '2 2'}
                    />
                    <circle r={2} fill={inRange ? '#F59E0B' : '#94A3B8'} />
                    <text
                      x={0}
                      y={18}
                      textAnchor="middle"
                      fill={inRange ? '#B45309' : '#64748B'}
                      stroke={darkMode ? 'rgba(10, 4, 16, 0.85)' : 'rgba(245, 240, 232, 0.88)'}
                      strokeWidth={2.5}
                      paintOrder="stroke"
                      fontSize={9}
                      fontFamily="Bitstream Vera Sans Mono, monospace"
                      fontWeight={700}
                    >
                      {shortCode}
                    </text>
                  </g>
                );
              })}
            </g>
          )}
        </g>
      </svg>

      {/* Symmetrical Edge Rulers (light ink ticks hugging all 4 inner borders symmetrically) */}
      {showCoverWorkspace && grid.showRulers && (
        <svg
          aria-hidden="true"
          className="pointer-events-none absolute inset-0 w-full h-full z-[6]"
          viewBox={`0 0 ${viewportSize.width} ${viewportSize.height}`}
          preserveAspectRatio="none"
        >
          <g>
            {rulerTicks.xTicks.map((tick, idx) => (
              <React.Fragment key={`rx-${idx}`}>
                <g transform={`translate(${tick.screen}, ${borderLayout.edgeThickness})`}>
                  <line
                    x1={0}
                    y1={0}
                    x2={0}
                    y2={tick.tier === 'mega' ? 9 : tick.tier === 'major' ? 7 : 4}
                    stroke={`rgba(${inkRgb}, ${tick.tier === 'mega' ? 0.58 : tick.tier === 'major' ? 0.38 : 0.22})`}
                    strokeWidth={tick.tier === 'mega' ? 1.25 : 0.9}
                  />
                  {tick.label && (
                    <text
                      x={0}
                      y={19}
                      textAnchor="middle"
                      fill={inkHex}
                      fillOpacity={tick.tier === 'mega' ? 0.72 : 0.52}
                      fontSize={tick.tier === 'mega' ? 10 : 9}
                      fontWeight={tick.tier === 'mega' ? 700 : 400}
                      fontFamily="UMTypewriter, monospace"
                    >
                      {tick.label}
                    </text>
                  )}
                </g>
                <g transform={`translate(${tick.screen}, ${viewportSize.height - borderLayout.edgeThickness})`}>
                  <line
                    x1={0}
                    y1={0}
                    x2={0}
                    y2={-(tick.tier === 'mega' ? 9 : tick.tier === 'major' ? 7 : 4)}
                    stroke={`rgba(${inkRgb}, ${tick.tier === 'mega' ? 0.48 : tick.tier === 'major' ? 0.3 : 0.18})`}
                    strokeWidth={tick.tier === 'mega' ? 1.25 : 0.9}
                  />
                </g>
              </React.Fragment>
            ))}

            {rulerTicks.yTicks.map((tick, idx) => (
              <React.Fragment key={`ry-${idx}`}>
                <g transform={`translate(${borderLayout.edgeThickness}, ${tick.screen})`}>
                  <line
                    x1={0}
                    y1={0}
                    x2={tick.tier === 'mega' ? 9 : tick.tier === 'major' ? 7 : 4}
                    y2={0}
                    stroke={`rgba(${inkRgb}, ${tick.tier === 'mega' ? 0.58 : tick.tier === 'major' ? 0.38 : 0.22})`}
                    strokeWidth={tick.tier === 'mega' ? 1.25 : 0.9}
                  />
                  {tick.label && (
                    <text
                      x={13}
                      y={3}
                      textAnchor="start"
                      fill={inkHex}
                      fillOpacity={tick.tier === 'mega' ? 0.72 : 0.52}
                      fontSize={tick.tier === 'mega' ? 10 : 9}
                      fontWeight={tick.tier === 'mega' ? 700 : 400}
                      fontFamily="UMTypewriter, monospace"
                    >
                      {tick.label}
                    </text>
                  )}
                </g>
                <g transform={`translate(${viewportSize.width - borderLayout.edgeThickness}, ${tick.screen})`}>
                  <line
                    x1={0}
                    y1={0}
                    x2={-(tick.tier === 'mega' ? 9 : tick.tier === 'major' ? 7 : 4)}
                    y2={0}
                    stroke={`rgba(${inkRgb}, ${tick.tier === 'mega' ? 0.48 : tick.tier === 'major' ? 0.3 : 0.18})`}
                    strokeWidth={tick.tier === 'mega' ? 1.25 : 0.9}
                  />
                </g>
              </React.Fragment>
            ))}
          </g>
        </svg>
      )}

      {/* Turning Page/Board Overlay Plane */}
      {turnAnim && (
        <div
          key={`leaf-${turnAnim.key}`}
          aria-hidden="true"
          className={`pointer-events-none absolute inset-0 z-[8] ${
            turnAnim.fromSubstrate === 'cover' || turnAnim.toSubstrate === 'cover'
              ? turnAnim.direction === 'next'
                ? 'animate-cover-next'
                : 'animate-cover-prev'
              : turnAnim.direction === 'next'
              ? 'animate-paper-next'
              : 'animate-paper-prev'
          }`}
          style={{
            backgroundColor: darkMode
              ? 'rgba(10, 10, 14, 0.92)'
              : turnAnim.fromSubstrate === 'cover'
              ? 'rgba(206, 202, 194, 0.92)'
              : 'rgba(239, 236, 230, 0.90)',
            boxShadow: darkMode
              ? '0 0 42px rgba(255, 255, 255, 0.18)'
              : turnAnim.fromSubstrate === 'cover'
              ? '0 0 48px rgba(0, 0, 0, 0.42)'
              : '0 0 32px rgba(28, 20, 34, 0.22)',
          }}
        />
      )}

      {/* Symmetrical Scan-Edge Border & Edge Animation Engine */}
      <ScanBorderLayer
        vw={viewportSize.width}
        vh={viewportSize.height}
        substrate={meta.substrate}
        darkMode={darkMode}
        turnAnim={turnAnim}
      />
    </div>
  );
});
