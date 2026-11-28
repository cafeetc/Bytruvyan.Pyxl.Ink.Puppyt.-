import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import {
  Grid3X3,
  LayoutGrid,
  Layers,
  Maximize2,
  Minus,
  Moon,
  Plus,
  Ruler,
  Square,
  Sun,
} from 'lucide-react';
import { BookStageCanvas } from './components/BookStageCanvas';
import { BottomAnimationDrawer } from './components/playground/BottomAnimationDrawer';
import { ModularWorkspaceOverlay } from './components/playground/ModularWorkspaceOverlay';
import { PoseStreamControl } from './components/PoseStreamControl';
import { JOINT_SPEC_MAP } from './data/digitalRig';
import { GridConfig, PageId } from './types/canvas';
import {
  ArmatureState,
  HandGesturePreset,
  JointId,
  JointMode,
  KeyframeSlot,
  MotionCurve,
  PhysicsModeId,
  PosePresetId,
  RigSystemsConfig,
  ToggleHingeFoldState,
  UserSavedPose,
} from './types/rig';
import {
  BottomDrawerTab,
  ModelModuleId,
  WORKSPACE_LAYOUT_STORAGE_KEY,
  WorkspaceLayoutConfig,
  createDefaultWorkspaceLayout,
} from './types/workspace';
import {
  CompositeFilter,
  SamplePlateId,
  Scene,
  SceneMedia,
  ScenePlaybackMode,
  StageTransform,
  WorldPin,
} from './types/scene';
import { ExportIRFormat } from './types/exportIR';
import {
  COLLAR_INWARD_NORMAL_LIMIT_DEG,
  COLLAR_SWIVEL_MAX_INWARD_DEG,
  DEFAULT_POSE_TRANSITION_MS,
  DEFAULT_TOGGLE_TRANSITION_MS,
  applyJointAngleConstraint,
  computeArmature,
  computePoseTransitionFrame,
  createPosePreset,
  getPhysicsMultiplier,
  hasArmaturePoseDelta,
  interpolateArmatures,
  mirrorPose,
  resolveCollarInwardBehavior,
  setBalancedSpineLean,
  setCollarToggleState,
  setHipToggleState,
  synthesizeProceduralBreakdown,
  toggleArmReverseSwivel,
} from './utils/kinematics';
import {
  PAPER_HEIGHT,
  PAPER_WIDTH,
  PYXL_PX,
  computeProportionalCanons,
  computeSymmetricalCenterPan,
  computeSymmetricalFitZoom,
  formatSignedNum,
} from './utils/proportions';
import {
  DEFAULT_SCENE_DURATION_SEC,
  DEFAULT_STAGE_TRANSFORM,
  SAMPLE_PLATES,
  advanceSequencePlayback,
  cloneSceneForNewShot,
  createShotBridgeSlots,
  createDefaultScene,
  createDefaultSceneMedia,
  createSamplePlateMedia,
  createWorldPinForJoint,
  solveWorldPins,
} from './utils/scene';
import {
  DEFAULT_SCREEN_FPS,
  ScreenFpsOption,
  getNextFpsOption,
  shouldEmitScreenFrame,
} from './utils/frameThrottle';
import {
  adaptIRToCanonicalJsonBlob,
  adaptIRToDragonBonesJsonBlob,
  adaptIRToLayeredSvgBlob,
  adaptIRToSpineJsonBlob,
  buildExportIR,
} from './utils/exportAdapters';
import {
  copyPoseDataToClipboard,
  downloadExportBlob,
  formatExportTimestamp,
  savePoseDataFile,
} from './utils/poseExport';
import { buildPoseTelemetrySnapshot } from './utils/poseTelemetry';

// Load directly with the Cover state as the active canvas backdrop
const INITIAL_PAGE: PageId = 'page-22';
const SAVED_POSES_STORAGE_KEY = 'bitruvius.savedPoses.v1';
const POSE_STREAM_WORKSPACE_ID = 'bitruvius-ink-manual-kinematics-studio';
const CANVAS_ORIGIN = { x: PAPER_WIDTH / 2, y: PAPER_HEIGHT / 2 };

function loadSavedPoses(): UserSavedPose[] {
  try {
    const raw = window.localStorage.getItem(SAVED_POSES_STORAGE_KEY);
    if (!raw) return [];
    const parsed = JSON.parse(raw);
    return Array.isArray(parsed) ? parsed : [];
  } catch {
    return [];
  }
}

function loadWorkspaceLayout(): WorkspaceLayoutConfig {
  const fallback = createDefaultWorkspaceLayout();
  try {
    const raw = window.localStorage.getItem(WORKSPACE_LAYOUT_STORAGE_KEY);
    if (!raw) return fallback;
    const parsed = JSON.parse(raw) as Partial<WorkspaceLayoutConfig>;
    const panels = { ...fallback.panels };
    if (parsed.panels) {
      (['rig_joint', 'digits', 'perspective', 'scene'] as ModelModuleId[]).forEach((id) => {
        const entry = parsed.panels?.[id];
        if (entry && typeof entry === 'object') {
          panels[id] = {
            isOpen: Boolean(entry.isOpen),
            isCollapsed: Boolean(entry.isCollapsed),
            dockMode: entry.dockMode === 'floating' ? 'floating' : 'docked',
            position:
              entry.position &&
              typeof entry.position.x === 'number' &&
              typeof entry.position.y === 'number'
                ? entry.position
                : fallback.panels[id].position,
            zIndex: typeof entry.zIndex === 'number' ? entry.zIndex : fallback.panels[id].zIndex,
          };
        }
      });
    }
    return {
      railVisible: typeof parsed.railVisible === 'boolean' ? parsed.railVisible : true,
      bottomDrawerOpen:
        typeof parsed.bottomDrawerOpen === 'boolean' ? parsed.bottomDrawerOpen : false,
      bottomDrawerTab:
        parsed.bottomDrawerTab === 'poses' || parsed.bottomDrawerTab === 'animation'
          ? parsed.bottomDrawerTab
          : 'timeline',
      panels,
    };
  } catch {
    return fallback;
  }
}

export default function App() {
  const [pageId] = useState<PageId>(INITIAL_PAGE);
  const [darkMode, setDarkMode] = useState(false);
  const [screenFps, setScreenFps] = useState<ScreenFpsOption>(DEFAULT_SCREEN_FPS);
  const [viewportSize, setViewportSize] = useState({ width: 1440, height: 780 });
  const viewportSizeRef = useRef(viewportSize);
  viewportSizeRef.current = viewportSize;
  const cursorXRef = useRef<HTMLSpanElement | null>(null);
  const cursorYRef = useRef<HTMLSpanElement | null>(null);
  const handleCursorMove = useCallback((coords: { x: number; y: number } | null) => {
    if (cursorXRef.current) {
      cursorXRef.current.textContent = coords
        ? `${formatSignedNum(coords.x / PYXL_PX)} pyx`
        : '+0.00 pyx';
    }
    if (cursorYRef.current) {
      cursorYRef.current.textContent = coords
        ? `${formatSignedNum(coords.y / PYXL_PX)} pyx`
        : '+0.00 pyx';
    }
  }, []);
  const [grid, setGrid] = useState<GridConfig>({
    showSubpyxl: true,
    showPyxl: true,
    showMegapyxl: true,
    showVitruvianGuide: true,
    showWorkingSquare: true,
    showEightHeadGuide: true,
    showRulers: true,
    inkStrength: 'light',
  });

  // Decoupled Scene architecture: Scene[0] preserves initial Back Cover workspace
  const [scenes, setScenes] = useState<Scene[]>(() => [
    createDefaultScene(0, { width: 1440, height: 780 }),
  ]);
  const [activeSceneId, setActiveSceneId] = useState<string>(() => '');
  const resolvedActiveScene = useMemo(
    () => scenes.find((s) => s.id === activeSceneId) ?? scenes[0],
    [activeSceneId, scenes]
  );

  // Sync activeSceneId if initial empty or deleted
  useEffect(() => {
    if (!scenes.some((s) => s.id === activeSceneId) && scenes[0]) {
      setActiveSceneId(scenes[0].id);
    }
  }, [activeSceneId, scenes]);

  const scenesRef = useRef(scenes);
  scenesRef.current = scenes;
  const activeSceneRef = useRef(resolvedActiveScene);
  activeSceneRef.current = resolvedActiveScene;

  const armature = resolvedActiveScene.armature;
  const activePreset = resolvedActiveScene.activePreset;
  const slots = resolvedActiveScene.slots;
  const activeSlot = resolvedActiveScene.activeSlot;
  const timelineT = resolvedActiveScene.timelineT;
  const motionCurve = resolvedActiveScene.motionCurve;
  const playbackSpeed = resolvedActiveScene.playbackSpeed;
  const showPerspective = resolvedActiveScene.showPerspective;
  const vanishingPoint = resolvedActiveScene.vanishingPoint;
  const zoom = resolvedActiveScene.camera.zoom;
  const pan = resolvedActiveScene.camera.pan;

  const [selectedJointId, setSelectedJointId] = useState<JointId>('humerusR');
  const [isPoseTransitioning, setIsPoseTransitioning] = useState(false);
  const historyRef = useRef<ArmatureState[]>([]);
  const screenFpsRef = useRef<ScreenFpsOption>(screenFps);
  screenFpsRef.current = screenFps;
  const poseTransitionRef = useRef<{
    sceneId: string;
    from: ArmatureState;
    to: ArmatureState;
    startMs: number;
    durationMs: number;
    curve: MotionCurve;
    lastEmitMs: number;
    rafId: number;
  } | null>(null);

  // Modular Workspace & Collapsible Bottom Drawer State
  const [workspaceLayout, setWorkspaceLayout] = useState<WorkspaceLayoutConfig>(() =>
    loadWorkspaceLayout()
  );
  const nextZRef = useRef(40);
  const [playbackMode, setPlaybackMode] = useState<ScenePlaybackMode>('scene');
  const [isPlaying, setIsPlaying] = useState(false);
  const [rigSystems, setRigSystems] = useState<RigSystemsConfig>(() => {
    const canons = computeProportionalCanons();
    return {
      softLimits: false,
      floorSnap: false,
      elasticPin: false,
      antiPhallic: true,
      groundY: PAPER_HEIGHT / 2 + canons.eightHeadTotalPx / 2,
      solver: 'analytical',
      showSwivelsAndJoints: true,
    };
  });

  const [savedPoses, setSavedPoses] = useState<UserSavedPose[]>(() => loadSavedPoses());

  useEffect(() => {
    try {
      window.localStorage.setItem(SAVED_POSES_STORAGE_KEY, JSON.stringify(savedPoses));
    } catch {
      // Ignore storage quota errors
    }
  }, [savedPoses]);

  useEffect(() => {
    try {
      window.localStorage.setItem(WORKSPACE_LAYOUT_STORAGE_KEY, JSON.stringify(workspaceLayout));
    } catch {
      // Ignore storage quota errors
    }
  }, [workspaceLayout]);

  useEffect(() => {
    document.documentElement.classList.toggle('dark', darkMode);
  }, [darkMode]);

  const computedJoints = useMemo(
    () => computeArmature(armature, CANVAS_ORIGIN),
    [armature]
  );
  const poseTelemetry = useMemo(
    () =>
      buildPoseTelemetrySnapshot({
        workspaceId: POSE_STREAM_WORKSPACE_ID,
        scene: {
          id: resolvedActiveScene.id,
          name: resolvedActiveScene.name,
          timelineT: resolvedActiveScene.timelineT,
          durationSec: resolvedActiveScene.durationSec,
          playbackSpeed: resolvedActiveScene.playbackSpeed,
        },
        armature,
        computedJoints,
      }),
    [
      armature,
      computedJoints,
      resolvedActiveScene.durationSec,
      resolvedActiveScene.id,
      resolvedActiveScene.name,
      resolvedActiveScene.playbackSpeed,
      resolvedActiveScene.timelineT,
    ]
  );

  const updateActiveScene = useCallback((updater: (scene: Scene) => Scene) => {
    setScenes((prev) => {
      const targetId = activeSceneRef.current?.id;
      return prev.map((s) => (s.id === targetId ? updater(s) : s));
    });
  }, []);

  const applyFloorSnapIfNeeded = useCallback(
    (candidate: ArmatureState): ArmatureState => {
      if (!rigSystems.floorSnap || candidate.isAnchorPinned) return candidate;
      const joints = computeArmature(candidate, CANVAS_ORIGIN);
      const groundY = rigSystems.pressurePlateY ?? rigSystems.groundY;
      const maxFootY = Math.max(
        joints.footR.endY,
        joints.footL.endY,
        joints.toeR.endY,
        joints.toeL.endY
      );
      if (maxFootY > groundY) {
        return {
          ...candidate,
          rootY: candidate.rootY - (maxFootY - groundY),
        };
      }
      return candidate;
    },
    [rigSystems.floorSnap, rigSystems.groundY, rigSystems.pressurePlateY]
  );

  const applySceneConstraints = useCallback(
    (candidate: ArmatureState, scene: Scene, tOverride?: number): ArmatureState => {
      let next = applyFloorSnapIfNeeded(candidate);
      if (scene.worldPins.length > 0) {
        next = solveWorldPins(
          next,
          scene.worldPins,
          tOverride ?? scene.timelineT,
          scene.stageTransform,
          CANVAS_ORIGIN,
          rigSystems
        );
      }
      return next;
    },
    [applyFloorSnapIfNeeded, rigSystems]
  );

  const fitSymmetricalView = useCallback(
    (size = viewportSize) => {
      const nextZoom = computeSymmetricalFitZoom(size.width, size.height);
      const nextPan = computeSymmetricalCenterPan(
        size.width,
        size.height,
        CANVAS_ORIGIN.x,
        CANVAS_ORIGIN.y,
        nextZoom
      );
      updateActiveScene((s) => ({
        ...s,
        camera: { zoom: nextZoom, pan: nextPan, userEdited: false },
      }));
    },
    [updateActiveScene, viewportSize]
  );

  const handleViewportResize = useCallback((nextSize: { width: number; height: number }) => {
    const prevSize = viewportSizeRef.current;
    if (prevSize.width === nextSize.width && prevSize.height === nextSize.height) return;
    viewportSizeRef.current = nextSize;
    setViewportSize(nextSize);
    setScenes((prev) =>
      prev.map((scene) => {
        if (!scene.camera.userEdited) {
          const nextZoom = computeSymmetricalFitZoom(nextSize.width, nextSize.height);
          return {
            ...scene,
            camera: {
              zoom: nextZoom,
              pan: computeSymmetricalCenterPan(
                nextSize.width,
                nextSize.height,
                CANVAS_ORIGIN.x,
                CANVAS_ORIGIN.y,
                nextZoom
              ),
              userEdited: false,
            },
          };
        }
        return {
          ...scene,
          camera: {
            ...scene.camera,
            pan: {
              x: scene.camera.pan.x + (nextSize.width - prevSize.width) / 2,
              y: scene.camera.pan.y + (nextSize.height - prevSize.height) / 2,
            },
          },
        };
      })
    );
  }, []);

  const handleZoomChange = useCallback(
    (nextZoom: number) => {
      updateActiveScene((s) => ({
        ...s,
        camera: { ...s.camera, zoom: nextZoom, userEdited: true },
      }));
    },
    [updateActiveScene]
  );

  const handlePanChange = useCallback(
    (nextPan: { x: number; y: number }) => {
      updateActiveScene((s) => ({
        ...s,
        camera: { ...s.camera, pan: nextPan, userEdited: true },
      }));
    },
    [updateActiveScene]
  );

  const handleZoomStep = useCallback(
    (factor: number) => {
      const curZoom = activeSceneRef.current.camera.zoom;
      const curPan = activeSceneRef.current.camera.pan;
      const minZoom = computeSymmetricalFitZoom(viewportSize.width, viewportSize.height);
      const nextZoom = Math.max(minZoom, Math.min(2.4, curZoom * factor));
      const cx = viewportSize.width / 2;
      const cy = viewportSize.height / 2;
      const worldX = (cx - curPan.x) / curZoom;
      const worldY = (cy - curPan.y) / curZoom;
      updateActiveScene((s) => ({
        ...s,
        camera: {
          zoom: nextZoom,
          pan: {
            x: cx - worldX * nextZoom,
            y: cy - worldY * nextZoom,
          },
          userEdited: true,
        },
      }));
    },
    [updateActiveScene, viewportSize.height, viewportSize.width]
  );

  const handleSetExactZoom = useCallback(
    (targetZoom: number) => {
      const minZoom = computeSymmetricalFitZoom(viewportSize.width, viewportSize.height);
      const nextZoom = Math.max(minZoom, Math.min(2.4, targetZoom));
      updateActiveScene((s) => ({
        ...s,
        camera: {
          zoom: nextZoom,
          pan: computeSymmetricalCenterPan(
            viewportSize.width,
            viewportSize.height,
            CANVAS_ORIGIN.x,
            CANVAS_ORIGIN.y,
            nextZoom
          ),
          userEdited: true,
        },
      }));
    },
    [updateActiveScene, viewportSize.height, viewportSize.width]
  );

  const handleFitClick = useCallback(() => {
    fitSymmetricalView(viewportSize);
  }, [fitSymmetricalView, viewportSize]);

  const handleTurnPage = useCallback((_target: PageId) => {
    // Page turning deferred until page-turn controls are re-enabled
  }, []);

  const handleCycleInk = useCallback(() => {
    setGrid((prev) => ({
      ...prev,
      inkStrength:
        prev.inkStrength === 'whisper'
          ? 'light'
          : prev.inkStrength === 'light'
          ? 'draft'
          : 'whisper',
    }));
  }, []);

  const handleCycleFps = useCallback(() => {
    setScreenFps((prev) => getNextFpsOption(prev));
  }, []);

  const cancelPoseTransition = useCallback(() => {
    if (poseTransitionRef.current) {
      cancelAnimationFrame(poseTransitionRef.current.rafId);
      poseTransitionRef.current = null;
      setIsPoseTransitioning(false);
    }
  }, []);

  useEffect(() => () => cancelPoseTransition(), [cancelPoseTransition]);

  const commitPoseHistory = useCallback(() => {
    const curScene = activeSceneRef.current;
    const pending = poseTransitionRef.current;
    const snapshot =
      pending && pending.sceneId === curScene?.id ? pending.to : curScene.armature;
    historyRef.current = [...historyRef.current.slice(-24), snapshot];
  }, []);

  const animateSceneArmature = useCallback(
    (
      buildNext: (baseArmature: ArmatureState, scene: Scene) => ArmatureState,
      options?: {
        durationMs?: number;
        curve?: MotionCurve;
        scenePatch?: Partial<Omit<Scene, 'armature'>>;
        tOverride?: number;
      }
    ) => {
      const curScene = activeSceneRef.current;
      if (!curScene) return;

      const pending = poseTransitionRef.current;
      const baseArmature =
        pending && pending.sceneId === curScene.id ? pending.to : curScene.armature;
      const fromArmature = curScene.armature;
      const rawTarget = buildNext(baseArmature, curScene);
      const targetArmature = applySceneConstraints(rawTarget, curScene, options?.tOverride);
      const durationMs = options?.durationMs ?? DEFAULT_POSE_TRANSITION_MS;
      const curve = options?.curve ?? curScene.motionCurve ?? 'flow';
      const scenePatch = options?.scenePatch;

      if (durationMs <= 0 || !hasArmaturePoseDelta(fromArmature, targetArmature)) {
        cancelPoseTransition();
        updateActiveScene((s) => ({
          ...s,
          ...(scenePatch ?? {}),
          armature: targetArmature,
        }));
        return;
      }

      if (poseTransitionRef.current) {
        cancelAnimationFrame(poseTransitionRef.current.rafId);
        poseTransitionRef.current = null;
      }

      if (scenePatch) {
        updateActiveScene((s) => ({
          ...s,
          ...scenePatch,
        }));
      }

      setIsPoseTransitioning(true);
      const transition = {
        sceneId: curScene.id,
        from: fromArmature,
        to: targetArmature,
        startMs: performance.now(),
        durationMs,
        curve,
        lastEmitMs: -1,
        rafId: 0,
      };
      poseTransitionRef.current = transition;

      const step = (now: number) => {
        if (
          poseTransitionRef.current !== transition ||
          activeSceneRef.current?.id !== transition.sceneId
        ) {
          return;
        }
        const elapsedMs = now - transition.startMs;
        const done = transition.durationMs <= 0 || elapsedMs >= transition.durationMs;

        if (done || shouldEmitScreenFrame(transition.lastEmitMs, now, screenFpsRef.current)) {
          transition.lastEmitMs = now;
          const resolvedFrame = done
            ? transition.to
            : computePoseTransitionFrame(
                transition.from,
                transition.to,
                elapsedMs,
                transition.durationMs,
                transition.curve
              ).armature;
          setScenes((prev) =>
            prev.map((s) =>
              s.id === transition.sceneId ? { ...s, armature: resolvedFrame } : s
            )
          );
        }

        if (done) {
          poseTransitionRef.current = null;
          setIsPoseTransitioning(false);
        } else {
          transition.rafId = requestAnimationFrame(step);
        }
      };

      transition.rafId = requestAnimationFrame(step);
    },
    [applySceneConstraints, cancelPoseTransition, updateActiveScene]
  );

  const handleAnimateArmature = useCallback(
    (next: ArmatureState, durationMs: number = DEFAULT_TOGGLE_TRANSITION_MS) => {
      animateSceneArmature(() => next, {
        durationMs,
        scenePatch: { activePreset: null, activeSlot: null },
      });
    },
    [animateSceneArmature]
  );

  const handleUpdateArmature = useCallback(
    (next: ArmatureState) => {
      cancelPoseTransition();
      updateActiveScene((s) => ({
        ...s,
        activePreset: null,
        activeSlot: null,
        armature: applyFloorSnapIfNeeded(next),
      }));
    },
    [applyFloorSnapIfNeeded, cancelPoseTransition, updateActiveScene]
  );

  const handleApplyPreset = useCallback(
    (preset: PosePresetId) => {
      commitPoseHistory();
      setIsPlaying(false);
      animateSceneArmature(
        (base) =>
          createPosePreset(preset, {
            ...base,
            rootX: PAPER_WIDTH / 2,
            rootY: PAPER_HEIGHT / 2,
            isAnchorPinned: true,
          }),
        {
          durationMs: DEFAULT_POSE_TRANSITION_MS,
          scenePatch: { activePreset: preset, activeSlot: null },
        }
      );
    },
    [animateSceneArmature, commitPoseHistory]
  );

  const handleMirrorPose = useCallback(() => {
    commitPoseHistory();
    setIsPlaying(false);
    animateSceneArmature((base) => mirrorPose(base), {
      durationMs: DEFAULT_TOGGLE_TRANSITION_MS,
      scenePatch: { activePreset: null },
    });
  }, [animateSceneArmature, commitPoseHistory]);

  const handleResetPose = useCallback(() => {
    commitPoseHistory();
    setIsPlaying(false);
    animateSceneArmature(
      (base) =>
        createPosePreset('tpose', {
          ...base,
          rootX: PAPER_WIDTH / 2,
          rootY: PAPER_HEIGHT / 2,
          isAnchorPinned: true,
          hipToggleSign: 1,
        }),
      {
        durationMs: DEFAULT_POSE_TRANSITION_MS,
        scenePatch: { activePreset: 'tpose', activeSlot: null },
      }
    );
  }, [animateSceneArmature, commitPoseHistory]);

  const handleCopyPoseData = useCallback(() => {
    const current = activeSceneRef.current.armature;
    const joints = computeArmature(current, CANVAS_ORIGIN);
    return copyPoseDataToClipboard(current, joints, activeSceneRef.current.activePreset);
  }, []);

  const handleSavePoseData = useCallback(() => {
    const current = activeSceneRef.current.armature;
    const joints = computeArmature(current, CANVAS_ORIGIN);
    savePoseDataFile(current, joints, activeSceneRef.current.activePreset);
  }, []);

  const handleExportIRFormat = useCallback((format: ExportIRFormat) => {
    const curScene = activeSceneRef.current;
    if (!curScene) return;
    const now = new Date();
    const ir = buildExportIR({
      armature: curScene.armature,
      preset: curScene.activePreset,
      sceneId: curScene.id,
      sceneName: curScene.name,
      slots: curScene.slots,
      motionCurve: curScene.motionCurve,
      durationSec: curScene.durationSec,
      fps: screenFpsRef.current,
      canvasOriginPx: CANVAS_ORIGIN,
      exportedAt: now.toISOString(),
    });
    const stamp = formatExportTimestamp(now);
    if (format === 'ir_json') {
      downloadExportBlob(adaptIRToCanonicalJsonBlob(ir), `bitruvius-ir-${stamp}.json`);
    } else if (format === 'layered_svg') {
      downloadExportBlob(adaptIRToLayeredSvgBlob(ir), `bitruvius-layered-${stamp}.svg`);
    } else if (format === 'spine_json') {
      downloadExportBlob(adaptIRToSpineJsonBlob(ir), `bitruvius-spine-${stamp}.json`);
    } else if (format === 'dragonbones_json') {
      downloadExportBlob(adaptIRToDragonBonesJsonBlob(ir), `bitruvius-dragonbones-${stamp}.json`);
    }
  }, []);

  const handleNudgeSelectedJoint = useCallback(
    (deltaDeg: number) => {
      const curScene = activeSceneRef.current;
      const pending = poseTransitionRef.current;
      const baseRefArmature =
        pending && pending.sceneId === curScene?.id ? pending.to : curScene.armature;
      const targetId: JointId =
        selectedJointId === 'clavOuterR'
          ? 'braceR'
          : selectedJointId === 'clavOuterL'
          ? 'braceL'
          : selectedJointId;
      const spec = JOINT_SPEC_MAP[targetId];
      const runtime = baseRefArmature.joints[targetId];
      if (!spec || spec.locked || !runtime || runtime.mode === 'PIN') return;
      const mult = getPhysicsMultiplier(runtime);
      if (mult === 0) return;
      commitPoseHistory();
      setIsPlaying(false);

      if (targetId === 'pelvis') {
        animateSceneArmature(
          (base) => ({
            ...base,
            rootRotation: applyJointAngleConstraint(
              base.rootRotation + deltaDeg * mult,
              base.continuousSpin
            ),
          }),
          {
            durationMs: DEFAULT_TOGGLE_TRANSITION_MS,
            scenePatch: { activePreset: null },
          }
        );
      } else {
        const isCollarSwivel = targetId === 'swivelR' || targetId === 'swivelL';
        const isClosedTriangleSwivel =
          targetId === 'hipR' ||
          targetId === 'hipL' ||
          targetId === 'braceR' ||
          targetId === 'braceL';
        const limits = spec.limits;

        animateSceneArmature(
          (base) => {
            const prevAngle = base.joints[targetId].localAngle;
            const rawNext = applyJointAngleConstraint(
              prevAngle + deltaDeg * mult,
              base.continuousSpin
            );
            let nextAngle = rawNext;
            if (isCollarSwivel) {
              const side: 'R' | 'L' = targetId === 'swivelR' ? 'R' : 'L';
              const otherId: JointId = side === 'R' ? 'swivelL' : 'swivelR';
              const prevInward = side === 'R' ? prevAngle : -prevAngle;
              const nextInward = side === 'R' ? rawNext : -rawNext;
              const wasSnapped = prevInward > COLLAR_INWARD_NORMAL_LIMIT_DEG + 0.5;
              const nudgedOutward = nextInward < prevInward - 1e-4;
              if (wasSnapped && nudgedOutward) {
                nextAngle =
                  side === 'R'
                    ? COLLAR_INWARD_NORMAL_LIMIT_DEG
                    : -COLLAR_INWARD_NORMAL_LIMIT_DEG;
              } else {
                const clampedRaw =
                  side === 'R'
                    ? Math.max(
                        -COLLAR_INWARD_NORMAL_LIMIT_DEG,
                        Math.min(COLLAR_SWIVEL_MAX_INWARD_DEG, rawNext)
                      )
                    : Math.max(
                        -COLLAR_SWIVEL_MAX_INWARD_DEG,
                        Math.min(COLLAR_INWARD_NORMAL_LIMIT_DEG, rawNext)
                      );
                nextAngle = resolveCollarInwardBehavior(
                  side,
                  clampedRaw,
                  base.joints[otherId].localAngle
                ).effectiveLocalAngle;
              }
            } else if (isClosedTriangleSwivel && limits) {
              nextAngle = Math.max(limits[0], Math.min(limits[1], rawNext));
            }
            return {
              ...base,
              joints: {
                ...base.joints,
                [targetId]: {
                  ...base.joints[targetId],
                  localAngle: nextAngle,
                },
              },
            };
          },
          {
            durationMs: DEFAULT_TOGGLE_TRANSITION_MS,
            scenePatch: { activePreset: null },
          }
        );
      }
    },
    [animateSceneArmature, commitPoseHistory, selectedJointId]
  );

  const handleZeroSelectedJoint = useCallback(() => {
    const targetId: JointId =
      selectedJointId === 'clavOuterR'
        ? 'braceR'
        : selectedJointId === 'clavOuterL'
        ? 'braceL'
        : selectedJointId;
    const spec = JOINT_SPEC_MAP[targetId];
    if (!spec || spec.locked) return;
    commitPoseHistory();
    setIsPlaying(false);
    animateSceneArmature(
      (base) =>
        targetId === 'pelvis'
          ? { ...base, rootRotation: 0 }
          : {
              ...base,
              joints: {
                ...base.joints,
                [targetId]: { ...base.joints[targetId], localAngle: 0 },
              },
            },
      {
        durationMs: DEFAULT_TOGGLE_TRANSITION_MS,
        scenePatch: { activePreset: null },
      }
    );
  }, [animateSceneArmature, commitPoseHistory, selectedJointId]);

  // Evaluate scene timeline slice + enforce active WorldPins
  const evaluateSceneArmatureAt = useCallback(
    (scene: Scene, t: number): ArmatureState => {
      const a = scene.slots.A;
      const b = scene.slots.B;
      const c = scene.slots.C;
      let candidate = a ?? scene.armature;
      if (a) {
        if (b && c) {
          candidate =
            t <= 0.5
              ? interpolateArmatures(a, b, t * 2, scene.motionCurve)
              : interpolateArmatures(b, c, (t - 0.5) * 2, scene.motionCurve);
        } else if (c) {
          candidate = interpolateArmatures(a, c, t, scene.motionCurve);
        } else if (b) {
          candidate = interpolateArmatures(a, b, t, scene.motionCurve);
        }
      }
      return applySceneConstraints(candidate, scene, t);
    },
    [applySceneConstraints]
  );

  const handleScrubChange = useCallback(
    (nextT: number) => {
      cancelPoseTransition();
      setIsPlaying(false);
      updateActiveScene((s) => ({
        ...s,
        activeSlot: null,
        activePreset: null,
        timelineT: nextT,
        armature: evaluateSceneArmatureAt(s, nextT),
      }));
    },
    [cancelPoseTransition, evaluateSceneArmatureAt, updateActiveScene]
  );

  // Playback RAF supporting both single-shot loops and multi-scene sequence advancement (throttled to screenFps)
  useEffect(() => {
    if (!isPlaying) return;
    cancelPoseTransition();
    let rafId = 0;
    let lastEmitMs = -1;

    const step = (now: number) => {
      if (shouldEmitScreenFrame(lastEmitMs, now, screenFpsRef.current)) {
        const dt =
          lastEmitMs < 0 ? 1 / screenFpsRef.current : Math.min(0.1, (now - lastEmitMs) / 1000);
        lastEmitMs = now;

        const curScenes = scenesRef.current;
        const curScene = activeSceneRef.current;
        if (curScene) {
          const { nextSceneId, nextT } = advanceSequencePlayback(
            curScenes,
            curScene.id,
            curScene.timelineT,
            dt,
            curScene.playbackSpeed,
            playbackMode
          );
          if (nextSceneId !== curScene.id) {
            setActiveSceneId(nextSceneId);
          }
          setScenes((prev) =>
            prev.map((s) =>
              s.id === nextSceneId
                ? {
                    ...s,
                    timelineT: nextT,
                    activePreset: null,
                    activeSlot: null,
                    armature: evaluateSceneArmatureAt(s, nextT),
                  }
                : s
            )
          );
        }
      }
      rafId = requestAnimationFrame(step);
    };

    rafId = requestAnimationFrame(step);
    return () => cancelAnimationFrame(rafId);
  }, [cancelPoseTransition, evaluateSceneArmatureAt, isPlaying, playbackMode]);

  // Playground Studio & Timeline Handlers
  const handleCaptureSlot = useCallback(
    (slot: KeyframeSlot) => {
      updateActiveScene((s) => ({
        ...s,
        activeSlot: slot,
        slots: { ...s.slots, [slot]: structuredClone(s.armature) },
      }));
    },
    [updateActiveScene]
  );

  const handleJumpToSlot = useCallback(
    (slot: KeyframeSlot) => {
      const pose = activeSceneRef.current.slots[slot];
      if (!pose) return;
      commitPoseHistory();
      setIsPlaying(false);
      const nextT = slot === 'A' ? 0 : slot === 'B' ? 0.5 : 1;
      animateSceneArmature(() => structuredClone(pose), {
        durationMs: DEFAULT_POSE_TRANSITION_MS,
        tOverride: nextT,
        scenePatch: { activeSlot: slot, activePreset: null, timelineT: nextT },
      });
    },
    [animateSceneArmature, commitPoseHistory]
  );

  const handleClearSlot = useCallback(
    (slot: KeyframeSlot) => {
      setIsPlaying(false);
      updateActiveScene((s) => ({
        ...s,
        activeSlot: s.activeSlot === slot ? null : s.activeSlot,
        slots: { ...s.slots, [slot]: null },
      }));
    },
    [updateActiveScene]
  );

  const handleSynthesizeBreakdown = useCallback(() => {
    const s = activeSceneRef.current;
    if (!s.slots.A || !s.slots.C) return;
    commitPoseHistory();
    setIsPlaying(false);
    const synth = synthesizeProceduralBreakdown(s.slots.A, s.slots.C);
    animateSceneArmature(() => synth, {
      durationMs: DEFAULT_POSE_TRANSITION_MS,
      tOverride: 0.5,
      scenePatch: {
        slots: { ...s.slots, B: synth },
        activeSlot: 'B',
        activePreset: null,
        timelineT: 0.5,
      },
    });
  }, [animateSceneArmature, commitPoseHistory]);

  const handleSaveUserPose = useCallback((name: string) => {
    const newEntry: UserSavedPose = {
      id: `pose-${Date.now()}-${Math.random().toString(36).slice(2, 6)}`,
      name,
      timestamp: Date.now(),
      armature: structuredClone(activeSceneRef.current.armature),
    };
    setSavedPoses((prev) => [newEntry, ...prev]);
  }, []);

  const handleApplyUserPose = useCallback(
    (pose: UserSavedPose) => {
      commitPoseHistory();
      setIsPlaying(false);
      animateSceneArmature(() => structuredClone(pose.armature), {
        durationMs: DEFAULT_POSE_TRANSITION_MS,
        scenePatch: { activePreset: null, activeSlot: null },
      });
    },
    [animateSceneArmature, commitPoseHistory]
  );

  const handleOverwriteUserPose = useCallback((id: string) => {
    const snap = structuredClone(activeSceneRef.current.armature);
    setSavedPoses((prev) =>
      prev.map((entry) =>
        entry.id === id ? { ...entry, timestamp: Date.now(), armature: snap } : entry
      )
    );
  }, []);

  const handleDeleteUserPose = useCallback((id: string) => {
    setSavedPoses((prev) => prev.filter((entry) => entry.id !== id));
  }, []);

  const handleLoadUserPoseToSlot = useCallback(
    (pose: UserSavedPose, slot: KeyframeSlot) => {
      updateActiveScene((s) => ({
        ...s,
        slots: { ...s.slots, [slot]: structuredClone(pose.armature) },
      }));
    },
    [updateActiveScene]
  );

  const handleBatchSetIK = useCallback(
    (enableAllLimbs: boolean) => {
      commitPoseHistory();
      updateActiveScene((s) => {
        const nextJoints = { ...s.armature.joints };
        const ikTargets: JointId[] = [
          'thorax',
          'neck',
          'head',
          'forearmR',
          'handR',
          'fingerR',
          'thumbR',
          'forearmL',
          'handL',
          'fingerL',
          'thumbL',
          'shinR',
          'footR',
          'toeR',
          'shinL',
          'footL',
          'toeL',
        ];
        for (const id of ikTargets) {
          if (nextJoints[id]) {
            nextJoints[id] = {
              ...nextJoints[id],
              mode: enableAllLimbs ? 'IK' : JOINT_SPEC_MAP[id]?.defaultMode ?? 'FK',
            };
          }
        }
        return { ...s, armature: { ...s.armature, joints: nextJoints } };
      });
    },
    [commitPoseHistory, updateActiveScene]
  );

  const handleSetJointMode = useCallback(
    (id: JointId, mode: JointMode) => {
      updateActiveScene((s) => ({
        ...s,
        armature: {
          ...s.armature,
          joints: { ...s.armature.joints, [id]: { ...s.armature.joints[id], mode } },
        },
      }));
    },
    [updateActiveScene]
  );

  const handleSetJointPhysics = useCallback(
    (id: JointId, physicsMode: PhysicsModeId) => {
      updateActiveScene((s) => ({
        ...s,
        armature: {
          ...s.armature,
          joints: {
            ...s.armature.joints,
            [id]: { ...s.armature.joints[id], physicsMode },
          },
        },
      }));
    },
    [updateActiveScene]
  );

  const handleSetJointAngle = useCallback(
    (id: JointId, localAngle: number) => {
      updateActiveScene((s) => ({
        ...s,
        activePreset: null,
        armature:
          id === 'lumbar' || id === 'thorax' || id === 'neck'
            ? setBalancedSpineLean(s.armature, id, localAngle)
            : id === 'pelvis'
            ? { ...s.armature, rootRotation: localAngle }
            : {
                ...s.armature,
                joints: {
                  ...s.armature.joints,
                  [id]: { ...s.armature.joints[id], localAngle },
                },
              },
      }));
    },
    [updateActiveScene]
  );

  const handleSetJointScale = useCallback(
    (id: JointId, scaleLength: number, scaleWidth: number) => {
      updateActiveScene((s) => ({
        ...s,
        armature: {
          ...s.armature,
          joints: {
            ...s.armature.joints,
            [id]: { ...s.armature.joints[id], scaleLength, scaleWidth },
          },
        },
      }));
    },
    [updateActiveScene]
  );

  const handleApplyHandGesture = useCallback(
    (side: 'right' | 'left' | 'both', gesture: HandGesturePreset) => {
      commitPoseHistory();
      setIsPlaying(false);
      const angles: Record<
        HandGesturePreset,
        { hand: number; thumb: number; finger: number }
      > = {
        open: { hand: 0, thumb: -14, finger: 0 },
        relaxed: { hand: 8, thumb: 14, finger: 24 },
        point: { hand: -4, thumb: -42, finger: 8 },
        pinch: { hand: 14, thumb: -48, finger: 54 },
        grasp: { hand: 12, thumb: -56, finger: 72 },
        fist: { hand: 10, thumb: -68, finger: 92 },
      };
      const p = angles[gesture];

      animateSceneArmature(
        (base) => {
          const nextJoints = { ...base.joints };
          if (side === 'right' || side === 'both') {
            nextJoints.handR = { ...nextJoints.handR, localAngle: p.hand };
            nextJoints.thumbR = { ...nextJoints.thumbR, localAngle: p.thumb };
            nextJoints.fingerR = { ...nextJoints.fingerR, localAngle: p.finger };
          }
          if (side === 'left' || side === 'both') {
            nextJoints.handL = { ...nextJoints.handL, localAngle: -p.hand };
            nextJoints.thumbL = { ...nextJoints.thumbL, localAngle: -p.thumb };
            nextJoints.fingerL = { ...nextJoints.fingerL, localAngle: -p.finger };
          }
          return { ...base, joints: nextJoints };
        },
        {
          durationMs: DEFAULT_TOGGLE_TRANSITION_MS,
          scenePatch: { activePreset: null },
        }
      );
    },
    [animateSceneArmature, commitPoseHistory]
  );

  // Scene Sequencer & Media Compositing Handlers
  const handleSelectScene = useCallback(
    (sceneId: string, clickCount = 1) => {
      cancelPoseTransition();
      setIsPlaying(false);
      setActiveSceneId(sceneId);
      const currentScenes = scenesRef.current;
      const sceneIndex = currentScenes.findIndex((scene) => scene.id === sceneId);
      const selectedScene = currentScenes[sceneIndex];
      if (!selectedScene) return;

      if (clickCount >= 3) {
        const tPose = createPosePreset('tpose', selectedScene.armature);
        setScenes((prev) =>
          prev.map((scene) =>
            scene.id === sceneId
              ? {
                  ...scene,
                  slots: { ...scene.slots, A: tPose },
                  activeSlot: null,
                  activePreset: null,
                }
              : scene
          )
        );
        return;
      }

      if (clickCount === 2 && sceneIndex > 0) {
        const previousScene = currentScenes[sceneIndex - 1];
        const previousEndPose = evaluateSceneArmatureAt(previousScene, 1);
        const nextScene = currentScenes[sceneIndex + 1];
        const slots = nextScene
          ? createShotBridgeSlots(
              previousEndPose,
              evaluateSceneArmatureAt(nextScene, 0),
              selectedScene.motionCurve
            )
          : { ...selectedScene.slots, A: structuredClone(previousEndPose) };
        setScenes((prev) =>
          prev.map((scene) =>
            scene.id === sceneId
              ? { ...scene, slots, activeSlot: null, activePreset: null }
              : scene
          )
        );
      }
    },
    [cancelPoseTransition, evaluateSceneArmatureAt]
  );

  const handleAddScene = useCallback(() => {
    cancelPoseTransition();
    setIsPlaying(false);
    const nextIdx = scenesRef.current.length;
    const newScene = createDefaultScene(nextIdx, viewportSize);
    setScenes((prev) => [...prev, newScene]);
    setActiveSceneId(newScene.id);
  }, [cancelPoseTransition, viewportSize]);

  const handleDuplicateScene = useCallback(
    (sceneId: string) => {
      const base = scenesRef.current.find((s) => s.id === sceneId);
      if (!base) return;
      cancelPoseTransition();
      setIsPlaying(false);
      const copy = cloneSceneForNewShot(base, scenesRef.current.length);
      setScenes((prev) => [...prev, copy]);
      setActiveSceneId(copy.id);
    },
    [cancelPoseTransition]
  );

  const handleDeleteScene = useCallback(
    (sceneId: string) => {
      if (scenesRef.current.length <= 1) return;
      cancelPoseTransition();
      setIsPlaying(false);
      setScenes((prev) => {
        const idx = prev.findIndex((s) => s.id === sceneId);
        const filtered = prev.filter((s) => s.id !== sceneId);
        if (activeSceneRef.current?.id === sceneId && filtered.length > 0) {
          const nextActive = filtered[Math.max(0, idx - 1)];
          setActiveSceneId(nextActive.id);
        }
        return filtered;
      });
    },
    [cancelPoseTransition]
  );

  const handleSelectPaperPlate = useCallback(
    (variant: 'cover' | 'backing') => {
      const baseMedia = createDefaultSceneMedia();
      updateActiveScene((s) => ({
        ...s,
        media: {
          ...baseMedia,
          paperVariant: variant,
          src:
            variant === 'backing'
              ? '/assets/manual/page_21_paper.png'
              : '/assets/manual/page_22_cover.png',
          name: variant === 'backing' ? 'Soft Archival Paper' : '1901 Manual Cover',
        },
        stageTransform: { ...DEFAULT_STAGE_TRANSFORM },
      }));
    },
    [updateActiveScene]
  );

  const handleSelectSamplePlate = useCallback(
    (sampleId: SamplePlateId) => {
      const plate = SAMPLE_PLATES[sampleId];
      const nextMedia = createSamplePlateMedia(sampleId);
      const nextTransform = { ...plate.recommendedTransform };
      commitPoseHistory();
      setIsPlaying(false);
      animateSceneArmature(
        (base, s) =>
          solveWorldPins(
            createPosePreset(plate.recommendedPreset, base),
            s.worldPins,
            s.timelineT,
            nextTransform,
            CANVAS_ORIGIN,
            rigSystems
          ),
        {
          durationMs: DEFAULT_POSE_TRANSITION_MS,
          scenePatch: {
            media: nextMedia,
            stageTransform: nextTransform,
            activePreset: plate.recommendedPreset,
            activeSlot: null,
          },
        }
      );
    },
    [animateSceneArmature, commitPoseHistory, rigSystems]
  );

  const handleUploadMediaFile = useCallback(
    (file: File) => {
      const url = URL.createObjectURL(file);
      const isVideo = file.type.startsWith('video/');
      const sceneId = activeSceneRef.current.id;

      if (isVideo) {
        const probe = document.createElement('video');
        probe.preload = 'metadata';
        probe.src = url;
        probe.onloadedmetadata = () => {
          const media: SceneMedia = {
            kind: 'video',
            src: url,
            name: file.name,
            duration: Number.isFinite(probe.duration) && probe.duration > 0 ? probe.duration : DEFAULT_SCENE_DURATION_SEC,
            fps: screenFps,
            naturalWidth: probe.videoWidth || 1280,
            naturalHeight: probe.videoHeight || 720,
            loop: true,
            playbackOffset: 0,
          };
          setScenes((prev) =>
            prev.map((s) =>
              s.id === sceneId
                ? {
                    ...s,
                    media,
                    durationSec: Math.max(1, Math.min(12, Math.round(media.duration * 2) / 2)),
                  }
                : s
            )
          );
        };
        probe.onerror = () => {
          setScenes((prev) =>
            prev.map((s) =>
              s.id === sceneId
                ? {
                    ...s,
                    media: {
                      kind: 'video',
                      src: url,
                      name: file.name,
                      duration: DEFAULT_SCENE_DURATION_SEC,
                      fps: screenFps,
                      naturalWidth: 1280,
                      naturalHeight: 720,
                      loop: true,
                      playbackOffset: 0,
                    },
                  }
                : s
            )
          );
        };
      } else {
        const img = new Image();
        img.onload = () => {
          setScenes((prev) =>
            prev.map((s) =>
              s.id === sceneId
                ? {
                    ...s,
                    media: {
                      kind: 'image',
                      src: url,
                      name: file.name,
                      duration: s.durationSec,
                      fps: screenFps,
                      naturalWidth: img.naturalWidth || 1600,
                      naturalHeight: img.naturalHeight || 1200,
                      loop: true,
                      playbackOffset: 0,
                    },
                  }
                : s
            )
          );
        };
        img.src = url;
      }
    },
    [screenFps]
  );

  const handleUpdateStageTransform = useCallback(
    (patch: Partial<StageTransform>) => {
      updateActiveScene((s) => {
        const nextTransform = { ...s.stageTransform, ...patch };
        const nextArmature =
          s.worldPins.length > 0
            ? solveWorldPins(
                s.armature,
                s.worldPins,
                s.timelineT,
                nextTransform,
                CANVAS_ORIGIN,
                rigSystems
              )
            : s.armature;
        return {
          ...s,
          stageTransform: nextTransform,
          armature: nextArmature,
        };
      });
    },
    [rigSystems, updateActiveScene]
  );

  const handleResetStageTransform = useCallback(() => {
    handleUpdateStageTransform({ ...DEFAULT_STAGE_TRANSFORM });
  }, [handleUpdateStageTransform]);

  const handleUpdateCompositeFilter = useCallback(
    (patch: Partial<CompositeFilter>) => {
      updateActiveScene((s) => ({
        ...s,
        compositeFilter: { ...s.compositeFilter, ...patch },
      }));
    },
    [updateActiveScene]
  );

  const handleToggleWorldPinForJoint = useCallback(
    (jointId: JointId) => {
      updateActiveScene((s) => {
        const exists = s.worldPins.some((p) => p.jointId === jointId);
        if (exists) {
          return {
            ...s,
            worldPins: s.worldPins.filter((p) => p.jointId !== jointId),
          };
        }
        const pin = createWorldPinForJoint(
          s.armature,
          jointId,
          s.stageTransform,
          CANVAS_ORIGIN
        );
        return {
          ...s,
          worldPins: [...s.worldPins, pin],
        };
      });
    },
    [updateActiveScene]
  );

  const handleMoveWorldPin = useCallback(
    (pinId: string, mediaSpaceXY: { x: number; y: number }) => {
      updateActiveScene((s) => {
        const nextPins = s.worldPins.map((p) =>
          p.id === pinId ? { ...p, mediaSpaceXY } : p
        );
        return {
          ...s,
          worldPins: nextPins,
          armature: solveWorldPins(
            s.armature,
            nextPins,
            s.timelineT,
            s.stageTransform,
            CANVAS_ORIGIN,
            rigSystems
          ),
        };
      });
    },
    [rigSystems, updateActiveScene]
  );

  const handleUpdateWorldPinConfig = useCallback(
    (pinId: string, patch: Partial<WorldPin>) => {
      updateActiveScene((s) => {
        const nextPins = s.worldPins.map((p) => (p.id === pinId ? { ...p, ...patch } : p));
        return {
          ...s,
          worldPins: nextPins,
          armature: solveWorldPins(
            s.armature,
            nextPins,
            s.timelineT,
            s.stageTransform,
            CANVAS_ORIGIN,
            rigSystems
          ),
        };
      });
    },
    [rigSystems, updateActiveScene]
  );

  const handleRemoveWorldPin = useCallback(
    (pinId: string) => {
      updateActiveScene((s) => ({
        ...s,
        worldPins: s.worldPins.filter((p) => p.id !== pinId),
      }));
    },
    [updateActiveScene]
  );

  // Stable memoized workspace & model module callbacks
  const handleVanishingPointChange = useCallback(
    (vp: { x: number; y: number }) => {
      updateActiveScene((s) => ({ ...s, vanishingPoint: vp }));
    },
    [updateActiveScene]
  );

  const handleToggleRail = useCallback(() => {
    setWorkspaceLayout((prev) => ({
      ...prev,
      railVisible: !prev.railVisible,
    }));
  }, []);

  const handleResetWorkspaceLayout = useCallback(() => {
    setWorkspaceLayout(createDefaultWorkspaceLayout());
  }, []);

  const handleToggleModule = useCallback((moduleId: ModelModuleId) => {
    setWorkspaceLayout((prev) => {
      const target = prev.panels[moduleId];
      nextZRef.current += 1;
      return {
        ...prev,
        panels: {
          ...prev.panels,
          [moduleId]: {
            ...target,
            isOpen: !target.isOpen,
            isCollapsed: false,
            zIndex: nextZRef.current,
          },
        },
      };
    });
  }, []);

  const handleSetModuleCollapsed = useCallback((moduleId: ModelModuleId, isCollapsed: boolean) => {
    setWorkspaceLayout((prev) => ({
      ...prev,
      panels: {
        ...prev.panels,
        [moduleId]: { ...prev.panels[moduleId], isCollapsed },
      },
    }));
  }, []);

  const handleSetModuleDockMode = useCallback(
    (
      moduleId: ModelModuleId,
      dockMode: 'docked' | 'floating',
      position?: { x: number; y: number }
    ) => {
      nextZRef.current += 1;
      setWorkspaceLayout((prev) => ({
        ...prev,
        panels: {
          ...prev.panels,
          [moduleId]: {
            ...prev.panels[moduleId],
            dockMode,
            position: position ?? prev.panels[moduleId].position,
            zIndex: nextZRef.current,
          },
        },
      }));
    },
    []
  );

  const handleSetModulePosition = useCallback(
    (moduleId: ModelModuleId, position: { x: number; y: number }) => {
      setWorkspaceLayout((prev) => ({
        ...prev,
        panels: {
          ...prev.panels,
          [moduleId]: { ...prev.panels[moduleId], position },
        },
      }));
    },
    []
  );

  const handleBringModuleToFront = useCallback((moduleId: ModelModuleId) => {
    nextZRef.current += 1;
    setWorkspaceLayout((prev) => ({
      ...prev,
      panels: {
        ...prev.panels,
        [moduleId]: { ...prev.panels[moduleId], zIndex: nextZRef.current },
      },
    }));
  }, []);

  const handleToggleBottomDrawer = useCallback(() => {
    setWorkspaceLayout((prev) => ({
      ...prev,
      bottomDrawerOpen: !prev.bottomDrawerOpen,
    }));
  }, []);

  const handleSelectBottomTab = useCallback((tab: BottomDrawerTab) => {
    setWorkspaceLayout((prev) => ({
      ...prev,
      bottomDrawerTab: tab,
    }));
  }, []);

  const handleUpdateRigSystems = useCallback(
    (patch: Partial<RigSystemsConfig>) => {
      setRigSystems((prev) => {
        const next = { ...prev, ...patch };
        if (patch.floorSnap && !prev.floorSnap) {
          animateSceneArmature(
            (base) => {
              if (base.isAnchorPinned) return base;
              const joints = computeArmature(base, CANVAS_ORIGIN);
              const groundY = next.pressurePlateY ?? next.groundY;
              const maxFootY = Math.max(
                joints.footR.endY,
                joints.footL.endY,
                joints.toeR.endY,
                joints.toeL.endY
              );
              if (maxFootY > groundY) {
                return {
                  ...base,
                  rootY: base.rootY - (maxFootY - groundY),
                };
              }
              return base;
            },
            { durationMs: DEFAULT_TOGGLE_TRANSITION_MS }
          );
        }
        return next;
      });
    },
    [animateSceneArmature]
  );

  const handleSetHipToggle = useCallback(
    (state: ToggleHingeFoldState) => {
      commitPoseHistory();
      setIsPlaying(false);
      animateSceneArmature((base) => setHipToggleState(base, state), {
        durationMs: DEFAULT_TOGGLE_TRANSITION_MS,
        scenePatch: { activePreset: null },
      });
    },
    [animateSceneArmature, commitPoseHistory]
  );

  const handleSetCollarToggle = useCallback(
    (side: 'R' | 'L', state: ToggleHingeFoldState) => {
      commitPoseHistory();
      setIsPlaying(false);
      animateSceneArmature((base) => setCollarToggleState(base, side, state), {
        durationMs: DEFAULT_TOGGLE_TRANSITION_MS,
        scenePatch: { activePreset: null },
      });
    },
    [animateSceneArmature, commitPoseHistory]
  );

  const handleToggleArmSwivel = useCallback(
    (armSide: 'R' | 'L') => {
      commitPoseHistory();
      setIsPlaying(false);
      animateSceneArmature((base) => toggleArmReverseSwivel(base, armSide), {
        durationMs: DEFAULT_TOGGLE_TRANSITION_MS,
        scenePatch: { activePreset: null },
      });
    },
    [animateSceneArmature, commitPoseHistory]
  );

  const handleTogglePerspective = useCallback(
    (show: boolean) => {
      updateActiveScene((s) => ({ ...s, showPerspective: show }));
    },
    [updateActiveScene]
  );

  const handleResetVanishingPoint = useCallback(() => {
    updateActiveScene((s) => ({
      ...s,
      vanishingPoint: { x: PAPER_WIDTH / 2, y: PAPER_HEIGHT / 2 - 110 },
    }));
  }, [updateActiveScene]);

  const handleUpdateSceneMedia = useCallback(
    (patch: Partial<SceneMedia>) => {
      updateActiveScene((s) => ({ ...s, media: { ...s.media, ...patch } }));
    },
    [updateActiveScene]
  );

  const handleTogglePlay = useCallback(() => {
    cancelPoseTransition();
    setIsPlaying((p) => !p);
  }, [cancelPoseTransition]);

  const handlePlaybackSpeedChange = useCallback(
    (speed: number) => {
      updateActiveScene((s) => ({ ...s, playbackSpeed: speed }));
    },
    [updateActiveScene]
  );

  const handleCurveChange = useCallback(
    (curve: MotionCurve) => {
      updateActiveScene((s) => ({ ...s, motionCurve: curve }));
    },
    [updateActiveScene]
  );

  const handleTogglePlaybackMode = useCallback(() => {
    setPlaybackMode((prev) => (prev === 'scene' ? 'sequence' : 'scene'));
  }, []);

  const handleRenameScene = useCallback(
    (name: string) => {
      updateActiveScene((s) => ({ ...s, name }));
    },
    [updateActiveScene]
  );

  const handleUpdateSceneDuration = useCallback(
    (durationSec: number) => {
      updateActiveScene((s) => ({ ...s, durationSec }));
    },
    [updateActiveScene]
  );

  useEffect(() => {
    const onKeyDown = (e: KeyboardEvent) => {
      if (e.key === 'Escape') {
        const openPanelIds = (['rig_joint', 'digits', 'perspective', 'scene'] as ModelModuleId[]).filter(
          (id) => workspaceLayout.panels[id].isOpen
        );
        if (openPanelIds.length > 0) {
          e.preventDefault();
          const topId = openPanelIds.reduce((best, id) =>
            workspaceLayout.panels[id].zIndex >= workspaceLayout.panels[best].zIndex ? id : best
          );
          handleToggleModule(topId);
          return;
        }
        if (workspaceLayout.bottomDrawerOpen) {
          e.preventDefault();
          handleToggleBottomDrawer();
          return;
        }
      }
      const target = e.target as HTMLElement | null;
      if (
        target &&
        (target.tagName === 'INPUT' || target.tagName === 'TEXTAREA' || target.tagName === 'SELECT')
      ) {
        return;
      }
      if ((e.metaKey || e.ctrlKey) && e.key.toLowerCase() === 'z') {
        const last = historyRef.current[historyRef.current.length - 1];
        if (last) {
          e.preventDefault();
          historyRef.current = historyRef.current.slice(0, -1);
          setIsPlaying(false);
          animateSceneArmature(() => last, {
            durationMs: DEFAULT_TOGGLE_TRANSITION_MS,
            scenePatch: { activePreset: null, activeSlot: null },
          });
        }
        return;
      }
      if (e.key === 'ArrowLeft' && e.shiftKey) {
        e.preventDefault();
        handleNudgeSelectedJoint(-5);
      } else if (e.key === 'ArrowRight' && e.shiftKey) {
        e.preventDefault();
        handleNudgeSelectedJoint(5);
      }
    };
    window.addEventListener('keydown', onKeyDown);
    return () => window.removeEventListener('keydown', onKeyDown);
  }, [
    animateSceneArmature,
    handleNudgeSelectedJoint,
    handleToggleBottomDrawer,
    handleToggleModule,
    workspaceLayout.bottomDrawerOpen,
    workspaceLayout.panels,
  ]);

  const isCover = pageId === 'page-22';
  const gridAnyVisible = grid.showSubpyxl || grid.showPyxl || grid.showMegapyxl;
  const showCalibrationOrigin = workspaceLayout.panels.scene.isOpen;

  return (
    <div className="h-screen w-screen flex flex-col bg-[#16001E] text-[#EFEAF4] overflow-hidden">
      {/* Top Bar — System & Stage Environment Only */}
      <header className="h-10 px-3 sm:px-6 bg-[#16001E] border-b border-[#EBE0F5]/14 flex items-center justify-between gap-2 sm:gap-4 shrink-0 z-20">
        <div className="flex items-center gap-3 shrink-0">
          <a
            href="#top"
            onClick={(e) => {
              e.preventDefault();
              handleFitClick();
            }}
            className="font-typewriter font-bold text-xs sm:text-base tracking-wider text-[#F4EFF9] whitespace-nowrap shrink-0 focus-visible:outline-2 focus-visible:outline-[#D8B4FE]"
          >
            Bitruvius
          </a>
        </div>

        {/* System Display, Workspace Layout & Theme */}
        <div className="flex items-center gap-1 sm:gap-1.5 shrink-0">
          <PoseStreamControl snapshot={poseTelemetry} />

          <button
            type="button"
            disabled={!isCover}
            aria-pressed={gridAnyVisible}
            onClick={() =>
              setGrid((prev) => {
                const nextOn = !(prev.showSubpyxl || prev.showPyxl || prev.showMegapyxl);
                return {
                  ...prev,
                  showSubpyxl: nextOn,
                  showPyxl: nextOn,
                  showMegapyxl: nextOn,
                };
              })
            }
            title="Toggle proportional light ink grid"
            className={`px-1.5 sm:px-2 py-1 font-vera text-[10.5px] font-bold rounded-[2px] border flex items-center gap-1 whitespace-nowrap shrink-0 transition-colors cursor-pointer disabled:opacity-35 disabled:cursor-not-allowed ${
              gridAnyVisible && isCover
                ? 'bg-[#2C143B] border-[#D8B4FE]/35 text-[#FAF6FF]'
                : 'bg-[#1D1026] border-[#EBE0F5]/14 text-[#B8A9C9] hover:text-[#FAF6FF]'
            }`}
          >
            <Grid3X3 className="w-3.5 h-3.5" />
            <span className="hidden sm:inline">Grid</span>
          </button>

          <button
            type="button"
            disabled={!isCover}
            aria-pressed={grid.showVitruvianGuide && grid.showWorkingSquare}
            onClick={() =>
              setGrid((prev) => {
                const nextOn = !(
                  prev.showVitruvianGuide ||
                  prev.showWorkingSquare ||
                  prev.showEightHeadGuide
                );
                return {
                  ...prev,
                  showVitruvianGuide: nextOn,
                  showWorkingSquare: nextOn,
                  showEightHeadGuide: nextOn,
                };
              })
            }
            title="Toggle Working Square, Vitruvian & 8-Head proportional guides"
            className={`px-1.5 sm:px-2 py-1 font-vera text-[10.5px] font-bold rounded-[2px] border flex items-center gap-1 whitespace-nowrap shrink-0 transition-colors cursor-pointer disabled:opacity-35 disabled:cursor-not-allowed ${
              (grid.showVitruvianGuide || grid.showWorkingSquare || grid.showEightHeadGuide) &&
              isCover
                ? 'bg-[#2C143B] border-[#D8B4FE]/35 text-[#FAF6FF]'
                : 'bg-[#1D1026] border-[#EBE0F5]/14 text-[#B8A9C9] hover:text-[#FAF6FF]'
            }`}
          >
            <Square className="w-3.5 h-3.5" />
            <span className="hidden md:inline">Guides</span>
          </button>

          <button
            type="button"
            disabled={!isCover}
            aria-pressed={grid.showRulers}
            onClick={() => setGrid((prev) => ({ ...prev, showRulers: !prev.showRulers }))}
            title="Toggle symmetrical edge rulers"
            className={`hidden sm:flex px-2 py-1 font-vera text-[10.5px] font-bold rounded-[2px] border items-center gap-1 whitespace-nowrap shrink-0 transition-colors cursor-pointer disabled:opacity-35 disabled:cursor-not-allowed ${
              grid.showRulers && isCover
                ? 'bg-[#2C143B] border-[#D8B4FE]/35 text-[#FAF6FF]'
                : 'bg-[#1D1026] border-[#EBE0F5]/14 text-[#B8A9C9] hover:text-[#FAF6FF]'
            }`}
          >
            <Ruler className="w-3.5 h-3.5" />
            <span className="hidden lg:inline">Rulers</span>
          </button>

          <button
            type="button"
            disabled={!isCover}
            onClick={handleCycleInk}
            title="Cycle light ink density (Whisper · Light · Draft)"
            className="hidden md:inline-flex px-2 py-1 font-vera text-[10.5px] font-bold rounded-[2px] border bg-[#1D1026] border-[#EBE0F5]/14 text-[#D9CBE8] hover:text-[#FAF6FF] whitespace-nowrap shrink-0 cursor-pointer disabled:opacity-35 disabled:cursor-not-allowed"
          >
            Ink: {grid.inkStrength === 'whisper' ? 'Whisper' : grid.inkStrength === 'light' ? 'Light' : 'Draft'}
          </button>

          <span className="h-4 w-px bg-[#EBE0F5]/14 mx-0.5 hidden sm:inline-block" aria-hidden="true" />

          <button
            type="button"
            aria-pressed={workspaceLayout.railVisible}
            onClick={handleToggleRail}
            title="Toggle right quick-toggle rail & model modules"
            className={`px-2 py-1 font-vera text-[10.5px] font-bold rounded-[2px] border flex items-center gap-1 whitespace-nowrap shrink-0 transition-colors cursor-pointer ${
              workspaceLayout.railVisible
                ? 'bg-[#2C143B] border-[#D8B4FE]/40 text-[#FAF6FF] hover:bg-[#361547]'
                : 'bg-[#1D1026] border-[#EBE0F5]/14 text-[#B8A9C9] hover:text-white'
            }`}
          >
            <Layers className="w-3.5 h-3.5" />
            <span>Modules</span>
          </button>

          <button
            type="button"
            onClick={handleResetWorkspaceLayout}
            title="Reset docked/floating modular menu layout"
            className="hidden sm:inline-flex px-2 py-1 font-vera text-[10.5px] font-bold rounded-[2px] border bg-[#1D1026] border-[#EBE0F5]/14 text-[#C8B2E0] hover:text-[#FAF6FF] items-center gap-1 whitespace-nowrap shrink-0 cursor-pointer"
          >
            <LayoutGrid className="w-3.5 h-3.5" />
            <span className="hidden lg:inline">Reset Layout</span>
          </button>

          <button
            type="button"
            aria-pressed={darkMode}
            onClick={() => setDarkMode((prev) => !prev)}
            title="Toggle Dark Mode (Black paper & white border)"
            className={`px-1.5 sm:px-2.5 py-1 font-vera text-[10.5px] font-bold rounded-[2px] border flex items-center gap-1 whitespace-nowrap shrink-0 transition-colors cursor-pointer ${
              darkMode
                ? 'bg-[#FAF6FF] border-white text-[#16001E]'
                : 'bg-[#1D1026] border-[#EBE0F5]/20 text-[#EFEAF4] hover:text-white'
            }`}
          >
            {darkMode ? <Sun className="w-3.5 h-3.5" /> : <Moon className="w-3.5 h-3.5" />}
            <span className="hidden sm:inline">{darkMode ? 'Light' : 'Dark'}</span>
          </button>
        </div>
      </header>

      {/* Full-Bleed Stage with Stackable / Attachable Modular Model Menus */}
      <div className="flex-1 min-h-0 flex flex-row relative">
        <main className="flex-1 w-full min-w-0 min-h-0 flex flex-col relative">
          <BookStageCanvas
            pageId={pageId}
            grid={grid}
            darkMode={darkMode}
            screenFps={screenFps}
            zoom={zoom}
            pan={pan}
            armature={armature}
            computedJoints={computedJoints}
            selectedJointId={selectedJointId}
            onZoomChange={handleZoomChange}
            onPanChange={handlePanChange}
            onViewportResize={handleViewportResize}
            onCursorMove={handleCursorMove}
            onTurnPage={handleTurnPage}
            onFitRequest={handleFitClick}
            onCopyPoseData={handleCopyPoseData}
            onSavePoseData={handleSavePoseData}
            onSelectJoint={setSelectedJointId}
            onUpdateArmature={handleUpdateArmature}
            onAnimateArmature={handleAnimateArmature}
            isPoseTransitioning={isPoseTransitioning}
            isPlaying={isPlaying}
            onCommitPoseHistory={commitPoseHistory}
            rigSystems={rigSystems}
            onUpdateRigSystems={handleUpdateRigSystems}
            showPerspective={showPerspective}
            vanishingPoint={vanishingPoint}
            onVanishingPointChange={handleVanishingPointChange}
            sceneMedia={resolvedActiveScene.media}
            stageTransform={resolvedActiveScene.stageTransform}
            compositeFilter={resolvedActiveScene.compositeFilter}
            worldPins={resolvedActiveScene.worldPins}
            timelineT={timelineT}
            sceneDurationSec={resolvedActiveScene.durationSec}
            showCalibrationOrigin={showCalibrationOrigin}
            onUpdateStageTransform={handleUpdateStageTransform}
            onUpdateWorldPin={handleMoveWorldPin}
          />

          <ModularWorkspaceOverlay
            railVisible={workspaceLayout.railVisible}
            panels={workspaceLayout.panels}
            onToggleRail={handleToggleRail}
            onToggleModule={handleToggleModule}
            onSetModuleCollapsed={handleSetModuleCollapsed}
            onSetModuleDockMode={handleSetModuleDockMode}
            onSetModulePosition={handleSetModulePosition}
            onBringModuleToFront={handleBringModuleToFront}
            armature={armature}
            selectedJointId={selectedJointId}
            rigSystems={rigSystems}
            showPerspective={showPerspective}
            vanishingPoint={vanishingPoint}
            activeScene={resolvedActiveScene}
            onMirrorPose={handleMirrorPose}
            onResetPose={handleResetPose}
            onBatchSetIK={handleBatchSetIK}
            onUpdateRigSystems={handleUpdateRigSystems}
            onUpdateArmature={handleAnimateArmature}
            onSetJointMode={handleSetJointMode}
            onSetJointPhysics={handleSetJointPhysics}
            onSetJointAngle={handleSetJointAngle}
            onSetJointScale={handleSetJointScale}
            onNudgeSelectedJoint={handleNudgeSelectedJoint}
            onZeroSelectedJoint={handleZeroSelectedJoint}
            onSetHipToggle={handleSetHipToggle}
            onSetCollarToggle={handleSetCollarToggle}
            onToggleArmSwivel={handleToggleArmSwivel}
            onApplyHandGesture={handleApplyHandGesture}
            onTogglePerspective={handleTogglePerspective}
            onResetVanishingPoint={handleResetVanishingPoint}
            onSelectPaperPlate={handleSelectPaperPlate}
            onSelectSamplePlate={handleSelectSamplePlate}
            onUploadMediaFile={handleUploadMediaFile}
            onUpdateSceneMedia={handleUpdateSceneMedia}
            onUpdateStageTransform={handleUpdateStageTransform}
            onResetStageTransform={handleResetStageTransform}
            onUpdateCompositeFilter={handleUpdateCompositeFilter}
            onToggleWorldPinForJoint={handleToggleWorldPinForJoint}
            onUpdateWorldPinConfig={handleUpdateWorldPinConfig}
            onRemoveWorldPin={handleRemoveWorldPin}
          />
        </main>
      </div>

      {/* Bottom Area — Unified Collapsible Drawer for Timeline, Pose Snapshots & Animation Settings */}
      {isCover && (
        <BottomAnimationDrawer
          isOpen={workspaceLayout.bottomDrawerOpen}
          activeTab={workspaceLayout.bottomDrawerTab}
          onToggleOpen={handleToggleBottomDrawer}
          onSelectTab={handleSelectBottomTab}
          slots={slots}
          activeSlot={activeSlot}
          timelineT={timelineT}
          isPlaying={isPlaying}
          playbackSpeed={playbackSpeed}
          motionCurve={motionCurve}
          onCaptureSlot={handleCaptureSlot}
          onJumpToSlot={handleJumpToSlot}
          onClearSlot={handleClearSlot}
          onScrubChange={handleScrubChange}
          onTogglePlay={handleTogglePlay}
          onPlaybackSpeedChange={handlePlaybackSpeedChange}
          onCurveChange={handleCurveChange}
          onSynthesizeBreakdown={handleSynthesizeBreakdown}
          activePreset={activePreset}
          savedPoses={savedPoses}
          onApplyPreset={handleApplyPreset}
          onMirrorPose={handleMirrorPose}
          onResetPose={handleResetPose}
          onSaveUserPose={handleSaveUserPose}
          onApplyUserPose={handleApplyUserPose}
          onOverwriteUserPose={handleOverwriteUserPose}
          onDeleteUserPose={handleDeleteUserPose}
          onLoadUserPoseToSlot={handleLoadUserPoseToSlot}
          scenes={scenes}
          activeScene={resolvedActiveScene}
          playbackMode={playbackMode}
          screenFps={screenFps}
          onCycleFps={handleCycleFps}
          onSelectScene={handleSelectScene}
          onAddScene={handleAddScene}
          onDuplicateScene={handleDuplicateScene}
          onDeleteScene={handleDeleteScene}
          onTogglePlaybackMode={handleTogglePlaybackMode}
          onRenameScene={handleRenameScene}
          onUpdateSceneDuration={handleUpdateSceneDuration}
          onExportFormat={handleExportIRFormat}
        />
      )}

      {/* Slim Bottom Coordinate & Symmetry Bar */}
      <footer className="h-9 px-3 sm:px-6 bg-[#16001E] border-t border-[#EBE0F5]/12 flex items-center justify-between gap-2 sm:gap-4 shrink-0 z-20 text-[11px]">
        <div className="flex items-center gap-3 min-w-0 overflow-hidden whitespace-nowrap font-vera tabular-nums">
          <span>
            <span className="text-[#9E8EB3] mr-1">X</span>
            <span ref={cursorXRef} className="font-typewriter font-bold text-[#FAF6FF]">
              +0.00 pyx
            </span>
          </span>
          <span>
            <span className="text-[#9E8EB3] mr-1">Y</span>
            <span ref={cursorYRef} className="font-typewriter font-bold text-[#FAF6FF]">
              +0.00 pyx
            </span>
          </span>
          <span className="hidden md:inline text-[#9E8EB3]">·</span>
          <span className="hidden md:inline text-[#C8B2E0]">
            {resolvedActiveScene.name} · {resolvedActiveScene.media.name}
          </span>
          <span className="hidden lg:inline text-[#9E8EB3]">·</span>
          <span className="hidden lg:inline text-[#C8B2E0]">
            Window {viewportSize.width} × {viewportSize.height}
          </span>
        </div>

        <div className="flex items-center gap-1 shrink-0 tabular-nums">
          <button
            type="button"
            onClick={() => handleZoomStep(0.85)}
            aria-label="Zoom out"
            title="Zoom out"
            className="p-1.5 text-[#E6DCF0] hover:bg-[#2A1D36] rounded-[2px] cursor-pointer"
          >
            <Minus className="w-3.5 h-3.5" />
          </button>
          <button
            type="button"
            onClick={() => handleSetExactZoom(1.0)}
            title="Set 100.00% scale"
            className="px-2 py-0.5 font-typewriter font-bold text-[11px] text-[#FAF6FF] hover:bg-[#2A1D36] rounded-[2px] cursor-pointer min-w-[48px] text-center whitespace-nowrap"
          >
            {(zoom * 100).toFixed(0)}%
          </button>
          <button
            type="button"
            onClick={() => handleZoomStep(1.18)}
            aria-label="Zoom in"
            title="Zoom in"
            className="p-1.5 text-[#E6DCF0] hover:bg-[#2A1D36] rounded-[2px] cursor-pointer"
          >
            <Plus className="w-3.5 h-3.5" />
          </button>
          <button
            type="button"
            onClick={handleFitClick}
            title="Fit symmetrically to window"
            className="px-2 py-0.5 font-vera font-bold text-[10.5px] text-[#D8B4FE] hover:bg-[#2A1D36] rounded-[2px] flex items-center gap-1 cursor-pointer whitespace-nowrap"
          >
            <Maximize2 className="w-3 h-3" />
            <span>Fit</span>
          </button>
        </div>
      </footer>
    </div>
  );
}
