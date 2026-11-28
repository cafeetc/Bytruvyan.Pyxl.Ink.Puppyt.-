import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import {
  Anchor,
  Maximize2,
  Minus,
  Moon,
  Plus,
  RotateCcw,
  ShieldCheck,
  SlidersHorizontal,
  Sun,
} from 'lucide-react';
import { BookStageCanvas } from './components/BookStageCanvas';
import {
  FirstScreenHudOverlay,
  FirstScreenPlanquetteSvgOverlay,
  FirstScreenSvgOverlay,
  IntroStage,
} from './components/FirstScreenIntro';
import { JOINT_SPEC_MAP } from './data/digitalRig';
import {
  DEFAULT_CANVAS_ORIGIN,
  PoseEngineSnapshot,
  createPoseEngine,
} from './engine/PoseEngine';
import { GridConfig, PageId } from './types/canvas';
import { ArmatureState, JointId, Point2D, RigSystemsConfig } from './types/rig';
import {
  DEFAULT_POSE_TRANSITION_MS,
  DEFAULT_TOGGLE_TRANSITION_MS,
  ELASTI_HOSE_TRANSITION_MS,
  TransitionEasing,
  computePoseTransitionFrame,
  createPosePreset,
  hasArmaturePoseDelta,
  solveFaceplateLookIK,
} from './utils/kinematics';
import {
  INTRO_FACEPLATE_THRESHOLD_PX,
  INTRO_TARGET_OFFSET_PX,
  IntroChoice,
  computeIntroCamera,
  computePlanquetteArc,
  computePlanquetteCamera,
  evaluateFaceplateChoice,
  hitTestPlanquetteArc,
  projectFaceplateToPlanquetteArc,
  resolveFaceplateTargetChoice,
} from './utils/introFraming';
import {
  PYXL_PX,
  computeSymmetricalCenterPan,
  computeSymmetricalFitZoom,
  formatSignedNum,
} from './utils/proportions';

const INITIAL_PAGE: PageId = 'page-22';

const DEFAULT_GRID: GridConfig = {
  showSubpyxl: true,
  showPyxl: true,
  showMegapyxl: true,
  showVitruvianGuide: true,
  showWorkingSquare: true,
  showEightHeadGuide: true,
};

const SOLVER_ORDER: readonly RigSystemsConfig['solver'][] = [
  'analytical',
  'fabrik',
  'ccd',
];

const SOLVER_LABELS: Record<RigSystemsConfig['solver'], string> = {
  analytical: 'Analytical',
  fabrik: 'FABRIK',
  ccd: 'CCD',
};

export default function App() {
  const [pageId] = useState<PageId>(INITIAL_PAGE);
  const [darkMode, setDarkMode] = useState(false);
  const [viewportSize, setViewportSize] = useState({ width: 1440, height: 780 });
  const viewportSizeRef = useRef(viewportSize);
  viewportSizeRef.current = viewportSize;

  const cursorXRef = useRef<HTMLSpanElement | null>(null);
  const cursorYRef = useRef<HTMLSpanElement | null>(null);

  const engineRef = useRef<ReturnType<typeof createPoseEngine> | null>(null);
  if (!engineRef.current) {
    engineRef.current = createPoseEngine({ canvasOrigin: DEFAULT_CANVAS_ORIGIN });
  }
  const engine = engineRef.current;

  const restLipRef = useRef<{ x: number; y: number } | null>(null);
  if (!restLipRef.current) {
    const initialLip = engine.getJoint('lip');
    restLipRef.current = { x: initialLip.x, y: initialLip.y };
  }
  const restLip = restLipRef.current;

  const [snapshot, setSnapshot] = useState<PoseEngineSnapshot>(() =>
    engine.getSnapshot()
  );
  const [selectedJointId, setSelectedJointId] = useState<JointId>('lip');
  const [isPoseTransitioning, setIsPoseTransitioning] = useState(false);

  const [introStage, setIntroStage] = useState<IntroStage>('greeting_1');
  const introStageRef = useRef<IntroStage>(introStage);
  introStageRef.current = introStage;
  const [isDraggingFaceplate, setIsDraggingFaceplate] = useState(false);
  const isDraggingFaceplateRef = useRef(false);
  const [hoveredChoice, setHoveredChoice] = useState<IntroChoice | null>(null);
  const hoveredChoiceRef = useRef<IntroChoice | null>(null);
  const faceplateDragWorldRef = useRef<{ pointer: Point2D; target: Point2D } | null>(null);
  const hoverDwellTimerRef = useRef<number | null>(null);
  const [pyrsonName, setPyrsonName] = useState<string | null>(null);
  const welcomeTimerRef = useRef<number | null>(null);
  const introTimersRef = useRef<number[]>([]);

  const [planquetteName, setPlanquetteName] = useState('');
  const [planquetteHoveredIndex, setPlanquetteHoveredIndex] = useState<number | null>(null);
  const planquetteHoveredIndexRef = useRef<number | null>(null);
  const planquetteHoveredCharRef = useRef<string | null>(null);
  const planquetteDwellTimerRef = useRef<number | null>(null);
  const dwellCommittedTileIdxRef = useRef<number | null>(null);
  const [planquetteSource, setPlanquetteSource] = useState<'faceplate' | 'handR' | 'handL' | null>(
    null
  );
  const [planquetteCursorWorld, setPlanquetteCursorWorld] = useState<Point2D | null>(null);

  const [camera, setCamera] = useState(() => {
    const introCam = computeIntroCamera(1440, 780, restLip);
    return {
      zoom: introCam.zoom,
      pan: introCam.pan,
      userEdited: false,
    };
  });
  const cameraRef = useRef(camera);
  cameraRef.current = camera;

  const historyRef = useRef<ArmatureState[]>([]);
  const transitionRef = useRef<{
    from: ArmatureState;
    to: ArmatureState;
    startMs: number;
    durationMs: number;
    targetPreset: 'tpose' | null;
    easing: TransitionEasing;
    rafId: number;
  } | null>(null);

  useEffect(() => {
    return engine.subscribe((next) => {
      setSnapshot(next);
    });
  }, [engine]);

  useEffect(() => {
    document.documentElement.classList.toggle('dark', darkMode);
  }, [darkMode]);

  const cancelPoseTransition = useCallback(() => {
    if (transitionRef.current) {
      cancelAnimationFrame(transitionRef.current.rafId);
      transitionRef.current = null;
      setIsPoseTransitioning(false);
    }
  }, []);

  useEffect(() => {
    return () => {
      cancelPoseTransition();
      if (hoverDwellTimerRef.current !== null) {
        window.clearTimeout(hoverDwellTimerRef.current);
      }
      if (planquetteDwellTimerRef.current !== null) {
        window.clearTimeout(planquetteDwellTimerRef.current);
      }
      if (welcomeTimerRef.current !== null) {
        window.clearTimeout(welcomeTimerRef.current);
      }
      for (const id of introTimersRef.current) {
        window.clearTimeout(id);
      }
    };
  }, [cancelPoseTransition]);

  const handleCursorMove = useCallback(
    (coords: { x: number; y: number } | null) => {
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
    },
    []
  );

  const handleViewportResize = useCallback(
    (nextSize: { width: number; height: number }) => {
      const prevSize = viewportSizeRef.current;
      if (prevSize.width === nextSize.width && prevSize.height === nextSize.height) {
        return;
      }
      viewportSizeRef.current = nextSize;
      setViewportSize(nextSize);
      setCamera((prev) => {
        if (!prev.userEdited) {
          if (introStageRef.current === 'input_name') {
            const planCam = computePlanquetteCamera(
              nextSize.width,
              nextSize.height,
              DEFAULT_CANVAS_ORIGIN.x
            );
            return {
              zoom: planCam.zoom,
              pan: planCam.pan,
              userEdited: false,
            };
          }
          if (introStageRef.current !== 'complete' && introStageRef.current !== 'welcomed') {
            const introCam = computeIntroCamera(nextSize.width, nextSize.height, restLip);
            return {
              zoom: introCam.zoom,
              pan: introCam.pan,
              userEdited: false,
            };
          }
          const nextZoom = computeSymmetricalFitZoom(nextSize.width, nextSize.height);
          return {
            zoom: nextZoom,
            pan: computeSymmetricalCenterPan(
              nextSize.width,
              nextSize.height,
              DEFAULT_CANVAS_ORIGIN.x,
              DEFAULT_CANVAS_ORIGIN.y,
              nextZoom
            ),
            userEdited: false,
          };
        }
        return {
          ...prev,
          pan: {
            x: prev.pan.x + (nextSize.width - prevSize.width) / 2,
            y: prev.pan.y + (nextSize.height - prevSize.height) / 2,
          },
        };
      });
    },
    [restLip]
  );

  const fitSymmetricalView = useCallback((size = viewportSizeRef.current) => {
    const nextZoom = computeSymmetricalFitZoom(size.width, size.height);
    const nextPan = computeSymmetricalCenterPan(
      size.width,
      size.height,
      DEFAULT_CANVAS_ORIGIN.x,
      DEFAULT_CANVAS_ORIGIN.y,
      nextZoom
    );
    setCamera({ zoom: nextZoom, pan: nextPan, userEdited: false });
  }, []);

  const handleZoomChange = useCallback((nextZoom: number) => {
    setCamera((prev) => ({ ...prev, zoom: nextZoom, userEdited: true }));
  }, []);

  const handlePanChange = useCallback((nextPan: { x: number; y: number }) => {
    setCamera((prev) => ({ ...prev, pan: nextPan, userEdited: true }));
  }, []);

  const handleZoomStep = useCallback((factor: number) => {
    const size = viewportSizeRef.current;
    setCamera((prev) => {
      const minZoom = computeSymmetricalFitZoom(size.width, size.height);
      const nextZoom = Math.max(minZoom, Math.min(2.4, prev.zoom * factor));
      const cx = size.width / 2;
      const cy = size.height / 2;
      const worldX = (cx - prev.pan.x) / prev.zoom;
      const worldY = (cy - prev.pan.y) / prev.zoom;
      return {
        zoom: nextZoom,
        pan: {
          x: cx - worldX * nextZoom,
          y: cy - worldY * nextZoom,
        },
        userEdited: true,
      };
    });
  }, []);

  const handleSetExactZoom = useCallback(
    (targetZoom: number) => {
      const size = viewportSizeRef.current;
      const minZoom = computeSymmetricalFitZoom(size.width, size.height);
      const nextZoom = Math.max(minZoom, Math.min(2.4, targetZoom));
      const anchor =
        introStageRef.current === 'input_name'
          ? { x: DEFAULT_CANVAS_ORIGIN.x, y: 462 }
          : introStageRef.current !== 'complete' && introStageRef.current !== 'welcomed'
          ? restLip
          : DEFAULT_CANVAS_ORIGIN;
      setCamera({
        zoom: nextZoom,
        pan: computeSymmetricalCenterPan(
          size.width,
          size.height,
          anchor.x,
          anchor.y,
          nextZoom
        ),
        userEdited: true,
      });
    },
    [restLip]
  );

  const commitPoseHistory = useCallback(() => {
    const current = transitionRef.current
      ? transitionRef.current.to
      : engine.getArmature();
    historyRef.current = [...historyRef.current.slice(-24), structuredClone(current)];
  }, [engine]);

  const animateArmatureTo = useCallback(
    (
      targetArmature: ArmatureState,
      durationMs: number = DEFAULT_POSE_TRANSITION_MS,
      targetPreset: 'tpose' | null = null,
      easing: TransitionEasing = 'smooth'
    ) => {
      const fromArmature = engine.getArmature();
      if (durationMs <= 0 || !hasArmaturePoseDelta(fromArmature, targetArmature)) {
        cancelPoseTransition();
        if (targetPreset === 'tpose') {
          engine.resetToTPose({ preserveConstraints: true });
        } else {
          engine.setArmature(targetArmature, targetPreset);
        }
        return;
      }

      cancelPoseTransition();
      setIsPoseTransitioning(true);

      const transition = {
        from: fromArmature,
        to: targetArmature,
        startMs: performance.now(),
        durationMs,
        targetPreset,
        easing,
        rafId: 0,
      };
      transitionRef.current = transition;

      const step = (now: number) => {
        if (transitionRef.current !== transition) return;
        const elapsedMs = now - transition.startMs;
        const frame = computePoseTransitionFrame(
          transition.from,
          transition.to,
          elapsedMs,
          transition.durationMs,
          transition.easing
        );

        if (frame.done) {
          transitionRef.current = null;
          setIsPoseTransitioning(false);
          if (transition.targetPreset === 'tpose') {
            engine.resetToTPose({ preserveConstraints: true });
          } else {
            engine.setArmature(transition.to, transition.targetPreset);
          }
        } else {
          engine.setArmature(frame.armature, null);
          transition.rafId = requestAnimationFrame(step);
        }
      };

      transition.rafId = requestAnimationFrame(step);
    },
    [cancelPoseTransition, engine]
  );

  const handleUpdateArmature = useCallback(
    (next: ArmatureState) => {
      cancelPoseTransition();
      engine.setArmature(next, null);
    },
    [cancelPoseTransition, engine]
  );

  const handleAnimateArmature = useCallback(
    (next: ArmatureState, durationMs: number = DEFAULT_TOGGLE_TRANSITION_MS) => {
      animateArmatureTo(next, durationMs, null);
    },
    [animateArmatureTo]
  );

  const handleElastiHoseSnap = useCallback(() => {
    if (introStageRef.current === 'welcomed') {
      setIntroStage('complete');
    }
    commitPoseHistory();
    const target = createPosePreset('tpose', {
      ...engine.getArmature(),
      rootX: DEFAULT_CANVAS_ORIGIN.x,
      rootY: DEFAULT_CANVAS_ORIGIN.y,
      isAnchorPinned: true,
      hipToggleSign: 1,
    });
    animateArmatureTo(target, ELASTI_HOSE_TRANSITION_MS, 'tpose', 'elastic');
  }, [animateArmatureTo, commitPoseHistory, engine]);

  const handleResetTPose = useCallback(() => {
    handleElastiHoseSnap();
  }, [handleElastiHoseSnap]);

  const handleUpdateRigSystems = useCallback(
    (patch: Partial<RigSystemsConfig>) => {
      engine.setConstraints(patch);
    },
    [engine]
  );

  const handleCycleSolver = useCallback(() => {
    const current = engine.getConstraints().solver;
    const nextIdx = (SOLVER_ORDER.indexOf(current) + 1) % SOLVER_ORDER.length;
    engine.setConstraints({ solver: SOLVER_ORDER[nextIdx] });
  }, [engine]);

  useEffect(() => {
    const onKeyDown = (e: KeyboardEvent) => {
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
          animateArmatureTo(last, DEFAULT_TOGGLE_TRANSITION_MS, null);
        }
        return;
      }
      if (e.key === 'ArrowLeft' && e.shiftKey) {
        e.preventDefault();
        commitPoseHistory();
        engine.nudgeJointAngle(selectedJointId, -5);
      } else if (e.key === 'ArrowRight' && e.shiftKey) {
        e.preventDefault();
        commitPoseHistory();
        engine.nudgeJointAngle(selectedJointId, 5);
      }
    };
    window.addEventListener('keydown', onKeyDown);
    return () => window.removeEventListener('keydown', onKeyDown);
  }, [animateArmatureTo, commitPoseHistory, engine, selectedJointId]);

  const selectedSpec = useMemo(() => JOINT_SPEC_MAP[selectedJointId], [selectedJointId]);
  const { pose, computedJoints, constraints } = snapshot;

  const faceplateEvaluation = useMemo(
    () =>
      evaluateFaceplateChoice(
        computedJoints.lip.x,
        restLip.x,
        INTRO_FACEPLATE_THRESHOLD_PX,
        computedJoints.lip.y,
        restLip.y
      ),
    [computedJoints.lip.x, computedJoints.lip.y, restLip.x, restLip.y]
  );

  const resetRigToNeutralTPose = useCallback(
    (durationMs = DEFAULT_TOGGLE_TRANSITION_MS) => {
      const target = createPosePreset('tpose', {
        ...engine.getArmature(),
        rootX: DEFAULT_CANVAS_ORIGIN.x,
        rootY: DEFAULT_CANVAS_ORIGIN.y,
        isAnchorPinned: true,
        hipToggleSign: 1,
      });
      animateArmatureTo(target, durationMs, 'tpose');
    },
    [animateArmatureTo, engine]
  );

  const clearHoverDwell = useCallback(() => {
    if (hoverDwellTimerRef.current !== null) {
      window.clearTimeout(hoverDwellTimerRef.current);
      hoverDwellTimerRef.current = null;
    }
  }, []);

  const clearPlanquetteDwell = useCallback(() => {
    if (planquetteDwellTimerRef.current !== null) {
      window.clearTimeout(planquetteDwellTimerRef.current);
      planquetteDwellTimerRef.current = null;
    }
  }, []);

  const resetPlanquetteTracking = useCallback(() => {
    clearPlanquetteDwell();
    planquetteHoveredIndexRef.current = null;
    planquetteHoveredCharRef.current = null;
    dwellCommittedTileIdxRef.current = null;
    setPlanquetteHoveredIndex(null);
    setPlanquetteCursorWorld(null);
    setPlanquetteSource(null);
  }, [clearPlanquetteDwell]);

  useEffect(() => {
    if (introStage === 'input_name' && !cameraRef.current.userEdited) {
      const planCam = computePlanquetteCamera(
        viewportSizeRef.current.width,
        viewportSizeRef.current.height,
        DEFAULT_CANVAS_ORIGIN.x
      );
      setCamera({
        zoom: planCam.zoom,
        pan: planCam.pan,
        userEdited: false,
      });
    }
  }, [introStage]);

  const handleGreetingSequenceComplete = useCallback(() => {
    if (introStageRef.current === 'greeting_1') {
      setIntroStage('greeting_2');
    } else if (introStageRef.current === 'greeting_2') {
      setIntroStage('prompt');
    }
  }, []);

  const handleChooseIntroSide = useCallback(
    (choice: IntroChoice) => {
      clearHoverDwell();
      hoveredChoiceRef.current = null;
      faceplateDragWorldRef.current = null;
      setHoveredChoice(null);
      const base = createPosePreset('tpose', {
        ...engine.getArmature(),
        rootX: DEFAULT_CANVAS_ORIGIN.x,
        rootY: DEFAULT_CANVAS_ORIGIN.y,
        isAnchorPinned: true,
        hipToggleSign: 1,
      });
      const targetX =
        restLip.x + (choice === 'Y' ? -INTRO_TARGET_OFFSET_PX : INTRO_TARGET_OFFSET_PX) * 0.82;
      const looked = solveFaceplateLookIK(
        base,
        targetX,
        restLip.y,
        DEFAULT_CANVAS_ORIGIN
      );
      animateArmatureTo(looked, DEFAULT_TOGGLE_TRANSITION_MS, null);

      if (choice === 'Y') {
        setIntroStage('n_response');
        window.setTimeout(() => {
          if (introStageRef.current === 'n_response') {
            setIntroStage('input_name');
          }
        }, 1200);
      } else {
        setIntroStage('input_name');
      }
    },
    [animateArmatureTo, clearHoverDwell, engine, restLip.x, restLip.y]
  );

  const handleHoverChoice = useCallback(
    (choice: IntroChoice | null) => {
      const gatedChoice =
        isDraggingFaceplateRef.current && introStageRef.current === 'prompt' ? choice : null;
      if (hoveredChoiceRef.current === gatedChoice) return;
      hoveredChoiceRef.current = gatedChoice;
      setHoveredChoice(gatedChoice);
      clearHoverDwell();
      if (gatedChoice) {
        hoverDwellTimerRef.current = window.setTimeout(() => {
          hoverDwellTimerRef.current = null;
          hoveredChoiceRef.current = null;
          handleChooseIntroSide(gatedChoice);
        }, 1000);
      }
    },
    [clearHoverDwell, handleChooseIntroSide]
  );

  const handleFaceplateDragMove = useCallback(
    (drag: { pointer: Point2D; target: Point2D }) => {
      faceplateDragWorldRef.current = drag;
      if (!isDraggingFaceplateRef.current || introStageRef.current !== 'prompt') {
        handleHoverChoice(null);
        return;
      }
      const currentLip = engine.getJoint('lip');
      const nextChoice = resolveFaceplateTargetChoice(true, restLip, [
        drag.pointer,
        drag.target,
        currentLip,
      ]);
      handleHoverChoice(nextChoice);
    },
    [engine, handleHoverChoice, restLip]
  );

  const handleFaceplateDragChange = useCallback(
    (isDragging: boolean) => {
      isDraggingFaceplateRef.current = isDragging;
      setIsDraggingFaceplate(isDragging);

      if (isDragging) {
        faceplateDragWorldRef.current = null;
        if (introStageRef.current === 'prompt') {
          const currentLip = engine.getJoint('lip');
          handleHoverChoice(resolveFaceplateTargetChoice(true, restLip, [currentLip]));
        }
        return;
      }

      const lastDrag = faceplateDragWorldRef.current;
      faceplateDragWorldRef.current = null;
      handleHoverChoice(null);

      if (introStageRef.current !== 'prompt') return;

      const currentLip = engine.getJoint('lip');
      const placedChoice = resolveFaceplateTargetChoice(true, restLip, [
        currentLip,
        lastDrag?.target,
        lastDrag?.pointer,
      ]);
      if (placedChoice) {
        handleChooseIntroSide(placedChoice);
      } else {
        handleElastiHoseSnap();
      }
    },
    [engine, handleChooseIntroSide, handleElastiHoseSnap, handleHoverChoice, restLip]
  );

  const handlePlanquetteDragMove = useCallback(
    (info: {
      source: 'faceplate' | 'handR' | 'handL';
      effectorJointId: 'lip' | 'handR' | 'handL' | 'fingerTipR' | 'fingerTipL';
      effectorWorld: Point2D;
      headWorld?: Point2D;
      lipWorld?: Point2D;
      pointerWorld: Point2D;
      pointerScreen: Point2D;
    }) => {
      if (introStageRef.current !== 'input_name') return;

      const arc = computePlanquetteArc(DEFAULT_CANVAS_ORIGIN.x, DEFAULT_CANVAS_ORIGIN.y);

      let functionalCursorWorld: Point2D;
      if (info.source === 'faceplate') {
        const headW = info.headWorld ?? engine.getJoint('head');
        const lipW = info.lipWorld ?? engine.getJoint('lip');
        functionalCursorWorld = projectFaceplateToPlanquetteArc(
          headW,
          lipW,
          info.pointerWorld,
          arc
        );
      } else {
        functionalCursorWorld = info.effectorWorld;
      }

      setPlanquetteSource(info.source);
      setPlanquetteCursorWorld(functionalCursorWorld);

      const letter = hitTestPlanquetteArc(
        functionalCursorWorld.x,
        functionalCursorWorld.y,
        arc
      );
      const nextIdx = letter ? letter.index : null;
      if (nextIdx !== planquetteHoveredIndexRef.current) {
        clearPlanquetteDwell();
        dwellCommittedTileIdxRef.current = null;
        planquetteHoveredIndexRef.current = nextIdx;
        planquetteHoveredCharRef.current = letter ? letter.char : null;
        setPlanquetteHoveredIndex(nextIdx);

        if (letter) {
          const committedIdx = letter.index;
          const committedChar = letter.char;
          planquetteDwellTimerRef.current = window.setTimeout(() => {
            planquetteDwellTimerRef.current = null;
            if (introStageRef.current !== 'input_name') return;
            if (planquetteHoveredIndexRef.current !== committedIdx) return;
            dwellCommittedTileIdxRef.current = committedIdx;
            setPlanquetteName((prev) => (prev.length < 32 ? prev + committedChar : prev));
          }, 1000);
        }
      }
    },
    [clearPlanquetteDwell, engine]
  );

  const handlePlanquetteDragChange = useCallback(
    (info: { isDragging: boolean; source: 'faceplate' | 'handR' | 'handL' | null }) => {
      if (info.isDragging) {
        if (introStageRef.current === 'input_name') {
          setPlanquetteSource(info.source);
        }
        return;
      }

      if (introStageRef.current === 'input_name') {
        const hoveredIdx = planquetteHoveredIndexRef.current;
        const hoveredChar = planquetteHoveredCharRef.current;
        if (
          hoveredIdx !== null &&
          hoveredIdx !== dwellCommittedTileIdxRef.current &&
          hoveredChar !== null
        ) {
          setPlanquetteName((prev) => (prev.length < 32 ? prev + hoveredChar : prev));
        }
      }
      resetPlanquetteTracking();
    },
    [resetPlanquetteTracking]
  );

  const handleSubmitPyrsonName = useCallback(
    (_rawName: string) => {
      resetPlanquetteTracking();
      setPlanquetteName('');
      handleElastiHoseSnap();
      setIntroStage('processing_1');

      for (const id of introTimersRef.current) {
        window.clearTimeout(id);
      }
      if (welcomeTimerRef.current !== null) {
        window.clearTimeout(welcomeTimerRef.current);
        welcomeTimerRef.current = null;
      }

      const ids: number[] = [];
      ids.push(window.setTimeout(() => setIntroStage('processing_2'), 500));
      ids.push(window.setTimeout(() => setIntroStage('processing_3'), 1000));
      ids.push(window.setTimeout(() => setIntroStage('processing_4'), 1500));
      ids.push(window.setTimeout(() => setIntroStage('processing_5'), 2000));
      ids.push(window.setTimeout(() => setIntroStage('processing_6'), 2500));
      ids.push(window.setTimeout(() => setIntroStage('not_found'), 3000));
      ids.push(
        window.setTimeout(() => {
          setIntroStage('facsimile');
        }, 6000)
      );
      ids.push(
        window.setTimeout(() => {
          setPyrsonName('Bytruvyus');
          setIntroStage('welcomed');
          if (!cameraRef.current.userEdited) {
            fitSymmetricalView(viewportSizeRef.current);
          }
        }, 8000)
      );
      introTimersRef.current = ids;
    },
    [fitSymmetricalView, handleElastiHoseSnap, resetPlanquetteTracking]
  );

  const handleBackToPrompt = useCallback(() => {
    resetPlanquetteTracking();
    setPlanquetteName('');
    setIntroStage('prompt');
    resetRigToNeutralTPose(DEFAULT_TOGGLE_TRANSITION_MS);
    if (!cameraRef.current.userEdited) {
      const introCam = computeIntroCamera(
        viewportSizeRef.current.width,
        viewportSizeRef.current.height,
        restLip
      );
      setCamera({
        zoom: introCam.zoom,
        pan: introCam.pan,
        userEdited: false,
      });
    }
  }, [resetPlanquetteTracking, resetRigToNeutralTPose, restLip]);

  return (
    <div className="h-screen w-screen flex flex-col bg-[#16001E] text-[#EFEAF4] overflow-hidden">
      {/* Slim Top Control Bar — Reset T-Pose, Core IK/Floor Constraints & Substrate */}
      <header className="h-10 px-3 sm:px-6 bg-[#16001E] border-b border-[#EBE0F5]/14 flex items-center justify-between gap-2 sm:gap-4 shrink-0 z-20">
        <div className="flex items-center gap-2.5 shrink-0 min-w-0">
          <a
            href="#top"
            onClick={(e) => {
              e.preventDefault();
              if (introStage === 'complete' || introStage === 'welcomed') {
                fitSymmetricalView();
              } else if (introStage === 'input_name') {
                const planCam = computePlanquetteCamera(
                  viewportSizeRef.current.width,
                  viewportSizeRef.current.height,
                  DEFAULT_CANVAS_ORIGIN.x
                );
                setCamera({
                  zoom: planCam.zoom,
                  pan: planCam.pan,
                  userEdited: false,
                });
              } else {
                const introCam = computeIntroCamera(
                  viewportSizeRef.current.width,
                  viewportSizeRef.current.height,
                  restLip
                );
                setCamera({
                  zoom: introCam.zoom,
                  pan: introCam.pan,
                  userEdited: false,
                });
              }
            }}
            className="font-typewriter font-bold text-xs sm:text-base tracking-wider text-[#F4EFF9] whitespace-nowrap shrink-0 focus-visible:outline-2 focus-visible:outline-[#D8B4FE]"
          >
            Bitruvius
          </a>
          {pyrsonName && (
            <span className="font-typewriter text-xs sm:text-sm text-[#D8B4FE] truncate">
              · {pyrsonName}
            </span>
          )}
        </div>

        <div className="flex items-center gap-1.5 shrink-0">
          <button
            type="button"
            onClick={handleCycleSolver}
            title="Cycle IK solver (Analytical · FABRIK · CCD)"
            className="px-2 py-1 font-vera text-[10.5px] font-bold rounded-[2px] border bg-[#1D1026] border-[#EBE0F5]/14 text-[#D9CBE8] hover:text-[#FAF6FF] flex items-center gap-1 whitespace-nowrap shrink-0 cursor-pointer"
          >
            <SlidersHorizontal className="w-3.5 h-3.5" />
            <span>IK: {SOLVER_LABELS[constraints.solver]}</span>
          </button>

          <button
            type="button"
            aria-pressed={constraints.softLimits}
            onClick={() =>
              handleUpdateRigSystems({ softLimits: !constraints.softLimits })
            }
            title="Toggle anatomical joint angle limits"
            className={`px-2 py-1 font-vera text-[10.5px] font-bold rounded-[2px] border flex items-center gap-1 whitespace-nowrap shrink-0 transition-colors cursor-pointer ${
              constraints.softLimits
                ? 'bg-[#2C143B] border-[#D8B4FE]/40 text-[#FAF6FF]'
                : 'bg-[#1D1026] border-[#EBE0F5]/14 text-[#B8A9C9] hover:text-[#FAF6FF]'
            }`}
          >
            <ShieldCheck className="w-3.5 h-3.5" />
            <span className="hidden sm:inline">Limits</span>
          </button>

          <button
            type="button"
            aria-pressed={constraints.floorSnap}
            onClick={() =>
              handleUpdateRigSystems({ floorSnap: !constraints.floorSnap })
            }
            title="Toggle ground floor snap constraint"
            className={`px-2 py-1 font-vera text-[10.5px] font-bold rounded-[2px] border flex items-center gap-1 whitespace-nowrap shrink-0 transition-colors cursor-pointer ${
              constraints.floorSnap
                ? 'bg-[#2C143B] border-[#D8B4FE]/40 text-[#FAF6FF]'
                : 'bg-[#1D1026] border-[#EBE0F5]/14 text-[#B8A9C9] hover:text-[#FAF6FF]'
            }`}
          >
            <Anchor className="w-3.5 h-3.5" />
            <span className="hidden sm:inline">Floor Snap</span>
          </button>

          <span className="h-4 w-px bg-[#EBE0F5]/14 mx-0.5" aria-hidden="true" />

          <button
            type="button"
            onClick={handleResetTPose}
            title="Reset rig to canonical T-Pose"
            className="px-2 py-1 font-vera text-[10.5px] font-bold rounded-[2px] border bg-[#1D1026] border-[#EBE0F5]/20 text-[#EFEAF4] hover:text-white hover:bg-[#2C143B] flex items-center gap-1 whitespace-nowrap shrink-0 cursor-pointer"
          >
            <RotateCcw className="w-3.5 h-3.5" />
            <span>T-Pose</span>
          </button>

          <button
            type="button"
            aria-pressed={darkMode}
            onClick={() => setDarkMode((prev) => !prev)}
            title="Toggle Dark Mode"
            className={`px-2 py-1 font-vera text-[10.5px] font-bold rounded-[2px] border flex items-center gap-1 whitespace-nowrap shrink-0 transition-colors cursor-pointer ${
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

      {/* Primary Interactive Posing Canvas */}
      <main className="flex-1 w-full min-w-0 min-h-0 flex flex-col relative">
        <BookStageCanvas
          pageId={pageId}
          grid={DEFAULT_GRID}
          darkMode={darkMode}
          zoom={camera.zoom}
          pan={camera.pan}
          armature={pose.armature}
          computedJoints={computedJoints}
          selectedJointId={selectedJointId}
          onZoomChange={handleZoomChange}
          onPanChange={handlePanChange}
          onViewportResize={handleViewportResize}
          onCursorMove={handleCursorMove}
          onSelectJoint={setSelectedJointId}
          onUpdateArmature={handleUpdateArmature}
          onAnimateArmature={handleAnimateArmature}
          isPoseTransitioning={isPoseTransitioning}
          onCommitPoseHistory={commitPoseHistory}
          rigSystems={constraints}
          onUpdateRigSystems={handleUpdateRigSystems}
          planquetteReachActive={introStage === 'input_name'}
          onFaceplateDragChange={handleFaceplateDragChange}
          onFaceplateDragMove={handleFaceplateDragMove}
          onPlanquetteDragChange={handlePlanquetteDragChange}
          onPlanquetteDragMove={handlePlanquetteDragMove}
          onDragEnd={handleElastiHoseSnap}
          worldOverlay={
            introStage === 'prompt' ? (
              <FirstScreenSvgOverlay
                restLip={restLip}
                currentLip={computedJoints.lip}
                evaluation={faceplateEvaluation}
                isDraggingFaceplate={isDraggingFaceplate}
                hoveredChoice={hoveredChoice}
                darkMode={darkMode}
              />
            ) : introStage === 'input_name' ? (
              <FirstScreenPlanquetteSvgOverlay
                origin={DEFAULT_CANVAS_ORIGIN}
                lipWorld={computedJoints.lip}
                hoveredIndex={planquetteHoveredIndex}
                cursorWorld={planquetteCursorWorld}
                source={planquetteSource}
                darkMode={darkMode}
              />
            ) : undefined
          }
        />

        <FirstScreenHudOverlay
          stage={introStage}
          pan={camera.pan}
          zoom={camera.zoom}
          restLip={restLip}
          figureCenter={DEFAULT_CANVAS_ORIGIN}
          viewportHeight={viewportSize.height}
          darkMode={darkMode}
          evaluation={faceplateEvaluation}
          isDraggingFaceplate={isDraggingFaceplate}
          hoveredChoice={hoveredChoice}
          committedName={pyrsonName}
          planquetteName={planquetteName}
          onChooseSide={handleChooseIntroSide}
          onSubmitName={handleSubmitPyrsonName}
          onHoverChoice={handleHoverChoice}
          onGreetingSequenceComplete={handleGreetingSequenceComplete}
        />
      </main>

      {/* Minimal Bottom Coordinate & Camera Bar */}
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
          <span className="hidden sm:inline text-[#9E8EB3]">·</span>
          <span className="hidden sm:inline text-[#C8B2E0]">
            {selectedSpec ? `${selectedSpec.label} (${selectedSpec.shortCode})` : selectedJointId}
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
            title="Set 100% scale"
            className="px-2 py-0.5 font-typewriter font-bold text-[11px] text-[#FAF6FF] hover:bg-[#2A1D36] rounded-[2px] cursor-pointer min-w-[48px] text-center whitespace-nowrap"
          >
            {(camera.zoom * 100).toFixed(0)}%
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
            onClick={() => fitSymmetricalView()}
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
