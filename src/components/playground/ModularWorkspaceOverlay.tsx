import React, { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import {
  Anchor,
  ArrowDownToLine,
  ChevronDown,
  ChevronUp,
  Compass,
  Crosshair,
  FlipHorizontal2,
  GripHorizontal,
  Hand,
  Image as ImageIcon,
  Link2,
  Move,
  PanelRight,
  PanelRightClose,
  Pin,
  RefreshCw,
  RotateCcw,
  ShieldCheck,
  SlidersHorizontal,
  Upload,
  X,
} from 'lucide-react';
import { JOINT_SPEC_MAP } from '../../data/digitalRig';
import {
  ArmatureState,
  BONE_SCALE_MAX,
  BONE_SCALE_MIN,
  EnvironmentalAnchorId,
  HandGesturePreset,
  JointId,
  JointMode,
  PhysicsModeId,
  RigSystemsConfig,
  ToggleHingeFoldState,
} from '../../types/rig';
import {
  CompositeBlendMode,
  CompositeFilter,
  SamplePlateId,
  Scene,
  SceneMedia,
  StageTransform,
  WorldPin,
} from '../../types/scene';
import {
  MODEL_MODULE_META,
  ModelModuleId,
  ModularPanelLayoutState,
} from '../../types/workspace';
import { FPS_OPTIONS, ScreenFpsOption } from '../../utils/frameThrottle';
import { formatSignedDeg } from '../../utils/kinematics';

export interface ModularWorkspaceOverlayProps {
  railVisible: boolean;
  panels: Record<ModelModuleId, ModularPanelLayoutState>;
  onToggleRail: () => void;
  onToggleModule: (moduleId: ModelModuleId) => void;
  onSetModuleCollapsed: (moduleId: ModelModuleId, collapsed: boolean) => void;
  onSetModuleDockMode: (
    moduleId: ModelModuleId,
    dockMode: 'docked' | 'floating',
    position?: { x: number; y: number }
  ) => void;
  onSetModulePosition: (moduleId: ModelModuleId, position: { x: number; y: number }) => void;
  onBringModuleToFront: (moduleId: ModelModuleId) => void;

  // Rig & Model State
  armature: ArmatureState;
  selectedJointId: JointId;
  rigSystems: RigSystemsConfig;
  showPerspective: boolean;
  vanishingPoint: { x: number; y: number };
  activeScene: Scene;

  // Quick Toggles & Model Handlers
  onMirrorPose: () => void;
  onResetPose: () => void;
  onBatchSetIK: (enableAllLimbs: boolean) => void;
  onUpdateRigSystems: (patch: Partial<RigSystemsConfig>) => void;
  onUpdateArmature: (next: ArmatureState) => void;
  onSetJointMode: (id: JointId, mode: JointMode) => void;
  onSetJointPhysics: (id: JointId, physics: PhysicsModeId) => void;
  onSetJointAngle: (id: JointId, angle: number) => void;
  onSetJointScale: (id: JointId, scaleLength: number, scaleWidth: number) => void;
  onNudgeSelectedJoint: (deltaDeg: number) => void;
  onZeroSelectedJoint: () => void;
  onSetHipToggle: (state: ToggleHingeFoldState) => void;
  onSetCollarToggle: (side: 'R' | 'L', state: ToggleHingeFoldState) => void;
  onToggleArmSwivel: (side: 'R' | 'L') => void;
  onApplyHandGesture: (side: 'right' | 'left' | 'both', gesture: HandGesturePreset) => void;
  onTogglePerspective: (show: boolean) => void;
  onResetVanishingPoint: () => void;

  // Scene & Plate Calibration Handlers
  onSelectPaperPlate: (variant: 'cover' | 'backing') => void;
  onSelectSamplePlate: (sampleId: SamplePlateId) => void;
  onUploadMediaFile: (file: File) => void;
  onUpdateSceneMedia: (patch: Partial<SceneMedia>) => void;
  onUpdateStageTransform: (patch: Partial<StageTransform>) => void;
  onResetStageTransform: () => void;
  onUpdateCompositeFilter: (patch: Partial<CompositeFilter>) => void;
  onToggleWorldPinForJoint: (jointId: JointId) => void;
  onUpdateWorldPinConfig: (pinId: string, patch: Partial<WorldPin>) => void;
  onRemoveWorldPin: (pinId: string) => void;
}

const HAND_GESTURES: ReadonlyArray<{ id: HandGesturePreset; label: string }> = [
  { id: 'open', label: 'Open Palm' },
  { id: 'relaxed', label: 'Relaxed' },
  { id: 'point', label: 'Point' },
  { id: 'pinch', label: 'Pinch' },
  { id: 'grasp', label: 'Grasp' },
  { id: 'fist', label: 'Fist' },
];

const ANCHOR_OPTIONS: ReadonlyArray<{ id: EnvironmentalAnchorId; label: string }> = [
  { id: 'navel', label: 'Navel (Keystone)' },
  { id: 'bottom_of_spine', label: 'Pelvis Root' },
  { id: 'feet', label: 'Feet Base' },
  { id: 'clavicle', label: 'Clavicle' },
  { id: 'neck', label: 'Neck' },
  { id: 'crown', label: 'Crown' },
];

const ARM_OR_DIGIT_JOINTS = new Set<JointId>([
  'humerusR',
  'forearmR',
  'handR',
  'thumbR',
  'fingerR',
  'thumbTipR',
  'fingerTipR',
  'humerusL',
  'forearmL',
  'handL',
  'thumbL',
  'fingerL',
  'thumbTipL',
  'fingerTipL',
]);

const MODULE_ICONS: Record<ModelModuleId, React.ReactNode> = {
  rig_joint: <SlidersHorizontal className="w-4 h-4" />,
  digits: <Hand className="w-4 h-4" />,
  perspective: <Crosshair className="w-4 h-4" />,
  scene: <ImageIcon className="w-4 h-4" />,
};

interface ActivePanelDrag {
  moduleId: ModelModuleId;
  pointerId: number;
  offsetX: number;
  offsetY: number;
  panelWidth: number;
  panelHeight: number;
  currentX: number;
  currentY: number;
  snapHint: 'right-dock' | 'left' | 'top' | 'bottom' | null;
}

const PANEL_WIDTH = 304;
const EDGE_MARGIN = 10;
const RAIL_RESERVE_PX = 62;
const DOCK_SNAP_THRESHOLD = 66;
const EDGE_SNAP_THRESHOLD = 22;

export const ModularWorkspaceOverlay: React.FC<ModularWorkspaceOverlayProps> = React.memo(({
  railVisible,
  panels,
  onToggleRail,
  onToggleModule,
  onSetModuleCollapsed,
  onSetModuleDockMode,
  onSetModulePosition,
  onBringModuleToFront,
  armature,
  selectedJointId,
  rigSystems,
  showPerspective,
  vanishingPoint,
  activeScene,
  onMirrorPose,
  onResetPose,
  onBatchSetIK,
  onUpdateRigSystems,
  onUpdateArmature,
  onSetJointMode,
  onSetJointPhysics,
  onSetJointAngle,
  onSetJointScale,
  onNudgeSelectedJoint,
  onZeroSelectedJoint,
  onSetHipToggle,
  onSetCollarToggle,
  onToggleArmSwivel,
  onApplyHandGesture,
  onTogglePerspective,
  onResetVanishingPoint,
  onSelectPaperPlate,
  onSelectSamplePlate,
  onUploadMediaFile,
  onUpdateSceneMedia,
  onUpdateStageTransform,
  onResetStageTransform,
  onUpdateCompositeFilter,
  onToggleWorldPinForJoint,
  onUpdateWorldPinConfig,
  onRemoveWorldPin,
}) => {
  const overlayRef = useRef<HTMLDivElement | null>(null);
  const [handSide, setHandSide] = useState<'right' | 'left' | 'both'>('both');
  const [dragState, setDragState] = useState<ActivePanelDrag | null>(null);
  const dragRef = useRef<ActivePanelDrag | null>(null);

  const selectedSpec = JOINT_SPEC_MAP[selectedJointId];
  const selectedRt = armature.joints[selectedJointId];
  const handRefSide = handSide === 'left' ? 'L' : 'R';
  const wristId: JointId = `hand${handRefSide}`;
  const thumbBaseId: JointId = `thumb${handRefSide}`;
  const fingerBaseId: JointId = `finger${handRefSide}`;

  const anyLimbIK = useMemo(() => {
    const limbIds: JointId[] = ['forearmR', 'forearmL', 'shinR', 'shinL', 'handR', 'handL'];
    return limbIds.some((id) => armature.joints[id]?.mode === 'IK');
  }, [armature.joints]);

  const dockedModules = useMemo(
    () =>
      MODEL_MODULE_META.filter(
        (m) => panels[m.id].isOpen && panels[m.id].dockMode === 'docked'
      ),
    [panels]
  );

  const floatingModules = useMemo(
    () =>
      MODEL_MODULE_META.filter(
        (m) => panels[m.id].isOpen && panels[m.id].dockMode === 'floating'
      ),
    [panels]
  );

  const computeDragPosition = useCallback(
    (
      clientX: number,
      clientY: number,
      offsetX: number,
      offsetY: number,
      panelWidth: number,
      panelHeight: number
    ) => {
      const bounds = overlayRef.current?.getBoundingClientRect();
      const stageW = bounds?.width ?? 1280;
      const stageH = bounds?.height ?? 720;
      const rawX = clientX - (bounds?.left ?? 0) - offsetX;
      const rawY = clientY - (bounds?.top ?? 0) - offsetY;

      const maxFloatingX = Math.max(EDGE_MARGIN, stageW - panelWidth - (railVisible ? RAIL_RESERVE_PX : EDGE_MARGIN));
      const maxFloatingY = Math.max(EDGE_MARGIN, stageH - Math.min(panelHeight, 120));

      let x = Math.max(EDGE_MARGIN, Math.min(maxFloatingX, rawX));
      let y = Math.max(EDGE_MARGIN, Math.min(maxFloatingY, rawY));
      let snapHint: ActivePanelDrag['snapHint'] = null;

      if (railVisible && rawX + panelWidth >= stageW - DOCK_SNAP_THRESHOLD) {
        snapHint = 'right-dock';
      } else if (rawX <= EDGE_SNAP_THRESHOLD) {
        x = EDGE_MARGIN;
        snapHint = 'left';
      } else if (rawY <= EDGE_SNAP_THRESHOLD) {
        y = EDGE_MARGIN;
        snapHint = 'top';
      } else if (rawY + panelHeight >= stageH - EDGE_SNAP_THRESHOLD) {
        y = Math.max(EDGE_MARGIN, stageH - panelHeight - EDGE_MARGIN);
        snapHint = 'bottom';
      }

      return { x, y, snapHint };
    },
    [railVisible]
  );

  const handleStartPanelDrag = useCallback(
    (
      e: React.PointerEvent<HTMLDivElement>,
      moduleId: ModelModuleId,
      panelEl: HTMLElement | null
    ) => {
      if (e.button !== 0 || !panelEl) return;
      e.preventDefault();
      e.stopPropagation();

      onBringModuleToFront(moduleId);
      const panelRect = panelEl.getBoundingClientRect();
      const stageRect = overlayRef.current?.getBoundingClientRect();
      const offsetX = e.clientX - panelRect.left;
      const offsetY = e.clientY - panelRect.top;
      const initX = panelRect.left - (stageRect?.left ?? 0);
      const initY = panelRect.top - (stageRect?.top ?? 0);

      if (panels[moduleId].dockMode === 'docked') {
        onSetModuleDockMode(moduleId, 'floating', {
          x: Math.max(EDGE_MARGIN, initX),
          y: Math.max(EDGE_MARGIN, initY),
        });
      }

      const nextDrag: ActivePanelDrag = {
        moduleId,
        pointerId: e.pointerId,
        offsetX,
        offsetY,
        panelWidth: panelRect.width || PANEL_WIDTH,
        panelHeight: panelRect.height || 260,
        currentX: Math.max(EDGE_MARGIN, initX),
        currentY: Math.max(EDGE_MARGIN, initY),
        snapHint: null,
      };
      dragRef.current = nextDrag;
      setDragState(nextDrag);
    },
    [onBringModuleToFront, onSetModuleDockMode, panels]
  );

  const isDraggingPanel = dragState !== null;

  useEffect(() => {
    if (!isDraggingPanel) return;

    const onPointerMove = (e: PointerEvent) => {
      const cur = dragRef.current;
      if (!cur || e.pointerId !== cur.pointerId) return;
      const nextPos = computeDragPosition(
        e.clientX,
        e.clientY,
        cur.offsetX,
        cur.offsetY,
        cur.panelWidth,
        cur.panelHeight
      );
      const updated: ActivePanelDrag = {
        ...cur,
        currentX: nextPos.x,
        currentY: nextPos.y,
        snapHint: nextPos.snapHint,
      };
      dragRef.current = updated;
      setDragState(updated);
    };

    const onPointerUp = (e: PointerEvent) => {
      const cur = dragRef.current;
      if (!cur || e.pointerId !== cur.pointerId) return;
      dragRef.current = null;
      setDragState(null);
      if (cur.snapHint === 'right-dock') {
        onSetModuleDockMode(cur.moduleId, 'docked');
      } else {
        onSetModulePosition(cur.moduleId, { x: cur.currentX, y: cur.currentY });
      }
    };

    window.addEventListener('pointermove', onPointerMove);
    window.addEventListener('pointerup', onPointerUp);
    window.addEventListener('pointercancel', onPointerUp);
    return () => {
      window.removeEventListener('pointermove', onPointerMove);
      window.removeEventListener('pointerup', onPointerUp);
      window.removeEventListener('pointercancel', onPointerUp);
    };
  }, [computeDragPosition, isDraggingPanel, onSetModuleDockMode, onSetModulePosition]);

  const renderModuleBody = (moduleId: ModelModuleId) => {
    if (moduleId === 'rig_joint') {
      return (
        <div className="space-y-3.5">
          {selectedSpec && selectedRt && (
            <div className="space-y-2.5">
              <div className="flex items-center justify-between gap-2">
                <div className="flex items-center gap-1.5 min-w-0">
                  <span className="text-[10px] font-bold uppercase tracking-widest text-[#D8B4FE] truncate">
                    {selectedSpec.label}
                  </span>
                  <span className="font-typewriter text-[10px] text-[#9E8EB3] shrink-0">
                    {selectedSpec.shortCode}
                  </span>
                </div>
                <span className="font-typewriter font-bold text-[#FAF6FF] tabular-nums shrink-0">
                  {formatSignedDeg(
                    selectedJointId === 'pelvis' ? armature.rootRotation : selectedRt.localAngle
                  )}
                </span>
              </div>

              <div className="grid grid-cols-3 gap-1 tabular-nums">
                <button
                  type="button"
                  disabled={selectedSpec.locked}
                  onClick={() => onNudgeSelectedJoint(-5)}
                  title="Rotate selected joint -5° (Shift+Left)"
                  className="py-1 px-2 bg-[#1B0825] border border-[#EBE0F5]/16 rounded-[2px] text-[#E6DCF0] hover:bg-[#361547] disabled:opacity-35 font-bold cursor-pointer whitespace-nowrap"
                >
                  -5°
                </button>
                <button
                  type="button"
                  disabled={selectedSpec.locked}
                  onClick={onZeroSelectedJoint}
                  title="Zero selected joint angle"
                  className="py-1 px-2 bg-[#1B0825] border border-[#EBE0F5]/16 rounded-[2px] text-[#C8B2E0] hover:bg-[#361547] disabled:opacity-35 font-bold cursor-pointer whitespace-nowrap"
                >
                  0°
                </button>
                <button
                  type="button"
                  disabled={selectedSpec.locked}
                  onClick={() => onNudgeSelectedJoint(5)}
                  title="Rotate selected joint +5° (Shift+Right)"
                  className="py-1 px-2 bg-[#1B0825] border border-[#EBE0F5]/16 rounded-[2px] text-[#E6DCF0] hover:bg-[#361547] disabled:opacity-35 font-bold cursor-pointer whitespace-nowrap"
                >
                  +5°
                </button>
              </div>

              {(selectedJointId === 'pelvis' ||
                selectedJointId === 'hipR' ||
                selectedJointId === 'hipL') && (
                <div className="space-y-1">
                  <span className="block text-[10px] text-[#B8A9C9]">Hip Fold</span>
                  <div className="grid grid-cols-3 gap-1" role="group" aria-label="Hip toggle fold">
                    {(['A', 'extended', 'V'] as ToggleHingeFoldState[]).map((state) => (
                      <button
                        key={`hip-toggle-${state}`}
                        type="button"
                        onClick={() => onSetHipToggle(state)}
                        className="py-1 px-2 bg-[#1B0825] border border-[#34D399]/35 rounded-[2px] text-[#6EE7B7] hover:bg-[#112A22] font-bold cursor-pointer whitespace-nowrap"
                      >
                        {state === 'extended' ? '180°' : state}
                      </button>
                    ))}
                  </div>
                </div>
              )}

              {(selectedJointId === 'braceR' ||
                selectedJointId === 'clavOuterR' ||
                selectedJointId === 'braceL' ||
                selectedJointId === 'clavOuterL') && (() => {
                const side: 'R' | 'L' =
                  selectedJointId === 'braceR' || selectedJointId === 'clavOuterR' ? 'R' : 'L';
                return (
                  <div className="space-y-1">
                    <span className="block text-[10px] text-[#B8A9C9]">Collar Fold ({side})</span>
                    <div
                      className="grid grid-cols-3 gap-1"
                      role="group"
                      aria-label="Collarbone toggle fold"
                    >
                      {(['V', 'extended', 'A'] as ToggleHingeFoldState[]).map((state) => (
                        <button
                          key={`collar-toggle-${state}`}
                          type="button"
                          onClick={() => onSetCollarToggle(side, state)}
                          className="py-1 px-2 bg-[#1B0825] border border-[#34D399]/35 rounded-[2px] text-[#6EE7B7] hover:bg-[#112A22] font-bold cursor-pointer whitespace-nowrap"
                        >
                          {state === 'extended' ? '180°' : state}
                        </button>
                      ))}
                    </div>
                  </div>
                );
              })()}

              {ARM_OR_DIGIT_JOINTS.has(selectedJointId) && (() => {
                const armSide: 'R' | 'L' = selectedJointId.endsWith('R') ? 'R' : 'L';
                const swivelDeg =
                  armSide === 'R' ? (armature.armSwivelR ?? 0) : (armature.armSwivelL ?? 0);
                const isReversed = Math.cos((swivelDeg * Math.PI) / 180) < 0;
                return (
                  <button
                    type="button"
                    aria-pressed={isReversed}
                    onClick={() => onToggleArmSwivel(armSide)}
                    title={`Swivel ${armSide} arm axially (0° <-> 180°)`}
                    className={`w-full py-1 px-2 border rounded-[2px] font-bold cursor-pointer whitespace-nowrap transition-colors flex items-center justify-between ${
                      isReversed
                        ? 'bg-[#112A22] border-[#34D399]/65 text-[#6EE7B7]'
                        : 'bg-[#1B0825] border-[#EBE0F5]/16 text-[#D9CBE8] hover:text-[#FAF6FF]'
                    }`}
                  >
                    <span>Arm Swivel ({armSide})</span>
                    <span className="font-typewriter tabular-nums">
                      {Math.round(Math.abs(swivelDeg))}°
                    </span>
                  </button>
                );
              })()}

              {selectedJointId !== 'pelvis' && !selectedSpec.locked && (
                <div className="grid grid-cols-3 gap-1">
                  {(['FK', 'IK', 'PIN'] as JointMode[]).map((mode) => {
                    const disabled = mode === 'IK' && !selectedSpec.ikChain;
                    return (
                      <button
                        key={mode}
                        type="button"
                        disabled={disabled}
                        onClick={() => onSetJointMode(selectedJointId, mode)}
                        className={`py-1 rounded-[2px] border font-bold text-[10px] cursor-pointer disabled:opacity-30 ${
                          selectedRt.mode === mode
                            ? 'bg-[#361547] border-[#D8B4FE]/60 text-white'
                            : 'bg-[#1B0825] border-[#EBE0F5]/14 text-[#B8A9C9]'
                        }`}
                      >
                        {mode}
                      </button>
                    );
                  })}
                </div>
              )}

              <label className="block space-y-1">
                <span className="text-[10px] text-[#B8A9C9]">Physics Damping</span>
                <select
                  value={selectedRt.physicsMode}
                  onChange={(e) =>
                    onSetJointPhysics(selectedJointId, e.target.value as PhysicsModeId)
                  }
                  className="w-full px-2 py-1 rounded-[2px] border border-[#EBE0F5]/18 bg-[#120319] text-[#FAF6FF]"
                >
                  <option value="twins">Twins (1.00× Direct)</option>
                  <option value="elastic_rubber">Elastic (1.50× Sweep)</option>
                  <option value="damp">Damp (0.50× Precision)</option>
                  <option value="rigid">Rigid (0.00× Locked)</option>
                </select>
              </label>

              <label className="block space-y-1">
                <div className="flex justify-between text-[10px] text-[#B8A9C9]">
                  <span>Segment Length</span>
                  <span className="font-typewriter tabular-nums">
                    {selectedRt.scaleLength.toFixed(2)}×
                  </span>
                </div>
                <input
                  type="range"
                  min={BONE_SCALE_MIN}
                  max={BONE_SCALE_MAX}
                  step={0.05}
                  value={selectedRt.scaleLength}
                  onChange={(e) =>
                    onSetJointScale(
                      selectedJointId,
                      parseFloat(e.target.value),
                      selectedRt.scaleWidth
                    )
                  }
                  className="w-full accent-[#D8B4FE]"
                />
              </label>
            </div>
          )}

          <div className="pt-3 border-t border-[#EBE0F5]/12 space-y-2">
            <span className="block text-[10px] font-bold uppercase tracking-widest text-[#B8A9C9]">
              Kinematics Solver Engine
            </span>
            <div className="grid grid-cols-3 gap-1">
              {(['analytical', 'fabrik', 'ccd'] as const).map((solverId) => (
                <button
                  key={solverId}
                  type="button"
                  onClick={() => onUpdateRigSystems({ solver: solverId })}
                  className={`py-1 px-2 rounded-[2px] border font-bold uppercase text-[10px] cursor-pointer ${
                    rigSystems.solver === solverId
                      ? 'bg-[#112A22] border-[#34D399]/60 text-[#6EE7B7]'
                      : 'bg-[#1B0825] border-[#EBE0F5]/14 text-[#C8B2E0] hover:text-white'
                  }`}
                >
                  {solverId}
                </button>
              ))}
            </div>
            <div className="grid grid-cols-2 gap-1.5">
              <button
                type="button"
                onClick={() => onBatchSetIK(true)}
                className="py-1 px-2 rounded-[2px] border border-[#34D399]/40 bg-[#112A22] text-[#6EE7B7] hover:bg-[#173D31] font-bold cursor-pointer text-[10px] whitespace-nowrap"
              >
                All Limbs → IK
              </button>
              <button
                type="button"
                onClick={() => onBatchSetIK(false)}
                className="py-1 px-2 rounded-[2px] border border-[#EBE0F5]/18 bg-[#1D0B28] text-[#D9CBE8] hover:text-white font-bold cursor-pointer text-[10px] whitespace-nowrap"
              >
                Reset All → FK
              </button>
            </div>
          </div>

          <div className="pt-3 border-t border-[#EBE0F5]/12 space-y-2">
            <label className="block space-y-1">
              <span className="text-[10px] text-[#B8A9C9]">Environmental Anchor</span>
              <select
                value={armature.environmentalAnchor}
                onChange={(e) =>
                  onUpdateArmature({
                    ...armature,
                    environmentalAnchor: e.target.value as EnvironmentalAnchorId,
                  })
                }
                className="w-full px-2 py-1 rounded-[2px] border border-[#EBE0F5]/18 bg-[#120319] text-[#FAF6FF]"
              >
                {ANCHOR_OPTIONS.map((opt) => (
                  <option key={opt.id} value={opt.id}>
                    {opt.label}
                  </option>
                ))}
              </select>
            </label>
          </div>
        </div>
      );
    }

    if (moduleId === 'digits') {
      return (
        <div className="space-y-3.5">
          <div>
            <span className="block text-[10px] font-bold uppercase tracking-widest text-[#B8A9C9] mb-1.5">
              Target Hand Sub-Rig
            </span>
            <div className="grid grid-cols-3 gap-1">
              {(['right', 'both', 'left'] as const).map((side) => (
                <button
                  key={side}
                  type="button"
                  onClick={() => setHandSide(side)}
                  className={`py-1 rounded-[2px] border font-bold capitalize cursor-pointer ${
                    handSide === side
                      ? 'bg-[#361547] border-[#34D399]/60 text-white'
                      : 'bg-[#1B0825] border-[#EBE0F5]/14 text-[#B8A9C9]'
                  }`}
                >
                  {side}
                </button>
              ))}
            </div>
          </div>

          <div>
            <span className="block text-[10px] font-bold uppercase tracking-widest text-[#B8A9C9] mb-1.5">
              Hand Gesture Presets
            </span>
            <div className="grid grid-cols-2 gap-1.5">
              {HAND_GESTURES.map((g) => (
                <button
                  key={g.id}
                  type="button"
                  onClick={() => onApplyHandGesture(handSide, g.id)}
                  className="px-2 py-1.5 rounded-[2px] border border-[#EBE0F5]/16 bg-[#1B0825] text-[#E9D5FF] hover:border-[#34D399]/55 hover:bg-[#240B31] font-bold cursor-pointer text-left truncate"
                >
                  {g.label}
                </button>
              ))}
            </div>
          </div>

          <div className="pt-3 border-t border-[#EBE0F5]/12 space-y-2">
            {(
              [
                { part: 'hand' as const, label: 'Wrist Flexion', id: wristId, min: -85, max: 85 },
                { part: 'thumb' as const, label: 'Thumb Opposition', id: thumbBaseId, min: -75, max: 75 },
                { part: 'finger' as const, label: 'Fingers Curl', id: fingerBaseId, min: -45, max: 95 },
              ]
            ).map((item) => {
              const val = armature.joints[item.id]?.localAngle ?? 0;
              return (
                <label key={item.part} className="block space-y-1">
                  <div className="flex justify-between text-[10.5px]">
                    <span className="text-[#D9CBE8]">{item.label}</span>
                    <span className="font-typewriter font-bold text-[#6EE7B7] tabular-nums">
                      {formatSignedDeg(val)}
                    </span>
                  </div>
                  <input
                    type="range"
                    min={item.min}
                    max={item.max}
                    step={1}
                    value={val}
                    onChange={(e) => {
                      const angle = parseFloat(e.target.value);
                      if (handSide === 'right') {
                        onSetJointAngle(`${item.part}R` as JointId, angle);
                      } else if (handSide === 'left') {
                        onSetJointAngle(`${item.part}L` as JointId, angle);
                      } else {
                        onUpdateArmature({
                          ...armature,
                          joints: {
                            ...armature.joints,
                            [`${item.part}R`]: {
                              ...armature.joints[`${item.part}R` as JointId],
                              localAngle: angle,
                            },
                            [`${item.part}L`]: {
                              ...armature.joints[`${item.part}L` as JointId],
                              localAngle: -angle,
                            },
                          },
                        });
                      }
                    }}
                    className="w-full accent-[#34D399]"
                  />
                </label>
              );
            })}

            <label className="block space-y-1">
              <div className="flex justify-between text-[10.5px]">
                <span className="text-[#D9CBE8]">Hand Scale</span>
                <span className="font-typewriter font-bold text-[#D8B4FE] tabular-nums">
                  {(armature.joints[wristId]?.scaleLength ?? 1).toFixed(2)}×
                </span>
              </div>
              <input
                type="range"
                min={0.7}
                max={1.5}
                step={0.05}
                value={armature.joints[wristId]?.scaleLength ?? 1}
                onChange={(e) => {
                  const s = parseFloat(e.target.value);
                  if (handSide === 'right') {
                    onSetJointScale('handR', s, s);
                  } else if (handSide === 'left') {
                    onSetJointScale('handL', s, s);
                  } else {
                    onUpdateArmature({
                      ...armature,
                      joints: {
                        ...armature.joints,
                        handR: { ...armature.joints.handR, scaleLength: s, scaleWidth: s },
                        handL: { ...armature.joints.handL, scaleLength: s, scaleWidth: s },
                      },
                    });
                  }
                }}
                className="w-full accent-[#D8B4FE]"
              />
            </label>
          </div>
        </div>
      );
    }

    if (moduleId === 'perspective') {
      return (
        <div className="space-y-3.5">
          <label className="flex items-center justify-between p-2 rounded-[2px] bg-[#1B0825] border border-[#EBE0F5]/14 cursor-pointer">
            <span className="font-bold">Perspective Horizon &amp; VP</span>
            <input
              type="checkbox"
              checked={showPerspective}
              onChange={(e) => onTogglePerspective(e.target.checked)}
              className="accent-[#34D399]"
            />
          </label>

          <div className="space-y-1.5 text-[10.5px]">
            <div className="flex justify-between">
              <span className="text-[#B8A9C9]">Vanishing Point X</span>
              <span className="font-typewriter text-[#FAF6FF] tabular-nums">
                {Math.round(vanishingPoint.x)} px
              </span>
            </div>
            <div className="flex justify-between">
              <span className="text-[#B8A9C9]">Horizon Y</span>
              <span className="font-typewriter text-[#FAF6FF] tabular-nums">
                {Math.round(vanishingPoint.y)} px
              </span>
            </div>
            <button
              type="button"
              onClick={onResetVanishingPoint}
              className="w-full py-1 rounded-[2px] border border-[#D8B4FE]/35 bg-[#2C143B] text-[#E9D5FF] hover:bg-[#3B1B50] font-bold cursor-pointer"
            >
              Center Vanishing Point
            </button>
          </div>

          <div className="pt-3 border-t border-[#EBE0F5]/12 space-y-2">
            <span className="block text-[10px] font-bold uppercase tracking-widest text-[#B8A9C9]">
              Spine Perspective Lean
            </span>
            {(['lumbar', 'thorax', 'neck'] as const).map((jid) => {
              const val = armature.joints[jid]?.localAngle ?? 0;
              return (
                <label key={jid} className="block space-y-1">
                  <div className="flex justify-between text-[10.5px]">
                    <span className="text-[#D9CBE8] capitalize">{jid} Lean</span>
                    <span className="font-typewriter font-bold text-[#D8B4FE] tabular-nums">
                      {formatSignedDeg(val)}
                    </span>
                  </div>
                  <input
                    type="range"
                    min={-55}
                    max={55}
                    step={0.5}
                    value={val}
                    onChange={(e) => onSetJointAngle(jid, parseFloat(e.target.value))}
                    className="w-full accent-[#D8B4FE]"
                  />
                </label>
              );
            })}
          </div>
        </div>
      );
    }

    // scene module
    return (
      <div className="space-y-3.5">
        <div className="space-y-2">
          <div className="flex items-center justify-between">
            <span className="text-[10px] font-bold uppercase tracking-widest text-[#B8A9C9]">
              Backdrop Plate
            </span>
            <span className="text-[10px] text-[#9E8EB3] tabular-nums">
              {activeScene.media.naturalWidth} × {activeScene.media.naturalHeight}
            </span>
          </div>
          <div className="grid grid-cols-2 gap-1.5">
            <button
              type="button"
              onClick={() => onSelectPaperPlate('cover')}
              className={`px-2 py-1.5 rounded-[2px] border text-left font-bold cursor-pointer ${
                activeScene.media.kind === 'paper' && activeScene.media.paperVariant !== 'backing'
                  ? 'bg-[#361547] border-[#34D399]/60 text-white'
                  : 'bg-[#1B0825] border-[#EBE0F5]/14 text-[#D9CBE8] hover:text-white'
              }`}
            >
              Manual Cover
            </button>
            <button
              type="button"
              onClick={() => onSelectPaperPlate('backing')}
              className={`px-2 py-1.5 rounded-[2px] border text-left font-bold cursor-pointer ${
                activeScene.media.kind === 'paper' && activeScene.media.paperVariant === 'backing'
                  ? 'bg-[#361547] border-[#34D399]/60 text-white'
                  : 'bg-[#1B0825] border-[#EBE0F5]/14 text-[#D9CBE8] hover:text-white'
              }`}
            >
              Soft Paper
            </button>
            <button
              type="button"
              onClick={() => onSelectSamplePlate('stone_archway')}
              className={`px-2 py-1.5 rounded-[2px] border text-left font-bold cursor-pointer ${
                activeScene.media.sampleId === 'stone_archway'
                  ? 'bg-[#361547] border-[#34D399]/60 text-white'
                  : 'bg-[#1B0825] border-[#EBE0F5]/14 text-[#D9CBE8] hover:text-white'
              }`}
            >
              Stone Archway
            </button>
            <button
              type="button"
              onClick={() => onSelectSamplePlate('botanical_pergola')}
              className={`px-2 py-1.5 rounded-[2px] border text-left font-bold cursor-pointer ${
                activeScene.media.sampleId === 'botanical_pergola'
                  ? 'bg-[#361547] border-[#34D399]/60 text-white'
                  : 'bg-[#1B0825] border-[#EBE0F5]/14 text-[#D9CBE8] hover:text-white'
              }`}
            >
              Courtyard Plate
            </button>
          </div>
          <label className="w-full py-1.5 px-2.5 rounded-[2px] border border-[#D8B4FE]/35 bg-[#2C143B] text-[#E9D5FF] hover:bg-[#3B1B50] font-bold flex items-center justify-center gap-1.5 cursor-pointer">
            <Upload className="w-3.5 h-3.5" />
            <span>Upload Image or Video</span>
            <input
              type="file"
              accept="image/*,video/*"
              className="hidden"
              onChange={(e) => {
                const file = e.target.files?.[0];
                if (file) onUploadMediaFile(file);
                e.target.value = '';
              }}
            />
          </label>
          {activeScene.media.kind === 'video' && (
            <div className="pt-2 space-y-2 border-t border-[#EBE0F5]/10">
              <div className="flex items-center justify-between text-[10px]">
                <span className="text-[#B8A9C9]">Video Cutout FPS</span>
                <select
                  value={activeScene.media.fps}
                  aria-label="Video plate cutout FPS"
                  onChange={(e) =>
                    onUpdateSceneMedia({ fps: parseInt(e.target.value, 10) as ScreenFpsOption })
                  }
                  className="px-1.5 py-0.5 rounded-[2px] border border-[#EBE0F5]/18 bg-[#120319] text-[#FAF6FF]"
                >
                  {FPS_OPTIONS.map((opt) => (
                    <option key={opt} value={opt}>
                      {opt} FPS
                    </option>
                  ))}
                </select>
              </div>
              <label className="flex items-center justify-between text-[10px] cursor-pointer">
                <span className="text-[#B8A9C9]">Loop Video with Timeline</span>
                <input
                  type="checkbox"
                  checked={activeScene.media.loop}
                  onChange={(e) => onUpdateSceneMedia({ loop: e.target.checked })}
                  className="accent-[#34D399]"
                />
              </label>
            </div>
          )}
        </div>

        <div className="pt-3 border-t border-[#EBE0F5]/12 space-y-2">
          <div className="flex items-center justify-between">
            <span className="text-[10px] font-bold uppercase tracking-widest text-[#B8A9C9]">
              PYX Calibration Grid
            </span>
            <button
              type="button"
              onClick={onResetStageTransform}
              className="text-[10px] text-[#D8B4FE] hover:text-white cursor-pointer"
            >
              Reset
            </button>
          </div>
          <label className="block space-y-1">
            <div className="flex justify-between text-[10px]">
              <span className="text-[#D9CBE8]">Origin X / Y</span>
              <span className="font-typewriter text-[#6EE7B7] tabular-nums">
                {Math.round(activeScene.stageTransform.originX)}, {Math.round(activeScene.stageTransform.originY)} px
              </span>
            </div>
            <div className="grid grid-cols-2 gap-1.5">
              <input
                type="range"
                min={-420}
                max={420}
                step={2}
                value={activeScene.stageTransform.originX}
                aria-label="Stage origin X"
                onChange={(e) => onUpdateStageTransform({ originX: parseFloat(e.target.value) })}
                className="w-full accent-[#34D399]"
              />
              <input
                type="range"
                min={-420}
                max={420}
                step={2}
                value={activeScene.stageTransform.originY}
                aria-label="Stage origin Y"
                onChange={(e) => onUpdateStageTransform({ originY: parseFloat(e.target.value) })}
                className="w-full accent-[#34D399]"
              />
            </div>
          </label>
          <label className="block space-y-1">
            <div className="flex justify-between text-[10px]">
              <span className="text-[#D9CBE8]">PYX Scale Ratio</span>
              <span className="font-typewriter text-[#D8B4FE] tabular-nums">
                {activeScene.stageTransform.pyxScale.toFixed(2)}×
              </span>
            </div>
            <input
              type="range"
              min={0.35}
              max={2.0}
              step={0.02}
              value={activeScene.stageTransform.pyxScale}
              aria-label="Stage PYX scale"
              onChange={(e) => onUpdateStageTransform({ pyxScale: parseFloat(e.target.value) })}
              className="w-full accent-[#D8B4FE]"
            />
          </label>
          <label className="block space-y-1">
            <div className="flex justify-between text-[10px]">
              <span className="text-[#D9CBE8]">Horizon Tilt</span>
              <span className="font-typewriter text-[#D8B4FE] tabular-nums">
                {formatSignedDeg(activeScene.stageTransform.rotation)}
              </span>
            </div>
            <input
              type="range"
              min={-35}
              max={35}
              step={0.5}
              value={activeScene.stageTransform.rotation}
              aria-label="Stage calibration rotation"
              onChange={(e) => onUpdateStageTransform({ rotation: parseFloat(e.target.value) })}
              className="w-full accent-[#D8B4FE]"
            />
          </label>
        </div>

        <div className="pt-3 border-t border-[#EBE0F5]/12 space-y-2">
          <span className="block text-[10px] font-bold uppercase tracking-widest text-[#B8A9C9]">
            World Pins &amp; Ink Blend
          </span>
          <div className="grid grid-cols-2 gap-1">
            {(
              [
                { id: 'footR' as const, label: 'Pin R Foot' },
                { id: 'footL' as const, label: 'Pin L Foot' },
                { id: 'handR' as const, label: 'Pin R Hand' },
                { id: 'handL' as const, label: 'Pin L Hand' },
              ]
            ).map((item) => {
              const pinned = activeScene.worldPins.some((p) => p.jointId === item.id);
              return (
                <button
                  key={item.id}
                  type="button"
                  onClick={() => onToggleWorldPinForJoint(item.id)}
                  className={`px-2 py-1 rounded-[2px] border font-bold flex items-center gap-1 cursor-pointer text-[10px] ${
                    pinned
                      ? 'bg-[#2C1B08] border-[#F59E0B]/60 text-[#FCD34D]'
                      : 'bg-[#1B0825] border-[#EBE0F5]/14 text-[#D9CBE8] hover:text-white'
                  }`}
                >
                  <Anchor className="w-3 h-3" />
                  <span>{item.label}</span>
                </button>
              );
            })}
          </div>
          {activeScene.worldPins.length > 0 && (
            <div className="space-y-1.5">
              {activeScene.worldPins.map((pin) => (
                <div
                  key={pin.id}
                  className="py-1.5 px-2 rounded-[2px] border border-[#F59E0B]/35 bg-[#1B0825] flex items-center justify-between gap-2 text-[10px]"
                >
                  <label className="flex items-center gap-1.5 font-bold text-[#FCD34D] cursor-pointer">
                    <input
                      type="checkbox"
                      checked={pin.enabled}
                      onChange={(e) =>
                        onUpdateWorldPinConfig(pin.id, { enabled: e.target.checked })
                      }
                      className="accent-[#F59E0B]"
                    />
                    <span>{JOINT_SPEC_MAP[pin.jointId]?.shortCode ?? pin.jointId}</span>
                  </label>
                  <span className="font-typewriter text-[#B8A9C9] tabular-nums">
                    {Math.round(pin.frameRange[0] * 100)}–{Math.round(pin.frameRange[1] * 100)}%
                  </span>
                  <button
                    type="button"
                    onClick={() => onRemoveWorldPin(pin.id)}
                    aria-label={`Remove pin ${pin.jointId}`}
                    className="text-[#9E8EB3] hover:text-rose-400 cursor-pointer"
                  >
                    ×
                  </button>
                </div>
              ))}
            </div>
          )}

          <div className="grid grid-cols-3 gap-1 pt-1">
            {(['multiply', 'darken', 'normal'] as CompositeBlendMode[]).map((mode) => (
              <button
                key={mode}
                type="button"
                onClick={() => onUpdateCompositeFilter({ blendMode: mode })}
                className={`py-1 px-1.5 rounded-[2px] border font-bold capitalize text-[10px] cursor-pointer ${
                  activeScene.compositeFilter.blendMode === mode
                    ? 'bg-[#112A22] border-[#34D399]/60 text-[#6EE7B7]'
                    : 'bg-[#1B0825] border-[#EBE0F5]/14 text-[#C8B2E0] hover:text-white'
                }`}
              >
                {mode}
              </button>
            ))}
          </div>
          <label className="block space-y-1">
            <div className="flex justify-between text-[10px]">
              <span className="text-[#D9CBE8]">Ink Wash Opacity</span>
              <span className="font-typewriter text-[#D8B4FE] tabular-nums">
                {Math.round(activeScene.compositeFilter.inkOpacity * 100)}%
              </span>
            </div>
            <input
              type="range"
              min={0.4}
              max={1}
              step={0.02}
              value={activeScene.compositeFilter.inkOpacity}
              onChange={(e) =>
                onUpdateCompositeFilter({ inkOpacity: parseFloat(e.target.value) })
              }
              className="w-full accent-[#D8B4FE]"
            />
          </label>
        </div>
      </div>
    );
  };

  const renderModuleCard = (
    moduleMeta: (typeof MODEL_MODULE_META)[number],
    layout: ModularPanelLayoutState,
    isFloating: boolean
  ) => {
    const isDraggingThis = dragState?.moduleId === moduleMeta.id;
    const x = isDraggingThis ? dragState.currentX : layout.position.x;
    const y = isDraggingThis ? dragState.currentY : layout.position.y;
    const isSnapDock = isDraggingThis && dragState.snapHint === 'right-dock';

    return (
      <section
        key={moduleMeta.id}
        data-model-module={moduleMeta.id}
        data-dock-mode={layout.dockMode}
        aria-label={moduleMeta.label}
        onPointerDown={isFloating ? () => onBringModuleToFront(moduleMeta.id) : undefined}
        style={
          isFloating
            ? {
                position: 'absolute',
                left: 0,
                top: 0,
                transform: `translate3d(${Math.round(x)}px, ${Math.round(y)}px, 0)`,
                zIndex: layout.zIndex,
                width: PANEL_WIDTH,
              }
            : undefined
        }
        className={`pointer-events-auto shrink-0 w-[296px] sm:w-[304px] bg-[#16001E]/90 backdrop-blur-md border rounded-[4px] shadow-2xl flex flex-col text-[#EFEAF4] overflow-hidden ${
          isSnapDock ? 'border-[#34D399]' : 'border-[#EBE0F5]/18'
        }`}
      >
        <div
          onPointerDown={(e) => {
            const cardEl = e.currentTarget.closest('section') as HTMLElement | null;
            handleStartPanelDrag(e, moduleMeta.id, cardEl);
          }}
          title="Drag to float or attach to edge"
          className="px-2.5 py-1.5 border-b border-[#EBE0F5]/14 bg-[#1B0825]/90 flex items-center justify-between gap-2 select-none cursor-grab active:cursor-grabbing shrink-0"
        >
          <div className="flex items-center gap-1.5 min-w-0">
            <GripHorizontal className="w-3.5 h-3.5 text-[#7D6E91] shrink-0" />
            <span className="text-[#34D399] shrink-0">{MODULE_ICONS[moduleMeta.id]}</span>
            <span className="font-typewriter font-bold text-[11px] tracking-wider text-[#FAF6FF] truncate">
              {moduleMeta.label.toUpperCase()}
            </span>
          </div>

          <div
            className="flex items-center gap-0.5 shrink-0"
            onPointerDown={(e) => e.stopPropagation()}
          >
            <button
              type="button"
              onClick={() =>
                onSetModuleDockMode(
                  moduleMeta.id,
                  isFloating ? 'docked' : 'floating',
                  isFloating ? undefined : { x: 24, y: 24 }
                )
              }
              aria-label={isFloating ? `Attach ${moduleMeta.label} to right dock` : `Float ${moduleMeta.label}`}
              title={isFloating ? 'Attach to right dock' : 'Detach to float'}
              className="p-1 rounded-[2px] text-[#B8A9C9] hover:text-[#6EE7B7] hover:bg-[#2C143B] cursor-pointer"
            >
              {isFloating ? <PanelRight className="w-3.5 h-3.5" /> : <Move className="w-3.5 h-3.5" />}
            </button>
            <button
              type="button"
              onClick={() => onSetModuleCollapsed(moduleMeta.id, !layout.isCollapsed)}
              aria-label={layout.isCollapsed ? `Expand ${moduleMeta.label}` : `Collapse ${moduleMeta.label}`}
              title={layout.isCollapsed ? 'Expand' : 'Collapse'}
              className="p-1 rounded-[2px] text-[#B8A9C9] hover:text-white hover:bg-[#2C143B] cursor-pointer"
            >
              {layout.isCollapsed ? (
                <ChevronDown className="w-3.5 h-3.5" />
              ) : (
                <ChevronUp className="w-3.5 h-3.5" />
              )}
            </button>
            <button
              type="button"
              onClick={() => onToggleModule(moduleMeta.id)}
              aria-label={`Close ${moduleMeta.label}`}
              title="Close module"
              className="p-1 rounded-[2px] text-[#B8A9C9] hover:text-white hover:bg-[#2C143B] cursor-pointer"
            >
              <X className="w-3.5 h-3.5" />
            </button>
          </div>
        </div>

        {!layout.isCollapsed && (
          <div className="p-3 overflow-y-auto max-h-[360px] font-vera text-[11px]">
            {renderModuleBody(moduleIdForMeta(moduleMeta.id))}
          </div>
        )}
      </section>
    );
  };

  function moduleIdForMeta(id: ModelModuleId) {
    return id;
  }

  return (
    <div
      ref={overlayRef}
      data-workspace-overlay="true"
      className="pointer-events-none absolute inset-0 z-30 overflow-hidden"
    >
      {/* Floating Undocked Model Panels */}
      {floatingModules.map((meta) =>
        renderModuleCard(meta, panels[meta.id], true)
      )}

      {/* Right Edge: Stackable Docked Panels + Slim Quick-Toggle Vertical Icon Menu */}
      {railVisible && (
        <div className="pointer-events-none absolute inset-y-2.5 right-2.5 z-40 flex items-start gap-2">
          {/* Stackable Edge-Docked Model Modules Column */}
          {(dockedModules.length > 0 || dragState?.snapHint === 'right-dock') && (
            <div
              data-dock-stack="true"
              className={`max-h-full overflow-y-auto flex flex-col gap-2 pr-0.5 rounded-[4px] ${
                dockedModules.length > 0 ? 'pointer-events-auto' : 'pointer-events-none'
              } ${
                dragState?.snapHint === 'right-dock'
                  ? 'ring-2 ring-[#34D399]/70 bg-[#112A22]/25 min-w-[296px] min-h-[96px]'
                  : ''
              }`}
            >
              {dockedModules.map((meta) =>
                renderModuleCard(meta, panels[meta.id], false)
              )}
            </div>
          )}

          {/* Slim Right Vertical Icon Menu (Quick Toggles + Module Launchers) */}
          <nav
            aria-label="Model quick toggles & modules"
            className="pointer-events-auto w-11 max-h-full overflow-y-auto shrink-0 bg-[#16001E]/88 backdrop-blur-md border border-[#EBE0F5]/16 rounded-[4px] p-1 flex flex-col items-center gap-0.5 shadow-xl"
          >
            {/* Quick Model Toggles */}
            <button
              type="button"
              aria-pressed={anyLimbIK}
              onClick={() => onBatchSetIK(!anyLimbIK)}
              title={anyLimbIK ? 'Switch all limbs to FK' : 'Switch all limbs to IK'}
              aria-label="Toggle All Limbs IK"
              className={`w-full min-h-[34px] rounded-[3px] border flex items-center justify-center cursor-pointer transition-colors ${
                anyLimbIK
                  ? 'bg-[#112A22] border-[#34D399]/65 text-[#6EE7B7]'
                  : 'border-transparent text-[#B8A9C9] hover:text-[#FAF6FF] hover:bg-[#240B31]/75'
              }`}
            >
              <Link2 className="w-4 h-4" />
            </button>

            <button
              type="button"
              aria-pressed={rigSystems.softLimits}
              onClick={() => onUpdateRigSystems({ softLimits: !rigSystems.softLimits })}
              title="Toggle Anatomical Soft Limits"
              aria-label="Toggle Soft Limits"
              className={`w-full min-h-[34px] rounded-[3px] border flex items-center justify-center cursor-pointer transition-colors ${
                rigSystems.softLimits
                  ? 'bg-[#112A22] border-[#34D399]/65 text-[#6EE7B7]'
                  : 'border-transparent text-[#B8A9C9] hover:text-[#FAF6FF] hover:bg-[#240B31]/75'
              }`}
            >
              <ShieldCheck className="w-4 h-4" />
            </button>

            <button
              type="button"
              aria-pressed={rigSystems.floorSnap}
              onClick={() => onUpdateRigSystems({ floorSnap: !rigSystems.floorSnap })}
              title="Toggle Floor Plane Snap"
              aria-label="Toggle Floor Snap"
              className={`w-full min-h-[34px] rounded-[3px] border flex items-center justify-center cursor-pointer transition-colors ${
                rigSystems.floorSnap
                  ? 'bg-[#112A22] border-[#34D399]/65 text-[#6EE7B7]'
                  : 'border-transparent text-[#B8A9C9] hover:text-[#FAF6FF] hover:bg-[#240B31]/75'
              }`}
            >
              <ArrowDownToLine className="w-4 h-4" />
            </button>

            <button
              type="button"
              aria-pressed={armature.isAnchorPinned}
              onClick={() =>
                onUpdateArmature({ ...armature, isAnchorPinned: !armature.isAnchorPinned })
              }
              title="Toggle Pin Keystone Anchor"
              aria-label="Toggle Pin Keystone Anchor"
              className={`w-full min-h-[34px] rounded-[3px] border flex items-center justify-center cursor-pointer transition-colors ${
                armature.isAnchorPinned
                  ? 'bg-[#2C143B] border-[#D8B4FE]/65 text-[#E9D5FF]'
                  : 'border-transparent text-[#B8A9C9] hover:text-[#FAF6FF] hover:bg-[#240B31]/75'
              }`}
            >
              <Pin className="w-4 h-4" />
            </button>

            <button
              type="button"
              aria-pressed={armature.continuousSpin}
              onClick={() =>
                onUpdateArmature({ ...armature, continuousSpin: !armature.continuousSpin })
              }
              title="Toggle Continuous 360° Spin"
              aria-label="Toggle 360 Spin"
              className={`w-full min-h-[34px] rounded-[3px] border flex items-center justify-center cursor-pointer transition-colors ${
                armature.continuousSpin
                  ? 'bg-[#112A22] border-[#34D399]/65 text-[#6EE7B7]'
                  : 'border-transparent text-[#B8A9C9] hover:text-[#FAF6FF] hover:bg-[#240B31]/75'
              }`}
            >
              <RefreshCw className="w-4 h-4" />
            </button>

            <button
              type="button"
              aria-pressed={showPerspective}
              onClick={() => onTogglePerspective(!showPerspective)}
              title="Toggle Perspective Horizon"
              aria-label="Toggle Perspective Horizon"
              className={`w-full min-h-[34px] rounded-[3px] border flex items-center justify-center cursor-pointer transition-colors ${
                showPerspective
                  ? 'bg-[#112A22] border-[#34D399]/65 text-[#6EE7B7]'
                  : 'border-transparent text-[#B8A9C9] hover:text-[#FAF6FF] hover:bg-[#240B31]/75'
              }`}
            >
              <Compass className="w-4 h-4" />
            </button>

            <button
              type="button"
              onClick={onMirrorPose}
              title="Mirror Pose Across Midline"
              aria-label="Mirror Pose"
              className="w-full min-h-[34px] rounded-[3px] border border-transparent text-[#B8A9C9] hover:text-[#FAF6FF] hover:bg-[#240B31]/75 flex items-center justify-center cursor-pointer transition-colors"
            >
              <FlipHorizontal2 className="w-4 h-4" />
            </button>

            <button
              type="button"
              onClick={onResetPose}
              title="Reset to T-Pose"
              aria-label="Reset T-Pose"
              className="w-full min-h-[34px] rounded-[3px] border border-transparent text-[#B8A9C9] hover:text-[#FAF6FF] hover:bg-[#240B31]/75 flex items-center justify-center cursor-pointer transition-colors"
            >
              <RotateCcw className="w-4 h-4" />
            </button>

            <span className="w-6 h-px bg-[#EBE0F5]/16 my-0.5 shrink-0" aria-hidden="true" />

            {/* Attachable / Stackable Model Modules */}
            {MODEL_MODULE_META.map((meta) => {
              const active = panels[meta.id].isOpen;
              return (
                <button
                  key={meta.id}
                  type="button"
                  aria-pressed={active}
                  aria-label={meta.label}
                  title={meta.label}
                  onClick={() => onToggleModule(meta.id)}
                  className={`w-full min-h-[36px] rounded-[3px] border flex items-center justify-center cursor-pointer transition-colors shrink-0 ${
                    active
                      ? 'bg-[#240B31] border-[#34D399]/70 text-[#6EE7B7]'
                      : 'border-transparent text-[#C8B2E0] hover:text-[#FAF6FF] hover:bg-[#240B31]/75'
                  }`}
                >
                  {MODULE_ICONS[meta.id]}
                </button>
              );
            })}

            <span className="w-6 h-px bg-[#EBE0F5]/16 my-0.5 shrink-0" aria-hidden="true" />

            <button
              type="button"
              onClick={onToggleRail}
              aria-label="Hide quick toggle rail"
              title="Hide quick toggle rail"
              className="w-full min-h-[32px] rounded-[3px] border border-transparent text-[#9E8EB3] hover:text-[#FAF6FF] hover:bg-[#240B31]/75 flex items-center justify-center cursor-pointer shrink-0"
            >
              <PanelRightClose className="w-4 h-4" />
            </button>
          </nav>
        </div>
      )}
    </div>
  );
});
