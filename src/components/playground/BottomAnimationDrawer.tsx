import React, { useState } from 'react';
import {
  ChevronDown,
  ChevronUp,
  Copy,
  Download,
  Film,
  FlipHorizontal2,
  Pause,
  Play,
  Plus,
  RotateCcw,
  Sparkles,
  Trash2,
} from 'lucide-react';
import { EXPORT_IR_SCHEMA, EXPORT_IR_VERSION, ExportIRFormat } from '../../types/exportIR';
import {
  ArmatureState,
  KeyframeSlot,
  MotionCurve,
  PosePresetId,
  UserSavedPose,
} from '../../types/rig';
import { Scene, ScenePlaybackMode } from '../../types/scene';
import { BottomDrawerTab } from '../../types/workspace';
import { ScreenFpsOption } from '../../utils/frameThrottle';
import { POSE_PRESET_LABELS } from '../../utils/kinematics';

export interface BottomAnimationDrawerProps {
  isOpen: boolean;
  activeTab: BottomDrawerTab;
  onToggleOpen: () => void;
  onSelectTab: (tab: BottomDrawerTab) => void;

  // Timeline
  slots: Record<KeyframeSlot, ArmatureState | null>;
  activeSlot: KeyframeSlot | null;
  timelineT: number;
  isPlaying: boolean;
  playbackSpeed: number;
  motionCurve: MotionCurve;
  onCaptureSlot: (slot: KeyframeSlot) => void;
  onJumpToSlot: (slot: KeyframeSlot) => void;
  onClearSlot: (slot: KeyframeSlot) => void;
  onScrubChange: (t: number) => void;
  onTogglePlay: () => void;
  onPlaybackSpeedChange: (speed: number) => void;
  onCurveChange: (curve: MotionCurve) => void;
  onSynthesizeBreakdown: () => void;

  // Pose Snapshots
  activePreset: PosePresetId | null;
  savedPoses: UserSavedPose[];
  onApplyPreset: (preset: PosePresetId) => void;
  onMirrorPose: () => void;
  onResetPose: () => void;
  onSaveUserPose: (name: string) => void;
  onApplyUserPose: (pose: UserSavedPose) => void;
  onOverwriteUserPose: (id: string) => void;
  onDeleteUserPose: (id: string) => void;
  onLoadUserPoseToSlot: (pose: UserSavedPose, slot: KeyframeSlot) => void;

  // Animation Settings, Shot Sequencer & Export IR
  scenes: readonly Scene[];
  activeScene: Scene;
  playbackMode: ScenePlaybackMode;
  screenFps: ScreenFpsOption;
  onCycleFps: () => void;
  onSelectScene: (sceneId: string) => void;
  onAddScene: () => void;
  onDuplicateScene: (sceneId: string) => void;
  onDeleteScene: (sceneId: string) => void;
  onTogglePlaybackMode: () => void;
  onRenameScene: (name: string) => void;
  onUpdateSceneDuration: (durationSec: number) => void;
  onExportFormat: (format: ExportIRFormat) => void;
}

const ALL_PRESETS: PosePresetId[] = ['tpose'];

const CURVE_OPTIONS: ReadonlyArray<{ value: MotionCurve; label: string }> = [
  { value: 'flow', label: 'Flow (Cubic)' },
  { value: 'snap', label: 'Snap (Quartic)' },
  { value: 'elastic', label: 'Elastic Bounce' },
  { value: 'mechanical', label: 'Linear' },
  { value: 'clockwork', label: 'Stepped (14f)' },
  { value: 'procedural', label: 'Sine Arc' },
];

const DRAWER_TABS: ReadonlyArray<{ id: BottomDrawerTab; label: string }> = [
  { id: 'timeline', label: 'Timeline' },
  { id: 'poses', label: 'Pose Snapshots' },
  { id: 'animation', label: 'Animation & Export' },
];

export const BottomAnimationDrawer: React.FC<BottomAnimationDrawerProps> = React.memo(({
  isOpen,
  activeTab,
  onToggleOpen,
  onSelectTab,
  slots,
  activeSlot,
  timelineT,
  isPlaying,
  playbackSpeed,
  motionCurve,
  onCaptureSlot,
  onJumpToSlot,
  onClearSlot,
  onScrubChange,
  onTogglePlay,
  onPlaybackSpeedChange,
  onCurveChange,
  onSynthesizeBreakdown,
  activePreset,
  savedPoses,
  onApplyPreset,
  onMirrorPose,
  onResetPose,
  onSaveUserPose,
  onApplyUserPose,
  onOverwriteUserPose,
  onDeleteUserPose,
  onLoadUserPoseToSlot,
  scenes,
  activeScene,
  playbackMode,
  screenFps,
  onCycleFps,
  onSelectScene,
  onAddScene,
  onDuplicateScene,
  onDeleteScene,
  onTogglePlaybackMode,
  onRenameScene,
  onUpdateSceneDuration,
  onExportFormat,
}) => {
  const [poseNameInput, setPoseNameInput] = useState('');
  const canPlay = Boolean(slots.A && (slots.B || slots.C));

  const handleTabClick = (tab: BottomDrawerTab) => {
    if (isOpen && activeTab === tab) {
      onToggleOpen();
    } else {
      onSelectTab(tab);
      if (!isOpen) onToggleOpen();
    }
  };

  return (
    <section
      aria-label="Animation and timeline drawer"
      data-bottom-drawer-open={isOpen ? 'true' : 'false'}
      className="bg-[#16001E] border-t border-[#EBE0F5]/14 shrink-0 z-20 flex flex-col"
    >
      {/* Always-Visible Collapsible Handle Bar */}
      <div className="h-8 px-3 sm:px-6 flex items-center justify-between gap-2 bg-[#180522]">
        <div
          role="tablist"
          aria-label="Bottom animation sections"
          className="flex items-center gap-1 min-w-0 overflow-x-auto"
        >
          {DRAWER_TABS.map((tab) => {
            const active = isOpen && activeTab === tab.id;
            return (
              <button
                key={tab.id}
                type="button"
                role="tab"
                aria-selected={active}
                onClick={() => handleTabClick(tab.id)}
                className={`px-2.5 py-1 rounded-[2px] border font-vera text-[10.5px] font-bold whitespace-nowrap shrink-0 cursor-pointer transition-colors ${
                  active
                    ? 'bg-[#240B31] border-[#34D399]/65 text-[#6EE7B7]'
                    : 'bg-[#1D0B28]/70 border-transparent text-[#B8A9C9] hover:text-[#FAF6FF] hover:border-[#EBE0F5]/14'
                }`}
              >
                {tab.label}
              </button>
            );
          })}
        </div>

        {/* Quick Collapsed Status & Expand Toggle */}
        <div className="flex items-center gap-2 shrink-0 font-vera text-[10.5px]">
          <div className="hidden sm:flex items-center gap-1.5 text-[#9E8EB3] tabular-nums">
            <span>{activeScene.name}</span>
            <span aria-hidden="true">·</span>
            <span>{Math.round(timelineT * 100)}%</span>
            {activePreset && (
              <>
                <span aria-hidden="true">·</span>
                <span>{POSE_PRESET_LABELS[activePreset]}</span>
              </>
            )}
          </div>

          <button
            type="button"
            disabled={!canPlay}
            onClick={onTogglePlay}
            aria-label={isPlaying ? 'Pause keyframe playback' : 'Play keyframe sequence'}
            className="px-2 py-0.5 rounded-[2px] border border-[#34D399]/40 bg-[#112A22] text-[#6EE7B7] hover:bg-[#173D31] disabled:opacity-35 disabled:cursor-not-allowed flex items-center gap-1 font-bold cursor-pointer whitespace-nowrap"
          >
            {isPlaying ? <Pause className="w-3 h-3" /> : <Play className="w-3 h-3" />}
            <span className="hidden md:inline">{isPlaying ? 'Pause' : 'Play'}</span>
          </button>

          <button
            type="button"
            aria-expanded={isOpen}
            aria-label={isOpen ? 'Collapse bottom animation drawer' : 'Expand bottom animation drawer'}
            title={isOpen ? 'Collapse drawer' : 'Expand Timeline, Pose Snapshots & Animation Settings'}
            onClick={onToggleOpen}
            className="p-1 rounded-[2px] border border-[#EBE0F5]/16 bg-[#1D0B28] text-[#D9CBE8] hover:text-white hover:bg-[#2C143B] cursor-pointer"
          >
            {isOpen ? <ChevronDown className="w-3.5 h-3.5" /> : <ChevronUp className="w-3.5 h-3.5" />}
          </button>
        </div>
      </div>

      {/* Expandable Bottom Drawer Body */}
      {isOpen && (
        <div
          role="tabpanel"
          aria-label={`Bottom drawer ${activeTab}`}
          className="h-[224px] overflow-y-auto px-3 sm:px-6 py-3 bg-[#15041D] border-t border-[#EBE0F5]/12 font-vera text-[11px]"
        >
          {/* TIMELINE TAB */}
          {activeTab === 'timeline' && (
            <div className="grid grid-cols-1 md:grid-cols-3 gap-4 items-start">
              {/* ABC Keyframe Slots */}
              <div className="space-y-2">
                <span className="block text-[10px] font-bold uppercase tracking-widest text-[#B8A9C9]">
                  ABC Keyframe Slots
                </span>
                <div className="space-y-1.5">
                  {(['A', 'B', 'C'] as KeyframeSlot[]).map((slot) => {
                    const captured = Boolean(slots[slot]);
                    const isActive = activeSlot === slot;
                    return (
                      <div
                        key={slot}
                        className={`px-2 py-1.5 rounded-[2px] border flex items-center justify-between gap-2 ${
                          isActive
                            ? 'border-[#34D399]/60 bg-[#1E102B]'
                            : 'border-[#EBE0F5]/14 bg-[#1B0825]'
                        }`}
                      >
                        <div className="flex items-center gap-2">
                          <span className="font-typewriter font-bold text-xs text-[#FAF6FF] w-4">
                            {slot}
                          </span>
                          <span className="text-[10.5px] text-[#B8A9C9]">
                            {captured ? 'Captured' : 'Empty'}
                          </span>
                        </div>
                        <div className="flex items-center gap-1">
                          <button
                            type="button"
                            onClick={() => onCaptureSlot(slot)}
                            className="px-2 py-0.5 rounded-[2px] border border-[#D8B4FE]/35 bg-[#2C143B] text-[#E9D5FF] hover:bg-[#3B1B50] font-bold cursor-pointer whitespace-nowrap"
                          >
                            Capture
                          </button>
                          <button
                            type="button"
                            disabled={!captured}
                            onClick={() => onJumpToSlot(slot)}
                            className="px-2 py-0.5 rounded-[2px] border border-[#34D399]/40 bg-[#112A22] text-[#6EE7B7] hover:bg-[#173D31] disabled:opacity-35 cursor-pointer whitespace-nowrap"
                          >
                            Load
                          </button>
                          <button
                            type="button"
                            disabled={!captured}
                            onClick={() => onClearSlot(slot)}
                            aria-label={`Clear slot ${slot}`}
                            className="px-1.5 py-0.5 rounded-[2px] border border-[#EBE0F5]/14 bg-[#14041C] text-[#9E8EB3] hover:text-rose-400 disabled:opacity-30 cursor-pointer"
                          >
                            ×
                          </button>
                        </div>
                      </div>
                    );
                  })}
                </div>
              </div>

              {/* Scrubber & Playback */}
              <div className="space-y-2.5">
                <div className="flex items-center justify-between">
                  <span className="text-[10px] font-bold uppercase tracking-widest text-[#B8A9C9]">
                    Interpolation Scrubber
                  </span>
                  <span className="font-typewriter font-bold text-[#34D399] tabular-nums">
                    {Math.round(timelineT * 100)}%
                  </span>
                </div>
                <input
                  type="range"
                  min={0}
                  max={1}
                  step={0.005}
                  value={timelineT}
                  disabled={!canPlay}
                  aria-label="Timeline scrubber"
                  onChange={(e) => onScrubChange(parseFloat(e.target.value))}
                  className="w-full accent-[#34D399] cursor-pointer disabled:opacity-35"
                />
                <div className="flex items-center gap-2">
                  <button
                    type="button"
                    disabled={!canPlay}
                    onClick={onTogglePlay}
                    className="flex-1 py-1.5 px-3 rounded-[2px] border border-[#34D399]/50 bg-[#112A22] text-[#6EE7B7] hover:bg-[#173D31] disabled:opacity-35 font-bold flex items-center justify-center gap-1.5 cursor-pointer whitespace-nowrap"
                  >
                    {isPlaying ? <Pause className="w-3.5 h-3.5" /> : <Play className="w-3.5 h-3.5" />}
                    <span>{isPlaying ? 'Pause Loop' : 'Play Loop'}</span>
                  </button>
                  <select
                    value={playbackSpeed}
                    aria-label="Playback speed"
                    onChange={(e) => onPlaybackSpeedChange(parseFloat(e.target.value))}
                    className="px-2 py-1.5 rounded-[2px] border border-[#EBE0F5]/18 bg-[#120319] text-[#FAF6FF]"
                  >
                    <option value={0.5}>0.5×</option>
                    <option value={1}>1.0×</option>
                    <option value={1.5}>1.5×</option>
                    <option value={2}>2.0×</option>
                  </select>
                </div>
              </div>

              {/* Curve & Breakdown */}
              <div className="space-y-2.5">
                <label className="block space-y-1">
                  <span className="text-[10px] font-bold uppercase tracking-widest text-[#B8A9C9]">
                    Motion Curve
                  </span>
                  <select
                    value={motionCurve}
                    onChange={(e) => onCurveChange(e.target.value as MotionCurve)}
                    className="w-full px-2 py-1.5 rounded-[2px] border border-[#EBE0F5]/18 bg-[#120319] text-[#FAF6FF]"
                  >
                    {CURVE_OPTIONS.map((opt) => (
                      <option key={opt.value} value={opt.value}>
                        {opt.label}
                      </option>
                    ))}
                  </select>
                </label>

                <button
                  type="button"
                  disabled={!slots.A || !slots.C}
                  onClick={onSynthesizeBreakdown}
                  className="w-full py-1.5 px-3 rounded-[2px] border border-[#D8B4FE]/40 bg-[#2C143B] text-[#E9D5FF] hover:bg-[#3B1B50] disabled:opacity-35 font-bold flex items-center justify-center gap-1.5 cursor-pointer whitespace-nowrap"
                >
                  <Sparkles className="w-3.5 h-3.5" />
                  <span>Synthesize Breakdown B (from A &amp; C)</span>
                </button>
              </div>
            </div>
          )}

          {/* POSE SNAPSHOTS TAB */}
          {activeTab === 'poses' && (
            <div className="grid grid-cols-1 lg:grid-cols-2 gap-4 items-start">
              {/* Canonical Presets */}
              <div className="space-y-2">
                <div className="flex items-center justify-between gap-2">
                  <span className="text-[10px] font-bold uppercase tracking-widest text-[#B8A9C9]">
                    Canonical Base Pose
                  </span>
                  <div className="flex items-center gap-1">
                    <button
                      type="button"
                      onClick={onMirrorPose}
                      className="px-2 py-0.5 rounded-[2px] border border-[#EBE0F5]/16 bg-[#1D0B28] text-[#D8B4FE] hover:bg-[#2C143B] flex items-center gap-1 cursor-pointer text-[10px] whitespace-nowrap"
                    >
                      <FlipHorizontal2 className="w-3 h-3" />
                      <span>Mirror</span>
                    </button>
                    <button
                      type="button"
                      onClick={onResetPose}
                      className="px-2 py-0.5 rounded-[2px] border border-[#EBE0F5]/16 bg-[#1D0B28] text-[#D8B4FE] hover:bg-[#2C143B] flex items-center gap-1 cursor-pointer text-[10px] whitespace-nowrap"
                    >
                      <RotateCcw className="w-3 h-3" />
                      <span>Reset</span>
                    </button>
                  </div>
                </div>
                <div className="grid grid-cols-3 sm:grid-cols-4 gap-1.5">
                  {ALL_PRESETS.map((preset) => (
                    <button
                      key={preset}
                      type="button"
                      onClick={() => onApplyPreset(preset)}
                      className={`px-2 py-1.5 rounded-[2px] border text-left font-bold transition-colors cursor-pointer truncate ${
                        activePreset === preset
                          ? 'bg-[#361547] border-[#34D399]/60 text-[#FAF6FF]'
                          : 'bg-[#1B0825] border-[#EBE0F5]/14 text-[#D9CBE8] hover:border-[#D8B4FE]/45 hover:text-white'
                      }`}
                    >
                      {POSE_PRESET_LABELS[preset]}
                    </button>
                  ))}
                </div>
              </div>

              {/* Saved Snapshots Library */}
              <div className="space-y-2">
                <span className="block text-[10px] font-bold uppercase tracking-widest text-[#B8A9C9]">
                  Saved Pose Snapshots ({savedPoses.length})
                </span>
                <form
                  onSubmit={(e) => {
                    e.preventDefault();
                    const trimmed = poseNameInput.trim() || `Study ${savedPoses.length + 1}`;
                    onSaveUserPose(trimmed);
                    setPoseNameInput('');
                  }}
                  className="flex items-center gap-1.5"
                >
                  <input
                    type="text"
                    value={poseNameInput}
                    onChange={(e) => setPoseNameInput(e.target.value)}
                    placeholder="Name current pose snapshot..."
                    aria-label="New pose name"
                    className="flex-1 px-2 py-1 rounded-[2px] border border-[#EBE0F5]/18 bg-[#120319] text-[#FAF6FF] placeholder-[#7D6E91] focus:outline-none focus:border-[#34D399]"
                  />
                  <button
                    type="submit"
                    className="px-2.5 py-1 rounded-[2px] border border-[#34D399]/50 bg-[#112A22] text-[#6EE7B7] hover:bg-[#173D31] font-bold flex items-center gap-1 cursor-pointer whitespace-nowrap"
                  >
                    <Plus className="w-3 h-3" />
                    <span>Save Snapshot</span>
                  </button>
                </form>

                {savedPoses.length === 0 ? (
                  <p className="py-2 text-[#9E8EB3] text-[10.5px]">
                    Save pose snapshots locally to recall them or load directly into Keyframes A, B, or C.
                  </p>
                ) : (
                  <div className="grid grid-cols-1 sm:grid-cols-2 gap-1.5 max-h-[120px] overflow-y-auto pr-1">
                    {savedPoses.map((entry) => (
                      <div
                        key={entry.id}
                        className="p-2 rounded-[2px] border border-[#EBE0F5]/14 bg-[#1B0825] space-y-1"
                      >
                        <div className="flex items-center justify-between gap-2">
                          <span className="font-bold text-[#FAF6FF] truncate">{entry.name}</span>
                          <button
                            type="button"
                            onClick={() => onDeleteUserPose(entry.id)}
                            aria-label={`Delete pose ${entry.name}`}
                            className="p-0.5 text-[#9E8EB3] hover:text-rose-400 cursor-pointer"
                          >
                            <Trash2 className="w-3 h-3" />
                          </button>
                        </div>
                        <div className="flex items-center gap-1">
                          <button
                            type="button"
                            onClick={() => onApplyUserPose(entry)}
                            className="flex-1 py-0.5 px-2 rounded-[2px] border border-[#34D399]/40 bg-[#112A22] text-[#6EE7B7] hover:bg-[#173D31] font-bold cursor-pointer"
                          >
                            Apply
                          </button>
                          <button
                            type="button"
                            onClick={() => onOverwriteUserPose(entry.id)}
                            className="py-0.5 px-2 rounded-[2px] border border-[#EBE0F5]/16 bg-[#14041C] text-[#C8B2E0] hover:text-white cursor-pointer"
                          >
                            Update
                          </button>
                          {(['A', 'B', 'C'] as KeyframeSlot[]).map((slot) => (
                            <button
                              key={slot}
                              type="button"
                              onClick={() => onLoadUserPoseToSlot(entry, slot)}
                              title={`Load into Keyframe ${slot}`}
                              className="py-0.5 px-1.5 rounded-[2px] border border-[#D8B4FE]/30 bg-[#2C143B] text-[#E9D5FF] hover:bg-[#3B1B50] font-typewriter font-bold cursor-pointer"
                            >
                              {slot}
                            </button>
                          ))}
                        </div>
                      </div>
                    ))}
                  </div>
                )}
              </div>
            </div>
          )}

          {/* ANIMATION & EXPORT TAB */}
          {activeTab === 'animation' && (
            <div className="grid grid-cols-1 md:grid-cols-3 gap-4 items-start">
              {/* Shot Sequencer */}
              <div className="space-y-2">
                <div className="flex items-center justify-between">
                  <span className="text-[10px] font-bold uppercase tracking-widest text-[#B8A9C9]">
                    Shot Sequencer ({scenes.length})
                  </span>
                  <div className="flex items-center gap-1">
                    <button
                      type="button"
                      onClick={onAddScene}
                      className="px-2 py-0.5 rounded-[2px] border border-[#34D399]/45 bg-[#112A22] text-[#6EE7B7] hover:bg-[#173D31] font-bold flex items-center gap-1 cursor-pointer whitespace-nowrap"
                    >
                      <Plus className="w-3 h-3" />
                      <span>Shot</span>
                    </button>
                    <button
                      type="button"
                      onClick={() => onDuplicateScene(activeScene.id)}
                      aria-label="Duplicate active shot"
                      title="Duplicate active shot"
                      className="p-1 rounded-[2px] border border-[#EBE0F5]/14 bg-[#1B0825] text-[#C8B2E0] hover:text-white cursor-pointer"
                    >
                      <Copy className="w-3 h-3" />
                    </button>
                    <button
                      type="button"
                      disabled={scenes.length <= 1}
                      onClick={() => onDeleteScene(activeScene.id)}
                      aria-label="Delete active shot"
                      title="Delete active shot"
                      className="p-1 rounded-[2px] border border-[#EBE0F5]/14 bg-[#1B0825] text-[#C8B2E0] hover:text-rose-400 disabled:opacity-30 cursor-pointer"
                    >
                      <Trash2 className="w-3 h-3" />
                    </button>
                  </div>
                </div>
                <div className="flex flex-wrap gap-1">
                  {scenes.map((scene, idx) => {
                    const isActive = scene.id === activeScene.id;
                    return (
                      <button
                        key={scene.id}
                        type="button"
                        onClick={() => onSelectScene(scene.id)}
                        aria-current={isActive ? 'true' : undefined}
                        className={`px-2 py-1 rounded-[2px] border flex items-center gap-1 cursor-pointer whitespace-nowrap ${
                          isActive
                            ? 'bg-[#361547] border-[#34D399]/60 text-[#FAF6FF] font-bold'
                            : 'bg-[#1B0825] border-[#EBE0F5]/14 text-[#C8B2E0] hover:text-white'
                        }`}
                      >
                        <span className="font-typewriter">{idx + 1}.</span>
                        <span className="truncate max-w-[96px]">{scene.name}</span>
                      </button>
                    );
                  })}
                </div>
              </div>

              {/* Animation Timing */}
              <div className="space-y-2">
                <span className="block text-[10px] font-bold uppercase tracking-widest text-[#B8A9C9]">
                  Timing &amp; Playback
                </span>
                <div className="flex items-center gap-1.5">
                  <input
                    type="text"
                    value={activeScene.name}
                    aria-label="Active shot name"
                    onChange={(e) => onRenameScene(e.target.value)}
                    className="flex-1 px-2 py-1 rounded-[2px] border border-[#EBE0F5]/18 bg-[#120319] text-[#FAF6FF]"
                  />
                  <label className="flex items-center gap-1 text-[10px] text-[#B8A9C9] shrink-0">
                    <span>Dur</span>
                    <input
                      type="number"
                      min={0.5}
                      max={30}
                      step={0.5}
                      value={activeScene.durationSec}
                      aria-label="Shot duration in seconds"
                      onChange={(e) => {
                        const val = parseFloat(e.target.value);
                        if (Number.isFinite(val)) {
                          onUpdateSceneDuration(Math.max(0.5, Math.min(30, val)));
                        }
                      }}
                      className="w-14 px-1.5 py-1 rounded-[2px] border border-[#EBE0F5]/18 bg-[#120319] text-[#FAF6FF] font-typewriter text-right tabular-nums"
                    />
                    <span>s</span>
                  </label>
                </div>

                <div className="grid grid-cols-2 gap-1.5">
                  <button
                    type="button"
                    onClick={onTogglePlaybackMode}
                    aria-pressed={playbackMode === 'sequence'}
                    className={`py-1.5 px-2 rounded-[2px] border font-bold flex items-center justify-center gap-1 cursor-pointer whitespace-nowrap ${
                      playbackMode === 'sequence'
                        ? 'bg-[#112A22] border-[#34D399]/55 text-[#6EE7B7]'
                        : 'bg-[#1B0825] border-[#EBE0F5]/16 text-[#D9CBE8] hover:text-white'
                    }`}
                  >
                    <Film className="w-3.5 h-3.5" />
                    <span>{playbackMode === 'sequence' ? 'Seq All' : 'Loop Shot'}</span>
                  </button>

                  <button
                    type="button"
                    onClick={onCycleFps}
                    data-fps-toggle="true"
                    title="Cycle animation frame cap (12 · 15 · 24 · 30 FPS)"
                    className="py-1.5 px-2 rounded-[2px] border border-[#EBE0F5]/16 bg-[#1B0825] text-[#D9CBE8] hover:text-white font-bold cursor-pointer tabular-nums whitespace-nowrap"
                  >
                    Cap: {screenFps} FPS
                  </button>
                </div>
              </div>

              {/* Export IR & Adapters */}
              <div className="space-y-2">
                <div className="flex items-center justify-between gap-2">
                  <span className="text-[10px] font-bold uppercase tracking-widest text-[#B8A9C9]">
                    Export IR &amp; Adapters
                  </span>
                  <span className="text-[9.5px] text-[#9E8EB3]">
                    {EXPORT_IR_SCHEMA} v{EXPORT_IR_VERSION}
                  </span>
                </div>
                <div className="grid grid-cols-2 gap-1.5">
                  <button
                    type="button"
                    data-export-ir="ir_json"
                    onClick={() => onExportFormat('ir_json')}
                    className="py-1.5 px-2 rounded-[2px] border border-[#EBE0F5]/16 bg-[#1B0825] text-[#FAF6FF] hover:border-[#D8B4FE]/50 hover:bg-[#2C143B] font-bold text-[10px] flex items-center justify-center gap-1 cursor-pointer whitespace-nowrap"
                  >
                    <Download className="w-3 h-3 shrink-0" />
                    <span>IR JSON</span>
                  </button>
                  <button
                    type="button"
                    data-export-ir="layered_svg"
                    onClick={() => onExportFormat('layered_svg')}
                    className="py-1.5 px-2 rounded-[2px] border border-[#EBE0F5]/16 bg-[#1B0825] text-[#FAF6FF] hover:border-[#D8B4FE]/50 hover:bg-[#2C143B] font-bold text-[10px] flex items-center justify-center gap-1 cursor-pointer whitespace-nowrap"
                  >
                    <Download className="w-3 h-3 shrink-0" />
                    <span>Layered SVG</span>
                  </button>
                  <button
                    type="button"
                    data-export-ir="spine_json"
                    onClick={() => onExportFormat('spine_json')}
                    className="py-1.5 px-2 rounded-[2px] border border-[#EBE0F5]/16 bg-[#1B0825] text-[#FAF6FF] hover:border-[#34D399]/50 hover:bg-[#112A22] font-bold text-[10px] flex items-center justify-center gap-1 cursor-pointer whitespace-nowrap"
                  >
                    <Download className="w-3 h-3 shrink-0" />
                    <span>Spine JSON</span>
                  </button>
                  <button
                    type="button"
                    data-export-ir="dragonbones_json"
                    onClick={() => onExportFormat('dragonbones_json')}
                    className="py-1.5 px-2 rounded-[2px] border border-[#EBE0F5]/16 bg-[#1B0825] text-[#FAF6FF] hover:border-[#34D399]/50 hover:bg-[#112A22] font-bold text-[10px] flex items-center justify-center gap-1 cursor-pointer whitespace-nowrap"
                  >
                    <Download className="w-3 h-3 shrink-0" />
                    <span>DragonBones</span>
                  </button>
                </div>
              </div>
            </div>
          )}
        </div>
      )}
    </section>
  );
});
