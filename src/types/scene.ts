import {
  ArmatureState,
  JointId,
  KeyframeSlot,
  MotionCurve,
  PosePresetId,
} from './rig';
import { ScreenFpsOption } from '../utils/frameThrottle';

export type SceneMediaKind = 'paper' | 'image' | 'video';

export type PaperPlateVariant = 'cover' | 'backing';

export type SamplePlateId = 'stone_archway' | 'botanical_pergola';

export type CompositeBlendMode = 'multiply' | 'darken' | 'normal';

export interface SceneMedia {
  kind: SceneMediaKind;
  paperVariant?: PaperPlateVariant;
  sampleId?: SamplePlateId;
  src: string;
  name: string;
  duration: number;
  fps: ScreenFpsOption;
  naturalWidth: number;
  naturalHeight: number;
  loop: boolean;
  playbackOffset: number;
}

export interface StageTransform {
  originX: number;
  originY: number;
  pyxScale: number;
  rotation: number;
}

export interface WorldPin {
  id: string;
  jointId: JointId;
  mediaSpaceXY: {
    x: number;
    y: number;
  };
  frameRange: readonly [number, number];
  enabled: boolean;
}

export interface CompositeFilter {
  blendMode: CompositeBlendMode;
  inkOpacity: number;
  edgeThreshold: number;
  paperGrain: number;
}

export interface SceneCameraState {
  zoom: number;
  pan: {
    x: number;
    y: number;
  };
  userEdited: boolean;
}

export interface Scene {
  id: string;
  name: string;
  durationSec: number;
  media: SceneMedia;
  stageTransform: StageTransform;
  compositeFilter: CompositeFilter;
  worldPins: WorldPin[];
  armature: ArmatureState;
  activePreset: PosePresetId | null;
  slots: Record<KeyframeSlot, ArmatureState | null>;
  activeSlot: KeyframeSlot | null;
  timelineT: number;
  motionCurve: MotionCurve;
  playbackSpeed: number;
  showPerspective: boolean;
  vanishingPoint: {
    x: number;
    y: number;
  };
  camera: SceneCameraState;
}

export type ScenePlaybackMode = 'scene' | 'sequence';
