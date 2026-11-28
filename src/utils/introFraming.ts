import { Point2D } from '../types/rig';
import { computeSymmetricalCenterPan } from './proportions';

export const INTRO_GREETING_TEXT = 'Greetings, pyrson. Have you no name?';
export const INTRO_GREETING_PART_1 = 'Greetings, pyrson....';
export const INTRO_GREETING_PART_2 = 'Have you no name?';
export const INTRO_NO_RESPONSE_TEXT = 'Have no name.';
export const INTRO_Y_LABEL = 'Y · Yes, I have no name';
export const INTRO_N_LABEL = 'N · No, I have a name';
export const INTRO_DRAG_PROMPT =
  'Drag face plate to Y or N and hover or place to choose your identity';
export const INTRO_DRAG_OR_HOVER_PROMPT = INTRO_DRAG_PROMPT;

export const INTRO_MARGIN_Y = 364; // ~1 head length above top of head (486 - 122)
export const INTRO_MARGIN_WIDTH = 640;
export const INTRO_MARGIN_HEIGHT = 118;

export const Y_EXCLAMATIONS = [
  'Congratulations!',
  'Zounds!',
  'Huzzah!',
  'Brilliant!',
  'Excellent!',
] as const;

export function getRandomYExclamation(): string {
  const index = Math.floor(Math.random() * Y_EXCLAMATIONS.length);
  return Y_EXCLAMATIONS[index];
}

export const INTRO_FACEPLATE_THRESHOLD_PX = 36;
export const INTRO_TARGET_OFFSET_PX = 140;
export const INTRO_TARGET_RADIUS_PX = 48;

export type IntroChoice = 'Y' | 'N';

export interface FaceplateChoiceEvaluation {
  choice: IntroChoice | null;
  deltaX: number;
  progressY: number;
  progressN: number;
  placedOnTop: boolean;
}

export function isPlacedOnTop(
  point: Point2D,
  target: Point2D,
  hitRadius = INTRO_TARGET_RADIUS_PX
): boolean {
  return Math.hypot(point.x - target.x, point.y - target.y) <= hitRadius;
}

export function resolveFaceplateTargetChoice(
  isFaceplateClickedDown: boolean,
  restLip: Point2D,
  points: readonly (Point2D | null | undefined)[],
  hitRadius = INTRO_TARGET_RADIUS_PX
): IntroChoice | null {
  if (!isFaceplateClickedDown) return null;
  const targetY: Point2D = { x: restLip.x - INTRO_TARGET_OFFSET_PX, y: restLip.y };
  const targetN: Point2D = { x: restLip.x + INTRO_TARGET_OFFSET_PX, y: restLip.y };

  for (const pt of points) {
    if (!pt) continue;
    if (isPlacedOnTop(pt, targetY, hitRadius)) return 'Y';
    if (isPlacedOnTop(pt, targetN, hitRadius)) return 'N';
  }
  return null;
}

export function computeIntroCamera(
  vw: number,
  vh: number,
  lipCenter: Point2D
): { zoom: number; pan: Point2D } {
  const zoom = 1.0;
  return {
    zoom,
    pan: computeSymmetricalCenterPan(vw, vh, lipCenter.x, lipCenter.y, zoom),
  };
}

export function evaluateFaceplateChoice(
  lipX: number,
  restLipX: number,
  thresholdPx: number = INTRO_FACEPLATE_THRESHOLD_PX,
  lipY?: number,
  restLipY?: number
): FaceplateChoiceEvaluation {
  const deltaX = lipX - restLipX;
  const safeThreshold = Math.max(1, thresholdPx);
  const progressY = Math.max(0, Math.min(1, -deltaX / safeThreshold));
  const progressN = Math.max(0, Math.min(1, deltaX / safeThreshold));

  const leftTargetX = restLipX - INTRO_TARGET_OFFSET_PX;
  const rightTargetX = restLipX + INTRO_TARGET_OFFSET_PX;
  const targetY = restLipY ?? 547.25;
  const currentY = lipY ?? targetY;

  const onTopOfY = Math.hypot(lipX - leftTargetX, currentY - targetY) <= INTRO_TARGET_RADIUS_PX;
  const onTopOfN = Math.hypot(lipX - rightTargetX, currentY - targetY) <= INTRO_TARGET_RADIUS_PX;

  const choice: IntroChoice | null =
    onTopOfY || deltaX <= -safeThreshold
      ? 'Y'
      : onTopOfN || deltaX >= safeThreshold
      ? 'N'
      : null;

  return {
    choice,
    deltaX,
    progressY: onTopOfY ? 1 : progressY,
    progressN: onTopOfN ? 1 : progressN,
    placedOnTop: onTopOfY || onTopOfN,
  };
}

export const DOUBLE_LINE_TOP_WORLD_Y = 370;
export const PLANQUETTE_TITLE = 'Spell your name';
export const PLANQUETTE_WARNING = 'WARNING RESULTS ARE FINAL';

export const PLANQUETTE_CHARS = [
  ' ',
  'A',
  'B',
  'C',
  'D',
  'E',
  'F',
  'G',
  'H',
  'I',
  'J',
  'K',
  'L',
  'M',
  'N',
  'O',
  'P',
  'Q',
  'R',
  'S',
  'T',
  'U',
  'V',
  'W',
  'X',
  'Y',
  'Z',
  ' ',
] as const;

export const PLANQUETTE_ARC_RADIUS = 510;
export const PLANQUETTE_ARC_CENTER_Y = 900;
export const PLANQUETTE_ARC_HALF_SPAN_DEG = 58;
export const PLANQUETTE_ARC_FONT_SIZE = 34;
export const PLANQUETTE_ARC_HIT_BAND_PX = 36;

export interface PlanquetteArcLetter {
  index: number;
  char: string;
  label: string;
  isSpace: boolean;
  angleDeg: number;
  x: number;
  y: number;
}

export interface PlanquetteArcLayout {
  centerX: number;
  centerY: number;
  radius: number;
  halfSpanDeg: number;
  stepDeg: number;
  fontSize: number;
  hitBandPx: number;
  apexY: number;
  letters: PlanquetteArcLetter[];
}

export function computePlanquetteCamera(
  vw: number,
  vh: number,
  originX = 681.5
): { zoom: number; pan: Point2D } {
  const safeW = Math.max(320, vw);
  const safeH = Math.max(320, vh);
  const zoom = Number(Math.max(0.68, Math.min(1.18, safeW / 1140, safeH / 680)).toFixed(4));
  const panX = safeW / 2 - originX * zoom;
  const rawPanY = safeH / 2 - 480 * zoom;
  const minTopPanY = 14 - 165 * zoom;
  return {
    zoom,
    pan: {
      x: panX,
      y: Math.max(rawPanY, minTopPanY),
    },
  };
}

export function computePlanquetteArc(
  originX = 681.5,
  originY = PLANQUETTE_ARC_CENTER_Y,
  radius = PLANQUETTE_ARC_RADIUS,
  halfSpanDeg = PLANQUETTE_ARC_HALF_SPAN_DEG
): PlanquetteArcLayout {
  const count = PLANQUETTE_CHARS.length;
  const stepDeg = (halfSpanDeg * 2) / Math.max(1, count - 1);
  const letters: PlanquetteArcLetter[] = PLANQUETTE_CHARS.map((char, index) => {
    const angleDeg = -halfSpanDeg + index * stepDeg;
    const rad = (angleDeg * Math.PI) / 180;
    const isSpace = char === ' ';
    return {
      index,
      char,
      label: isSpace ? '␣' : char,
      isSpace,
      angleDeg,
      x: originX + radius * Math.sin(rad),
      y: originY - radius * Math.cos(rad),
    };
  });

  return {
    centerX: originX,
    centerY: originY,
    radius,
    halfSpanDeg,
    stepDeg,
    fontSize: PLANQUETTE_ARC_FONT_SIZE,
    hitBandPx: PLANQUETTE_ARC_HIT_BAND_PX,
    apexY: originY - radius,
    letters,
  };
}

export function hitTestPlanquetteArc(
  worldX: number,
  worldY: number,
  layout: PlanquetteArcLayout
): PlanquetteArcLetter | null {
  const dx = worldX - layout.centerX;
  const dy = layout.centerY - worldY;
  const radialDist = Math.hypot(dx, dy);
  if (Math.abs(radialDist - layout.radius) > layout.hitBandPx) {
    return null;
  }

  const angleDeg = (Math.atan2(dx, dy) * 180) / Math.PI;
  const maxAngle = layout.halfSpanDeg + layout.stepDeg * 0.65;
  if (Math.abs(angleDeg) > maxAngle) {
    return null;
  }

  const rawIdx = Math.round((angleDeg + layout.halfSpanDeg) / layout.stepDeg);
  const idx = Math.max(0, Math.min(layout.letters.length - 1, rawIdx));
  return layout.letters[idx] ?? null;
}

export function projectFaceplateToPlanquetteArc(
  headWorld: Point2D,
  lipWorld: Point2D,
  pointerWorld: Point2D,
  layout: PlanquetteArcLayout
): Point2D {
  const ptrDx = pointerWorld.x - headWorld.x;
  const ptrDy = pointerWorld.y - headWorld.y;
  const ptrLen = Math.hypot(ptrDx, ptrDy);

  // If pointer is already on the outer arc band, track it directly
  const ptrRadius = Math.hypot(pointerWorld.x - layout.centerX, layout.centerY - pointerWorld.y);
  if (
    Math.abs(ptrRadius - layout.radius) <= layout.hitBandPx &&
    pointerWorld.y <= layout.centerY
  ) {
    return pointerWorld;
  }

  // Raycast upward/outward faceplate sightline onto the Ouija arc circle
  const aimingAtArc = ptrLen > 12 && pointerWorld.y <= headWorld.y + 40;
  if (!aimingAtArc) {
    return lipWorld;
  }

  const ux = ptrDx / ptrLen;
  const uy = ptrDy / ptrLen;
  const ox = headWorld.x - layout.centerX;
  const oy = headWorld.y - layout.centerY;
  const b = ox * ux + oy * uy;
  const c = ox * ox + oy * oy - layout.radius * layout.radius;
  const disc = b * b - c;
  if (disc < 0) {
    return lipWorld;
  }

  const t = -b + Math.sqrt(disc);
  if (t <= 0) {
    return lipWorld;
  }

  const hitX = headWorld.x + t * ux;
  const hitY = headWorld.y + t * uy;
  const angleDeg = (Math.atan2(hitX - layout.centerX, layout.centerY - hitY) * 180) / Math.PI;
  const clampedAngle = Math.max(
    -layout.halfSpanDeg - layout.stepDeg * 0.4,
    Math.min(layout.halfSpanDeg + layout.stepDeg * 0.4, angleDeg)
  );
  const rad = (clampedAngle * Math.PI) / 180;
  return {
    x: layout.centerX + layout.radius * Math.sin(rad),
    y: layout.centerY - layout.radius * Math.cos(rad),
  };
}
