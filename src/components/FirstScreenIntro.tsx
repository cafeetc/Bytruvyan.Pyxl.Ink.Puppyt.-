import React, { useEffect, useState } from 'react';
import { Point2D } from '../types/rig';
import {
  DOUBLE_LINE_TOP_WORLD_Y,
  FaceplateChoiceEvaluation,
  INTRO_DRAG_OR_HOVER_PROMPT,
  INTRO_GREETING_PART_1,
  INTRO_GREETING_PART_2,
  INTRO_MARGIN_Y,
  INTRO_N_LABEL,
  INTRO_TARGET_OFFSET_PX,
  INTRO_Y_LABEL,
  IntroChoice,
  PLANQUETTE_TITLE,
  PLANQUETTE_WARNING,
  computePlanquetteArc,
  getRandomYExclamation,
} from '../utils/introFraming';

export type IntroStage =
  | 'greeting_1'
  | 'greeting_2'
  | 'prompt'
  | 'n_response'
  | 'input_name'
  | 'processing_1'
  | 'processing_2'
  | 'processing_3'
  | 'processing_4'
  | 'processing_5'
  | 'processing_6'
  | 'not_found'
  | 'facsimile'
  | 'welcomed'
  | 'complete';

function usePrefersReducedMotion(): boolean {
  const [reduced, setReduced] = useState(() =>
    typeof window !== 'undefined' &&
    typeof window.matchMedia === 'function' &&
    window.matchMedia('(prefers-reduced-motion: reduce)').matches
  );

  useEffect(() => {
    if (typeof window === 'undefined' || typeof window.matchMedia !== 'function') {
      return;
    }
    const mq = window.matchMedia('(prefers-reduced-motion: reduce)');
    const onChange = () => setReduced(mq.matches);
    mq.addEventListener('change', onChange);
    return () => mq.removeEventListener('change', onChange);
  }, []);

  return reduced;
}

function useTimedTyping(
  text: string,
  startDelayMs = 1000,
  targetDurationMs = 1000
): { displayed: string; isComplete: boolean; isTyping: boolean } {
  const reducedMotion = usePrefersReducedMotion();
  const [count, setCount] = useState(() => (reducedMotion ? text.length : 0));
  const [started, setStarted] = useState(false);

  useEffect(() => {
    if (reducedMotion) {
      setCount(text.length);
      setStarted(true);
      return;
    }
    setCount(0);
    setStarted(false);
    if (!text) return;

    if (targetDurationMs <= 0) {
      setCount(text.length);
      setStarted(true);
      return;
    }

    const delayTimer = window.setTimeout(() => {
      setStarted(true);
      const charTime = Math.max(14, Math.round(targetDurationMs / Math.max(1, text.length)));
      let current = 0;
      const typeTimer = window.setInterval(() => {
        current += 1;
        setCount(current);
        if (current >= text.length) {
          window.clearInterval(typeTimer);
        }
      }, charTime);
      return () => window.clearInterval(typeTimer);
    }, startDelayMs);

    return () => {
      window.clearTimeout(delayTimer);
    };
  }, [reducedMotion, startDelayMs, targetDurationMs, text]);

  const isComplete = started && count >= text.length;
  const isTyping = started && !isComplete;

  return {
    displayed: text.slice(0, count),
    isComplete,
    isTyping,
  };
}

interface FirstScreenSvgOverlayProps {
  restLip: Point2D;
  currentLip: Point2D;
  evaluation: FaceplateChoiceEvaluation;
  isDraggingFaceplate: boolean;
  hoveredChoice: IntroChoice | null;
  darkMode: boolean;
}

export const FirstScreenSvgOverlay: React.FC<FirstScreenSvgOverlayProps> = ({
  restLip,
  currentLip,
  evaluation,
  isDraggingFaceplate,
  hoveredChoice,
  darkMode,
}) => {
  const leftX = restLip.x - INTRO_TARGET_OFFSET_PX;
  const rightX = restLip.x + INTRO_TARGET_OFFSET_PX;
  const axisY = restLip.y;

  const baseInk = darkMode ? '#FAF6FF' : '#16001E';
  const activeAccent = darkMode ? '#6EE7B7' : '#0F766E';
  const leftActive = evaluation.choice === ('Y' as IntroChoice) || hoveredChoice === ('Y' as IntroChoice);
  const rightActive = evaluation.choice === ('N' as IntroChoice) || hoveredChoice === ('N' as IntroChoice);

  const leftColor = leftActive ? activeAccent : baseInk;
  const rightColor = rightActive ? activeAccent : baseInk;

  const leftOpacity = leftActive
    ? 1
    : 0.75 + Math.max(evaluation.progressY, hoveredChoice === ('Y' as IntroChoice) ? 1 : 0) * 0.25;
  const rightOpacity = rightActive
    ? 1
    : 0.75 + Math.max(evaluation.progressN, hoveredChoice === ('N' as IntroChoice) ? 1 : 0) * 0.25;

  const circleCircumference = 2 * Math.PI * 26; // ~163.36

  return (
    <g data-first-screen-targets="true">
      {/* Symmetrical dashed alignment datum line connecting Y and N */}
      <line
        x1={leftX + 26}
        y1={axisY}
        x2={rightX - 26}
        y2={axisY}
        stroke={baseInk}
        strokeOpacity={darkMode ? 0.22 : 0.18}
        strokeWidth={1}
        strokeDasharray="3 4"
        vectorEffect="non-scaling-stroke"
        pointerEvents="none"
      />

      {/* Dynamic faceplate drag connector */}
      {Math.abs(evaluation.deltaX) > 2 && (
        <line
          x1={restLip.x}
          y1={axisY}
          x2={currentLip.x}
          y2={currentLip.y}
          stroke={
            evaluation.choice
              ? activeAccent
              : darkMode
              ? '#D8B4FE'
              : '#583168'
          }
          strokeOpacity={0.88}
          strokeWidth={1.8}
          vectorEffect="non-scaling-stroke"
          pointerEvents="none"
        />
      )}

      {/* Faceplate drag locator indicator */}
      <g
        transform={`translate(${currentLip.x}, ${currentLip.y})`}
        pointerEvents="none"
      >
        <circle
          r={33}
          fill="none"
          stroke={
            evaluation.choice
              ? activeAccent
              : isDraggingFaceplate
              ? darkMode
                ? '#D8B4FE'
                : '#583168'
              : baseInk
          }
          strokeOpacity={
            evaluation.choice ? 0.95 : isDraggingFaceplate ? 0.8 : 0.35
          }
          strokeWidth={evaluation.choice ? 2 : 1.1}
          strokeDasharray={evaluation.choice ? undefined : '4 3'}
          vectorEffect="non-scaling-stroke"
        />
        <path
          d="M -39 -4 L -44 0 L -39 4"
          fill="none"
          stroke={leftActive ? activeAccent : baseInk}
          strokeOpacity={leftActive ? 0.95 : 0.45}
          strokeWidth={1.3}
          strokeLinecap="square"
          vectorEffect="non-scaling-stroke"
        />
        <path
          d="M 39 -4 L 44 0 L 39 4"
          fill="none"
          stroke={rightActive ? activeAccent : baseInk}
          strokeOpacity={rightActive ? 0.95 : 0.45}
          strokeWidth={1.3}
          strokeLinecap="square"
          vectorEffect="non-scaling-stroke"
        />
      </g>

      {/* Left target & text: Y (to the left of the head) */}
      <g
        data-intro-target="Y"
        data-active={leftActive ? 'true' : 'false'}
        aria-label="Yes, I have no name (drag faceplate over Y and hover 1s or place on top)"
        transform={`translate(${leftX}, ${axisY})`}
        pointerEvents="none"
      >
        <circle r={36} fill="transparent" />

        {/* 1-second radial click-hover dwell progress circle */}
        <circle
          r={26}
          fill="none"
          stroke={activeAccent}
          strokeWidth={2.4}
          strokeDasharray={circleCircumference}
          strokeDashoffset={hoveredChoice === 'Y' ? 0 : circleCircumference}
          strokeLinecap="round"
          transform="rotate(-90)"
          style={{
            transition:
              hoveredChoice === 'Y'
                ? 'stroke-dashoffset 1000ms linear'
                : 'stroke-dashoffset 160ms ease-out',
          }}
          vectorEffect="non-scaling-stroke"
        />

        {/* Architectural corner bracket frame */}
        <path
          d="M -18 -12 L -18 -18 L -12 -18 M 12 -18 L 18 -18 L 18 -12 M -18 12 L -18 18 L -12 18 M 12 18 L 18 18 L 18 12"
          fill="none"
          stroke={leftColor}
          strokeOpacity={leftOpacity}
          strokeWidth={leftActive ? 2 : 1.25}
          vectorEffect="non-scaling-stroke"
        />

        <text
          x={0}
          y={9}
          textAnchor="middle"
          fill={leftColor}
          fillOpacity={leftOpacity}
          fontSize={26}
          fontWeight={700}
          fontFamily="UMTypewriter, Courier New, monospace"
        >
          Y
        </text>

        {/* Text to the left of the head */}
        <text
          x={-34}
          y={6}
          textAnchor="end"
          fill={leftColor}
          fillOpacity={leftActive ? 1 : 0.85}
          fontSize={14}
          fontWeight={leftActive ? 700 : 500}
          fontFamily="Bitstream Vera Sans Mono, monospace"
          letterSpacing="0.4"
        >
          {INTRO_Y_LABEL}
        </text>
      </g>

      {/* Right target & text: N (to the right of the head) */}
      <g
        data-intro-target="N"
        data-active={rightActive ? 'true' : 'false'}
        aria-label="No, I have a name (drag faceplate over N and hover 1s or place on top)"
        transform={`translate(${rightX}, ${axisY})`}
        pointerEvents="none"
      >
        <circle r={36} fill="transparent" />

        {/* 1-second radial click-hover dwell progress circle */}
        <circle
          r={26}
          fill="none"
          stroke={activeAccent}
          strokeWidth={2.4}
          strokeDasharray={circleCircumference}
          strokeDashoffset={hoveredChoice === 'N' ? 0 : circleCircumference}
          strokeLinecap="round"
          transform="rotate(-90)"
          style={{
            transition:
              hoveredChoice === 'N'
                ? 'stroke-dashoffset 1000ms linear'
                : 'stroke-dashoffset 160ms ease-out',
          }}
          vectorEffect="non-scaling-stroke"
        />

        {/* Architectural corner bracket frame */}
        <path
          d="M -18 -12 L -18 -18 L -12 -18 M 12 -18 L 18 -18 L 18 -12 M -18 12 L -18 18 L -12 18 M 12 18 L 18 18 L 18 12"
          fill="none"
          stroke={rightColor}
          strokeOpacity={rightOpacity}
          strokeWidth={rightActive ? 2 : 1.25}
          vectorEffect="non-scaling-stroke"
        />

        <text
          x={0}
          y={9}
          textAnchor="middle"
          fill={rightColor}
          fillOpacity={rightOpacity}
          fontSize={26}
          fontWeight={700}
          fontFamily="UMTypewriter, Courier New, monospace"
        >
          N
        </text>

        {/* Text to the right of the head */}
        <text
          x={34}
          y={6}
          textAnchor="start"
          fill={rightColor}
          fillOpacity={rightActive ? 1 : 0.85}
          fontSize={14}
          fontWeight={rightActive ? 700 : 500}
          fontFamily="Bitstream Vera Sans Mono, monospace"
          letterSpacing="0.4"
        >
          {INTRO_N_LABEL}
        </text>
      </g>
    </g>
  );
};

interface FirstScreenPlanquetteSvgOverlayProps {
  origin: Point2D;
  lipWorld: Point2D;
  hoveredIndex: number | null;
  cursorWorld: Point2D | null;
  source: 'faceplate' | 'handR' | 'handL' | null;
  darkMode: boolean;
}

function describeWorldArc(
  cx: number,
  cy: number,
  radius: number,
  startDeg: number,
  endDeg: number
): string {
  const sRad = (startDeg * Math.PI) / 180;
  const eRad = (endDeg * Math.PI) / 180;
  const x1 = cx + radius * Math.sin(sRad);
  const y1 = cy - radius * Math.cos(sRad);
  const x2 = cx + radius * Math.sin(eRad);
  const y2 = cy - radius * Math.cos(eRad);
  return `M ${x1.toFixed(2)} ${y1.toFixed(2)} A ${radius} ${radius} 0 0 1 ${x2.toFixed(2)} ${y2.toFixed(2)}`;
}

export const FirstScreenPlanquetteSvgOverlay: React.FC<FirstScreenPlanquetteSvgOverlayProps> = ({
  origin,
  lipWorld,
  hoveredIndex,
  cursorWorld,
  source,
  darkMode,
}) => {
  const arc = computePlanquetteArc(origin.x, origin.y);
  const baseInk = darkMode ? '#FAF6FF' : '#16001E';
  const activeAccent = darkMode ? '#6EE7B7' : '#0F766E';
  const ringCircumference = 2 * Math.PI * 20; // ~125.66
  const trackSpan = arc.halfSpanDeg + 2.4;

  return (
    <g data-planquette-ouija-arc="true" pointerEvents="none">
      {/* Classic Ouija-board hairline guide arcs (no tiles) */}
      <path
        d={describeWorldArc(arc.centerX, arc.centerY, arc.radius + 25, -trackSpan, trackSpan)}
        fill="none"
        stroke={baseInk}
        strokeOpacity={darkMode ? 0.24 : 0.2}
        strokeWidth={1.1}
        vectorEffect="non-scaling-stroke"
      />
      <path
        d={describeWorldArc(arc.centerX, arc.centerY, arc.radius - 25, -trackSpan, trackSpan)}
        fill="none"
        stroke={baseInk}
        strokeOpacity={darkMode ? 0.16 : 0.14}
        strokeWidth={0.9}
        strokeDasharray="4 6"
        vectorEffect="non-scaling-stroke"
      />

      {arc.letters.map((letter) => {
        const isHovered = hoveredIndex === letter.index;
        return (
          <g
            key={`ouija-letter-${letter.index}`}
            data-planquette-letter={letter.index}
            data-planquette-char={letter.char}
            data-hovered={isHovered ? 'true' : 'false'}
            transform={`translate(${letter.x.toFixed(2)}, ${letter.y.toFixed(2)}) rotate(${letter.angleDeg.toFixed(2)})`}
          >
            {isHovered && (
              <circle
                key={`arc-dwell-${letter.index}`}
                r={20}
                fill={activeAccent}
                fillOpacity={darkMode ? 0.16 : 0.12}
                stroke={activeAccent}
                strokeWidth={2.2}
                strokeDasharray={ringCircumference}
                strokeLinecap="round"
                transform="rotate(-90)"
                style={{
                  animation: 'planquetteArcRingFill 1000ms linear forwards',
                }}
                vectorEffect="non-scaling-stroke"
              />
            )}
            <text
              x={0}
              y={letter.isSpace ? 9 : 11}
              textAnchor="middle"
              fill={isHovered ? activeAccent : baseInk}
              fillOpacity={isHovered ? 1 : 0.92}
              fontSize={letter.isSpace ? 26 : arc.fontSize}
              fontWeight={700}
              fontFamily="UMTypewriter, Courier New, monospace"
            >
              {letter.label}
            </text>
            {letter.isSpace && (
              <text
                x={0}
                y={21}
                textAnchor="middle"
                fill={isHovered ? activeAccent : baseInk}
                fillOpacity={isHovered ? 0.9 : 0.52}
                fontSize={8.5}
                fontWeight={700}
                fontFamily="Bitstream Vera Sans Mono, monospace"
                letterSpacing="0.6"
              >
                SPC
              </text>
            )}
          </g>
        );
      })}

      {/* Faceplate sightline + functional world cursor reticle */}
      {cursorWorld && (
        <g>
          {source === 'faceplate' && (
            <line
              x1={lipWorld.x}
              y1={lipWorld.y}
              x2={cursorWorld.x}
              y2={cursorWorld.y}
              stroke={activeAccent}
              strokeOpacity={0.75}
              strokeWidth={1.6}
              strokeDasharray="4 4"
              vectorEffect="non-scaling-stroke"
            />
          )}
          <g
            data-planquette-cursor="true"
            data-planquette-source={source ?? ''}
            transform={`translate(${cursorWorld.x.toFixed(2)}, ${cursorWorld.y.toFixed(2)})`}
          >
            <circle
              r={16}
              fill="none"
              stroke={activeAccent}
              strokeWidth={2}
              strokeOpacity={0.95}
              vectorEffect="non-scaling-stroke"
            />
            <circle r={3.2} fill={activeAccent} />
          </g>
        </g>
      )}
    </g>
  );
};

interface FirstScreenHudOverlayProps {
  stage: IntroStage;
  pan: Point2D;
  zoom: number;
  restLip: Point2D;
  figureCenter: Point2D;
  viewportHeight: number;
  darkMode: boolean;
  evaluation: FaceplateChoiceEvaluation;
  isDraggingFaceplate: boolean;
  hoveredChoice: IntroChoice | null;
  committedName: string | null;
  planquetteName: string;
  onChooseSide: (choice: IntroChoice) => void;
  onSubmitName: (name: string) => void;
  onHoverChoice: (choice: IntroChoice | null) => void;
  onGreetingSequenceComplete: () => void;
  onTypingStateChange?: (isTyping: boolean) => void;
}

export const FirstScreenHudOverlay: React.FC<FirstScreenHudOverlayProps> = ({
  stage,
  pan,
  zoom,
  restLip,
  figureCenter,
  viewportHeight,
  darkMode,
  committedName,
  planquetteName,
  onSubmitName,
  onGreetingSequenceComplete,
  onTypingStateChange,
}) => {
  const [yExclamation] = useState(() => getRandomYExclamation());

  // Derive text for current stage
  let textToType = '';
  let startDelayMs = 0;
  let targetDurationMs = 1000;

  if (stage === 'greeting_1') {
    textToType = INTRO_GREETING_PART_1; // "Greetings, pyrson...."
    startDelayMs = 1000; // 1 second on start
    targetDurationMs = 1000; // 1 second typing duration
  } else if (stage === 'greeting_2') {
    textToType = INTRO_GREETING_PART_2; // "Have you no name?"
    startDelayMs = 200;
    targetDurationMs = 1000; // 1 second typing duration
  } else if (stage === 'prompt') {
    textToType = INTRO_GREETING_PART_2;
    startDelayMs = 0;
    targetDurationMs = 0;
  } else if (stage === 'n_response') {
    textToType = `${yExclamation} Tell me your name.`;
    startDelayMs = 100;
    targetDurationMs = 1000;
  } else if (stage === 'input_name') {
    textToType = PLANQUETTE_TITLE;
    startDelayMs = 0;
    targetDurationMs = 0;
  } else if (stage === 'processing_1') {
    textToType = 'Procyssyng Name...';
    startDelayMs = 0;
    targetDurationMs = 0;
  } else if (stage === 'processing_2') {
    textToType = 'Confirming identity....';
    startDelayMs = 0;
    targetDurationMs = 0;
  } else if (stage === 'processing_3') {
    textToType = 'sorting tokens....';
    startDelayMs = 0;
    targetDurationMs = 0;
  } else if (stage === 'processing_4') {
    textToType = 'arranging letters.....';
    startDelayMs = 0;
    targetDurationMs = 0;
  } else if (stage === 'processing_5') {
    textToType = 'finding origin....';
    startDelayMs = 0;
    targetDurationMs = 0;
  } else if (stage === 'processing_6') {
    textToType = 'finalizing name....';
    startDelayMs = 0;
    targetDurationMs = 0;
  } else if (stage === 'not_found') {
    textToType = 'Name not found.';
    startDelayMs = 0;
    targetDurationMs = 0;
  } else if (stage === 'facsimile') {
    textToType = 'Please accept this new facsimile instead';
    startDelayMs = 0;
    targetDurationMs = 0;
  } else if (stage === 'welcomed') {
    textToType = `Welcome, ${committedName || 'Bytruvyus'}.`;
    startDelayMs = 0;
    targetDurationMs = 0;
  }

  const typed = useTimedTyping(textToType, startDelayMs, targetDurationMs);

  useEffect(() => {
    onTypingStateChange?.(typed.isTyping);
  }, [typed.isTyping, onTypingStateChange]);

  useEffect(() => {
    if (stage === 'greeting_1' && typed.isComplete) {
      // Last 1 second after typing completes, then transition to greeting_2
      const t = window.setTimeout(() => {
        onGreetingSequenceComplete();
      }, 1000);
      return () => window.clearTimeout(t);
    }
    if (stage === 'greeting_2' && typed.isComplete) {
      const t = window.setTimeout(() => {
        onGreetingSequenceComplete();
      }, 500);
      return () => window.clearTimeout(t);
    }
  }, [stage, typed.isComplete, onGreetingSequenceComplete]);

  if (stage === 'complete') return null;

  // Margin center coordinates: ~1 head length up from top of head, centered horizontally over faceplate
  const marginCenterX = pan.x + restLip.x * zoom;
  const marginCenterY = pan.y + INTRO_MARGIN_Y * zoom;

  // Title screen coordinates: centered horizontally over the figure, resting on top of the 2nd box line (y = 370)
  const figureScreenX = pan.x + figureCenter.x * zoom;
  const secondBoxLineScreenY = pan.y + DOUBLE_LINE_TOP_WORLD_Y * zoom;
  const titleAnchorY = Math.max(76, secondBoxLineScreenY - 4);

  const plaqueTheme = darkMode
    ? 'bg-[#0C0712]/94 border-[#D8B4FE]/35 text-[#FAF6FF]'
    : 'bg-[#F3EFE7]/96 border-[#16001E]/30 text-[#16001E]';

  const titlePlaqueTheme = darkMode
    ? 'bg-[#0C0712]/78 border-[#D8B4FE]/45 text-[#FAF6FF]'
    : 'bg-[#F3EFE7]/82 border-[#16001E]/35 text-[#16001E]';

  const primaryBtnTheme = darkMode
    ? 'border-[#6EE7B7]/60 bg-[#064E3B] text-[#FAF6FF] hover:bg-[#065F46]'
    : 'border-[#16001E] bg-[#16001E] text-[#F7F4EE] hover:bg-[#2E0E3E]';

  if (stage === 'welcomed') {
    return (
      <div
        data-first-screen-overlay="true"
        data-intro-stage={stage}
        data-welcome-title-screen="true"
        className="pointer-events-none absolute inset-0 z-10"
      >
        <div
          className="absolute -translate-x-1/2 -translate-y-full px-4 flex flex-col items-center justify-end text-center"
          style={{
            left: `${Math.round(figureScreenX)}px`,
            top: `${Math.round(titleAnchorY)}px`,
          }}
        >
          <div
            className={`px-7 py-3.5 sm:px-12 sm:py-5 rounded-[4px] border shadow-2xl backdrop-blur-[2px] flex flex-col items-center justify-center gap-2 ${titlePlaqueTheme}`}
          >
            <div className="w-24 h-px bg-current opacity-35" />
            <h1
              aria-live="polite"
              className="font-typewriter font-bold text-2xl sm:text-4xl md:text-5xl tracking-wider whitespace-nowrap"
            >
              {`Welcome, ${committedName || 'Bytruvyus'}.`}
            </h1>
            <div className="w-24 h-px bg-current opacity-35" />
          </div>
        </div>
      </div>
    );
  }

  if (stage === 'input_name') {
    const doubleLineTopScreenY = pan.y + (DOUBLE_LINE_TOP_WORLD_Y - 14) * zoom;
    const promptBottomPx = Math.max(10, Math.round(viewportHeight - doubleLineTopScreenY));

    return (
      <div
        data-first-screen-overlay="true"
        data-intro-stage={stage}
        data-planquette-active="true"
        className="pointer-events-none absolute inset-0 z-10"
      >
        {/* Prompt, blank line space, flashing magenta-red warning, and irreversible Claim readout above the double line */}
        <div
          className="absolute -translate-x-1/2 w-[min(96vw,760px)] px-3 flex flex-col items-center justify-end text-center"
          style={{
            left: `${Math.round(figureScreenX)}px`,
            bottom: `${promptBottomPx}px`,
          }}
        >
          <div
            className={`px-5 py-2.5 sm:px-8 sm:py-3 rounded-[4px] border shadow-xl inline-flex flex-col items-center justify-center max-w-full ${plaqueTheme}`}
          >
            <p className="font-typewriter font-bold text-base sm:text-xl tracking-wide leading-tight">
              {PLANQUETTE_TITLE}
            </p>

            {/* Explicit blank line space */}
            <div className="h-3 sm:h-4" aria-hidden="true" />

            <p className="animate-warning-flash font-vera font-bold text-xs sm:text-sm tracking-[0.14em] uppercase">
              {PLANQUETTE_WARNING}
            </p>

            <div className="mt-2.5 flex items-center justify-center gap-2.5 w-full max-w-lg pointer-events-auto">
              <div
                aria-live="polite"
                data-planquette-name-display="true"
                className={`flex-1 min-w-[180px] sm:min-w-[260px] min-h-[34px] px-3 py-1 rounded-[2px] border font-typewriter font-bold text-base sm:text-lg tracking-widest flex items-center justify-center whitespace-pre overflow-x-auto ${
                  darkMode
                    ? 'bg-[#050308]/92 border-[#EBE0F5]/35 text-[#FAF6FF]'
                    : 'bg-[#FBF9F4]/96 border-[#16001E]/38 text-[#16001E]'
                }`}
              >
                {planquetteName ? (
                  <span>{planquetteName.replace(/ /g, '␣')}</span>
                ) : (
                  <span className="opacity-40 text-xs sm:text-sm tracking-normal font-vera">
                    Drag faceplate or either hand along arc
                  </span>
                )}
              </div>

              <button
                type="button"
                disabled={!planquetteName.trim()}
                onClick={() => {
                  if (planquetteName.trim()) {
                    onSubmitName(planquetteName);
                  }
                }}
                className={`px-4 py-1.5 rounded-[2px] border font-vera text-xs sm:text-sm font-bold whitespace-nowrap cursor-pointer disabled:opacity-35 disabled:cursor-not-allowed shadow-sm transition-colors ${primaryBtnTheme}`}
              >
                Claim
              </button>
            </div>
          </div>
        </div>
      </div>
    );
  }

  return (
    <div
      data-first-screen-overlay="true"
      data-intro-stage={stage}
      className="pointer-events-none absolute inset-0 z-10"
    >
      {/* Words appear in margin ~1 head length up from top of head, centered over faceplate (even on each side) */}
      <div
        className="absolute -translate-x-1/2 -translate-y-1/2 w-[min(94vw,760px)] px-4 flex flex-col items-center justify-center text-center transition-all duration-300"
        style={{
          left: `${Math.round(marginCenterX)}px`,
          top: `${Math.round(marginCenterY)}px`,
        }}
      >
        <div
          className={`px-8 py-5 sm:px-11 sm:py-7 rounded-[4px] border shadow-xl inline-flex flex-col items-center justify-center max-w-full min-w-[min(92vw,560px)] ${plaqueTheme}`}
        >
          <p
            aria-live="polite"
            className="font-typewriter font-bold text-lg sm:text-2xl md:text-[26px] tracking-wide leading-relaxed"
          >
            <span>{typed.displayed}</span>
            {!typed.isComplete && (
              <span aria-hidden="true" className="inline-block ml-1 opacity-80 animate-pulse">
                _
              </span>
            )}
          </p>

          {stage === 'prompt' && (
            <p className="mt-2.5 font-vera text-xs sm:text-sm uppercase tracking-wider opacity-80">
              {INTRO_DRAG_OR_HOVER_PROMPT}
            </p>
          )}
        </div>
      </div>
    </div>
  );
};
