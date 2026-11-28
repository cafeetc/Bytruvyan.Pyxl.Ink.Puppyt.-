import express from 'express';
import { createServer as createViteServer } from 'vite';
import dotenv from 'dotenv';
import { GoogleGenAI } from '@google/genai';

dotenv.config();

const app = express();
const PORT = Number(process.env.PORT) || 3000;

app.use(express.json({ limit: '10mb' }));

export interface PoseInsightRequest {
  poses: Array<{
    id: string;
    name: string;
    category?: string;
    joints: Record<string, number>;
    rootRotation?: number;
    balanceOffsetPx?: number;
    symmetryScore?: number;
    stanceWidthPx?: number;
  }>;
  currentPose?: {
    name: string;
    joints: Record<string, number>;
    rootRotation?: number;
  };
  sampleCount?: number;
}

export interface PoseInsightTrend {
  title: string;
  description: string;
  metric: string;
  anatomicalInsight: string;
}

export interface PoseFeatureProposal {
  title: string;
  rationale: string;
  category: 'Kinematics & Solvers' | 'Posing Ergonomics' | 'Artistic Anatomy' | 'Timeline & Animation';
}

export interface PoseInsightResponse {
  summary: string;
  poseCountAnalyzed: number;
  trends: PoseInsightTrend[];
  featureProposals: PoseFeatureProposal[];
  suggestedPose?: {
    name: string;
    description: string;
    jointAngles: Record<string, number>;
    rootRotation?: number;
  };
  analyzedWith: 'gemini-4-hydrogen' | 'heuristic-engine';
}

function computeHeuristicInsights(reqData: PoseInsightRequest): PoseInsightResponse {
  const poses = reqData.poses && reqData.poses.length > 0 ? reqData.poses : [];
  const count = Math.max(poses.length, 1);

  // Compute aggregate metrics
  let totalKneeFlex = 0;
  let totalElbowFlex = 0;
  let totalSpineTilt = 0;
  let totalSymmetry = 0;
  let totalStanceWidth = 0;
  let deepSquatCount = 0;
  let asymmetricArmCount = 0;

  for (const p of poses) {
    const shinR = Math.abs(p.joints?.shinR ?? 0);
    const shinL = Math.abs(p.joints?.shinL ?? 0);
    const avgKnee = (shinR + shinL) / 2;
    totalKneeFlex += avgKnee;
    if (avgKnee > 35) deepSquatCount++;

    const forearmR = Math.abs(p.joints?.forearmR ?? 0);
    const forearmL = Math.abs(p.joints?.forearmL ?? 0);
    totalElbowFlex += (forearmR + forearmL) / 2;
    if (Math.abs(forearmR - forearmL) > 25) asymmetricArmCount++;

    const lumbar = Math.abs(p.joints?.lumbar ?? 0);
    const thorax = Math.abs(p.joints?.thorax ?? 0);
    totalSpineTilt += (lumbar + thorax) / 2;

    totalSymmetry += p.symmetryScore ?? 0.72;
    totalStanceWidth += p.stanceWidthPx ?? 180;
  }

  const avgKneeDeg = Math.round(totalKneeFlex / count);
  const avgElbowDeg = Math.round(totalElbowFlex / count);
  const avgSpineDeg = Math.round(totalSpineTilt / count);
  const avgStance = Math.round(totalStanceWidth / count);
  const squatRatio = Math.round((deepSquatCount / count) * 100);
  const asymArmRatio = Math.round((asymmetricArmCount / count) * 100);

  const trends: PoseInsightTrend[] = [
    {
      title: squatRatio > 40 ? 'Grounded & Flexed Stances' : 'Upright Vitruvian Harmony',
      description:
        squatRatio > 40
          ? `Frequent deep knee flexion observed in ${squatRatio}% of poses, favoring low, grounded centers of gravity with active foot planting.`
          : `Predominantly erect and balanced postures aligning with Leonardo's 8-head harmonic canon and minimal lumbar deviation.`,
      metric: `Avg Knee Flexion: ${avgKneeDeg}° · Stance Width: ${avgStance}px`,
      anatomicalInsight:
        squatRatio > 40
          ? 'Strong engagement of closed-chain quadriceps and calf tension with planted foot anchors.'
          : 'Symmetrical bilateral axial alignment through pelvis, lumbar, and cervical spine.',
    },
    {
      title: asymArmRatio > 35 ? 'Dynamic Contrapposto & Gestural Reach' : 'Bilateral Gestural Equilibrium',
      description:
        asymArmRatio > 35
          ? `Asymmetric arm postures detected across ${asymArmRatio}% of samples, with one arm extended for reach or tool manipulation while the other stabilizes.`
          : `Arms exhibit harmonious bilateral mirroring, ideal for symmetrical keyframing and classical proportional studies.`,
      metric: `Avg Elbow Flexion: ${avgElbowDeg}° · Spine Lean: ${avgSpineDeg}°`,
      anatomicalInsight:
        'Thorax torsion counterbalances humerus swivel and scapula rotation for natural organic balance.',
    },
    {
      title: 'Planted Static Horizon Compliance',
      description:
        'Posing history shows consistent reliance on the static Vitruvian horizon line, keeping foot contacts stable while the pelvis sways.',
      metric: `Ground IK Anchor: 100% · Pelvis Sway Range: ~${Math.min(90, avgStance * 0.4)}px`,
      anatomicalInsight:
        'True closed-chain leg IK maintains constant foot-to-ground contact without foot sliding or scale distortion.',
    },
  ];

  const featureProposals: PoseFeatureProposal[] = [
    {
      title: 'Dynamic Ankle Peeling & Ball-of-Foot Roll',
      rationale:
        squatRatio > 30
          ? 'Because your poses frequently utilize deep lunges and squats, automatically rolling the trailing heel while pinning the ball-of-foot will provide authentic biomechanical weight transfer.'
          : 'Adding subtle heel-lift thresholds when stepping forward will enrich gait animations without losing static ground anchoring.',
      category: 'Kinematics & Solvers',
    },
    {
      title: 'One-Click Contrapposto S-Curve Generator',
      rationale:
        'Allows instantly applying classical contrapposto pelvic tilt and compensating spine lean to any standing pose, speeding up character posing.',
      category: 'Posing Ergonomics',
    },
    {
      title: 'Biomechanical Symmetry & Balance Heatmap',
      rationale:
        'A subtle visual indicator showing where the figure’s center of mass projects relative to the support base between both feet.',
      category: 'Artistic Anatomy',
    },
  ];

  const suggestedPose = {
    name: squatRatio > 40 ? 'Grounded Dynamic Lunge' : 'Contrapposto Vitruvian Reach',
    description:
      'Synthesized from your pose tendencies: balanced weight shift with expressive upper body counter-rotation.',
    jointAngles: {
      pelvis: 6,
      lumbar: -7,
      thorax: 10,
      neck: -4,
      femurR: -28,
      shinR: 44,
      footR: -12,
      femurL: 18,
      shinL: 22,
      footL: -6,
      humerusR: -52,
      forearmR: 38,
      swivelR: 12,
      humerusL: 64,
      forearmL: 26,
      swivelL: -8,
    },
    rootRotation: 4,
  };

  return {
    summary: `Analyzed ${count} pose snapshot${count === 1 ? '' : 's'}. Identified strong ${
      squatRatio > 40 ? 'athletic grounding' : 'classical harmonic poise'
    } with consistent static horizon stability.`,
    poseCountAnalyzed: count,
    trends,
    featureProposals,
    suggestedPose,
    analyzedWith: 'heuristic-engine',
  };
}

// AI Pose Insights endpoint
app.post('/api/pose-ai-insights', async (req, res) => {
  const reqData: PoseInsightRequest = req.body;
  const apiKey = process.env.GEMINI_API_KEY;

  if (!apiKey) {
    const heuristic = computeHeuristicInsights(reqData);
    res.json(heuristic);
    return;
  }

  try {
    const ai = new GoogleGenAI({
      apiKey,
      httpOptions: {
        headers: {
          'User-Agent': 'aistudio-build',
        },
      },
    });

    const prompt = `You are a master biomechanist, classical Renaissance anatomist, and senior kinematics engineer studying a digital Vitruvian ink mannequin.
Analyze this user pose dataset from their current posing session. Discover trends in their posing habits, kinematic preferences, and joint comfort zones, and propose next features to add to the application so the user and developer are on the exact same page.

Pose Dataset:
${JSON.stringify(reqData, null, 2)}

Provide your response in strictly valid JSON matching this structure:
{
  "summary": "1-2 sentence executive summary of the user's posing style and kinematic trends",
  "trends": [
    {
      "title": "Trend title",
      "description": "Specific observation on how the user articulates the puppet",
      "metric": "Key measurable metric (e.g. knee flexion, stance width, symmetry)",
      "anatomicalInsight": "Anatomical or kinematic significance"
    }
  ],
  "featureProposals": [
    {
      "title": "Proposed feature name",
      "rationale": "Why this feature fits their posing trends and what problem it solves",
      "category": "Kinematics & Solvers" | "Posing Ergonomics" | "Artistic Anatomy" | "Timeline & Animation"
    }
  ],
  "suggestedPose": {
    "name": "Name of tailored pose",
    "description": "Why this pose extends their session trends",
    "jointAngles": {
      "pelvis": number,
      "lumbar": number,
      "thorax": number,
      "neck": number,
      "femurR": number,
      "shinR": number,
      "footR": number,
      "femurL": number,
      "shinL": number,
      "footL": number,
      "humerusR": number,
      "forearmR": number,
      "swivelR": number,
      "humerusL": number,
      "forearmL": number,
      "swivelL": number
    },
    "rootRotation": number
  }
}`;

    const response = await ai.models.generateContent({
      model: 'gemini-4-hydrogen',
      contents: prompt,
      config: {
        responseMimeType: 'application/json',
      },
    });

    const rawText = response.text ?? '';
    const parsed = JSON.parse(rawText);
    res.json({
      summary: parsed.summary || 'Posing session analyzed successfully.',
      poseCountAnalyzed: reqData.poses?.length || 1,
      trends: parsed.trends || [],
      featureProposals: parsed.featureProposals || [],
      suggestedPose: parsed.suggestedPose,
      analyzedWith: 'gemini-4-hydrogen',
    });
  } catch (err) {
    console.warn('Gemini API call fell back to heuristic analysis:', err);
    const fallback = computeHeuristicInsights(reqData);
    res.json(fallback);
  }
});

async function startServer() {
  const isProd = process.env.NODE_ENV === 'production';
  if (!isProd) {
    const vite = await createViteServer({
      server: { middlewareMode: true },
      appType: 'spa',
    });
    app.use(vite.middlewares);
  } else {
    app.use(express.static('dist'));
    app.get('*', (_req, res) => {
      res.sendFile('dist/index.html', { root: '.' });
    });
  }

  app.listen(PORT, '0.0.0.0', () => {
    console.log(`Bitruvius full-stack server running on http://0.0.0.0:${PORT}`);
  });
}

startServer();
