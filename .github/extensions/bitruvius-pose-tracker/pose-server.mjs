import { createHash, randomBytes, randomUUID, timingSafeEqual } from "node:crypto";
import { createServer } from "node:http";
import { mkdir, readFile, rename, unlink, writeFile } from "node:fs/promises";
import os from "node:os";
import path from "node:path";

const CANVAS_ID = "bitruvius-live-pose-tracker";
const FORMAT = "bitruvius-live-pose-tracker";
const VERSION = 1;
const EXPECTED_JOINT_COUNT = 43;
const MAX_BODY_BYTES = 1024 * 1024;
const DATA_DIRECTORY = path.join(
    process.env.COPILOT_HOME && process.env.COPILOT_HOME.length > 0
        ? process.env.COPILOT_HOME
        : path.join(os.homedir(), ".copilot"),
    "extension-data",
    CANVAS_ID,
);

function isRecord(value) {
    return value !== null && typeof value === "object" && !Array.isArray(value);
}

function validateWorkspaceId(value) {
    if (typeof value !== "string" || !/^[a-zA-Z0-9._-]{1,160}$/.test(value)) {
        throw new Error("workspaceId must contain 1-160 letters, numbers, dots, underscores, or hyphens.");
    }
    return value;
}

function storagePath(workspaceId) {
    const identity = createHash("sha256").update(workspaceId).digest("hex");
    return path.join(DATA_DIRECTORY, `${identity}.json`);
}

function validateSnapshot(snapshot, workspaceId) {
    if (
        !isRecord(snapshot) ||
        snapshot.format !== "bitruvius-live-pose" ||
        snapshot.version !== 1 ||
        snapshot.workspaceId !== workspaceId ||
        typeof snapshot.capturedAt !== "string" ||
        !isRecord(snapshot.scene) ||
        typeof snapshot.scene.id !== "string" ||
        typeof snapshot.scene.name !== "string" ||
        !Number.isFinite(snapshot.scene.timelineT) ||
        !Number.isFinite(snapshot.scene.durationSec) ||
        !Number.isFinite(snapshot.scene.playbackSpeed) ||
        !isRecord(snapshot.armature) ||
        !isRecord(snapshot.armature.joints) ||
        !isRecord(snapshot.joints) ||
        Object.keys(snapshot.joints).length !== EXPECTED_JOINT_COUNT ||
        Object.keys(snapshot.armature.joints).length !== EXPECTED_JOINT_COUNT ||
        !Number.isFinite(snapshot.armature.rootX) ||
        !Number.isFinite(snapshot.armature.rootY) ||
        !Number.isFinite(snapshot.armature.rootRotation)
    ) {
        throw new Error("Pose data does not match the supported 43-joint telemetry format.");
    }

    for (const [jointId, joint] of Object.entries(snapshot.joints)) {
        if (
            !isRecord(joint) ||
            !isRecord(joint.state) ||
            !Number.isFinite(joint.x) ||
            !Number.isFinite(joint.y) ||
            !Number.isFinite(joint.endX) ||
            !Number.isFinite(joint.endY) ||
            !Number.isFinite(joint.localAngle) ||
            !Number.isFinite(joint.worldAngle) ||
            !Number.isFinite(joint.effectiveLengthPx) ||
            !Number.isFinite(joint.effectiveWidthScale) ||
            !Array.isArray(joint.boneMatrix) ||
            joint.boneMatrix.length !== 6 ||
            !joint.boneMatrix.every(Number.isFinite) ||
            !Number.isFinite(joint.state.localAngle) ||
            !Number.isFinite(joint.state.modeERatio) ||
            !Number.isFinite(joint.state.scaleLength) ||
            !Number.isFinite(joint.state.scaleWidth) ||
            typeof joint.state.mode !== "string" ||
            typeof joint.state.physicsMode !== "string" ||
            !Object.hasOwn(snapshot.armature.joints, jointId)
        ) {
            throw new Error(`Pose data for joint "${jointId}" is incomplete or invalid.`);
        }
    }
    return snapshot;
}

async function ensureDataDirectory() {
    await mkdir(DATA_DIRECTORY, { recursive: true });
}

async function readStoredPose(workspaceId) {
    await ensureDataDirectory();
    const target = storagePath(workspaceId);
    let text;
    try {
        text = await readFile(target, "utf8");
    } catch (error) {
        if (error?.code === "ENOENT") return null;
        throw new Error(`Unable to read saved pose data: ${error.message}`);
    }

    let record;
    try {
        record = JSON.parse(text);
    } catch (error) {
        throw new Error(`Saved pose data is corrupt and was not changed: ${error.message}`);
    }
    if (
        !isRecord(record) ||
        record.format !== FORMAT ||
        record.version !== VERSION ||
        record.workspaceId !== workspaceId ||
        typeof record.updatedAt !== "string"
    ) {
        throw new Error("Saved pose data has an unsupported format or workspace identity.");
    }
    validateSnapshot(record.snapshot, workspaceId);
    return record;
}

async function writeStoredPose(workspaceId, snapshot) {
    await readStoredPose(workspaceId);
    await ensureDataDirectory();
    const target = storagePath(workspaceId);
    const temporary = path.join(DATA_DIRECTORY, `.${path.basename(target)}.${randomUUID()}.tmp`);
    const record = {
        format: FORMAT,
        version: VERSION,
        workspaceId,
        updatedAt: new Date().toISOString(),
        snapshot,
    };
    try {
        await writeFile(temporary, JSON.stringify(record), { encoding: "utf8", mode: 0o600, flag: "wx" });
        await rename(temporary, target);
    } finally {
        try {
            await unlink(temporary);
        } catch (error) {
            if (error?.code !== "ENOENT") throw error;
        }
    }
    return record;
}

function isAllowedOrigin(origin) {
    if (typeof origin !== "string") return false;
    try {
        const url = new URL(origin);
        const isLoopback = url.hostname === "localhost" || url.hostname === "127.0.0.1";
        const port = Number(url.port || (url.protocol === "https:" ? 443 : 80));
        return url.protocol === "http:" && isLoopback && port >= 3000 && port <= 3999;
    } catch {
        return false;
    }
}

function setCommonHeaders(response, origin) {
    response.setHeader("Cache-Control", "no-store");
    response.setHeader("X-Content-Type-Options", "nosniff");
    response.setHeader("Referrer-Policy", "no-referrer");
    if (isAllowedOrigin(origin)) {
        response.setHeader("Access-Control-Allow-Origin", origin);
        response.setHeader("Access-Control-Allow-Methods", "POST, OPTIONS");
        response.setHeader("Access-Control-Allow-Headers", "Authorization, Content-Type");
        response.setHeader("Access-Control-Allow-Private-Network", "true");
        response.setHeader("Vary", "Origin");
    }
}

function sendJson(response, status, value) {
    response.writeHead(status, { "Content-Type": "application/json; charset=utf-8" });
    response.end(JSON.stringify(value));
}

function readRequestBody(request) {
    return new Promise((resolve, reject) => {
        const chunks = [];
        let size = 0;
        let tooLarge = false;
        request.on("data", (chunk) => {
            size += chunk.length;
            if (size > MAX_BODY_BYTES) {
                tooLarge = true;
                return;
            }
            if (!tooLarge) chunks.push(chunk);
        });
        request.on("end", () => {
            if (tooLarge) {
                reject(new Error("Pose payload exceeds the 1 MiB request limit."));
                return;
            }
            resolve(Buffer.concat(chunks).toString("utf8"));
        });
        request.on("error", reject);
    });
}

function hasValidToken(request, token) {
    const supplied = request.headers.authorization;
    if (typeof supplied !== "string" || !supplied.startsWith("Bearer ")) return false;
    const expectedBuffer = Buffer.from(token);
    const suppliedBuffer = Buffer.from(supplied.slice(7));
    return (
        expectedBuffer.length === suppliedBuffer.length &&
        timingSafeEqual(expectedBuffer, suppliedBuffer)
    );
}

function htmlPage(workspaceId, token) {
    const safeWorkspaceId = workspaceId.replaceAll("&", "&amp;").replaceAll("<", "&lt;").replaceAll(">", "&gt;").replaceAll('"', "&quot;").replaceAll("'", "&#39;");
    const safeToken = token;
    return `<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <meta name="color-scheme" content="dark">
  <title>Bitruvius Live Pose Tracker</title>
  <style>
    :root{font:14px/1.5 ui-monospace,SFMono-Regular,Menlo,monospace;color:#f1e8f8;background:#16001e}
    *{box-sizing:border-box}body{margin:0;padding:24px;max-width:1400px;margin-inline:auto}
    h1{font-size:20px;margin:0 0 6px}p{color:#c7b6d6;margin:6px 0}
    .card{border:1px solid #553b63;background:#21102b;border-radius:6px;padding:14px;margin:16px 0}
    .row{display:flex;gap:8px;align-items:center;flex-wrap:wrap}.muted{color:#a994ba}
    label{display:block;color:#d4c3df;margin:8px 0 4px}input{font:inherit;color:#f1e8f8;background:#16001e;border:1px solid #654d72;border-radius:3px;padding:8px;min-width:260px;max-width:100%}
    button{font:inherit;color:#f1e8f8;background:#362044;border:1px solid #8a65a0;border-radius:3px;padding:8px 12px;cursor:pointer}
    button:hover{background:#482a59}button:focus-visible,input:focus-visible{outline:2px solid #d8b4fe;outline-offset:2px}
    [role=status]{min-height:1.5em;color:#6ee7b7}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(170px,1fr));gap:8px}
    .metric{border-left:2px solid #8b5fb1;padding-left:9px}.metric b{display:block;color:#fff}
    .table-wrap{overflow:auto;max-height:60vh;border:1px solid #493252;border-radius:4px}
    table{width:100%;border-collapse:collapse;white-space:nowrap}th,td{text-align:right;padding:7px 9px;border-bottom:1px solid #382442}
    th{position:sticky;top:0;background:#2a1535;color:#d8b4fe}th:first-child,td:first-child{text-align:left}
    tbody tr:hover{background:#2c1935}code{color:#e9d5ff;overflow-wrap:anywhere}
    @media(max-width:600px){body{padding:14px}.card{padding:10px}input{width:100%;min-width:0}}
  </style>
</head>
<body>
  <header><h1>Bitruvius Live Pose Tracker</h1><p>Live telemetry for all 43 rig joints. The app streams the current pose whenever it changes.</p></header>
  <section class="card" aria-labelledby="connect-heading">
    <h2 id="connect-heading">Connect the app</h2>
    <p>In Bitruvius, open <b>Pose Stream</b>, paste this endpoint and access key, then select Connect.</p>
    <label for="endpoint">Receiver endpoint</label>
    <div class="row"><input id="endpoint" readonly aria-label="Pose receiver endpoint"><button id="copy-endpoint" type="button">Copy endpoint</button></div>
    <label for="access-key">Access key</label>
    <div class="row"><input id="access-key" readonly aria-label="Pose stream access key"><button id="copy-key" type="button">Copy access key</button></div>
    <p id="copy-status" role="status" aria-live="polite"></p>
  </section>
  <section class="card" aria-label="Latest pose status">
    <p id="connection-status" role="status" aria-live="polite">Waiting for the app to connect…</p>
    <div id="scene-metrics" class="grid"></div>
    <div id="root-metrics" class="grid"></div>
  </section>
  <section class="card">
    <div class="row"><h2>All joints</h2><label for="joint-filter" class="muted">Filter</label><input id="joint-filter" type="search" placeholder="Joint name or mode" aria-label="Filter pose joints"></div>
    <p id="joint-count" class="muted">No pose received yet.</p>
    <div class="table-wrap">
      <table>
        <thead><tr><th scope="col">Joint</th><th scope="col">Mode</th><th scope="col">Local °</th><th scope="col">World °</th><th scope="col">Start X</th><th scope="col">Start Y</th><th scope="col">End X</th><th scope="col">End Y</th><th scope="col">Length</th><th scope="col">Scale L</th><th scope="col">Scale W</th><th scope="col">Physics</th><th scope="col">Strained</th></tr></thead>
        <tbody id="joints"></tbody>
      </table>
    </div>
  </section>
  <script>
    const accessToken = ${JSON.stringify(safeToken)};
    const endpoint = location.origin + "/api/pose";
    const endpointInput = document.querySelector("#endpoint");
    const tokenInput = document.querySelector("#access-key");
    endpointInput.value = endpoint;
    tokenInput.value = accessToken;
    const statusNode = document.querySelector("#connection-status");
    const copyStatus = document.querySelector("#copy-status");
    const jointBody = document.querySelector("#joints");
    const filterInput = document.querySelector("#joint-filter");
    let latest = null;
    let lastUpdated = null;
    const makeMetric = (label, value) => {
      const wrapper = document.createElement("div");
      wrapper.className = "metric";
      const strong = document.createElement("b");
      strong.textContent = value == null ? "—" : String(value);
      const caption = document.createElement("span");
      caption.textContent = label;
      wrapper.append(strong, caption);
      return wrapper;
    };
    async function copyField(input, label) {
      try {
        await navigator.clipboard.writeText(input.value);
        copyStatus.textContent = label + " copied.";
      } catch {
        input.focus();
        input.select();
        copyStatus.textContent = "Copy is unavailable; the value is selected for manual copying.";
      }
    }
    document.querySelector("#copy-endpoint").addEventListener("click", () => copyField(endpointInput, "Endpoint"));
    document.querySelector("#copy-key").addEventListener("click", () => copyField(tokenInput, "Access key"));
    function render() {
      const rows = Object.entries(latest.joints).filter(([id, joint]) => {
        const search = filterInput.value.trim().toLowerCase();
        return !search || (id + " " + (joint.mode || "") + " " + (joint.state.physicsMode || "")).toLowerCase().includes(search);
      });
      jointBody.replaceChildren();
      for (const [id, joint] of rows) {
        const row = document.createElement("tr");
        const cells = [
          id, joint.mode, joint.localAngle, joint.worldAngle, joint.x, joint.y, joint.endX, joint.endY,
          joint.effectiveLengthPx, joint.state.scaleLength, joint.state.scaleWidth, joint.state.physicsMode,
          joint.isStrained ? "Yes" : "No",
        ];
        for (const value of cells) {
          const cell = document.createElement("td");
          cell.textContent = typeof value === "number" ? value.toFixed(2) : String(value ?? "—");
          row.append(cell);
        }
        jointBody.append(row);
      }
      document.querySelector("#joint-count").textContent = rows.length + " of " + Object.keys(latest.joints).length + " joints";
      document.querySelector("#scene-metrics").replaceChildren(
        makeMetric("Scene", latest.scene.name),
        makeMetric("Timeline", (latest.scene.timelineT * 100).toFixed(1) + "%"),
        makeMetric("Captured", new Date(latest.capturedAt).toLocaleTimeString()),
        makeMetric("Workspace", latest.workspaceId),
      );
      document.querySelector("#root-metrics").replaceChildren(
        makeMetric("Root X", latest.armature.rootX.toFixed(2)),
        makeMetric("Root Y", latest.armature.rootY.toFixed(2)),
        makeMetric("Root rotation", latest.armature.rootRotation.toFixed(2) + "°"),
        makeMetric("Pose joints", Object.keys(latest.armature.joints).length),
      );
      statusNode.textContent = "Live pose received · " + new Date(lastUpdated).toLocaleTimeString();
    }
    filterInput.addEventListener("input", () => { if (latest) render(); });
    async function poll() {
      try {
        const response = await fetch("/api/pose", { cache: "no-store" });
        if (!response.ok) throw new Error("Unable to read pose state (" + response.status + ").");
        const result = await response.json();
        if (result.snapshot) {
          latest = result.snapshot;
          lastUpdated = result.updatedAt;
          render();
        } else {
          statusNode.textContent = "Waiting for the app to connect…";
        }
      } catch (error) {
        statusNode.textContent = "Pose feed unavailable: " + error.message;
      }
    }
    poll();
    setInterval(poll, 500);
  </script>
</body>
</html>`;
}

export async function createPoseServer(workspaceId) {
    validateWorkspaceId(workspaceId);
    await readStoredPose(workspaceId);
    const token = randomBytes(32).toString("hex");
    const server = createServer(async (request, response) => {
        const origin = request.headers.origin;
        setCommonHeaders(response, origin);
        let requestUrl;
        try {
            requestUrl = new URL(request.url || "/", "http://127.0.0.1");
        } catch {
            sendJson(response, 400, { error: "Malformed request URL." });
            return;
        }

        if (request.method === "OPTIONS" && requestUrl.pathname === "/api/pose") {
            if (!isAllowedOrigin(origin)) {
                sendJson(response, 403, { error: "Origin is not permitted." });
                return;
            }
            response.writeHead(204);
            response.end();
            return;
        }

        if (request.method === "GET" && requestUrl.pathname === "/") {
            response.writeHead(200, {
                "Content-Type": "text/html; charset=utf-8",
                "Content-Security-Policy": "default-src 'none'; script-src 'unsafe-inline'; style-src 'unsafe-inline'; connect-src 'self'; base-uri 'none'; form-action 'none'",
            });
            response.end(htmlPage(workspaceId, token));
            return;
        }

        if (request.method === "GET" && requestUrl.pathname === "/api/pose") {
            try {
                const record = await readStoredPose(workspaceId);
                sendJson(response, 200, { snapshot: record?.snapshot ?? null, updatedAt: record?.updatedAt ?? null });
            } catch (error) {
                sendJson(response, 500, { error: error.message });
            }
            return;
        }

        if (request.method === "POST" && requestUrl.pathname === "/api/pose") {
            if (!isAllowedOrigin(origin)) {
                sendJson(response, 403, { error: "Origin is not permitted." });
                return;
            }
            if (!hasValidToken(request, token)) {
                sendJson(response, 401, { error: "Invalid pose stream access key." });
                return;
            }
            try {
                const text = await readRequestBody(request);
                const body = JSON.parse(text);
                if (!isRecord(body) || body.workspaceId !== workspaceId) {
                    sendJson(response, 400, { error: "Pose payload workspace identity does not match." });
                    return;
                }
                const snapshot = validateSnapshot(body, workspaceId);
                let record;
                try {
                    record = await writeStoredPose(workspaceId, snapshot);
                } catch (error) {
                    sendJson(response, 500, { error: `Unable to save pose data: ${error.message}` });
                    return;
                }
                sendJson(response, 200, { ok: true, updatedAt: record.updatedAt });
            } catch (error) {
                const status = error instanceof SyntaxError ? 400 : error.message.includes("1 MiB") ? 413 : 400;
                sendJson(response, status, { error: error.message });
            }
            return;
        }

        sendJson(response, 404, { error: "Not found." });
    });

    await new Promise((resolve, reject) => {
        server.once("error", reject);
        server.listen(0, "127.0.0.1", resolve);
    });
    const address = server.address();
    if (!address || typeof address === "string") {
        await new Promise((resolve) => server.close(resolve));
        throw new Error("Pose tracker server did not receive a valid loopback address.");
    }
    return {
        server,
        token,
        workspaceId,
        url: `http://127.0.0.1:${address.port}/`,
        readSnapshot: async () => (await readStoredPose(workspaceId))?.snapshot ?? null,
        close: () => new Promise((resolve, reject) => {
            server.close((error) => error ? reject(error) : resolve());
        }),
    };
}
