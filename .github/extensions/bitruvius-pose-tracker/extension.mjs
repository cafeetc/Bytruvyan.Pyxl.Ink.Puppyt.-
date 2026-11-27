import { CanvasError, createCanvas, joinSession } from "@github/copilot-sdk/extension";
import declaration from "./extension.json" with { type: "json" };
import { createPoseServer } from "./pose-server.mjs";

const instances = new Map();

function validateWorkspaceId(value) {
    if (typeof value !== "string" || !/^[a-zA-Z0-9._-]{1,160}$/.test(value)) {
        throw new CanvasError("invalid_input", "workspaceId must be a non-empty workspace identifier.");
    }
    return value;
}

function toCanvasError(error) {
    if (error instanceof CanvasError) return error;
    return new CanvasError("pose_tracker_error", error instanceof Error ? error.message : String(error));
}

const actionHandlers = {
    get_latest_pose: async ({ instanceId }) => {
        const entry = instances.get(instanceId);
        if (!entry) {
            throw new CanvasError("instance_not_open", "Open the Bitruvius Live Pose Tracker canvas before requesting its pose.");
        }
        try {
            return await entry.readSnapshot();
        } catch (error) {
            throw toCanvasError(error);
        }
    },
};

await joinSession({
    canvases: declaration.canvases.map((canvas) => createCanvas({
        ...canvas,
        actions: canvas.actions?.map((action) => ({
            ...action,
            handler: actionHandlers[action.name],
        })),
        open: async ({ instanceId, input }) => {
            const workspaceId = validateWorkspaceId(input?.workspaceId);
            let entry = instances.get(instanceId);
            if (entry && entry.workspaceId !== workspaceId) {
                throw new CanvasError("instance_identity_mismatch", "This canvas instance is already bound to another workspace.");
            }
            try {
                if (!entry) {
                    entry = await createPoseServer(workspaceId);
                    instances.set(instanceId, entry);
                } else {
                    await entry.readSnapshot();
                }
                return { title: canvas.displayName, url: entry.url };
            } catch (error) {
                throw toCanvasError(error);
            }
        },
        onClose: async ({ instanceId }) => {
            const entry = instances.get(instanceId);
            if (!entry) return;
            instances.delete(instanceId);
            try {
                await entry.close();
            } catch (error) {
                throw toCanvasError(error);
            }
        },
    })),
});
