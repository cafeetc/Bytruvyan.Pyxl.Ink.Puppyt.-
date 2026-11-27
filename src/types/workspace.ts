export type ModelModuleId = 'rig_joint' | 'digits' | 'perspective' | 'scene';

export type BottomDrawerTab = 'timeline' | 'poses' | 'animation';

export interface ModularPanelLayoutState {
  isOpen: boolean;
  isCollapsed: boolean;
  dockMode: 'docked' | 'floating';
  position: { x: number; y: number };
  zIndex: number;
}

export interface WorkspaceLayoutConfig {
  railVisible: boolean;
  bottomDrawerOpen: boolean;
  bottomDrawerTab: BottomDrawerTab;
  panels: Record<ModelModuleId, ModularPanelLayoutState>;
}

export const WORKSPACE_LAYOUT_STORAGE_KEY = 'bitruvius.workspaceLayout.v2';

export const MODEL_MODULE_META: ReadonlyArray<{
  id: ModelModuleId;
  label: string;
  shortLabel: string;
}> = [
  { id: 'rig_joint', label: 'Rig & Joint', shortLabel: 'Rig' },
  { id: 'digits', label: 'Hands & Digits', shortLabel: 'Digits' },
  { id: 'perspective', label: 'Perspective', shortLabel: 'Persp' },
  { id: 'scene', label: 'Scene & Backdrop', shortLabel: 'Scene' },
];

export function createDefaultWorkspaceLayout(): WorkspaceLayoutConfig {
  return {
    railVisible: true,
    bottomDrawerOpen: false,
    bottomDrawerTab: 'timeline',
    panels: {
      rig_joint: {
        isOpen: false,
        isCollapsed: false,
        dockMode: 'docked',
        position: { x: 24, y: 24 },
        zIndex: 31,
      },
      digits: {
        isOpen: false,
        isCollapsed: false,
        dockMode: 'docked',
        position: { x: 48, y: 48 },
        zIndex: 32,
      },
      perspective: {
        isOpen: false,
        isCollapsed: false,
        dockMode: 'docked',
        position: { x: 72, y: 72 },
        zIndex: 33,
      },
      scene: {
        isOpen: false,
        isCollapsed: false,
        dockMode: 'docked',
        position: { x: 96, y: 96 },
        zIndex: 34,
      },
    },
  };
}
