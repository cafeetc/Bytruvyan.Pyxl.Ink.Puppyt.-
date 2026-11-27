import { ChangeEvent, useEffect, useRef, useState } from 'react';
import { Download, FileUp, FolderOpen } from 'lucide-react';
import { PoseTelemetrySnapshot } from '../utils/poseTelemetry';
import {
  addPoseTelemetrySample,
  createEmptyPoseUsageLog,
  parsePoseTelemetrySnapshot,
  parsePoseUsageLog,
} from '../utils/poseUsageLog';
import { downloadExportBlob, formatExportTimestamp } from '../utils/poseExport';

const MAX_SNAPSHOT_SIZE_BYTES = 1_000_000;

interface PoseUsageWritable {
  write(data: string): Promise<void>;
  close(): Promise<void>;
}

interface PoseUsageDocumentHandle {
  name: string;
  getFile(): Promise<File>;
  createWritable(): Promise<PoseUsageWritable>;
}

function isPoseUsageDocumentHandle(value: unknown): value is PoseUsageDocumentHandle {
  return (
    typeof value === 'object' &&
    value !== null &&
    typeof Reflect.get(value, 'name') === 'string' &&
    typeof Reflect.get(value, 'getFile') === 'function' &&
    typeof Reflect.get(value, 'createWritable') === 'function'
  );
}

function errorMessage(error: unknown): string {
  return error instanceof Error ? error.message : 'Unable to update the pose usage document.';
}

interface PoseUsageImportProps {
  snapshot: PoseTelemetrySnapshot;
}

export function PoseUsageImport({ snapshot }: PoseUsageImportProps) {
  const [documentHandle, setDocumentHandle] = useState<PoseUsageDocumentHandle | null>(null);
  const [poseLabel, setPoseLabel] = useState(snapshot.scene.name);
  const [status, setStatus] = useState('Choose the project JSON document to begin.');
  const [statusKind, setStatusKind] = useState<'idle' | 'success' | 'error'>('idle');
  const [isImporting, setIsImporting] = useState(false);
  const fileInputRef = useRef<HTMLInputElement | null>(null);

  useEffect(() => {
    setPoseLabel(snapshot.scene.name);
  }, [snapshot.scene.name]);

  const handleSaveSnapshot = () => {
    const blob = new Blob([`${JSON.stringify(snapshot, null, 2)}\n`], {
      type: 'application/json;charset=utf-8',
    });
    downloadExportBlob(blob, `bitruvius-pose-${formatExportTimestamp()}.json`);
    setStatus('Pose snapshot saved. Import that file to update the usage document.');
    setStatusKind('success');
  };

  const handleChooseDocument = async () => {
    const picker = Reflect.get(window, 'showSaveFilePicker');
    if (typeof picker !== 'function') {
      setStatus('This browser cannot write project files directly. Use a Chromium-based browser.');
      setStatusKind('error');
      return;
    }

    try {
      const selected: unknown = await Reflect.apply(picker, window, [
        {
          suggestedName: 'pose-updates.json',
          types: [
            {
              description: 'JSON document',
              accept: { 'application/json': ['.json'] },
            },
          ],
        },
      ]);
      if (!isPoseUsageDocumentHandle(selected)) {
        throw new Error('The selected project file cannot be written by this browser.');
      }

      setDocumentHandle(selected);
      setStatus(`Project document selected: ${selected.name}`);
      setStatusKind('success');
    } catch (error) {
      if (error instanceof DOMException && error.name === 'AbortError') return;
      setStatus(errorMessage(error));
      setStatusKind('error');
    }
  };

  const handleImportFile = async (event: ChangeEvent<HTMLInputElement>) => {
    const file = event.currentTarget.files?.[0];
    if (!file) return;
    if (!documentHandle) {
      setStatus('Choose the project JSON document before importing a snapshot.');
      setStatusKind('error');
      return;
    }
    if (file.size > MAX_SNAPSHOT_SIZE_BYTES) {
      setStatus('Snapshot file is larger than 1 MB; no data was written.');
      setStatusKind('error');
      return;
    }

    setIsImporting(true);
    try {
      const snapshotData = parsePoseTelemetrySnapshot(await file.text());
      const targetFile = await documentHandle.getFile();
      const existingLog = targetFile.size
        ? parsePoseUsageLog(await targetFile.text())
        : createEmptyPoseUsageLog();
      const updatedLog = addPoseTelemetrySample(existingLog, snapshotData, poseLabel);
      const writable = await documentHandle.createWritable();
      await writable.write(`${JSON.stringify(updatedLog, null, 2)}\n`);
      await writable.close();

      if (fileInputRef.current) fileInputRef.current.value = '';
      setStatus(
        `Added ${updatedLog.sampleCount} total sample${updatedLog.sampleCount === 1 ? '' : 's'} to ${documentHandle.name}.`
      );
      setStatusKind('success');
    } catch (error) {
      setStatus(errorMessage(error));
      setStatusKind('error');
    } finally {
      setIsImporting(false);
    }
  };

  return (
    <section className="mb-3 border-b border-[#60466F]/70 pb-3">
      <h2 className="font-typewriter font-bold text-sm">Manual pose update</h2>
      <p className="mt-1 text-[10px] leading-relaxed text-[#B8A9C9]">
        Save a snapshot, then ingest it into one project JSON file. Only pose counts and joint
        angle ranges are kept; raw snapshots are not stored.
      </p>

      <div className="mt-2 flex flex-wrap items-center gap-2">
        <button
          type="button"
          onClick={handleSaveSnapshot}
          className="inline-flex items-center gap-1 rounded-[2px] border border-[#34D399]/40 bg-[#112A22] px-2 py-1 text-[10px] font-bold text-[#6EE7B7] hover:bg-[#173D31] focus-visible:outline-2 focus-visible:outline-[#D8B4FE]"
        >
          <Download className="h-3 w-3" aria-hidden="true" />
          Save current JSON
        </button>
        <button
          type="button"
          onClick={handleChooseDocument}
          className="inline-flex items-center gap-1 rounded-[2px] border border-[#60466F] px-2 py-1 text-[10px] text-[#D9CBE8] hover:bg-[#2C143B] focus-visible:outline-2 focus-visible:outline-[#D8B4FE]"
        >
          <FolderOpen className="h-3 w-3" aria-hidden="true" />
          {documentHandle ? documentHandle.name : 'Choose project JSON'}
        </button>
      </div>

      <label className="mt-2 block text-[10px] font-bold text-[#D9CBE8]" htmlFor="pose-usage-label">
        Pose group
      </label>
      <input
        id="pose-usage-label"
        value={poseLabel}
        onChange={(event) => setPoseLabel(event.target.value)}
        maxLength={80}
        className="mt-1 w-full rounded-[2px] border border-[#60466F] bg-[#100817] px-2 py-1.5 text-[10px] text-white focus-visible:outline-2 focus-visible:outline-[#D8B4FE]"
      />

      <label className="mt-2 flex cursor-pointer items-center gap-1.5 rounded-[2px] border border-[#60466F] px-2 py-1.5 text-[10px] text-[#D9CBE8] hover:bg-[#2C143B]">
        <FileUp className="h-3 w-3" aria-hidden="true" />
        <span>{isImporting ? 'Ingesting snapshot…' : 'Ingest snapshot JSON'}</span>
        <input
          ref={fileInputRef}
          type="file"
          accept=".json,application/json"
          disabled={!documentHandle || isImporting}
          onChange={handleImportFile}
          className="sr-only"
        />
      </label>

      <p
        role="status"
        aria-live="polite"
        className={`mt-2 text-[10px] ${
          statusKind === 'error'
            ? 'text-[#FCA5A5]'
            : statusKind === 'success'
              ? 'text-[#6EE7B7]'
              : 'text-[#B8A9C9]'
        }`}
      >
        {status}
      </p>
    </section>
  );
}
