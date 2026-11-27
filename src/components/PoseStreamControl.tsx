import { FormEvent, useEffect, useRef, useState } from 'react';
import { Activity } from 'lucide-react';
import { PoseTelemetrySnapshot } from '../utils/poseTelemetry';
import { PoseUsageImport } from './PoseUsageImport';

const STORAGE_KEY = 'bitruvius.poseStream.v1';
const MIN_SEND_INTERVAL_MS = 100;
const INITIAL_RETRY_DELAY_MS = 500;
const MAX_RETRY_DELAY_MS = 10_000;

interface StreamConfiguration {
  endpoint: string;
  accessKey: string;
}

interface StoredConfigurationResult {
  configuration: StreamConfiguration;
  error: string | null;
}

function isValidEndpoint(value: string): boolean {
  try {
    const endpoint = new URL(value);
    return (
      endpoint.protocol === 'http:' &&
      (endpoint.hostname === '127.0.0.1' || endpoint.hostname === 'localhost') &&
      Number(endpoint.port) >= 3000 &&
      Number(endpoint.port) <= 3999 &&
      endpoint.pathname === '/api/pose' &&
      !endpoint.username &&
      !endpoint.password &&
      !endpoint.search &&
      !endpoint.hash
    );
  } catch {
    return false;
  }
}

function readConfiguration(): StoredConfigurationResult {
  try {
    const raw = window.localStorage.getItem(STORAGE_KEY);
    if (!raw) return { configuration: { endpoint: '', accessKey: '' }, error: null };
    const value = JSON.parse(raw);
    if (
      value &&
      typeof value.endpoint === 'string' &&
      isValidEndpoint(value.endpoint) &&
      typeof value.accessKey === 'string' &&
      /^[a-f0-9]{64}$/.test(value.accessKey)
    ) {
      return { configuration: value, error: null };
    }
    return {
      configuration: { endpoint: '', accessKey: '' },
      error: 'Saved pose stream settings are invalid. Reconnect using the canvas values.',
    };
  } catch {
    return {
      configuration: { endpoint: '', accessKey: '' },
      error: 'Unable to read saved pose stream settings from this browser.',
    };
  }
}

interface PoseStreamControlProps {
  snapshot: PoseTelemetrySnapshot;
}

export function PoseStreamControl({ snapshot }: PoseStreamControlProps) {
  const initial = useRef<StoredConfigurationResult | null>(null);
  if (!initial.current) initial.current = readConfiguration();
  const [configuration, setConfiguration] = useState(initial.current.configuration);
  const [endpointDraft, setEndpointDraft] = useState(configuration.endpoint);
  const [keyDraft, setKeyDraft] = useState(configuration.accessKey);
  const [status, setStatus] = useState(initial.current.error ?? 'Not connected');
  const [statusKind, setStatusKind] = useState<'idle' | 'connected' | 'error'>(
    initial.current.error ? 'error' : 'idle'
  );
  const lastSentAt = useRef(0);
  const retryDelay = useRef(INITIAL_RETRY_DELAY_MS);
  const retryAt = useRef(0);
  const requestInFlight = useRef(false);

  useEffect(() => {
    if (!configuration.endpoint || !configuration.accessKey) return;
    let active = true;
    let timer = 0;

    const sendLatestPose = async () => {
      if (!active) return;
      if (requestInFlight.current) {
        timer = window.setTimeout(sendLatestPose, 50);
        return;
      }

      requestInFlight.current = true;
      try {
        const response = await fetch(configuration.endpoint, {
          method: 'POST',
          headers: {
            Authorization: `Bearer ${configuration.accessKey}`,
            'Content-Type': 'application/json',
          },
          body: JSON.stringify(snapshot),
        });
        if (!response.ok) {
          const result = await response.json().catch(() => null);
          throw new Error(
            typeof result?.error === 'string'
              ? result.error
              : `Pose stream returned HTTP ${response.status}.`
          );
        }
        lastSentAt.current = Date.now();
        retryDelay.current = INITIAL_RETRY_DELAY_MS;
        retryAt.current = 0;
        if (active) {
          setStatus(`Live · sent ${new Date().toLocaleTimeString()}`);
          setStatusKind('connected');
        }
      } catch (error) {
        if (active) {
          const message = error instanceof Error ? error.message : 'Pose stream request failed.';
          setStatus(`${message} Retrying…`);
          setStatusKind('error');
        }
        retryAt.current = Date.now() + retryDelay.current;
        retryDelay.current = Math.min(retryDelay.current * 2, MAX_RETRY_DELAY_MS);
      } finally {
        requestInFlight.current = false;
      }
    };

    const wait = Math.max(
      0,
      MIN_SEND_INTERVAL_MS - (Date.now() - lastSentAt.current),
      retryAt.current - Date.now()
    );
    timer = window.setTimeout(sendLatestPose, wait);
    return () => {
      active = false;
      window.clearTimeout(timer);
    };
  }, [configuration, snapshot]);

  const handleConnect = (event: FormEvent<HTMLFormElement>) => {
    event.preventDefault();
    const endpoint = endpointDraft.trim();
    const accessKey = keyDraft.trim();
    if (!isValidEndpoint(endpoint)) {
      setStatus('Use the canvas receiver URL on localhost, ending in /api/pose.');
      setStatusKind('error');
      return;
    }
    if (!/^[a-f0-9]{64}$/.test(accessKey)) {
      setStatus('The access key must be copied from the live pose canvas.');
      setStatusKind('error');
      return;
    }

    const next = { endpoint, accessKey };
    try {
      window.localStorage.setItem(STORAGE_KEY, JSON.stringify(next));
      setConfiguration(next);
      retryAt.current = 0;
      retryDelay.current = INITIAL_RETRY_DELAY_MS;
      setStatus('Connecting to the live pose canvas…');
      setStatusKind('idle');
    } catch {
      setStatus('Unable to save pose stream settings in this browser.');
      setStatusKind('error');
    }
  };

  const handleDisconnect = () => {
    try {
      window.localStorage.removeItem(STORAGE_KEY);
      setConfiguration({ endpoint: '', accessKey: '' });
      setEndpointDraft('');
      setKeyDraft('');
      setStatus('Not connected');
      setStatusKind('idle');
    } catch {
      setStatus('Unable to clear saved pose stream settings.');
      setStatusKind('error');
    }
  };

  return (
    <details className="relative shrink-0">
      <summary className="flex items-center gap-1.5 px-2 py-1 font-vera text-[10.5px] font-bold rounded-[2px] border border-[#34D399]/35 bg-[#112A22] text-[#6EE7B7] hover:bg-[#173D31] cursor-pointer list-none focus-visible:outline-2 focus-visible:outline-[#D8B4FE]">
        <Activity className="w-3.5 h-3.5" aria-hidden="true" />
        <span>Pose Data</span>
        <span
          className={`inline-block w-1.5 h-1.5 rounded-full ${
            statusKind === 'connected'
              ? 'bg-[#6EE7B7]'
              : statusKind === 'error'
                ? 'bg-[#FCA5A5]'
                : 'bg-[#A994BA]'
          }`}
          aria-hidden="true"
        />
      </summary>
      <section className="absolute right-0 top-full mt-2 z-50 w-[min(24rem,calc(100vw-1rem))] p-3 rounded border border-[#60466F] bg-[#1D1026] shadow-2xl text-[#EFEAF4]">
        <PoseUsageImport snapshot={snapshot} />
        <h2 className="font-typewriter font-bold text-sm">Optional live pose connection</h2>
        <p className="mt-1 text-[10px] leading-relaxed text-[#B8A9C9]">
          Copy the receiver endpoint and key from the Bitruvius Live Pose Tracker canvas.
          All 43 joints stream locally as the rig changes.
        </p>
        <form className="mt-3 space-y-2" onSubmit={handleConnect}>
          <label className="block text-[10px] font-bold text-[#D9CBE8]" htmlFor="pose-stream-endpoint">
            Receiver endpoint
          </label>
          <input
            id="pose-stream-endpoint"
            type="url"
            autoComplete="url"
            value={endpointDraft}
            onChange={(event) => setEndpointDraft(event.target.value)}
            placeholder="http://127.0.0.1:PORT/api/pose"
            className="w-full rounded-[2px] border border-[#60466F] bg-[#100817] px-2 py-1.5 text-[10px] text-white focus-visible:outline-2 focus-visible:outline-[#D8B4FE]"
          />
          <label className="block text-[10px] font-bold text-[#D9CBE8]" htmlFor="pose-stream-key">
            Access key
          </label>
          <input
            id="pose-stream-key"
            type="password"
            autoComplete="off"
            value={keyDraft}
            onChange={(event) => setKeyDraft(event.target.value)}
            placeholder="Paste the key displayed in the canvas"
            className="w-full rounded-[2px] border border-[#60466F] bg-[#100817] px-2 py-1.5 text-[10px] text-white focus-visible:outline-2 focus-visible:outline-[#D8B4FE]"
          />
          <div className="flex items-center gap-2">
            <button
              type="submit"
              className="rounded-[2px] border border-[#34D399]/40 bg-[#112A22] px-2 py-1 text-[10px] font-bold text-[#6EE7B7] hover:bg-[#173D31] focus-visible:outline-2 focus-visible:outline-[#D8B4FE]"
            >
              Connect
            </button>
            {configuration.endpoint && (
              <button
                type="button"
                onClick={handleDisconnect}
                className="rounded-[2px] border border-[#EBE0F5]/20 px-2 py-1 text-[10px] text-[#D9CBE8] hover:bg-[#2C143B] focus-visible:outline-2 focus-visible:outline-[#D8B4FE]"
              >
                Disconnect
              </button>
            )}
          </div>
        </form>
        <p
          role="status"
          aria-live="polite"
          className={`mt-2 text-[10px] ${
            statusKind === 'error' ? 'text-[#FCA5A5]' : 'text-[#6EE7B7]'
          }`}
        >
          {status}
        </p>
      </section>
    </details>
  );
}
