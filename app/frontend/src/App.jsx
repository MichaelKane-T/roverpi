import { useEffect, useMemo, useState } from 'react'

function ActionButton({ active = false, children, className = '', ...props }) {
  return (
    <button
      className={[
        'rounded-lg border px-3 py-2 text-[0.65rem] font-extrabold tracking-[0.16em] transition-colors',
        active
          ? 'border-sky-300/60 bg-sky-400/30 text-sky-100'
          : 'border-sky-400/40 bg-sky-400/10 text-sky-300 hover:bg-sky-400/20',
        className,
      ].join(' ')}
      {...props}
    >
      {children}
    </button>
  )
}

function PanelCard({ title, children }) {
  return (
    <section className="rounded-xl border border-cyan-300/20 bg-[rgba(8,16,24,0.9)] p-3 sm:p-4">
      <h2 className="mb-3 text-[0.6rem] uppercase tracking-[0.25em] text-cyan-300/80 sm:text-[0.65rem]">
        {title}
      </h2>
      {children}
    </section>
  )
}

function TelemetryRow({ label, value, valueClass = '' }) {
  return (
    <div className="flex items-center justify-between rounded-md bg-white/5 px-2 py-1.5">
      <span className="text-slate-400">{label}</span>
      <span className={valueClass}>{value}</span>
    </div>
  )
}

export default function App() {
  const [online, setOnline] = useState(false)
  const [pan, setPan] = useState(90)
  const [tilt, setTilt] = useState(90)
  const [tick, setTick] = useState(Date.now())
  const [logs, setLogs] = useState([])
  const [telemetry, setTelemetry] = useState({
    distance: '—',
    path: '—',
    pathOk: false,
    mode: '—',
    ready: '—',
    steps: '—',
    epsilon: '—',
    esp: '—',
  })

  const videoSrc = '/video'
  const mapSrc = useMemo(() => `/map?t=${tick}`, [tick])

  const addLog = (message, ok = true) => {
    const entry = {
      id: `${Date.now()}-${Math.random().toString(36).slice(2, 8)}`,
      time: new Date().toLocaleTimeString(),
      message,
      ok,
    }
    setLogs((prev) => [entry, ...prev].slice(0, 60))
  }

  const safeFetchJson = async (url, options) => {
    const response = await fetch(url, options)
    if (!response.ok) throw new Error(`Request failed: ${response.status}`)
    return response.json()
  }

  const cmd = async (command) => {
    try {
      const data = await safeFetchJson(`/cmd/${command}`)
      addLog(`CMD ${command.toUpperCase()} → ${data.esp32 || 'sent'}`)
    } catch {
      addLog(`CMD ${command.toUpperCase()} failed`, false)
    }
  }

  const setMode = async (mode) => {
    try {
      await fetch(mode === 'AUTO' ? '/auto/start' : '/auto/stop')
      setTelemetry((prev) => ({ ...prev, mode }))
      addLog(`Mode → ${mode}`)
    } catch {
      addLog(`Mode change failed → ${mode}`, false)
    }
  }

  const updateCamera = async (axis, value) => {
    try {
      await fetch(`/cam/${axis}/${value}`)
    } catch {
      addLog(`${axis.toUpperCase()} update failed`, false)
    }
  }

  const centerCam = async () => {
    try {
      await fetch('/cam/center')
      setPan(90)
      setTilt(90)
      addLog('Camera centered')
    } catch {
      addLog('Camera center failed', false)
    }
  }

  useEffect(() => {
    let timer

    const poll = async () => {
      try {
        const [status, esp] = await Promise.all([
          safeFetchJson('/status'),
          safeFetchJson('/esp32_status'),
        ])

        setOnline(true)
        setTelemetry({
          distance: esp.dist >= 0 ? `${Number(esp.dist).toFixed(1)}cm` : '—',
          path: esp.path === 1 ? 'CLEAR' : 'BLOCKED',
          pathOk: esp.path === 1,
          mode: status.mode,
          ready: status.system_ready ? 'YES' : 'NO',
          steps: status.agent_steps,
          epsilon: status.epsilon,
          esp: esp.raw || status.esp32_msg || '—',
        })
        setTick(Date.now())
      } catch {
        setOnline(false)
      }
    }

    addLog('React dashboard loaded')
    poll()
    timer = window.setInterval(poll, 800)

    return () => window.clearInterval(timer)
  }, [])

  return (
    <div className="min-h-screen bg-[#05070b] px-4 py-4 font-mono text-slate-200 antialiased sm:px-6">
      <header className="mx-auto mb-4 flex max-w-6xl items-center gap-2 sm:mb-6">
        <span
          className={[
            'inline-block h-2.5 w-2.5 rounded-full transition-all',
            online
              ? 'bg-emerald-400 shadow-[0_0_8px_rgba(52,211,153,0.85)]'
              : 'bg-rose-400',
          ].join(' ')}
        />
        <h1 className="text-xs tracking-[0.25em] text-cyan-300 sm:text-sm">
          ROVERPI LOCAL DASHBOARD
        </h1>
      </header>

      <main className="mx-auto max-w-6xl space-y-4 lg:space-y-6">
        <section className="rounded-xl border border-cyan-300/20 bg-[rgba(8,16,24,0.9)] p-3 sm:p-4">
          <h2 className="mb-3 text-[0.6rem] uppercase tracking-[0.25em] text-cyan-300/80 sm:text-[0.65rem]">
            Camera Feed
          </h2>
          <div className="flex w-full justify-center">
            <div className="w-full max-w-3xl overflow-hidden rounded-lg bg-black">
              <img
                src={videoSrc}
                alt="Rover camera feed"
                className="aspect-video h-auto w-full object-contain"
              />
            </div>
          </div>
        </section>

        <section className="grid grid-cols-1 items-start gap-4 md:grid-cols-2 xl:grid-cols-3 lg:gap-5">
          <div className="space-y-4">
            <PanelCard title="Mode">
              <div className="flex flex-wrap gap-2">
                <ActionButton active={telemetry.mode === 'AUTO'} onClick={() => setMode('AUTO')}>
                  AUTO
                </ActionButton>
                <ActionButton active={telemetry.mode === 'MANUAL'} onClick={() => setMode('MANUAL')}>
                  MANUAL
                </ActionButton>
              </div>
            </PanelCard>

            <PanelCard title="Drive">
              <div className="mt-2 grid grid-cols-3 gap-2 text-[0.65rem] font-extrabold tracking-[0.14em]">
                <div />
                <ActionButton onClick={() => cmd('forward')}>▲ FWD</ActionButton>
                <div />

                <ActionButton onClick={() => cmd('left')}>◀ LEFT</ActionButton>
                <button
                  className="rounded-lg border border-rose-400/60 bg-rose-400/10 px-2 py-2 text-rose-300 transition-colors hover:bg-rose-400/20"
                  onClick={() => cmd('stop')}
                >
                  ■ STOP
                </button>
                <ActionButton onClick={() => cmd('right')}>RIGHT ▶</ActionButton>

                <div />
                <ActionButton onClick={() => cmd('backward')}>▼ BACK</ActionButton>
                <div />
              </div>
            </PanelCard>
          </div>

          <div className="space-y-4">
            <PanelCard title="Gimbal">
              <div className="mb-1 flex items-center justify-between text-[0.65rem] text-slate-500">
                <span>Pan</span>
                <span className="text-slate-300">{pan}°</span>
              </div>
              <input
                type="range"
                min="0"
                max="180"
                value={pan}
                onChange={(e) => setPan(Number(e.target.value))}
                onMouseUp={() => updateCamera('pan', pan)}
                onTouchEnd={() => updateCamera('pan', pan)}
                className="range-base range-pan"
              />

              <div className="mb-1 mt-3 flex items-center justify-between text-[0.65rem] text-slate-500">
                <span>Tilt</span>
                <span className="text-slate-300">{tilt}°</span>
              </div>
              <input
                type="range"
                min="15"
                max="145"
                value={tilt}
                onChange={(e) => setTilt(Number(e.target.value))}
                onMouseUp={() => updateCamera('tilt', tilt)}
                onTouchEnd={() => updateCamera('tilt', tilt)}
                className="range-base range-tilt"
              />

              <div className="mt-3 flex flex-wrap gap-2">
                <ActionButton onClick={centerCam}>⊙ Center</ActionButton>
                <ActionButton onClick={() => cmd('scan')}>Scan</ActionButton>
              </div>
            </PanelCard>

            <PanelCard title="Telemetry">
              <div className="space-y-1.5 text-[0.7rem] [font-variant-numeric:tabular-nums_lining-nums]">
                <TelemetryRow label="Distance" value={telemetry.distance} valueClass="font-semibold text-cyan-300" />
                <TelemetryRow
                  label="Path"
                  value={telemetry.path}
                  valueClass={telemetry.pathOk ? 'font-semibold text-emerald-400' : 'font-semibold text-red-400'}
                />
                <TelemetryRow label="Mode" value={telemetry.mode} valueClass="font-semibold text-cyan-300" />
                <TelemetryRow label="System Ready" value={telemetry.ready} valueClass="font-semibold text-cyan-300" />
                <TelemetryRow label="Agent Steps" value={telemetry.steps} valueClass="font-semibold text-cyan-300" />
                <TelemetryRow label="Epsilon" value={telemetry.epsilon} valueClass="font-semibold text-cyan-300" />
                <TelemetryRow
                  label="ESP32 raw"
                  value={telemetry.esp}
                  valueClass="max-w-[10rem] truncate text-[0.6rem] font-semibold text-cyan-300 md:max-w-[12rem]"
                />
              </div>
            </PanelCard>
          </div>

          <PanelCard title="Occupancy Map">
            <div className="overflow-hidden rounded-lg bg-black">
              <img
                src={mapSrc}
                alt="Occupancy map"
                className="h-auto w-full object-contain"
                style={{ imageRendering: 'pixelated' }}
              />
            </div>
          </PanelCard>
        </section>

        <PanelCard title="Event Log">
          <div className="h-40 space-y-0.5 overflow-y-auto rounded-lg bg-black/40 p-2 text-[0.65rem] sm:h-48">
            {logs.map((entry) => (
              <div key={entry.id} className="flex gap-2 border-b border-slate-400/20 py-[2px] text-slate-400">
                <span className={entry.ok ? 'text-emerald-400' : 'text-red-400'}>
                  {entry.time}
                </span>
                <span>{entry.message}</span>
              </div>
            ))}
          </div>
        </PanelCard>
      </main>
    </div>
  )
}