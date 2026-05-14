import { useEffect, useMemo, useState } from 'react'

function ActionButton({ active = false, danger = false, children, className = '', ...props }) {
  return (
    <button
      className={[
        'rounded-lg border px-3 py-2 text-[0.65rem] font-extrabold tracking-[0.16em] transition-colors disabled:cursor-not-allowed disabled:opacity-40',
        danger
          ? 'border-rose-400/60 bg-rose-400/10 text-rose-300 hover:bg-rose-400/20'
          : active
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
    <section className="rounded-xl border border-cyan-300/20 bg-[rgba(8,16,24,0.9)] p-3 shadow-lg shadow-cyan-950/20 sm:p-4">
      <h2 className="mb-3 text-[0.6rem] uppercase tracking-[0.25em] text-cyan-300/80 sm:text-[0.65rem]">
        {title}
      </h2>
      {children}
    </section>
  )
}

function TelemetryRow({ label, value, valueClass = '' }) {
  return (
    <div className="flex items-center justify-between gap-3 rounded-md bg-white/5 px-2 py-1.5">
      <span className="text-slate-400">{label}</span>
      <span className={['text-right font-semibold text-cyan-300', valueClass].join(' ')}>
        {value}
      </span>
    </div>
  )
}

function SensorPill({ label, active }) {
  return (
    <div
      className={[
        'rounded-lg border px-2 py-3 text-center text-[0.65rem] font-black tracking-[0.14em]',
        active
          ? 'border-red-400/60 bg-red-500/25 text-red-200 shadow-[0_0_14px_rgba(248,113,113,0.25)]'
          : 'border-emerald-400/30 bg-emerald-500/10 text-emerald-300',
      ].join(' ')}
    >
      <div>{label}</div>
      <div className="mt-1 text-[0.55rem] opacity-70">{active ? 'BLOCKED' : 'CLEAR'}</div>
    </div>
  )
}

function Bar({ label, value, suffix = '%', danger = false }) {
  const pct = Math.max(0, Math.min(100, Number(value) || 0))

  return (
    <div>
      <div className="mb-1 flex justify-between text-[0.65rem] text-slate-400">
        <span>{label}</span>
        <span className={danger ? 'text-rose-300' : 'text-cyan-300'}>
          {Number(value || 0).toFixed(1)}
          {suffix}
        </span>
      </div>
      <div className="h-2 overflow-hidden rounded-full bg-white/10">
        <div
          className={['h-full rounded-full', danger ? 'bg-rose-400' : 'bg-cyan-400'].join(' ')}
          style={{ width: `${pct}%` }}
        />
      </div>
    </div>
  )
}

function Compass({ yaw }) {
  const angle = Number(yaw) || 0

  return (
    <div className="flex flex-col items-center justify-center py-2">
      <div className="relative h-28 w-28 rounded-full border border-cyan-400/40 bg-black/30">
        <div className="absolute left-1/2 top-1 text-[0.55rem] text-slate-400">N</div>
        <div className="absolute bottom-1 left-1/2 text-[0.55rem] text-slate-400">S</div>
        <div className="absolute left-1 top-1/2 text-[0.55rem] text-slate-400">W</div>
        <div className="absolute right-1 top-1/2 text-[0.55rem] text-slate-400">E</div>

        <div
          className="absolute inset-0 transition-transform"
          style={{ transform: `rotate(${angle}deg)` }}
        >
          <div className="absolute left-1/2 top-3 h-10 w-1 -translate-x-1/2 rounded-full bg-cyan-300 shadow-[0_0_12px_rgba(103,232,249,0.8)]" />
        </div>

        <div className="absolute left-1/2 top-1/2 h-3 w-3 -translate-x-1/2 -translate-y-1/2 rounded-full bg-cyan-300" />
      </div>

      <div className="mt-2 text-xs font-bold text-cyan-300">{angle.toFixed(1)}°</div>
    </div>
  )
}

function actionName(action) {
  const map = {
    0: 'FORWARD',
    1: 'LEFT',
    2: 'RIGHT',
    3: 'BACKWARD',
    4: 'STOP',
  }

  return map[action] ?? action ?? '—'
}

export default function App() {
  const [online, setOnline] = useState(false)
  const [pan, setPan] = useState(90)
  const [tilt, setTilt] = useState(90)
  const [mapTick, setMapTick] = useState(Date.now())
  const [logs, setLogs] = useState([])

  const [status, setStatus] = useState({})
  const [esp, setEsp] = useState({})
  const [battery, setBattery] = useState({})
  const [gyro, setGyro] = useState({})

  const videoSrc = '/video'
  const mapSrc = useMemo(() => `/map?t=${mapTick}`, [mapTick])

  const addLog = (message, ok = true) => {
    const entry = {
      id: `${Date.now()}-${Math.random().toString(36).slice(2, 8)}`,
      time: new Date().toLocaleTimeString(),
      message,
      ok,
    }

    setLogs((prev) => [entry, ...prev].slice(0, 80))
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
      setStatus((prev) => ({ ...prev, mode }))
      addLog(`Mode → ${mode}`)
    } catch {
      addLog(`Mode change failed → ${mode}`, false)
    }
  }

  const updateCamera = async (axis, value) => {
    try {
      await fetch(`/cam/${axis}/${value}`)
      addLog(`Camera ${axis} → ${value}°`)
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
    const poll = async () => {
      try {
        const [statusData, espData, batteryData, gyroData] = await Promise.all([
          safeFetchJson('/status'),
          safeFetchJson('/esp32_status'),
          safeFetchJson('/battery'),
          safeFetchJson('/gyro'),
        ])

        setOnline(true)
        setStatus(statusData)
        setEsp(espData)
        setBattery(batteryData)
        setGyro(gyroData)

        if (typeof statusData.pan_angle === 'number') setPan(statusData.pan_angle)
        if (typeof statusData.tilt_angle === 'number') setTilt(statusData.tilt_angle)
      } catch {
        setOnline(false)
      }
    }

    addLog('Rover dashboard loaded')
    poll()

    const telemetryTimer = window.setInterval(poll, 600)
    const mapTimer = window.setInterval(() => setMapTick(Date.now()), 2500)

    return () => {
      window.clearInterval(telemetryTimer)
      window.clearInterval(mapTimer)
    }
  }, [])

  const pathOk = Number(esp.path) === 1
  const battPct = Number(battery.percent ?? esp.batt_pct ?? 0)
  const battVoltage = Number(battery.voltage ?? esp.batt ?? 0)
  const battLow = Number(battery.state ?? esp.batt_state ?? 0) >= 1 || battPct < 25
  const yaw = Number(gyro.yaw ?? esp.yaw ?? 0)
  const gz = Number(gyro.gz ?? esp.gz ?? 0)

  const irFront = Number(esp.ir_front || 0)
  const irLeft = Number(esp.ir_left || 0)
  const irRight = Number(esp.ir_right || 0)
  const irFrontUp = Number(esp.ir_front_up || 0)

  const scan = status.scan || {}

  return (
    <div className="min-h-screen bg-[#05070b] px-4 py-4 font-mono text-slate-200 antialiased sm:px-6">
      <header className="mx-auto mb-4 flex max-w-7xl items-center justify-between gap-3 sm:mb-6">
        <div className="flex items-center gap-2">
          <span
            className={[
              'inline-block h-2.5 w-2.5 rounded-full transition-all',
              online
                ? 'bg-emerald-400 shadow-[0_0_8px_rgba(52,211,153,0.85)]'
                : 'bg-rose-400',
            ].join(' ')}
          />
          <h1 className="text-xs tracking-[0.25em] text-cyan-300 sm:text-sm">
            ROVERPI CONTROL STATION
          </h1>
        </div>

        <div className="hidden text-[0.65rem] uppercase tracking-[0.2em] text-slate-500 sm:block">
          {online ? 'LINK ONLINE' : 'LINK OFFLINE'}
        </div>
      </header>

      <main className="mx-auto grid max-w-7xl grid-cols-1 gap-4 xl:grid-cols-[1.35fr_0.9fr]">
        <div className="space-y-4">
          <PanelCard title="Camera Feed">
            <div className="relative overflow-hidden rounded-lg bg-black">
              <img
                src={videoSrc}
                alt="Rover camera feed"
                className="aspect-video h-auto w-full object-contain"
              />

              <div className="absolute left-3 top-3 rounded bg-black/70 px-2 py-1 text-[0.6rem] text-cyan-300">
                MODE {status.mode || '—'}
              </div>

              <div className="absolute right-3 top-3 rounded bg-black/70 px-2 py-1 text-[0.6rem] text-cyan-300">
                DIST {esp.dist >= 0 ? `${Number(esp.dist).toFixed(1)}cm` : '—'}
              </div>

              {!pathOk && (
                <div className="absolute bottom-3 left-1/2 -translate-x-1/2 rounded border border-red-400/50 bg-red-950/80 px-3 py-1 text-[0.7rem] font-black tracking-[0.18em] text-red-200">
                  OBSTACLE / PATH BLOCKED
                </div>
              )}
            </div>
          </PanelCard>

          <section className="grid grid-cols-1 gap-4 lg:grid-cols-2">
            <PanelCard title="Drive Control">
              <div className="flex flex-wrap gap-2">
                <ActionButton active={status.mode === 'AUTO'} onClick={() => setMode('AUTO')}>
                  AUTO
                </ActionButton>
                <ActionButton active={status.mode === 'MANUAL'} onClick={() => setMode('MANUAL')}>
                  MANUAL
                </ActionButton>
              </div>

              <div className="mt-4 grid grid-cols-3 gap-2 text-[0.65rem] font-extrabold tracking-[0.14em]">
                <div />
                <ActionButton onClick={() => cmd('forward')}>▲ FWD</ActionButton>
                <div />

                <ActionButton onClick={() => cmd('left')}>◀ LEFT</ActionButton>
                <ActionButton danger onClick={() => cmd('stop')}>
                  ■ STOP
                </ActionButton>
                <ActionButton onClick={() => cmd('right')}>RIGHT ▶</ActionButton>

                <div />
                <ActionButton onClick={() => cmd('backward')}>▼ BACK</ActionButton>
                <div />
              </div>

              <div className="mt-3 flex flex-wrap gap-2">
                <ActionButton onClick={() => cmd('scan')}>SCAN</ActionButton>
                <ActionButton onClick={() => cmd('fault_clear')}>FAULT CLEAR</ActionButton>
              </div>
            </PanelCard>

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
                className="w-full"
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
                className="w-full"
              />

              <div className="mt-3 flex flex-wrap gap-2">
                <ActionButton onClick={centerCam}>⊙ CENTER</ActionButton>
                <ActionButton onClick={() => fetch('/cam/left')}>PAN LEFT</ActionButton>
                <ActionButton onClick={() => fetch('/cam/right')}>PAN RIGHT</ActionButton>
              </div>
            </PanelCard>
          </section>

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
        </div>

        <div className="space-y-4">
          <PanelCard title="System Telemetry">
            <div className="space-y-1.5 text-[0.7rem] [font-variant-numeric:tabular-nums_lining-nums]">
              <TelemetryRow
                label="Path"
                value={pathOk ? 'CLEAR' : 'BLOCKED'}
                valueClass={pathOk ? 'text-emerald-400' : 'text-red-400'}
              />
              <TelemetryRow
                label="Distance"
                value={esp.dist >= 0 ? `${Number(esp.dist).toFixed(1)}cm` : 'OPEN / TIMEOUT'}
              />
              <TelemetryRow label="Mode" value={status.mode || '—'} />
              <TelemetryRow label="System Ready" value={status.system_ready ? 'YES' : 'NO'} />
              <TelemetryRow label="Direction" value={actionName(esp.dir)} />
              <TelemetryRow label="Fault" value={esp.fault ? 'YES' : 'NO'} valueClass={esp.fault ? 'text-red-400' : 'text-emerald-400'} />
            </div>
          </PanelCard>

          <PanelCard title="Battery">
            <div className="space-y-3">
              <Bar label="Battery" value={battPct} danger={battLow} />
              <TelemetryRow
                label="Voltage"
                value={battVoltage ? `${battVoltage.toFixed(2)}V` : '—'}
                valueClass={battLow ? 'text-rose-300' : 'text-cyan-300'}
              />
              <TelemetryRow
                label="State"
                value={battery.state_label || (battLow ? 'LOW' : 'OK')}
                valueClass={battLow ? 'text-rose-300' : 'text-emerald-400'}
              />
            </div>
          </PanelCard>

          <PanelCard title="IMU / Gyro">
            <Compass yaw={yaw} />
            <div className="space-y-1.5 text-[0.7rem]">
              <TelemetryRow label="Yaw" value={`${yaw.toFixed(2)}°`} />
              <TelemetryRow label="GZ" value={`${gz.toFixed(2)} dps`} />
            </div>
          </PanelCard>

          <PanelCard title="IR Sensors">
            <div className="grid grid-cols-2 gap-2">
              <SensorPill label="FRONT" active={irFront} />
              <SensorPill label="FRONT-UP" active={irFrontUp} />
              <SensorPill label="LEFT" active={irLeft} />
              <SensorPill label="RIGHT" active={irRight} />
            </div>
          </PanelCard>

          <PanelCard title="Learning / Agent">
            <div className="space-y-1.5 text-[0.7rem]">
              <TelemetryRow label="Agent Steps" value={status.agent_steps ?? '—'} />
              <TelemetryRow label="Replay Buffer" value={status.buffer_size ?? '—'} />
              <TelemetryRow label="Training" value={status.training ? 'YES' : 'NO'} />
              <TelemetryRow label="Epsilon" value={status.epsilon ?? '—'} />
              <TelemetryRow label="Exploration" value={`${status.exploration ?? 0}%`} />
            </div>
          </PanelCard>

          <PanelCard title="Scan Readings">
            <div className="grid grid-cols-3 gap-2 text-center text-[0.65rem]">
              <div className="rounded-lg bg-white/5 p-2">
                <div className="text-slate-500">RIGHT 30°</div>
                <div className="text-cyan-300">{scan[30] ?? '—'}cm</div>
              </div>
              <div className="rounded-lg bg-white/5 p-2">
                <div className="text-slate-500">FRONT 90°</div>
                <div className="text-cyan-300">{scan[90] ?? '—'}cm</div>
              </div>
              <div className="rounded-lg bg-white/5 p-2">
                <div className="text-slate-500">LEFT 150°</div>
                <div className="text-cyan-300">{scan[150] ?? '—'}cm</div>
              </div>
            </div>
          </PanelCard>

          <PanelCard title="ESP32 Raw">
            <div className="max-h-24 overflow-y-auto rounded-lg bg-black/40 p-2 text-[0.6rem] text-cyan-300">
              {esp.raw || status.esp32_msg || '—'}
            </div>
          </PanelCard>

          <PanelCard title="Event Log">
            <div className="h-44 space-y-0.5 overflow-y-auto rounded-lg bg-black/40 p-2 text-[0.65rem]">
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
        </div>
      </main>
    </div>
  )
}