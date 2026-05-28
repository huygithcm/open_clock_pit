// Gauge Dashboard Application - DEBUG VERSION
const POLL_MS = 250;

const $ = (id) => document.getElementById(id);

// Gauge instances
let gauges = {};
let attitudeCanvas = null;

function fmt(v, digits = 2) {
  if (v === null || v === undefined) return "--";
  if (typeof v === "number") return v.toFixed(digits);
  return String(v);
}

function updateLink(ok) {
  const b = $("link-status");
  if (ok) {
    b.classList.remove("offline");
    b.classList.add("online");
    b.textContent = "LINK ONLINE";
  } else {
    b.classList.remove("online");
    b.classList.add("offline");
    b.textContent = "LINK OFFLINE";
  }
}

function updateClock() {
  const d = new Date();
  const el = $("clock");
  if (el) el.textContent = d.toTimeString().slice(0, 8);
}

// Create a gauge
function createGauge(elementId, minValue, maxValue, title) {
  const element = $(elementId);
  if (!element) {
    console.warn(`Gauge element not found: ${elementId}`);
    return null;
  }

  const opts = {
    angle: 0.15,
    lineWidth: 0.44,
    radiusScale: 1,
    pointer: {
      length: 0.75,
      strokeWidth: 0.035,
      color: '#00ff88'
    },
    limitMax: false,
    limitMin: false,
    colorStart: '#00ff88',
    colorStop: '#ffaa00',
    strokeColor: '#2d3a2d',
    generateGradient: true,
    highDpiSupport: true,
    renderTicks: {
      divisions: 5,
      divWidth: 1.1,
      divLength: 0.8,
      divColor: '#909090',
      subDivisions: 3,
      subLength: 0.5,
      subWidth: 0.6,
      subColor: '#606060'
    },
    staticZones: [
      { strokeStyle: '#00ff88', min: minValue, max: maxValue * 0.7 },
      { strokeStyle: '#ffaa00', min: maxValue * 0.7, max: maxValue * 0.85 },
      { strokeStyle: '#ff3333', min: maxValue * 0.85, max: maxValue }
    ]
  };

  try {
    const gauge = new Gauge(element);
    gauge.setOptions(opts);
    gauge.maxValue = maxValue;
    gauge.setMinValue(minValue);
    gauge.set(0);
    console.log(`✓ Gauge created: ${elementId}`);
    return gauge;
  } catch (e) {
    console.error(`Failed to create gauge ${elementId}:`, e);
    return null;
  }
}

function initGauges() {
  console.log("Initializing gauges...");
  gauges.altitude = createGauge('gauge-altitude', 0, 500);
  gauges.speed = createGauge('gauge-speed', 0, 50);
  gauges.vspeed = createGauge('gauge-vspeed', -10, 10);
  gauges.battery = createGauge('gauge-battery', 10, 17);
  gauges.current = createGauge('gauge-current', 0, 50);
  gauges.throttle = createGauge('gauge-throttle', 0, 100);
  gauges.rssi = createGauge('gauge-rssi', 0, 255);

  // Attitude indicator
  initAttitudeIndicator();
  console.log("✓ All gauges initialized");
}

function initAttitudeIndicator() {
  const canvas = $('attitude-canvas');
  if (!canvas) {
    console.warn("Attitude canvas not found");
    return;
  }

  attitudeCanvas = {
    canvas: canvas,
    ctx: canvas.getContext('2d'),
    pitch: 0,
    roll: 0,
    yaw: 0
  };

  // Set canvas size
  const rect = canvas.getBoundingClientRect();
  canvas.width = rect.width * 2;
  canvas.height = rect.height * 2;
  attitudeCanvas.ctx.scale(2, 2);
  console.log("✓ Attitude indicator initialized");
}

function drawAttitudeIndicator(pitch, roll, yaw) {
  if (!attitudeCanvas) return;

  const ctx = attitudeCanvas.ctx;
  const w = attitudeCanvas.canvas.width / 2;
  const h = attitudeCanvas.canvas.height / 2;

  // Clear
  ctx.fillStyle = '#0a0e0a';
  ctx.fillRect(0, 0, w, h);

  // Save state
  ctx.save();

  // Move to center
  ctx.translate(w / 2, h / 2);

  // Rotate for roll
  ctx.rotate((roll * Math.PI) / 180);

  // Sky (blue)
  ctx.fillStyle = '#001a4d';
  ctx.fillRect(-w / 2, -h * 0.6, w, h * 0.6);

  // Ground (brown)
  ctx.fillStyle = '#4d3300';
  ctx.fillRect(-w / 2, 0, w, h * 0.6);

  // Pitch lines
  const pitchStep = h / 10;
  ctx.strokeStyle = '#909090';
  ctx.lineWidth = 1;
  ctx.font = '10px monospace';
  ctx.fillStyle = '#909090';
  ctx.textAlign = 'center';

  for (let i = -5; i <= 5; i++) {
    const y = (pitch * pitchStep) / 10 + i * pitchStep;
    if (i % 2 === 0) {
      ctx.fillText(i * 10, 0, y);
      ctx.beginPath();
      ctx.moveTo(-w / 4, y);
      ctx.lineTo(w / 4, y);
      ctx.stroke();
    } else {
      ctx.beginPath();
      ctx.moveTo(-w / 8, y);
      ctx.lineTo(w / 8, y);
      ctx.stroke();
    }
  }

  // Center crosshair
  ctx.strokeStyle = '#00ff88';
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(-20, 0);
  ctx.lineTo(20, 0);
  ctx.moveTo(0, -20);
  ctx.lineTo(0, 20);
  ctx.stroke();

  // Restore state
  ctx.restore();

  // Yaw indicator (top)
  ctx.fillStyle = '#2d3a2d';
  ctx.fillRect(0, 0, w, 30);
  ctx.strokeStyle = '#00ff88';
  ctx.strokeRect(0, 0, w, 30);

  ctx.fillStyle = '#00ff88';
  ctx.font = 'bold 12px monospace';
  ctx.textAlign = 'center';
  ctx.fillText(`${Math.round(yaw)}°`, w / 2, 20);

  // Boundary
  ctx.strokeStyle = '#00ff88';
  ctx.lineWidth = 2;
  ctx.strokeRect(0, 0, w, h);
}

function safeUpdate(elementId, value) {
  const el = $(elementId);
  if (el) {
    el.textContent = value;
  }
}

async function pollTelemetry() {
  try {
    const r = await fetch('/api/telemetry');
    if (!r.ok) {
      updateLink(false);
      return;
    }

    const j = await r.json();
    if (!j.ok) {
      updateLink(false);
      console.warn("API returned ok=false");
      return;
    }

    const d = j.data || {};
    updateLink(true);

    console.log("Telemetry received:", { alt: d.alt, pitch: d.pitch, speed_3d: d.speed_3d });

    // Update gauges
    if (gauges.altitude) gauges.altitude.set(d.alt || 0);
    if (gauges.speed) gauges.speed.set(d.speed_3d || 0);
    if (gauges.vspeed) gauges.vspeed.set(d.v_speed || 0);
    if (gauges.battery) gauges.battery.set(d.vbat || 12);
    if (gauges.current) gauges.current.set(d.current || 0);
    if (gauges.throttle) gauges.throttle.set(((d.throttle || 1000) - 1000) / 10);
    if (gauges.rssi) gauges.rssi.set(d.rssi || 0);

    // Update attitude
    drawAttitudeIndicator(d.pitch || 0, d.roll || 0, d.yaw || 0);

    // Update attitude values
    safeUpdate('attitude-pitch', fmt(d.pitch, 1) + '°');
    safeUpdate('attitude-roll', fmt(d.roll, 1) + '°');
    safeUpdate('attitude-yaw', fmt(d.yaw, 1) + '°');

    // Update gauge info displays
    safeUpdate('alt-value', fmt(d.alt, 0));
    safeUpdate('speed-value', fmt(d.speed_3d, 1));
    safeUpdate('vspeed-value', fmt(d.v_speed, 2));
    safeUpdate('battery-value', fmt(d.vbat, 2));
    safeUpdate('current-value', fmt(d.current, 1));
    safeUpdate('throttle-value', fmt(((d.throttle || 1000) - 1000) / 10, 0));
    safeUpdate('rssi-value', fmt(d.rssi, 0));

    // GPS info
    safeUpdate('gps-sats', d.sats ?? '0');
    safeUpdate('gps-course', (d.course ?? 0) + '°');
    safeUpdate('gps-lat', fmt(d.lat, 5));
    safeUpdate('gps-lon', fmt(d.lon, 5));
    safeUpdate('home-dist', (d.home_dist ?? 0) + ' m');
    safeUpdate('home-dir', (d.home_dir ?? 0) + '°');

    // Telemetry tab
    safeUpdate('tel-pitch', fmt(d.pitch, 1));
    safeUpdate('tel-roll', fmt(d.roll, 1));
    safeUpdate('tel-yaw', fmt(d.yaw, 1));
    safeUpdate('tel-alt', fmt(d.alt, 1));
    safeUpdate('tel-vspeed', fmt(d.v_speed, 2));
    safeUpdate('tel-speed3d', fmt(d.speed_3d, 1));
    safeUpdate('tel-speed', fmt(d.speed, 1));
    safeUpdate('tel-vbat', fmt(d.vbat, 2));
    safeUpdate('tel-current', fmt(d.current, 2));
    safeUpdate('tel-throttle', d.throttle ?? '--');
    safeUpdate('tel-rssi', d.rssi ?? '--');
    safeUpdate('tel-sats', d.sats ?? '--');
    safeUpdate('tel-course', d.course ?? '--');
    safeUpdate('tel-lat', fmt(d.lat, 5));
    safeUpdate('tel-lon', fmt(d.lon, 5));
    safeUpdate('tel-homedist', d.home_dist ?? '--');
    safeUpdate('tel-homedir', d.home_dir ?? '--');
  } catch (e) {
    updateLink(false);
    console.error("Poll error:", e);
  }
}

async function loadConfig() {
  try {
    const stateR = await fetch('/api/state');
    const state = await stateR.json();
    const slots = Object.keys(state.selected).sort();
    const modules = ['', ...(state.available_modules || [])];

    const wrap = $('config-rows');
    if (!wrap) return;

    wrap.innerHTML = '';
    slots.forEach((slot) => {
      const row = document.createElement('div');
      row.className = 'form-row';
      const lbl = document.createElement('label');
      lbl.textContent = slot.toUpperCase();
      const sel = document.createElement('select');
      sel.dataset.slot = slot;
      modules.forEach((m) => {
        const opt = document.createElement('option');
        opt.value = m;
        opt.textContent = m || '— none —';
        if (state.selected[slot] === m) opt.selected = true;
        sel.appendChild(opt);
      });
      row.appendChild(lbl);
      row.appendChild(sel);
      wrap.appendChild(row);
    });
  } catch (e) {
    console.error('Config load failed', e);
  }
}

async function saveConfig() {
  const sels = document.querySelectorAll('#config-rows select');
  const displays = {};
  sels.forEach((s) => {
    if (s.value) displays[s.dataset.slot] = s.value;
  });
  const status = $('apply-status');
  if (!status) return;

  status.textContent = 'SAVING...';
  try {
    const r = await fetch('/api/config', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ displays }),
    });
    const j = await r.json();
    if (j.ok) {
      status.textContent = 'SAVED ✓ RESTARTING...';
      setTimeout(() => location.reload(), 1000);
    } else {
      status.textContent = 'ERROR: ' + (j.error || '?');
    }
  } catch (e) {
    status.textContent = 'NETWORK ERROR';
  }
}

async function loadPorts() {
  const sel = $('fc-port');
  if (!sel) return;

  sel.innerHTML = '';
  try {
    const r = await fetch('/api/ports');
    const j = await r.json();
    const ports = j.ports || [];
    ports.forEach((p) => {
      const o = document.createElement('option');
      o.value = p.path;
      o.textContent = `${p.label}`;
      sel.appendChild(o);
    });
  } catch (e) {
    console.error('Port scan failed', e);
  }
}

// Tab switching
function setupTabs() {
  document.querySelectorAll('.tab').forEach((btn) => {
    btn.addEventListener('click', () => {
      const tab = btn.dataset.tab;
      document.querySelectorAll('.tab').forEach((b) => b.classList.remove('active'));
      document.querySelectorAll('.tab-panel').forEach((p) => p.classList.remove('active'));
      btn.classList.add('active');
      const panel = document.querySelector(`[data-panel="${tab}"]`);
      if (panel) panel.classList.add('active');
    });
  });
}

function setupButtons() {
  $('btn-rescan')?.addEventListener('click', loadPorts);
  $('btn-apply')?.addEventListener('click', saveConfig);
  $('btn-fc-test')?.addEventListener('click', async () => {
    const port = $('fc-port')?.value;
    const baud = parseInt($('fc-baud')?.value || '115200', 10);
    try {
      const r = await fetch('/api/port/test', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ port, baudrate: baud }),
      });
      const j = await r.json();
      alert(j.output || 'Test completed');
    } catch (e) {
      alert('Error: ' + e.message);
    }
  });
}

// Init
window.addEventListener('DOMContentLoaded', () => {
  console.log("=== OpenCockpit Gauge Dashboard Loading ===");
  setupTabs();
  setupButtons();
  initGauges();
  loadConfig();
  loadPorts();

  setInterval(updateClock, 1000);
  setInterval(pollTelemetry, POLL_MS);

  updateClock();
  pollTelemetry();

  console.log("=== Dashboard Ready ===");
});
