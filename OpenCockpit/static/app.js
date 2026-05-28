// OpenCockpit live UI with Charts
const POLL_MS = 250;
const MAX_DATA_POINTS = 240; // ~60 seconds at 250ms poll

const $ = (id) => document.getElementById(id);

// Data history for charts
let dataHistory = {
  time: [],
  altitude: [],
  vspeed: [],
  speed_3d: [],
  speed: [],
  pitch: [],
  roll: [],
  yaw: [],
  vbat: [],
  current: [],
  lat: [],
  lon: [],
  sats: [],
  rssi: []
};

// Chart instances
let charts = {};

function fmt(v, digits = 2) {
  if (v === null || v === undefined) return "--";
  if (typeof v === "number") return v.toFixed(digits);
  return String(v);
}

function setBar(el, value, min, max) {
  if (!el || value === null || value === undefined) return;
  const pct = Math.max(0, Math.min(100, ((value - min) / (max - min)) * 100));
  el.style.width = pct + "%";
}

function setBarSigned(el, value, range) {
  if (!el || value === null || value === undefined) return;
  const pct = Math.max(0, Math.min(100, 50 + (value / range) * 50));
  el.style.width = pct + "%";
}

let lastOk = 0;
function updateLink(ok) {
  const b = $("link-status");
  if (ok) {
    b.classList.remove("offline");
    b.classList.add("online");
    b.textContent = "LINK ONLINE";
    lastOk = Date.now();
  } else {
    b.classList.remove("online");
    b.classList.add("offline");
    b.textContent = "LINK OFFLINE";
  }
}

function updateClock() {
  const d = new Date();
  $("clock").textContent = d.toTimeString().slice(0, 8);
}

function addDataPoint(data) {
  // Keep history trimmed to MAX_DATA_POINTS
  if (dataHistory.time.length >= MAX_DATA_POINTS) {
    Object.keys(dataHistory).forEach(key => {
      dataHistory[key].shift();
    });
  }

  const now = new Date().toLocaleTimeString();
  dataHistory.time.push(now);
  dataHistory.altitude.push(data.alt);
  dataHistory.vspeed.push(data.v_speed);
  dataHistory.speed_3d.push(data.speed_3d);
  dataHistory.speed.push(data.speed);
  dataHistory.pitch.push(data.pitch);
  dataHistory.roll.push(data.roll);
  dataHistory.yaw.push(data.yaw);
  dataHistory.vbat.push(data.vbat);
  dataHistory.current.push(data.current);
  dataHistory.lat.push(data.lat);
  dataHistory.lon.push(data.lon);
  dataHistory.sats.push(data.sats);
  dataHistory.rssi.push(data.rssi);

  updateCharts();
  $("chart-info").textContent = `Data points: ${dataHistory.time.length}`;
}

function initCharts() {
  const colors = {
    primary: '#00ff88',
    secondary: '#ff6600',
    tertiary: '#00ccff',
    danger: '#ff3333'
  };

  // Altitude Chart
  const altCtx = $("chart-altitude");
  if (altCtx) {
    charts.altitude = new Chart(altCtx, {
      type: 'line',
      data: {
        labels: dataHistory.time,
        datasets: [
          {
            label: 'Altitude (m)',
            data: dataHistory.altitude,
            borderColor: colors.primary,
            backgroundColor: 'rgba(0, 255, 136, 0.1)',
            tension: 0.1
          },
          {
            label: 'V/S (m/s)',
            data: dataHistory.vspeed,
            borderColor: colors.secondary,
            backgroundColor: 'rgba(255, 102, 0, 0.1)',
            tension: 0.1,
            yAxisID: 'y1'
          }
        ]
      },
      options: {
        responsive: true,
        maintainAspectRatio: true,
        scales: {
          y: { title: { display: true, text: 'Altitude (m)' } },
          y1: { type: 'linear', position: 'right', title: { display: true, text: 'V/S (m/s)' } }
        },
        plugins: { legend: { display: true } }
      }
    });
  }

  // Speed Chart
  const spdCtx = $("chart-speed");
  if (spdCtx) {
    charts.speed = new Chart(spdCtx, {
      type: 'line',
      data: {
        labels: dataHistory.time,
        datasets: [
          { label: '3D Speed (m/s)', data: dataHistory.speed_3d, borderColor: colors.primary, backgroundColor: 'rgba(0, 255, 136, 0.1)', tension: 0.1 },
          { label: 'Horizontal (m/s)', data: dataHistory.speed, borderColor: colors.secondary, backgroundColor: 'rgba(255, 102, 0, 0.1)', tension: 0.1 }
        ]
      },
      options: {
        responsive: true,
        maintainAspectRatio: true,
        plugins: { legend: { display: true } }
      }
    });
  }

  // Attitude Chart
  const attCtx = $("chart-attitude");
  if (attCtx) {
    charts.attitude = new Chart(attCtx, {
      type: 'line',
      data: {
        labels: dataHistory.time,
        datasets: [
          { label: 'Pitch (°)', data: dataHistory.pitch, borderColor: colors.primary, backgroundColor: 'rgba(0, 255, 136, 0.1)', tension: 0.1 },
          { label: 'Roll (°)', data: dataHistory.roll, borderColor: colors.secondary, backgroundColor: 'rgba(255, 102, 0, 0.1)', tension: 0.1 },
          { label: 'Yaw (°)', data: dataHistory.yaw, borderColor: colors.tertiary, backgroundColor: 'rgba(0, 204, 255, 0.1)', tension: 0.1 }
        ]
      },
      options: {
        responsive: true,
        maintainAspectRatio: true,
        plugins: { legend: { display: true } }
      }
    });
  }

  // Power Chart
  const pwrCtx = $("chart-power");
  if (pwrCtx) {
    charts.power = new Chart(pwrCtx, {
      type: 'line',
      data: {
        labels: dataHistory.time,
        datasets: [
          { label: 'Voltage (V)', data: dataHistory.vbat, borderColor: colors.primary, backgroundColor: 'rgba(0, 255, 136, 0.1)', tension: 0.1 },
          { label: 'Current (A)', data: dataHistory.current, borderColor: colors.danger, backgroundColor: 'rgba(255, 51, 51, 0.1)', tension: 0.1, yAxisID: 'y1' }
        ]
      },
      options: {
        responsive: true,
        maintainAspectRatio: true,
        scales: {
          y: { title: { display: true, text: 'Voltage (V)' } },
          y1: { type: 'linear', position: 'right', title: { display: true, text: 'Current (A)' } }
        },
        plugins: { legend: { display: true } }
      }
    });
  }

  // Position Chart
  const posCtx = $("chart-position");
  if (posCtx) {
    charts.position = new Chart(posCtx, {
      type: 'scatter',
      data: {
        datasets: [
          {
            label: 'Flight Path',
            data: dataHistory.lat.map((lat, i) => ({ x: dataHistory.lon[i], y: lat })),
            borderColor: colors.primary,
            backgroundColor: 'rgba(0, 255, 136, 0.3)',
            showLine: false
          }
        ]
      },
      options: {
        responsive: true,
        maintainAspectRatio: true,
        scales: {
          x: { title: { display: true, text: 'Longitude' } },
          y: { title: { display: true, text: 'Latitude' } }
        }
      }
    });
  }

  // Signal Chart
  const sigCtx = $("chart-signal");
  if (sigCtx) {
    charts.signal = new Chart(sigCtx, {
      type: 'line',
      data: {
        labels: dataHistory.time,
        datasets: [
          { label: 'Satellites', data: dataHistory.sats, borderColor: colors.primary, backgroundColor: 'rgba(0, 255, 136, 0.1)', tension: 0.1 },
          { label: 'RSSI', data: dataHistory.rssi, borderColor: colors.secondary, backgroundColor: 'rgba(255, 102, 0, 0.1)', tension: 0.1, yAxisID: 'y1' }
        ]
      },
      options: {
        responsive: true,
        maintainAspectRatio: true,
        scales: {
          y: { title: { display: true, text: 'Satellites' } },
          y1: { type: 'linear', position: 'right', title: { display: true, text: 'RSSI' } }
        },
        plugins: { legend: { display: true } }
      }
    });
  }
}

function updateCharts() {
  if (charts.altitude) {
    charts.altitude.data.labels = dataHistory.time;
    charts.altitude.data.datasets[0].data = dataHistory.altitude;
    charts.altitude.data.datasets[1].data = dataHistory.vspeed;
    charts.altitude.update('none');
  }
  if (charts.speed) {
    charts.speed.data.labels = dataHistory.time;
    charts.speed.data.datasets[0].data = dataHistory.speed_3d;
    charts.speed.data.datasets[1].data = dataHistory.speed;
    charts.speed.update('none');
  }
  if (charts.attitude) {
    charts.attitude.data.labels = dataHistory.time;
    charts.attitude.data.datasets[0].data = dataHistory.pitch;
    charts.attitude.data.datasets[1].data = dataHistory.roll;
    charts.attitude.data.datasets[2].data = dataHistory.yaw;
    charts.attitude.update('none');
  }
  if (charts.power) {
    charts.power.data.labels = dataHistory.time;
    charts.power.data.datasets[0].data = dataHistory.vbat;
    charts.power.data.datasets[1].data = dataHistory.current;
    charts.power.update('none');
  }
  if (charts.position) {
    charts.position.data.datasets[0].data = dataHistory.lat.map((lat, i) => ({ x: dataHistory.lon[i], y: lat }));
    charts.position.update('none');
  }
  if (charts.signal) {
    charts.signal.data.labels = dataHistory.time;
    charts.signal.data.datasets[0].data = dataHistory.sats;
    charts.signal.data.datasets[1].data = dataHistory.rssi;
    charts.signal.update('none');
  }
}

async function pollTelemetry() {
  try {
    const r = await fetch("/api/telemetry");
    const j = await r.json();
    if (!j.ok) {
      updateLink(false);
      return;
    }
    const d = j.data || {};
    updateLink(true);

    // Add to history
    addDataPoint(d);

    // Update live display
    $("v-pitch").textContent = fmt(d.pitch, 1);
    $("v-roll").textContent  = fmt(d.roll, 1);
    $("v-yaw").textContent   = fmt(d.yaw, 1);
    setBarSigned($("b-pitch"), d.pitch, 90);
    setBarSigned($("b-roll"),  d.roll, 90);

    $("v-alt").textContent    = fmt(d.alt, 1);
    $("v-vspeed").textContent = fmt(d.v_speed, 2);
    $("v-spd3d").textContent  = fmt(d.speed_3d, 1);
    $("v-spd").textContent    = fmt(d.speed, 1);

    $("v-lat").textContent    = fmt(d.lat, 5);
    $("v-lon").textContent    = fmt(d.lon, 5);
    $("v-sats").textContent   = d.sats ?? "--";
    $("v-course").textContent = d.course ?? "--";
    $("v-homed").textContent  = d.home_dist ?? "--";
    $("v-homeb").textContent  = d.home_dir ?? "--";

    $("v-vbat").textContent    = fmt(d.vbat, 2);
    $("v-current").textContent = fmt(d.current, 2);
    $("v-thr").textContent     = d.throttle ?? "--";
    $("v-rssi").textContent    = d.rssi ?? "--";
    setBar($("b-vbat"), d.vbat, 13.5, 16.8);
    setBar($("b-thr"),  d.throttle, 1000, 2000);

    $("raw-json").textContent = JSON.stringify(d, null, 2);
  } catch (e) {
    updateLink(false);
  }
}

async function loadConfig() {
  try {
    const [stateR, cfgR] = await Promise.all([
      fetch("/api/state"),
      fetch("/api/config"),
    ]);
    const state = await stateR.json();
    const cfg = await cfgR.json();

    const slots = Object.keys(state.selected).sort();
    const saved = (cfg && cfg.displays) ? cfg.displays : state.selected;
    const modules = ["", ...(state.available_modules || [])];

    const wrap = $("config-rows");
    wrap.innerHTML = "";
    slots.forEach((slot) => {
      const row = document.createElement("div");
      row.className = "cfg-row";
      const lbl = document.createElement("label");
      lbl.textContent = slot.toUpperCase();
      const sel = document.createElement("select");
      sel.dataset.slot = slot;
      modules.forEach((m) => {
        const opt = document.createElement("option");
        opt.value = m;
        opt.textContent = m || "— none —";
        if ((saved[slot] || state.selected[slot]) === m) opt.selected = true;
        sel.appendChild(opt);
      });
      row.appendChild(lbl);
      row.appendChild(sel);
      wrap.appendChild(row);
    });
  } catch (e) {
    console.error("Config load failed", e);
  }
}

async function saveConfig() {
  const sels = document.querySelectorAll("#config-rows select");
  const displays = {};
  sels.forEach((s) => {
    if (s.value) displays[s.dataset.slot] = s.value;
  });
  const status = $("apply-status");
  status.textContent = "SAVING...";
  try {
    const r = await fetch("/api/config", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ displays }),
    });
    const j = await r.json();
    if (j.ok) {
      status.textContent = "SAVED ✓  RESTARTING...";
      setTimeout(() => location.reload(), 1000);
    } else {
      status.textContent = "ERROR: " + (j.error || "?");
    }
  } catch (e) {
    status.textContent = "NETWORK ERROR";
  }
}

async function loadPorts(preserveSelection = false) {
  const sel = $("fc-port");
  const prev = preserveSelection ? sel.value : null;
  sel.innerHTML = "";

  try {
    const r = await fetch("/api/ports");
    const j = await r.json();
    const ports = j.ports || [];
    if (!ports.length) {
      const o = document.createElement("option");
      o.value = "";
      o.textContent = "— no ports found —";
      sel.appendChild(o);
      return;
    }
    ports.forEach((p) => {
      const o = document.createElement("option");
      o.value = p.path;
      const kindTag = p.kind ? `[${p.kind}]` : "";
      o.textContent = `${kindTag} ${p.label}`;
      sel.appendChild(o);
    });
    if (prev) sel.value = prev;
  } catch (e) {
    console.error("Port scan failed", e);
  }
}

async function loadFcConfig() {
  try {
    const r = await fetch("/api/fc-config");
    const fc = await r.json();

    const sel = $("fc-port");
    if (fc.port && ![...sel.options].some((o) => o.value === fc.port)) {
      const o = document.createElement("option");
      o.value = fc.port;
      o.textContent = `[saved] ${fc.port}`;
      sel.prepend(o);
    }
    if (fc.port) sel.value = fc.port;

    if (fc.baudrate) $("fc-baud").value = String(fc.baudrate);
    if (fc.protocol) $("fc-proto").value = fc.protocol;
  } catch (e) {
    console.error("FC config load failed", e);
  }
}

async function testFcLink() {
  const port = $("fc-port").value;
  const baud = parseInt($("fc-baud").value, 10);
  const out = $("fc-test-out");
  out.hidden = true;

  try {
    const r = await fetch("/api/port/test", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ port, baudrate: baud }),
    });
    const j = await r.json();
    out.hidden = false;
    out.textContent = j.output || "No output";
  } catch (e) {
    out.hidden = false;
    out.textContent = "ERROR: " + e.message;
  }
}

// Tab switching
function setupTabs() {
  document.querySelectorAll(".tab").forEach((btn) => {
    btn.addEventListener("click", () => {
      const tab = btn.dataset.tab;
      document.querySelectorAll(".tab").forEach((b) => b.classList.remove("active"));
      document.querySelectorAll(".tab-panel").forEach((p) => p.classList.remove("active"));
      btn.classList.add("active");
      document.querySelector(`[data-panel="${tab}"]`).classList.add("active");
    });
  });
}

function setupButtons() {
  $("btn-rescan")?.addEventListener("click", () => loadPorts(true));
  $("btn-fc-test")?.addEventListener("click", testFcLink);
  $("btn-apply")?.addEventListener("click", saveConfig);
  $("btn-clear-charts")?.addEventListener("click", () => {
    dataHistory = {
      time: [], altitude: [], vspeed: [], speed_3d: [], speed: [],
      pitch: [], roll: [], yaw: [], vbat: [], current: [],
      lat: [], lon: [], sats: [], rssi: []
    };
    updateCharts();
    $("chart-info").textContent = "Data points: 0";
  });
  $("btn-scan-diag")?.addEventListener("click", async () => {
    try {
      const r = await fetch("/api/ports");
      const j = await r.json();
      $("diag-ports").textContent = JSON.stringify(j, null, 2);
    } catch (e) {
      $("diag-ports").textContent = "ERROR: " + e.message;
    }
  });
}

// Init
window.addEventListener("DOMContentLoaded", () => {
  setupTabs();
  setupButtons();
  loadConfig();
  loadPorts();
  loadFcConfig();
  initCharts();
  setInterval(updateClock, 1000);
  setInterval(pollTelemetry, POLL_MS);
  updateClock();
  pollTelemetry();
});
