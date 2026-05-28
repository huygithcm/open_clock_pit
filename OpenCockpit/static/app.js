// OpenCockpit live UI
const POLL_MS = 250;
const LCD_REFRESH_MS = 200;     // LCD preview snapshot rate
const LCD_SCALE = 2;            // upscale factor for the preview (e.g. 128→256)

const $ = (id) => document.getElementById(id);

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
  // 0 deg = 50%; +range = 100%; -range = 0%
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

    // Attitude
    $("v-pitch").textContent = fmt(d.pitch, 1);
    $("v-roll").textContent  = fmt(d.roll, 1);
    $("v-yaw").textContent   = fmt(d.yaw, 1);
    setBarSigned($("b-pitch"), d.pitch, 90);
    setBarSigned($("b-roll"),  d.roll, 90);

    // Flight
    $("v-alt").textContent    = fmt(d.alt, 1);
    $("v-vspeed").textContent = fmt(d.v_speed, 2);
    $("v-spd3d").textContent  = fmt(d.speed_3d, 1);
    $("v-spd").textContent    = fmt(d.speed, 1);

    // GPS
    $("v-lat").textContent    = fmt(d.lat, 5);
    $("v-lon").textContent    = fmt(d.lon, 5);
    $("v-sats").textContent   = d.sats ?? "--";
    $("v-course").textContent = d.course ?? "--";
    $("v-homed").textContent  = d.home_dist ?? "--";
    $("v-homeb").textContent  = d.home_dir ?? "--";

    // Power
    $("v-vbat").textContent    = fmt(d.vbat, 2);
    $("v-current").textContent = fmt(d.current, 2);
    $("v-thr").textContent     = d.throttle ?? "--";
    $("v-rssi").textContent    = d.rssi ?? "--";
    setBar($("b-vbat"), d.vbat, 13.5, 16.8);
    setBar($("b-thr"),  d.throttle, 1000, 2000);

    // Raw
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
  const status = $("save-status");
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
      triggerRestart();
    } else {
      status.textContent = "ERROR: " + (j.error || "?");
    }
  } catch (e) {
    status.textContent = "NETWORK ERROR";
  }
}

// ---------------- LCD preview ----------------

let _lcdCells = {};   // slot -> { img, native_w, native_h }
let _lcdBuildKey = ""; // signature so we only rebuild when displays change

async function buildLcdPreview() {
  try {
    const r = await fetch("/api/state");
    const j = await r.json();
    const sizes = j.sizes || {};
    const running = j.running || [];

    // Build a signature to detect changes
    const key = running
      .slice().sort()
      .map((s) => `${s}:${sizes[s]?.module}:${sizes[s]?.width}x${sizes[s]?.height}`)
      .join("|");
    if (key === _lcdBuildKey) return;
    _lcdBuildKey = key;

    const grid = $("preview-grid");
    grid.innerHTML = "";
    _lcdCells = {};

    if (!running.length) {
      grid.innerHTML = '<div style="grid-column:1/-1;text-align:center;color:var(--text-dim);padding:20px;">NO DISPLAYS ACTIVE</div>';
      return;
    }

    running.forEach((slot) => {
      const meta = sizes[slot] || {};
      const w = meta.width || 128;
      const h = meta.height || 128;

      const cell = document.createElement("div");
      cell.className = "lcd-cell";

      const tag = document.createElement("div");
      tag.className = "lcd-tag";
      tag.innerHTML =
        `<span>${slot.replace("_", " ")} · ${meta.module || "?"}</span>` +
        `<span class="lcd-size">${w}×${h}</span>`;

      const img = document.createElement("img");
      img.className = "lcd-screen";
      img.width = w * LCD_SCALE;
      img.height = h * LCD_SCALE;
      img.alt = slot;

      cell.appendChild(tag);
      cell.appendChild(img);
      grid.appendChild(cell);

      _lcdCells[slot] = { img, cell, w, h };
    });
  } catch (e) {
    console.error("buildLcdPreview failed", e);
  }
}

function refreshLcdImages() {
  const t = Date.now();
  Object.entries(_lcdCells).forEach(([slot, c]) => {
    // Use Image preloader to avoid flashing on slow networks
    const next = new Image();
    next.onload = () => {
      c.img.src = next.src;
      c.cell.classList.remove("offline");
    };
    next.onerror = () => { c.cell.classList.add("offline"); };
    next.src = `/api/display/${slot}.png?t=${t}`;
  });
}

// ---------------- FC LINK panel ----------------

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

    // Pre-fill port: add a synthetic option if current port not in scanned list
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
  const status = $("fc-status");
  status.textContent = "TESTING...";
  out.hidden = true;

  try {
    const r = await fetch("/api/port/test", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ port, baudrate: baud }),
    });
    const j = await r.json();
    out.hidden = false;
    if (j.ok) {
      status.textContent = `OK ✓  ${j.bytes}B  proto=${j.protocol_hint}`;
      out.textContent =
        `port    : ${j.port}\n` +
        `baud    : ${j.baudrate}\n` +
        `bytes   : ${j.bytes}\n` +
        `protocol: ${j.protocol_hint}\n` +
        `head_hex: ${j.head_hex || "(none)"}`;
    } else {
      status.textContent = "FAIL";
      out.textContent = "Error: " + (j.error || "unknown");
    }
  } catch (e) {
    status.textContent = "NETWORK ERR";
    out.textContent = String(e);
    out.hidden = false;
  }
}

async function saveFcConfig() {
  const body = {
    port: $("fc-port").value,
    baudrate: parseInt($("fc-baud").value, 10),
    protocol: $("fc-proto").value,
  };
  const status = $("fc-status");
  status.textContent = "SAVING...";
  try {
    const r = await fetch("/api/fc-config", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body),
    });
    const j = await r.json();
    if (j.ok) {
      status.textContent = "SAVED ✓  RESTARTING...";
      triggerRestart();
    } else {
      status.textContent = "ERROR: " + (j.error || "?");
    }
  } catch (e) {
    status.textContent = "NETWORK ERROR";
  }
}

// ---------------- Auto-restart with overlay ----------------

function showRestartOverlay() {
  let o = $("restart-overlay");
  if (!o) {
    o = document.createElement("div");
    o.id = "restart-overlay";
    o.innerHTML = `
      <div class="restart-box">
        <div class="restart-spinner"></div>
        <div class="restart-title">RESTARTING SYSTEM</div>
        <div class="restart-sub" id="restart-sub">Spawning new process…</div>
      </div>`;
    document.body.appendChild(o);
  } else {
    o.style.display = "flex";
  }
}

function hideRestartOverlay() {
  const o = $("restart-overlay");
  if (o) o.style.display = "none";
}

function setRestartSub(text) {
  const el = $("restart-sub");
  if (el) el.textContent = text;
}

async function waitForServerBack(maxTries = 60, intervalMs = 1000) {
  for (let i = 0; i < maxTries; i++) {
    try {
      const r = await fetch("/api/state", { cache: "no-store" });
      if (r.ok) {
        const j = await r.json();
        // Confirm the new process is actually running displays
        if ((j.running || []).length > 0) {
          setRestartSub(`Back online — ${j.running.length} displays running`);
          return true;
        }
      }
    } catch (e) { /* server still down */ }
    setRestartSub(`Waiting for server… (${i + 1}s)`);
    await new Promise((res) => setTimeout(res, intervalMs));
  }
  return false;
}

async function triggerRestart() {
  showRestartOverlay();
  setRestartSub("Sending restart signal…");
  try {
    await fetch("/api/restart", { method: "POST" });
  } catch (e) { /* expected: server may die before responding */ }

  setRestartSub("Old process dying…");
  // Give old process time to be killed by new one
  await new Promise((res) => setTimeout(res, 2000));

  const ok = await waitForServerBack();
  if (ok) {
    setRestartSub("Reloading page…");
    setTimeout(() => location.reload(), 600);
  } else {
    setRestartSub("Timeout. Please reload manually.");
  }
}

// Bootstrap
$("btn-save").addEventListener("click", saveConfig);
$("btn-rescan").addEventListener("click", () => loadPorts(true));
$("btn-fc-test").addEventListener("click", testFcLink);
$("btn-fc-save").addEventListener("click", saveFcConfig);
$("poll-info").textContent = "POLL " + POLL_MS + "MS";

loadConfig();
loadPorts().then(loadFcConfig);
buildLcdPreview();
setInterval(updateClock, 1000); updateClock();
setInterval(pollTelemetry, POLL_MS); pollTelemetry();
setInterval(buildLcdPreview, 5000);              // rebuild when displays change
setInterval(refreshLcdImages, LCD_REFRESH_MS);
refreshLcdImages();
