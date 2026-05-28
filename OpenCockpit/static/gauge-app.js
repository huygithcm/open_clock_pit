// Gauge Dashboard Application - Swarm-OS Edition
const POLL_MS = 250;

const $ = (id) => document.getElementById(id);

let altitudeHistory = [];
const MAX_CHART_POINTS = 60;
let mapTime = 0;

function fmt(v, digits = 2) {
  if (v === null || v === undefined) return "--";
  if (typeof v === "number") return v.toFixed(digits);
  return String(v);
}

function updateLink(ok) {
  const b = $("link-status");
  if (!b) return;
  if (ok) {
    b.className = "badge online";
    b.textContent = "LINK ONLINE";
  } else {
    b.className = "badge offline";
    b.textContent = "LINK OFFLINE";
  }
}

function updateClock() {
  const d = new Date();
  const el = $("clock");
  if (el) el.textContent = d.toTimeString().slice(0, 8);
}

function safeUpdate(elementId, value) {
  const el = $(elementId);
  if (el) {
    el.textContent = value;
  }
}

// ========== CANVAS MAP RENDERING ==========
function drawSwarmMap(snap) {
  const canvas = $('swarm-map');
  if (!canvas) return;
  const ctx = canvas.getContext('2d');
  
  const rect = canvas.getBoundingClientRect();
  canvas.width = rect.width * 2;
  canvas.height = rect.height * 2;
  ctx.scale(2, 2);
  
  const w = rect.width;
  const h = rect.height;
  
  // Clear background
  ctx.fillStyle = '#030a04';
  ctx.fillRect(0, 0, w, h);
  
  // Draw tactical grids
  ctx.strokeStyle = 'rgba(255, 255, 255, 0.03)';
  ctx.lineWidth = 1;
  const gridSize = 30;
  for (let x = 0; x < w; x += gridSize) {
    ctx.beginPath();
    ctx.moveTo(x, 0);
    ctx.lineTo(x, h);
    ctx.stroke();
  }
  for (let y = 0; y < h; y += gridSize) {
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(w, y);
    ctx.stroke();
  }
  
  mapTime += 0.015;
  
  // Define flight path vectors
  const homeX = w * 0.35;
  const homeY = h * 0.8;
  const zoneX = w * 0.65;
  const zoneY = h * 0.25;
  
  // Draw dotted pathways
  ctx.save();
  ctx.strokeStyle = 'rgba(57, 255, 20, 0.12)';
  ctx.lineWidth = 1.5;
  ctx.setLineDash([4, 4]);
  
  const drawPathOffset = (offset) => {
    ctx.beginPath();
    ctx.moveTo(homeX + offset, homeY);
    ctx.lineTo(zoneX + offset, zoneY);
    ctx.stroke();
  };
  
  drawPathOffset(-12);
  drawPathOffset(0);
  drawPathOffset(12);
  ctx.restore();
  
  // Draw home point
  ctx.save();
  ctx.strokeStyle = '#39ff14';
  ctx.fillStyle = 'rgba(57, 255, 20, 0.12)';
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.arc(homeX, homeY, 6, 0, Math.PI * 2);
  ctx.fill();
  ctx.stroke();
  
  ctx.beginPath();
  ctx.arc(homeX, homeY, 12, 0, Math.PI * 2);
  ctx.strokeStyle = 'rgba(57, 255, 20, 0.22)';
  ctx.setLineDash([2, 3]);
  ctx.stroke();
  ctx.restore();

  // Draw mission warning ZONE (oval)
  ctx.save();
  ctx.strokeStyle = '#f6931f';
  ctx.fillStyle = 'rgba(246, 147, 31, 0.06)';
  ctx.lineWidth = 1.5;
  ctx.setLineDash([5, 3]);
  
  ctx.beginPath();
  ctx.ellipse(zoneX, zoneY, 36, 22, -Math.PI / 6, 0, Math.PI * 2);
  ctx.fill();
  ctx.stroke();
  
  ctx.fillStyle = '#f6931f';
  ctx.font = '800 8px "Orbitron", monospace';
  ctx.textAlign = 'center';
  ctx.fillText('ZONE', zoneX, zoneY + 3);
  ctx.restore();

  // Animate flight path progress
  const progress = (mapTime * 0.03) % 1.0;
  
  // UAV-01 (Live telemetry)
  let factor1 = progress;
  if (snap && snap.home_dist > 0) {
    factor1 = Math.min(0.9, snap.home_dist / 350.0);
  }
  const u1x = homeX + (zoneX - homeX) * factor1;
  const u1y = homeY + (zoneY - homeY) * factor1;
  const u1yaw = snap && snap.yaw !== undefined ? snap.yaw : -35;
  
  // UAV-02 (Simulated)
  const factor2 = (factor1 + 0.3) % 0.85;
  const u2x = homeX + (zoneX - homeX) * factor2 - 12;
  const u2y = homeY + (zoneY - homeY) * factor2 + 6;
  const u2yaw = -30 + Math.sin(mapTime * 1.5) * 6;

  // UAV-04 (Simulated Alert)
  const factor4 = 0.72 + Math.sin(mapTime * 0.8) * 0.05;
  const u4x = homeX + (zoneX - homeX) * factor4 + 12;
  const u4y = homeY + (zoneY - homeY) * factor4 - 8;
  const u4yaw = -40 + Math.cos(mapTime * 1.2) * 8;

  const drawChevron = (x, y, yawDeg, name, color, size = 6, isPulse = false) => {
    ctx.save();
    ctx.translate(x, y);
    ctx.rotate((yawDeg * Math.PI) / 180);
    
    if (isPulse) {
      ctx.strokeStyle = color;
      ctx.beginPath();
      ctx.arc(0, 0, size * (1.6 + Math.sin(mapTime * 6) * 0.3), 0, Math.PI * 2);
      ctx.lineWidth = 1;
      ctx.stroke();
    }
    
    ctx.strokeStyle = color;
    ctx.fillStyle = 'rgba(5, 7, 10, 0.85)';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(0, -size);
    ctx.lineTo(size * 0.75, size * 0.8);
    ctx.lineTo(0, size * 0.25);
    ctx.lineTo(-size * 0.75, size * 0.8);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();
    ctx.restore();
    
    ctx.fillStyle = color;
    ctx.font = '700 8px "Orbitron", monospace';
    ctx.textAlign = 'left';
    ctx.fillText(name, x + size + 3, y + 3);
  };

  drawChevron(u1x, u1y, u1yaw, 'UAV-01', '#39ff14', 6, true);
  drawChevron(u2x, u2y, u2yaw, 'UAV-02', '#6bba4f', 6, false);
  drawChevron(u4x, u4y, u4yaw, 'UAV-04', '#f5a623', 6, true);
}

// ========== ALTITUDE LINE CHART ==========
function drawAltitudeChart(alt) {
  const canvas = $('swarm-altitude-chart');
  if (!canvas) return;
  const ctx = canvas.getContext('2d');
  
  const rect = canvas.getBoundingClientRect();
  canvas.width = rect.width * 2;
  canvas.height = rect.height * 2;
  ctx.scale(2, 2);
  
  const w = rect.width;
  const h = rect.height;
  
  ctx.fillStyle = '#030a04';
  ctx.fillRect(0, 0, w, h);
  
  // Grid lines
  ctx.strokeStyle = 'rgba(255, 255, 255, 0.03)';
  ctx.lineWidth = 1;
  for (let y = 15; y < h; y += 22) {
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(w, y);
    ctx.stroke();
  }
  for (let x = 0; x < w; x += 35) {
    ctx.beginPath();
    ctx.moveTo(x, 0);
    ctx.lineTo(x, h);
    ctx.stroke();
  }

  if (alt !== undefined && alt !== null) {
    altitudeHistory.push(alt);
    if (altitudeHistory.length > MAX_CHART_POINTS) {
      altitudeHistory.shift();
    }
  }

  if (altitudeHistory.length < 2) return;

  const min = Math.min(...altitudeHistory);
  let max = Math.max(...altitudeHistory);
  if (max - min < 10) {
    max = min + 10;
  }
  
  const points = [];
  for (let i = 0; i < altitudeHistory.length; i++) {
    const x = (i / (MAX_CHART_POINTS - 1)) * w;
    const y = h - 8 - ((altitudeHistory[i] - min) / (max - min)) * (h - 16);
    points.push({ x, y });
  }

  // Draw area gradient fill
  ctx.save();
  ctx.beginPath();
  ctx.moveTo(points[0].x, h);
  points.forEach(p => ctx.lineTo(p.x, p.y));
  ctx.lineTo(points[points.length - 1].x, h);
  ctx.closePath();
  
  const grad = ctx.createLinearGradient(0, 0, 0, h);
  grad.addColorStop(0, 'rgba(57, 255, 20, 0.22)');
  grad.addColorStop(1, 'rgba(26, 122, 16, 0.0)');
  ctx.fillStyle = grad;
  ctx.fill();
  ctx.restore();

  // Draw path stroke
  ctx.strokeStyle = '#39ff14';
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.moveTo(points[0].x, points[0].y);
  for (let i = 1; i < points.length; i++) {
    ctx.lineTo(points[i].x, points[i].y);
  }
  ctx.stroke();
  
  // Alt text markers
  ctx.fillStyle = 'rgba(107, 186, 79, 0.7)';
  ctx.font = '700 8px "Orbitron", monospace';
  ctx.textAlign = 'right';
  ctx.fillText(`${Math.round(max)}m`, w - 6, 12);
  ctx.fillText(`${Math.round(min)}m`, w - 6, h - 6);
}

// ========== SIMULATED CAMERA FEEDS ==========
function drawCameraFeeds(snap) {
  const ids = ['canvas-feed-01', 'canvas-feed-02', 'canvas-feed-04'];
  ids.forEach((id, index) => {
    const canvas = $(id);
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    
    const rect = canvas.getBoundingClientRect();
    canvas.width = rect.width;
    canvas.height = rect.height;
    
    const w = rect.width;
    const h = rect.height;
    
    // Camera feed screen background
    ctx.fillStyle = '#020a03';
    ctx.fillRect(0, 0, w, h);
    
    // Crosshair target
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.08)';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(w / 2 - 8, h / 2);
    ctx.lineTo(w / 2 + 8, h / 2);
    ctx.moveTo(w / 2, h / 2 - 8);
    ctx.lineTo(w / 2, h / 2 + 8);
    ctx.stroke();
    
    // Flight scrolling grids
    const flow = (Date.now() * 0.01) % 30;
    ctx.strokeStyle = 'rgba(57, 255, 20, 0.025)';
    for (let y = flow; y < h; y += 30) {
      ctx.beginPath();
      ctx.moveTo(0, y);
      ctx.lineTo(w, y);
      ctx.stroke();
    }
    
    ctx.fillStyle = 'rgba(57, 255, 20, 0.5)';
    ctx.font = '700 7px "Orbitron", monospace';
    if (index === 0) {
      const altVal = snap && snap.alt !== undefined ? Math.round(snap.alt) : 0;
      const yawVal = snap && snap.yaw !== undefined ? Math.round(snap.yaw) : 0;
      ctx.fillText(`ALT: ${altVal}M`, 6, h - 6);
      ctx.fillText(`HDG: ${yawVal.toString().padStart(3, '0')}°`, w - 46, h - 6);
      ctx.fillStyle = '#39ff14';
      ctx.fillText(`LIVE ●`, w - 30, 10);
    } else if (index === 1) {
      ctx.fillStyle = '#6bba4f';
      ctx.fillText(`ALT: 152M`, 6, h - 6);
      ctx.fillText(`HDG: 325°`, w - 46, h - 6);
      ctx.fillText(`STBY`, w - 24, 10);
    } else {
      ctx.fillStyle = '#f6931f';
      ctx.fillText(`ALT: 88M`, 6, h - 6);
      ctx.fillText(`HDG: 190°`, w - 46, h - 6);
      ctx.fillText(`WARN ●`, w - 34, 10);
    }
  });
}

// ========== PROGRESS/BAR CHARTS UPDATES ==========
function updateMetricBars(d) {
  // Battery V: 10.0V - 17.0V range
  const vbat = d.vbat !== undefined ? d.vbat : 12.0;
  const vbatPct = Math.max(0, Math.min(100, ((vbat - 10.0) / 7.0) * 100));
  const vbatFill = $('fill-vbat');
  if (vbatFill) {
    vbatFill.style.width = `${vbatPct}%`;
    vbatFill.className = 'metric-fill';
    if (vbat < 11.2) {
      vbatFill.classList.add('critical');
    } else if (vbat < 11.8) {
      vbatFill.classList.add('warning');
    }
  }
  safeUpdate('metric-vbat', fmt(vbat, 2));

  // Current A: 0A - 50A range
  const current = d.current !== undefined ? d.current : 0;
  const currentPct = Math.max(0, Math.min(100, (current / 50.0) * 100));
  const currentFill = $('fill-current');
  if (currentFill) {
    currentFill.style.width = `${currentPct}%`;
    currentFill.className = 'metric-fill';
    if (current > 40) {
      currentFill.classList.add('critical');
    } else if (current > 28) {
      currentFill.classList.add('warning');
    }
  }
  safeUpdate('metric-current', fmt(current, 1));

  // Throttle: 1000 - 2000 range
  const throttleRaw = d.throttle !== undefined ? d.throttle : 1000;
  const throttlePct = Math.max(0, Math.min(100, ((throttleRaw - 1000) / 1000) * 100));
  const throttleFill = $('fill-throttle');
  if (throttleFill) {
    throttleFill.style.width = `${throttlePct}%`;
  }
  safeUpdate('metric-throttle', fmt(throttlePct, 0));

  // RSSI: 0 - 255 range
  const rssi = d.rssi !== undefined ? d.rssi : 0;
  const rssiPct = Math.max(0, Math.min(100, (rssi / 255.0) * 100));
  const rssiFill = $('fill-rssi');
  if (rssiFill) {
    rssiFill.style.width = `${rssiPct}%`;
    rssiFill.className = 'metric-fill';
    if (rssi < 90) {
      rssiFill.classList.add('critical');
    } else if (rssi < 130) {
      rssiFill.classList.add('warning');
    }
  }
  safeUpdate('metric-rssi', fmt(rssi, 0));

  // Speed 3D: 0 - 50m/s range
  const speed = d.speed_3d !== undefined ? d.speed_3d : 0;
  const speedPct = Math.max(0, Math.min(100, (speed / 50.0) * 100));
  const speedFill = $('fill-speed');
  if (speedFill) {
    speedFill.style.width = `${speedPct}%`;
  }
  safeUpdate('metric-speed', fmt(speed, 1));

  // Vertical Speed: -10 to +10 m/s dual progress fills
  const vspeed = d.v_speed !== undefined ? d.v_speed : 0;
  const vspeedDownFill = $('fill-vspeed-down');
  const vspeedUpFill = $('fill-vspeed-up');
  if (vspeedDownFill && vspeedUpFill) {
    if (vspeed < 0) {
      const downPct = Math.max(0, Math.min(100, (Math.abs(vspeed) / 10.0) * 100));
      vspeedDownFill.style.width = `${downPct}%`;
      vspeedUpFill.style.width = `0%`;
    } else {
      const upPct = Math.max(0, Math.min(100, (vspeed / 10.0) * 100));
      vspeedUpFill.style.width = `${upPct}%`;
      vspeedDownFill.style.width = `0%`;
    }
  }
  safeUpdate('metric-vspeed', fmt(vspeed, 2));
}

// ========== TELEMETRY POLLING HANDLER ==========
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
      return;
    }

    const d = j.data || {};
    updateLink(true);

    // 1. Render canvas loops
    drawSwarmMap(d);
    drawAltitudeChart(d.alt || 0);
    drawCameraFeeds(d);

    // 2. Update real-time progress bar charts
    updateMetricBars(d);

    // 3. Update left sidebar live UAV card details
    safeUpdate('uav-01-alt', `${Math.round(d.alt || 0)}m`);
    safeUpdate('uav-01-speed', `${Math.round(d.speed_3d || 0)}m/s`);
    safeUpdate('uav-01-vbat', `${fmt(d.vbat, 1)}V`);
    safeUpdate('uav-01-current', `${fmt(d.current, 1)}A`);
    safeUpdate('uav-01-throttle', `${fmt(((d.throttle || 1000) - 1000) / 10, 0)}%`);
    safeUpdate('uav-01-rssi', d.rssi !== undefined ? d.rssi : '--');
    safeUpdate('uav-01-gps', `${d.sats || 0} Sats`);
    
    // 4. Distance reads
    safeUpdate('dist-uav-01', `${((d.home_dist || 0) / 1000).toFixed(2)} km`);

    // 5. Update Telemetry Tab metrics
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

// ========== CONFIGURATION LOADING & SAVING ==========
async function loadFcConfig() {
  try {
    const r = await fetch('/api/fc-config');
    const fc = await r.json();

    const sel = $('fc-port');
    if (sel && fc.port) {
      if (![...sel.options].some((o) => o.value === fc.port)) {
        const o = document.createElement('option');
        o.value = fc.port;
        o.textContent = `[saved] ${fc.port}`;
        sel.prepend(o);
      }
      sel.value = fc.port;
    }

    if (fc.baudrate && $('fc-baud')) $('fc-baud').value = String(fc.baudrate);
    if (fc.protocol && $('fc-proto')) $('fc-proto').value = fc.protocol;
  } catch (e) {
    console.error('FC config load failed', e);
  }
}

async function saveFcConfig() {
  const port = $('fc-port')?.value;
  const baud = parseInt($('fc-baud')?.value || '115200', 10);
  const proto = $('fc-proto')?.value || 'auto';
  const status = $('fc-status');
  if (!status) return;

  status.textContent = 'SAVING...';
  try {
    const r = await fetch('/api/fc-config', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ port, baudrate: baud, protocol: proto }),
    });
    const j = await r.json();
    if (j.ok) {
      status.textContent = 'SAVED ✓ RESTARTING...';
      fetch('/api/restart', { method: 'POST' }).catch(() => {});
      setTimeout(() => location.reload(), 2000);
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

// ========== TABS SWITCHING & BUTTON BINDINGS ==========
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
  $('btn-fc-save')?.addEventListener('click', saveFcConfig);
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
      alert(j.output || `Test completed:\nBytes read: ${j.bytes}\nProtocol hint: ${j.protocol_hint}\nHex data: ${j.head_hex}`);
    } catch (e) {
      alert('Error: ' + e.message);
    }
  });

  // Swarm-OS Button Bindings
  $('btn-end-mission')?.addEventListener('click', () => {
    if (confirm("Are you sure you want to abort the mission? This will command all UAVs to return to home base.")) {
      alert("Abort command sent. UAVs returning to home base.");
    }
  });

  $('btn-apply-one')?.addEventListener('click', () => {
    const targetAlt = $('action-alt')?.value;
    const txGa = $('action-tx-ga')?.value;
    const txAg = $('action-tx-ag')?.value;
    alert(`Command sent to UAV-01:\nTarget Altitude: ${targetAlt}m\nTX G->A: ${txGa}mW\nTX A->G: ${txAg}mW`);
  });

  $('btn-apply-all')?.addEventListener('click', () => {
    const targetAlt = $('action-alt')?.value;
    const txGa = $('action-tx-ga')?.value;
    const txAg = $('action-tx-ag')?.value;
    alert(`Broadcast command sent to ALL Swarm units:\nTarget Altitude: ${targetAlt}m\nTX G->A: ${txGa}mW\nTX A->G: ${txAg}mW`);
  });
}

// ========== INITIALIZATION ==========
window.addEventListener('DOMContentLoaded', () => {
  console.log("=== Swarm-OS Dashboard Loading ===");
  setupTabs();
  setupButtons();
  loadPorts().then(loadFcConfig);

  setInterval(updateClock, 1000);
  setInterval(pollTelemetry, POLL_MS);

  updateClock();
  pollTelemetry();

  console.log("=== Swarm-OS Dashboard Ready ===");
});
