# 🚁 OpenCockpit Update — Swarm-OS Dashboard

This update overhauls the web interface, replacing the circular analog gauges and simulated LCD frames with a high-resolution, custom-drawn **Swarm-OS Tactical Dashboard** based on HTML5 Canvas.

## 🌟 What's New

### 1. Swarm-OS Tab (Primary Dashboard)
A sleek, premium, 3-column military/sci-fi layout:
- **Left Panel (UAV Status List)**: Displays real-time data for 4 UAV swarm units. UAV-01 utilizes live flight controller telemetry (Battery, Current, Throttle, RSSI, GPS), while UAV-02, UAV-04, and UAV-05 display simulated drift coordinates.
- **Center Panel (Tactical Map & Video Feeds)**: 
  - A custom HTML5 Canvas rendering a dark vector grid map, showing home base coordinates, flight pathways, rotating chevrons, and warning boundaries (`ZONE`).
  - Contains an interactive **End Mission** button.
  - Three live simulated camera feed viewfinders showing status overlays at the bottom.
- **Right Panel (Metrics, Graphs, Actions)**:
  - Uplink & Downlink network bandwidth progress indicators.
  - Custom canvas-rendered real-time **Altitude History Line Graph** with ambient glowing fills.
  - Inline distance readouts.
  - **Quick Actions Form**: Target ALT and transmission power (TX G🡲A, TX A🡲G) inputs with glowing double-border **APPLY** buttons.

### 2. Segmented progress bar metrics
Traditional analog gauges have been replaced by high-performance linear progress meters with custom segmented tally markings and automatic warnings:
- **Battery**: Glows cyan under normal loads, turns orange (warning) at <11.8V, and red (critical) at <11.2V.
- **Current Draw**: Glows cyan under normal loads, turns orange/red on high current draws.
- **Throttle**: Linear throttle meter.
- **Signal (RSSI)**: Segmented signal strength readout.
- **Speed 3D**: Horizontal ground speed meter.
- **Vertical Speed**: Dual-direction symmetric progress indicator, expanding left (descending) or right (ascending) from a center divider.

### 3. Streamlined Configuration Tab
- Removed the deprecated physical display configurations (`DISPLAY SETTINGS`).
- Centered the **FC LINK CONFIGURATION** block in a single-column layout.
- Added a new direct **SAVE & RESTART** button that updates the flight controller port configuration and triggers daemon restart in a single click.

---

## 📁 Staged Files

The following files have been updated for this release:
- `OpenCockpit/templates/index.html` — Layout structures for the new tabs, forms, progress bars, and canvas elements.
- `OpenCockpit/static/gauge-style.css` — Swarm-OS variables, grid layouts, neon gradient fills, double-border animations, and responsive queries.
- `OpenCockpit/static/gauge-app.js` — Core javascript logic, Canvas loops for map/graphs, dynamic bar updates, and action click bindings.
