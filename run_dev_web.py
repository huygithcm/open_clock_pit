import sys
import os
from unittest.mock import MagicMock

# 1. Force Pygame to run headless without opening physical windows on desktop
os.environ["SDL_VIDEODRIVER"] = "dummy"

# 2. Mock CircuitPython and GPIO hardware modules before importing displays
sys.modules["board"] = MagicMock()
sys.modules["digitalio"] = MagicMock()
sys.modules["busio"] = MagicMock()
sys.modules["adafruit_rgb_display"] = MagicMock()
sys.modules["adafruit_rgb_display.st7735"] = MagicMock()
sys.modules["adafruit_rgb_display.st7789"] = MagicMock()
sys.modules["RPi"] = MagicMock()
sys.modules["RPi.GPIO"] = MagicMock()

# Set up paths for importing OpenCockpit modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "OpenCockpit"))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "OpenCockpit", "Simulator"))

# Mock MSP_Read_pi daemon thread using MSP_Sim
import MSP_Sim
sys.modules["MSP_Read_pi"] = MSP_Sim

import threading
import time
import pygame

# Initialize Pygame in headless mode
pygame.init()

# Import the actual cockpit display files
import HUD_pi_085
import MFD_pi_096
import MAP_pi_096
import INFO_pi_096
import web_app

# Simulation display selections
SELECTED_DISPLAYS = {
    "Display_1": "HUD_0.85",
    "Display_2": "MAP_0.96",
    "Display_3": "MFD_0.96",
    "Display_4": "INFO_0.96",
}

MODULE_MAP = {
    "HUD_0.85": HUD_pi_085,
    "MAP_0.96": MAP_pi_096,
    "MFD_0.96": MFD_pi_096,
    "INFO_0.96": INFO_pi_096
}

def desktop_display_loop(module, width, height, fps):
    """Headless render thread matching display_loop in main.py."""
    # Override global screen inside the imported display file with our Surface
    module.screen = pygame.Surface((width, height))
    module.WIDTH, module.HEIGHT = width, height
    module.CENTER_X, module.CENTER_Y = width / 2, height / 2

    # Draw initial static surfaces
    if hasattr(module, "render_mfd_fixed"):
        module.render_mfd_fixed()
    if hasattr(module, "render_info_fixed"):
        module.render_info_fixed()

    clock = pygame.time.Clock()
    while True:
        clock.tick(fps)
        
        # Atomically snap simulated telemetry data
        with MSP_Sim.data_lock:
            snap = dict(MSP_Sim.data)

        # Extraction with safety defaults
        pitch = snap.get("pitch") if snap.get("pitch") is not None else 0.0
        roll  = snap.get("roll")  if snap.get("roll")  is not None else 0.0
        yaw   = snap.get("yaw")   if snap.get("yaw")   is not None else 0.0
        alt = snap.get("alt") if snap.get("alt") is not None else 0.0
        lat = snap.get("lat") if snap.get("lat") is not None else 10.7769
        lon = snap.get("lon") if snap.get("lon") is not None else 106.7009
        v_speed = snap.get("v_speed") if snap.get("v_speed") is not None else 0.0
        speed_3d = snap.get("speed_3d") if snap.get("speed_3d") is not None else 0.0
        sats = snap.get("sats") if snap.get("sats") is not None else 0
        course = snap.get("course") if snap.get("course") is not None else 0
        vbat = snap.get("vbat") if snap.get("vbat") is not None else 0.0
        current = snap.get("current") if snap.get("current") is not None else 0.0
        rssi = snap.get("rssi") if snap.get("rssi") is not None else 0.0
        throttle = snap.get("throttle") if snap.get("throttle") is not None else 0.0
        home_dist = snap.get("home_dist") if snap.get("home_dist") is not None else 0
        home_dir = snap.get("home_dir") if snap.get("home_dir") is not None else 0

        # Invoke actual Pygame canvas drawing operations
        if hasattr(module, "render_hud"):
            module.render_hud(pitch, roll, yaw, v_speed, alt, speed_3d, sats, course, vbat, current, home_dist, home_dir)
        elif hasattr(module, "render_mfd_dynamic"):
            module.render_mfd_dynamic(pitch, roll, yaw, v_speed, alt, speed_3d, sats, course, vbat, current, home_dist, home_dir)
        elif hasattr(module, "render_map"):
            module.render_map(yaw, v_speed, alt, lat, lon, speed_3d, sats, course, vbat, current, home_dist, home_dir)
        elif hasattr(module, "render_info_dynamic"):
            module.render_info_dynamic(vbat, current, rssi, throttle)

        # Composite screen buffers for MFD and INFO modules
        if module == MFD_pi_096 or module == INFO_pi_096:
            module.screen.blit(module.background_surface, (0, 0))
            module.screen.blit(module.dynamic_surface, (0, 0))
            module.screen.blit(module.fixed_surface, (0, 0))

if __name__ == "__main__":
    print("Starting Web App in Simulation Mode...")
    
    # Start telemetry simulator thread
    sim_thread = threading.Thread(target=MSP_Sim.main, daemon=True)
    sim_thread.start()
    
    # Initialize Flask web app configuration
    web_app.init(MSP_Sim, SELECTED_DISPLAYS, list(MODULE_MAP.keys()))
    
    # Start background display threads for the rendering loops
    for slot, mod_key in SELECTED_DISPLAYS.items():
        module_obj = MODULE_MAP.get(mod_key)
        if module_obj:
            cfg = getattr(module_obj, "DISPLAY_CONFIG", {})
            width = cfg.get("width", 128)
            height = cfg.get("height", 128)
            fps = 30 if "HUD" in mod_key else 15
            
            # Start background Pygame render thread
            t = threading.Thread(target=desktop_display_loop, args=(module_obj, width, height, fps), daemon=True)
            t.start()
            
            # Register module screen reference to serve PNG snapshots on the web API
            web_app.mark_running(slot)
            web_app.register_display(slot, module_obj)
            print(f"[OK] Registered simulated renderer loop for: {slot} ({mod_key})")
            
    web_app.app.run(host="127.0.0.1", port=5050, debug=False, use_reloader=False)
