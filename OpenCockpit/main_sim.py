"""Run displays with the simulator instead of a real Flight Controller.

Identical to main.py but swaps MSP_Read_pi for Simulator.MSP_Sim.
"""

import sys
import os

# Make sibling Simulator package importable
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "Simulator"))

import MSP_Sim  # noqa: E402

# Inject the simulator as MSP_Read_pi before main.py imports it
sys.modules["MSP_Read_pi"] = MSP_Sim

import main  # noqa: E402

if __name__ == "__main__":
    main.main()
