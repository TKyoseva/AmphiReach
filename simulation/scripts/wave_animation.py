#!/usr/bin/env python3
"""
Wave animation script for Gazebo simulation
Creates 1m wave height animation on water surface
"""

import math
import time

# This script can be used with gz-sim to animate waves
# For now, the animated pose plugin in the SDF handles basic wave motion

def calculate_wave_height(x, y, t, amplitude=0.5, period=5.0, wavelength=10.0):
    """
    Calculate wave height at position (x, y) at time t
    Using simple sine wave: h = A * sin(2*pi*(t/T - x/L))
    """
    return amplitude * math.sin(2 * math.pi * (t / period - x / wavelength))

if __name__ == "__main__":
    print("Wave animation helper script")
    print("Wave parameters:")
    print("  - Amplitude: 0.5m (total height: 1m)")
    print("  - Period: 5.0s")
    print("  - Wavelength: 10.0m")
    print("\nThe animated pose plugin in the SDF file handles wave motion.")
