# Armored Turtle Automated Filament Changer
#
# Copyright (C) 2024-2026 Armored Turtle
#
# This file may be distributed under the terms of the GNU GPLv3 license.

# FPS (Filament Position Sensor) Buffer Driver
#
# Uses an analog Hall-effect sensor (or any proportional filament position
# sensor) to provide proportional buffer control for any AFC unit type.
# Instead of the TurtleNeck's bang-bang approach (two mechanical switches
# toggling between high/low multipliers), this driver reads a continuous
# 0.0-1.0 value from an ADC pin and applies a smooth, proportional
# rotation-distance adjustment to keep the buffer at its set_point.
#
# Compatible with any ADC-based filament position sensor including:
#   - OpenAMS FPS boards
#   - Proportional Sync Feedback (PSF) sensors (DRV5055A2 / MT9105ET Hall)
#   - Any Hall-effect + spring slider producing a 0.0-1.0 analog signal
#
# FPS reading semantics:
#   0.1 (low)  -> buffer stretched / tension -> increase feed
#   0.5 (mid)  -> buffer centered / ideal state
#   0.9 (high) -> buffer compressed / pushing -> decrease feed
#
# PSF users: this driver accepts PSF-compatible config aliases:
#   neutral_point  -> set_point    (default 0.5)
#   max_tension    -> low_point    (default 0.1)
#   max_compression -> high_point  (default 0.9)
#
# The driver is unit-agnostic and registers itself as an AFC buffer so any
# lane can reference it via  buffer: <name>  in its config.
#
# For units WITHOUT stepper motors (OpenAMS, etc.), the driver simply
# provides the ADC reading.  The unit's own code (AFC_OpenAMS) manages
# tool_start_state and virtual sensors exactly as it already does — this
# driver just acts as the FPS hardware interface and config entry point.
# Set  pin_tool_start: buffer  and  buffer: <FPS_buffer_name>  on
# the extruder so Klipper treats it as a buffer, not a GPIO pin.

from __future__ import annotations



# ---------------------------------------------------------------------------
#  Virtual filament sensor & extruder patch
#
#  When pin_tool_start references an FPS buffer name (e.g. "FPS_buffer1"),
#  AFCExtruder.__init__ would try to register it as a GPIO pin and fail.
#  The patch below temporarily rewrites the config value to "buffer" (which
#  the base init harmlessly skips), then creates a lightweight virtual
#  filament sensor so the extruder has a valid fila_tool_start object.
# ---------------------------------------------------------------------------
