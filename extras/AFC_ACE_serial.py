# Armored Turtle Automated Filament Changer
#
# ACE Serial Communication Layer
# Direct serial communication with Anycubic ACE PRO hardware via JSON-RPC
# over USB serial. No dependency on ACEPRO or DuckACE.
#
# Protocol: Binary frames with CRC-16/CCITT-FALSE (reflected) checksums
# wrapping JSON-RPC payloads.
# Frame format: [0xFF 0xAA] [length_le16] [json_payload] [crc_le16] [0xFE]
#
# Reconnection and health monitoring inspired by Kobra-S1/ACEPRO.
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from __future__ import annotations

import json
import logging
import struct
import traceback

from typing import TYPE_CHECKING, Any, Dict, Optional

if TYPE_CHECKING:
    pass


_logger = logging.getLogger("AFC_ACE_serial")

# Frame constants
FRAME_HEADER = b'\xff\xaa'
FRAME_FOOTER = b'\xfe'
MAX_PAYLOAD_SIZE = 1024
DEFAULT_BAUD = 115200
REQUEST_TIMEOUT = 5.0

# Reconnection constants
RECONNECT_BACKOFF_MIN = 5.0
RECONNECT_BACKOFF_MAX = 30.0
RECONNECT_BACKOFF_FACTOR = 1.5

# Health monitoring constants
HEARTBEAT_INTERVAL = 2.0  # Must be < 3s to prevent ACE USB autosuspend
COMM_SUPERVISION_WINDOW = 30.0
COMM_TIMEOUT_THRESHOLD = 15
COMM_UNSOLICITED_THRESHOLD = 15
SUPERVISION_CHECK_INTERVAL = 10.0


def crc16_ccitt_reflected(data: bytes) -> int:
    """Calculate CRC-16/CCITT-FALSE (reflected) checksum.

    Compatible with ACEPRO's _calc_crc implementation.
    Uses polynomial 0x8408 (bit-reflected 0x1021).
    """
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0x8408
            else:
                crc >>= 1
    return crc & 0xFFFF


class ACESerialError(Exception):
    """Raised when ACE serial communication fails."""
    pass


class ACETimeoutError(ACESerialError):
    """Raised when an ACE command times out."""
    pass


class ACEConnection:
    """Serial connection to a single ACE PRO hardware unit.

    Integrates with Klipper's reactor for non-blocking I/O. Commands are
    sent as JSON-RPC over a binary-framed USB serial protocol. Responses
    are matched by request ID and delivered via reactor completions.

    Features (modelled after Kobra-S1/ACEPRO serial_manager.py):
    - Automatic reconnection with exponential backoff
    - Periodic heartbeat to detect dead connections
    - Communication health supervision (timeout/unsolicited tracking)
    """

    def __init__(self, reactor, serial_port, logger=None, baud_rate=DEFAULT_BAUD):
        self._reactor = reactor
        self._serial_port = serial_port
        self._baud_rate = baud_rate
        self._logger = logger or _logger

        self._serial = None
        self._fd_handle = None
        self._connected = False

        # Request tracking
        self._next_request_id = 0
        self._pending = {}  # request_id -> reactor.completion()

        # Read buffer for frame reassembly
        self._read_buffer = b''

        # Device info cache (populated on connect)
        self.device_info = {}
        self.slot_count = 4  # ACE PRO always has 4 slots

        # Reconnection state
        self._reconnect_timer = None
        self._reconnect_backoff = RECONNECT_BACKOFF_MIN
        self._reconnect_enabled = True

        # Heartbeat state
        self._heartbeat_timer = None
        self._last_rx_time = 0.0

        # Health supervision
        self._timeout_timestamps = []
        self._unsolicited_timestamps = []
        self._last_supervision_check = 0.0

        # Callback for status updates (set by unit)
        self.status_callback = None

        # Called after a successful reconnect (not initial connect)
        self.reconnect_callback = None

    @property
    def connected(self):
        return self._connected

    def connect(self):
        """Open the serial port and register with Klipper's reactor for reads."""
        if self._connected:
            return

        try:
            import serial as pyserial
        except ImportError:
            raise ACESerialError(
                "pyserial not installed. Install with: pip install pyserial"
            )

        try:
            self._serial = pyserial.Serial(
                port=self._serial_port,
                baudrate=self._baud_rate,
                timeout=0,           # Non-blocking reads
                write_timeout=0.5,
            )
            self._serial.reset_input_buffer()
            self._serial.reset_output_buffer()
        except Exception as e:
            raise ACESerialError(
                f"Failed to open ACE serial port {self._serial_port}: {e}"
            )

        # Register fd with reactor for incoming data
        self._fd_handle = self._reactor.register_fd(
            self._serial.fileno(), self._handle_read
        )
        self._connected = True
        self._last_rx_time = self._reactor.monotonic()
        self._logger.info(
            f"ACE serial connected: {self._serial_port} @ {self._baud_rate}"
        )

        # Query device info
        try:
            self.device_info = self.send_command("get_info", timeout=3.0)
            self._logger.info(f"ACE device info: {self.device_info}")
        except Exception as e:
            self._logger.debug(f"ACE get_info failed (non-fatal): {e}")

        # Reset backoff on successful connect
        self._reconnect_backoff = RECONNECT_BACKOFF_MIN

        # Start heartbeat
        self._start_heartbeat()

    def disconnect(self):
        """Close serial port and clean up reactor registrations."""
        was_connected = self._connected
        self._connected = False

        self._stop_heartbeat()

        if self._fd_handle is not None:
            try:
                self._reactor.unregister_fd(self._fd_handle)
            except Exception:
                pass
            self._fd_handle = None

        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None

        # Fail any pending requests
        for req_id, completion in list(self._pending.items()):
            try:
                completion.complete(None)
            except Exception:
                pass
        self._pending.clear()
        self._read_buffer = b''

        if was_connected:
            self._logger.info("ACE serial disconnected")

    def reconnect(self):
        """Disconnect and schedule automatic reconnection."""
        if not self._reconnect_enabled:
            return

        self.disconnect()

        delay = self._reconnect_backoff
        self._reconnect_backoff = min(
            self._reconnect_backoff * RECONNECT_BACKOFF_FACTOR,
            RECONNECT_BACKOFF_MAX,
        )

        self._logger.info(
            f"ACE scheduling reconnect in {delay:.0f}s "
            f"(next backoff: {self._reconnect_backoff:.0f}s)"
        )

        def _reconnect_callback(eventtime):
            if not self._reconnect_enabled:
                return self._reactor.NEVER
            try:
                self.connect()
                self._logger.info("ACE reconnected successfully")
                if self.reconnect_callback:
                    try:
                        self.reconnect_callback()
                    except Exception as e:
                        self._logger.warning(
                            f"ACE reconnect callback failed: {e}"
                        )
                return self._reactor.NEVER
            except Exception as e:
                self._logger.warning(f"ACE reconnect failed: {e}")
                next_delay = self._reconnect_backoff
                self._reconnect_backoff = min(
                    self._reconnect_backoff * RECONNECT_BACKOFF_FACTOR,
                    RECONNECT_BACKOFF_MAX,
                )
                return eventtime + next_delay

        if self._reconnect_timer is not None:
            try:
                self._reactor.unregister_timer(self._reconnect_timer)
            except Exception:
                pass

        self._reconnect_timer = self._reactor.register_timer(
            _reconnect_callback,
            self._reactor.monotonic() + delay,
        )

    def _quick_reconnect(self):
        """Fast reconnect for USB autosuspend - no backoff delay."""
        if not self._reconnect_enabled:
            return

        self.disconnect()

        def _quick_reconnect_cb(eventtime):
            if not self._reconnect_enabled:
                return self._reactor.NEVER
            try:
                self.connect()
                self._logger.debug("ACE quick reconnect succeeded")
                if self.reconnect_callback:
                    try:
                        self.reconnect_callback()
                    except Exception as e:
                        self._logger.warning(
                            f"ACE reconnect callback failed: {e}"
                        )
                return self._reactor.NEVER
            except Exception as e:
                self._logger.warning(f"ACE quick reconnect failed: {e}")
                # Fall back to normal reconnect with backoff
                self.reconnect()
                return self._reactor.NEVER

        if self._reconnect_timer is not None:
            try:
                self._reactor.unregister_timer(self._reconnect_timer)
            except Exception:
                pass

        # Reconnect after just 0.5s - enough for USB to re-enumerate
        self._reconnect_timer = self._reactor.register_timer(
            _quick_reconnect_cb,
            self._reactor.monotonic() + 0.5,
        )

    def send_command(self, method, params=None, timeout=REQUEST_TIMEOUT):
        """Send a JSON-RPC command and wait for the response.

        Uses Klipper's reactor completion mechanism to block the calling
        greenlet while still allowing other reactor events to be processed.
        """
        if not self._connected or self._serial is None:
            raise ACESerialError("ACE not connected")

        request_id = self._next_request_id
        self._next_request_id += 1

        request = {"id": request_id, "method": method}
        if params is not None:
            request["params"] = params

        payload = json.dumps(request, separators=(',', ':')).encode('utf-8')
        if len(payload) > MAX_PAYLOAD_SIZE:
            raise ACESerialError(
                f"ACE payload too large ({len(payload)} > {MAX_PAYLOAD_SIZE})"
            )

        completion = self._reactor.completion()
        self._pending[request_id] = completion

        frame = self._build_frame(payload)
        try:
            self._serial.write(frame)
            self._serial.flush()
        except Exception as e:
            self._pending.pop(request_id, None)
            self._track_timeout()
            raise ACESerialError(f"ACE write failed: {e}")

        self._logger.debug(f"ACE TX: {request}")

        deadline = self._reactor.monotonic() + timeout
        result = completion.wait(deadline)

        self._pending.pop(request_id, None)

        if result is None:
            self._track_timeout()
            raise ACETimeoutError(
                f"ACE command '{method}' (id={request_id}) timed out after {timeout}s"
            )

        # Check for error response
        if isinstance(result, dict):
            code = result.get("code", 0)
            msg = result.get("msg", "")
            if code != 0 or (msg and msg.upper() == "FORBIDDEN"):
                if not msg:
                    msg = "unknown error"
                raise ACESerialError(
                    f"ACE command '{method}' failed: code={code}, msg={msg}"
                )
            return result.get("result", result)

        return result

    def send_command_async(self, method, params=None):
        """Send a JSON-RPC command without waiting for a response."""
        if not self._connected or self._serial is None:
            return

        request_id = self._next_request_id
        self._next_request_id += 1

        request = {"id": request_id, "method": method}
        if params is not None:
            request["params"] = params

        payload = json.dumps(request, separators=(',', ':')).encode('utf-8')
        frame = self._build_frame(payload)

        try:
            self._serial.write(frame)
            self._serial.flush()
        except Exception as e:
            self._logger.debug(f"ACE async write failed: {e}")

        self._logger.debug(f"ACE TX (async): {request}")

    # ---- Heartbeat ----

    def _start_heartbeat(self):
        """Start periodic heartbeat to detect dead connections."""
        if self._heartbeat_timer is not None:
            return
        self._heartbeat_timer = self._reactor.register_timer(
            self._heartbeat_tick,
            self._reactor.monotonic() + HEARTBEAT_INTERVAL,
        )

    def _stop_heartbeat(self):
        """Stop heartbeat timer."""
        if self._heartbeat_timer is not None:
            try:
                self._reactor.unregister_timer(self._heartbeat_timer)
            except Exception:
                pass
            self._heartbeat_timer = None

    def _heartbeat_tick(self, eventtime):
        """Send periodic get_status to verify connection is alive."""
        if not self._connected:
            return self._reactor.NEVER

        # Check if we've received anything recently
        silence = eventtime - self._last_rx_time
        if silence > HEARTBEAT_INTERVAL * 4:
            self._logger.warning(
                f"ACE no data received for {silence:.0f}s, reconnecting"
            )
            self.reconnect()
            return self._reactor.NEVER

        # Send heartbeat status request
        try:
            self.send_command_async("get_status")
        except Exception:
            pass

        # Run health supervision check
        self._supervision_check()

        return eventtime + HEARTBEAT_INTERVAL

    # ---- Health Supervision ----

    def _track_timeout(self):
        """Record a timeout event."""
        now = self._reactor.monotonic()
        self._timeout_timestamps.append(now)
        cutoff = now - COMM_SUPERVISION_WINDOW
        self._timeout_timestamps = [
            t for t in self._timeout_timestamps if t > cutoff
        ]

    def _track_unsolicited(self):
        """Record an unsolicited message event."""
        now = self._reactor.monotonic()
        self._unsolicited_timestamps.append(now)
        cutoff = now - COMM_SUPERVISION_WINDOW
        self._unsolicited_timestamps = [
            t for t in self._unsolicited_timestamps if t > cutoff
        ]

    def _supervision_check(self):
        """Check communication health; force reconnect if degraded."""
        now = self._reactor.monotonic()
        if now - self._last_supervision_check < SUPERVISION_CHECK_INTERVAL:
            return
        self._last_supervision_check = now

        if not self._connected:
            return

        cutoff = now - COMM_SUPERVISION_WINDOW
        self._timeout_timestamps = [
            t for t in self._timeout_timestamps if t > cutoff
        ]
        self._unsolicited_timestamps = [
            t for t in self._unsolicited_timestamps if t > cutoff
        ]

        timeout_count = len(self._timeout_timestamps)
        unsolicited_count = len(self._unsolicited_timestamps)

        if (timeout_count >= COMM_TIMEOUT_THRESHOLD
                and unsolicited_count >= COMM_UNSOLICITED_THRESHOLD):
            self._logger.warning(
                f"ACE communication unhealthy: {timeout_count} timeouts + "
                f"{unsolicited_count} unsolicited in {COMM_SUPERVISION_WINDOW}s, "
                "forcing reconnect"
            )
            self._timeout_timestamps.clear()
            self._unsolicited_timestamps.clear()
            self.reconnect()

    # ---- Frame Protocol ----

    def _build_frame(self, payload: bytes) -> bytes:
        """Build a binary frame wrapping the JSON payload."""
        length = len(payload)
        crc = crc16_ccitt_reflected(payload)
        return (
            FRAME_HEADER
            + struct.pack('<H', length)
            + payload
            + struct.pack('<H', crc)
            + FRAME_FOOTER
        )

    def _handle_read(self, eventtime):
        """Reactor callback invoked when data is available on the serial fd."""
        try:
            data = self._serial.read(4096)
        except Exception as e:
            err_msg = str(e)
            if "returned no data" in err_msg or "disconnected" in err_msg:
                # USB autosuspend - device briefly disappeared.
                # Quick reconnect without backoff since it's not a real failure.
                self._logger.debug(
                    "ACE USB idle disconnect detected, quick reconnect"
                )
                self._reconnect_backoff = RECONNECT_BACKOFF_MIN
                self._quick_reconnect()
            else:
                self._logger.error(f"ACE read error: {e}")
                self.reconnect()
            return

        if not data:
            return

        self._last_rx_time = eventtime
        self._read_buffer += data
        self._parse_frames()

    # Maximum sane payload size to reject false headers with garbage lengths
    _MAX_PAYLOAD = 8192

    def _parse_frames(self):
        """Extract and process complete frames from the read buffer."""
        while True:
            header_pos = self._read_buffer.find(FRAME_HEADER)
            if header_pos < 0:
                self._read_buffer = b''
                return
            if header_pos > 0:
                self._read_buffer = self._read_buffer[header_pos:]

            if len(self._read_buffer) < 7:
                return

            payload_length = struct.unpack_from('<H', self._read_buffer, 2)[0]

            # Reject impossibly large payloads (likely a false header in data)
            if payload_length > self._MAX_PAYLOAD:
                self._logger.debug(
                    f"ACE frame: payload length {payload_length} exceeds max, "
                    "skipping false header"
                )
                self._read_buffer = self._read_buffer[2:]
                continue

            frame_size = 2 + 2 + payload_length + 2 + 1
            if len(self._read_buffer) < frame_size:
                return

            payload = self._read_buffer[4:4 + payload_length]
            crc_received = struct.unpack_from(
                '<H', self._read_buffer, 4 + payload_length
            )[0]
            footer = self._read_buffer[4 + payload_length + 2]

            # Validate footer and CRC BEFORE consuming the frame.
            # On failure, skip past just the header bytes and rescan,
            # since a false header in data would produce garbage framing.
            if footer != FRAME_FOOTER[0]:
                self._logger.debug(
                    f"ACE frame: invalid footer byte 0x{footer:02x}, "
                    "rescanning for next header"
                )
                self._read_buffer = self._read_buffer[2:]
                continue

            crc_calculated = crc16_ccitt_reflected(payload)
            if crc_received != crc_calculated:
                self._logger.debug(
                    f"ACE frame: CRC mismatch (recv=0x{crc_received:04x}, "
                    f"calc=0x{crc_calculated:04x}), rescanning"
                )
                self._read_buffer = self._read_buffer[2:]
                continue

            # Frame is valid - consume it from the buffer
            self._read_buffer = self._read_buffer[frame_size:]

            try:
                response = json.loads(payload.decode('utf-8'))
            except (json.JSONDecodeError, UnicodeDecodeError) as e:
                self._logger.warning(f"ACE frame: JSON parse error: {e}")
                continue

            self._logger.debug(f"ACE RX: {response}")
            self._handle_response(response)

    def _handle_response(self, response: dict):
        """Route a parsed response to its pending request completion."""
        response_id = response.get("id")

        if response_id is None:
            # Unsolicited notification
            self._track_unsolicited()
            if self.status_callback:
                try:
                    self.status_callback(response)
                except Exception as e:
                    self._logger.debug(f"ACE status_callback error: {e}")
            return

        completion = self._pending.get(response_id)
        if completion is not None:
            try:
                completion.complete(response)
            except Exception:
                pass
        else:
            # Heartbeat (send_command_async) responses land here - they have
            # a valid id but no pending completion.  Forward to the status
            # callback so the unit can process slot data in near-real-time.
            self._track_unsolicited()
            if self.status_callback:
                try:
                    self.status_callback(response)
                except Exception as e:
                    self._logger.debug(f"ACE status_callback error: {e}")
            self._logger.debug(
                f"ACE response for unknown request id={response_id}"
            )

    # ---- High-Level Hardware Commands ----

    def get_status(self, timeout=3.0):
        """Query current device status (temperatures, sensors, slots)."""
        return self.send_command("get_status", timeout=timeout)

    def get_filament_info(self, slot_index, timeout=3.0):
        """Get RFID/NFC filament data for a specific slot (0-based)."""
        return self.send_command(
            "get_filament_info",
            params={"index": slot_index},
            timeout=timeout,
        )

    def feed_filament(self, slot_index, length_mm, speed_mm_s):
        """Feed filament forward from the specified slot.

        :param slot_index: Slot index (0-based)
        :param length_mm: Distance to feed in mm
        :param speed_mm_s: Feed speed in mm/s (ACE firmware native unit)
        """
        return self.send_command(
            "feed_filament",
            params={
                "index": slot_index,
                "length": length_mm,
                "speed": speed_mm_s,
            },
            timeout=max(REQUEST_TIMEOUT, (length_mm / max(speed_mm_s, 1)) + 10),
        )

    def stop_feed_filament(self, slot_index):
        """Stop an active feed operation on the specified slot."""
        self.send_command_async(
            "stop_feed_filament", params={"index": slot_index}
        )

    def unwind_filament(self, slot_index, length_mm, speed_mm_s, mode="normal"):
        """Retract/unwind filament back into the specified slot.

        :param slot_index: Slot index (0-based)
        :param length_mm: Distance to retract in mm
        :param speed_mm_s: Retract speed in mm/s (ACE firmware native unit)
        :param mode: Unwind mode ("normal" or other firmware-supported modes)
        """
        return self.send_command(
            "unwind_filament",
            params={
                "index": slot_index,
                "length": length_mm,
                "speed": speed_mm_s,
                "mode": mode,
            },
            timeout=max(REQUEST_TIMEOUT, (length_mm / max(speed_mm_s, 1)) + 10),
        )

    def stop_unwind_filament(self, slot_index):
        """Stop an active unwind/retract operation."""
        self.send_command_async(
            "stop_unwind_filament", params={"index": slot_index}
        )

    def start_feed_assist(self, slot_index):
        """Enable continuous motorized feed assist for a slot."""
        return self.send_command(
            "start_feed_assist", params={"index": slot_index}
        )

    def stop_feed_assist(self, slot_index):
        """Disable feed assist for a slot."""
        self.send_command_async(
            "stop_feed_assist", params={"index": slot_index}
        )

    def update_feeding_speed(self, slot_index, speed_mm_min):
        """Update the speed of an active feed operation."""
        self.send_command_async(
            "update_feeding_speed",
            params={"index": slot_index, "speed": speed_mm_min},
        )

    def update_unwinding_speed(self, slot_index, speed_mm_min):
        """Update the speed of an active unwind operation."""
        self.send_command_async(
            "update_unwinding_speed",
            params={"index": slot_index, "speed": speed_mm_min},
        )

    def start_drying(self, temp_c, fan_speed_rpm, duration_min):
        """Start a filament drying cycle."""
        return self.send_command(
            "drying",
            params={
                "temp": temp_c,
                "fan_speed": fan_speed_rpm,
                "duration": duration_min,
            },
        )

    def stop_drying(self):
        """Stop an active drying cycle."""
        return self.send_command("drying_stop")

    def enable_rfid(self):
        """Enable RFID/NFC tag detection."""
        return self.send_command("enable_rfid")

    def disable_rfid(self):
        """Disable RFID/NFC tag detection."""
        return self.send_command("disable_rfid")