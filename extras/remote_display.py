#!/usr/bin/env python3
# Remote Display — serves the printer's physical display over HTTP
# with touch input support.  Appears as a camera in Mainsail/Fluidd.
#
# Configuration:
#   [remote_display]
#   port: 8092
#   bind: 0.0.0.0
#   display_backend: auto          # auto, drm, or fbdev
#   drm_device:                    # auto-detect, or e.g. /dev/dri/card2
#   framebuffer_device: /dev/fb0   # only used when backend is fbdev
#   touch_device:                  # blank = auto-detect the touchscreen by
#                                  # capability (recommended). Override with a
#                                  # stable path if needed, e.g.
#                                  # /dev/input/by-path/platform-XXXX.i2c-event
#                                  # (a bare /dev/input/eventN can renumber).
#   stream_fps: 5
#
# Provenance / license
# ---------------------
# This module is ADAPTED from `fb-http.py`, the remote-screen tool that ships in
# the Snapmaker U1 "PAXX" Extended Firmware (where it serves the U1's on-device
# display over HTTP at 127.0.0.1:8092):
#
#     https://github.com/paxx12-snapmaker-u1/SnapmakerU1-Extended-Firmware
#     Copyright (c) paxx12  —  GNU General Public License v3.0
#
# Seeing that tool on the U1 is what prompted this. It reuses fb-http.py's
# framebuffer/DRM screen capture, MJPEG streaming, and evdev touch-input code,
# repackaged as a standalone Klipper `[remote_display]` object so the same
# feature works on any Klipper host, not just the U1. Changes here: wrapped as a
# Klipper config object (config options, printer event hooks, gcode STATUS
# command) and touchscreen auto-detection by evdev capability.
#
# Because it is a derivative work of GPL-3.0 code, this module is likewise
# distributed under the GNU General Public License v3.0. Original tool and design
# credit: paxx12 / SnapmakerU1-Extended-Firmware.
#
# (The low-level DRM/fbdev/evdev ioctl constants both files use are Linux kernel
# UAPI; frame encoding uses Pillow; the HTTP/MJPEG server uses the Python
# standard library.)

import ctypes
import fcntl
import glob
import hashlib
import logging
import mmap
import os
import struct
import threading
import chelper
import time
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from io import BytesIO
from urllib.parse import urlparse, parse_qs


FBIOGET_VSCREENINFO = 0x4600
FBIOGET_FSCREENINFO = 0x4602

EV_SYN = 0x00
EV_KEY = 0x01
EV_ABS = 0x03
SYN_REPORT = 0x00
BTN_TOUCH = 0x14A
ABS_X = 0x00
ABS_Y = 0x01
ABS_MT_SLOT = 0x2F
ABS_MT_TRACKING_ID = 0x39
ABS_MT_POSITION_X = 0x35
ABS_MT_POSITION_Y = 0x36

MJPEG_BOUNDARY = b"frameboundary"

# ── Touchscreen auto-detection (self-healing device resolution) ──────
# Query evdev capabilities so we can find the touchscreen by what it IS,
# not by a hardcoded eventN that can renumber across reboots/driver loads.
INPUT_PROP_DIRECT = 0x01

def _eviocgbit(ev_type, length):
    # EVIOCGBIT(ev_type, len): read the code bitmap for an event type.
    return (2 << 30) | (length << 16) | (0x45 << 8) | (0x20 + ev_type)

def _eviocgprop(length):
    # EVIOCGPROP(len): read the INPUT_PROP_* bitmap.
    return (2 << 30) | (length << 16) | (0x45 << 8) | 0x09

def _eviocgname(length):
    # EVIOCGNAME(len): read the device name.
    return (2 << 30) | (length << 16) | (0x45 << 8) | 0x06

def _test_bit(bitmap, bit):
    idx = bit // 8
    return idx < len(bitmap) and bool(bitmap[idx] & (1 << (bit % 8)))

def _device_is_touchscreen(path):
    """Return (is_touchscreen, name_str) by querying evdev capabilities."""
    try:
        fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
    except OSError:
        return False, None
    try:
        name = None
        try:
            buf = bytearray(256)
            fcntl.ioctl(fd, _eviocgname(len(buf)), buf)
            name = bytes(buf).split(b'\x00', 1)[0].decode('utf-8', 'replace')
        except OSError:
            pass
        abs_bits = bytearray(8)   # ABS_CNT (0x40) bits
        try:
            fcntl.ioctl(fd, _eviocgbit(EV_ABS, len(abs_bits)), abs_bits)
        except OSError:
            return False, name
        has_xy = (_test_bit(abs_bits, ABS_MT_POSITION_X)
                  or (_test_bit(abs_bits, ABS_X) and _test_bit(abs_bits, ABS_Y)))
        if not has_xy:
            return False, name
        # Confirm it's a touch surface, not a joystick/tablet: BTN_TOUCH or
        # INPUT_PROP_DIRECT.
        key_bits = bytearray(96)
        has_touch = False
        try:
            fcntl.ioctl(fd, _eviocgbit(EV_KEY, len(key_bits)), key_bits)
            has_touch = _test_bit(key_bits, BTN_TOUCH)
        except OSError:
            pass
        prop_bits = bytearray(4)
        is_direct = False
        try:
            fcntl.ioctl(fd, _eviocgprop(len(prop_bits)), prop_bits)
            is_direct = _test_bit(prop_bits, INPUT_PROP_DIRECT)
        except OSError:
            pass
        return (has_touch or is_direct), name
    finally:
        os.close(fd)

def _prefer_by_path(dev):
    """Return a stable /dev/input/by-path symlink for dev, if one exists, so
    the resolved path survives eventN renumbering."""
    bp_dir = '/dev/input/by-path'
    try:
        target = os.path.realpath(dev)
        for link in sorted(os.listdir(bp_dir)):
            full = os.path.join(bp_dir, link)
            try:
                if os.path.realpath(full) == target:
                    return full
            except OSError:
                pass
    except OSError:
        pass
    return dev

def _find_touchscreen():
    """Scan /dev/input/event* for a touchscreen; return (stable_path, name)
    or (None, None)."""
    for dev in sorted(glob.glob('/dev/input/event*')):
        ok, name = _device_is_touchscreen(dev)
        if ok:
            return _prefer_by_path(dev), name
    return None, None

# ── DRM/KMS constants ──────────────────────────────────────────────

def _iowr(type_val, nr, size):
    return (3 << 30) | (size << 16) | (type_val << 8) | nr

def _iow(type_val, nr, size):
    return (1 << 30) | (size << 16) | (type_val << 8) | nr

_DRM_IOCTL_MAP_DUMB = _iowr(0x64, 0xB3, 16)
_DRM_IOCTL_GEM_CLOSE = _iow(0x64, 0x09, 8)


class _DrmMapDumb(ctypes.Structure):
    _fields_ = [('handle', ctypes.c_uint32), ('pad', ctypes.c_uint32),
                ('offset', ctypes.c_uint64)]


class _DrmGemClose(ctypes.Structure):
    _fields_ = [('handle', ctypes.c_uint32), ('pad', ctypes.c_uint32)]


class _DrmModeModeInfo(ctypes.Structure):
    _fields_ = [
        ('clock', ctypes.c_uint32),
        ('hdisplay', ctypes.c_uint16), ('hsync_start', ctypes.c_uint16),
        ('hsync_end', ctypes.c_uint16), ('htotal', ctypes.c_uint16),
        ('hskew', ctypes.c_uint16), ('vdisplay', ctypes.c_uint16),
        ('vsync_start', ctypes.c_uint16), ('vsync_end', ctypes.c_uint16),
        ('vtotal', ctypes.c_uint16), ('vscan', ctypes.c_uint16),
        ('vrefresh', ctypes.c_uint32), ('flags', ctypes.c_uint32),
        ('type', ctypes.c_uint32), ('name', ctypes.c_char * 32)]


class _DrmModeRes(ctypes.Structure):
    _fields_ = [
        ('count_fbs', ctypes.c_int),
        ('fbs', ctypes.POINTER(ctypes.c_uint32)),
        ('count_crtcs', ctypes.c_int),
        ('crtcs', ctypes.POINTER(ctypes.c_uint32)),
        ('count_connectors', ctypes.c_int),
        ('connectors', ctypes.POINTER(ctypes.c_uint32)),
        ('count_encoders', ctypes.c_int),
        ('encoders', ctypes.POINTER(ctypes.c_uint32)),
        ('min_width', ctypes.c_uint32), ('max_width', ctypes.c_uint32),
        ('min_height', ctypes.c_uint32), ('max_height', ctypes.c_uint32)]


class _DrmModeCrtc(ctypes.Structure):
    _fields_ = [
        ('crtc_id', ctypes.c_uint32), ('buffer_id', ctypes.c_uint32),
        ('x', ctypes.c_uint32), ('y', ctypes.c_uint32),
        ('width', ctypes.c_uint32), ('height', ctypes.c_uint32),
        ('mode_valid', ctypes.c_int), ('mode', _DrmModeModeInfo),
        ('gamma_size', ctypes.c_int)]


class _DrmModeFB(ctypes.Structure):
    _fields_ = [
        ('fb_id', ctypes.c_uint32),
        ('width', ctypes.c_uint32), ('height', ctypes.c_uint32),
        ('pitch', ctypes.c_uint32), ('bpp', ctypes.c_uint32),
        ('depth', ctypes.c_uint32), ('handle', ctypes.c_uint32)]


class FbVarScreeninfo(ctypes.Structure):
    _fields_ = [
        ("xres", ctypes.c_uint32),
        ("yres", ctypes.c_uint32),
        ("xres_virtual", ctypes.c_uint32),
        ("yres_virtual", ctypes.c_uint32),
        ("xoffset", ctypes.c_uint32),
        ("yoffset", ctypes.c_uint32),
        ("bits_per_pixel", ctypes.c_uint32),
        ("grayscale", ctypes.c_uint32),
        ("red", ctypes.c_uint32 * 3),
        ("green", ctypes.c_uint32 * 3),
        ("blue", ctypes.c_uint32 * 3),
        ("transp", ctypes.c_uint32 * 3),
        ("nonstd", ctypes.c_uint32),
        ("activate", ctypes.c_uint32),
        ("height", ctypes.c_uint32),
        ("width", ctypes.c_uint32),
        ("accel_flags", ctypes.c_uint32),
        ("pixclock", ctypes.c_uint32),
        ("left_margin", ctypes.c_uint32),
        ("right_margin", ctypes.c_uint32),
        ("upper_margin", ctypes.c_uint32),
        ("lower_margin", ctypes.c_uint32),
        ("hsync_len", ctypes.c_uint32),
        ("vsync_len", ctypes.c_uint32),
        ("sync", ctypes.c_uint32),
        ("vmode", ctypes.c_uint32),
        ("rotate", ctypes.c_uint32),
        ("colorspace", ctypes.c_uint32),
        ("reserved", ctypes.c_uint32 * 4),
    ]


class FbFixScreeninfo(ctypes.Structure):
    _fields_ = [
        ("id", ctypes.c_char * 16),
        ("smem_start", ctypes.c_ulong),
        ("smem_len", ctypes.c_uint32),
        ("type", ctypes.c_uint32),
        ("type_aux", ctypes.c_uint32),
        ("visual", ctypes.c_uint32),
        ("xpanstep", ctypes.c_uint16),
        ("ypanstep", ctypes.c_uint16),
        ("ywrapstep", ctypes.c_uint16),
        ("line_length", ctypes.c_uint32),
        ("mmio_start", ctypes.c_ulong),
        ("mmio_len", ctypes.c_uint32),
        ("accel", ctypes.c_uint32),
        ("capabilities", ctypes.c_uint16),
        ("reserved", ctypes.c_uint16 * 2),
    ]


class Framebuffer:
    def __init__(self, device):
        self.device = device
        self.fd = None
        self.mm = None
        self.width = 0
        self.height = 0
        self.bpp = 0
        self.line_length = 0
        self._virtual_height = 0
        self._cache_hash = None
        self._cache_jpeg = None
        self._cache_png = None
        self._lock = threading.Lock()

    def open(self):
        self.fd = os.open(self.device, os.O_RDONLY)
        vinfo = FbVarScreeninfo()
        fcntl.ioctl(self.fd, FBIOGET_VSCREENINFO, vinfo)
        finfo = FbFixScreeninfo()
        fcntl.ioctl(self.fd, FBIOGET_FSCREENINFO, finfo)
        self.width = vinfo.xres
        self.height = vinfo.yres
        self._virtual_height = vinfo.yres_virtual
        self.bpp = vinfo.bits_per_pixel
        self.line_length = finfo.line_length
        size = self.line_length * self._virtual_height
        self.mm = mmap.mmap(self.fd, size, mmap.MAP_SHARED, mmap.PROT_READ)

    def _read_raw(self):
        vinfo = FbVarScreeninfo()
        fcntl.ioctl(self.fd, FBIOGET_VSCREENINFO, vinfo)
        offset = vinfo.yoffset * self.line_length
        self.mm.seek(offset)
        return self.mm.read(self.line_length * self.height)

    def _raw_to_image(self, raw):
        try:
            from PIL import Image
        except ImportError:
            raise RuntimeError(
                "remote_display requires Pillow. "
                "Install with: pip install Pillow")
        if self.bpp == 32:
            img = Image.frombytes(
                'RGBA', (self.width, self.height), raw,
                'raw', 'BGRA', self.line_length)
            return img.convert('RGB')
        elif self.bpp == 16:
            return Image.frombytes(
                'RGB', (self.width, self.height), raw,
                'raw', 'BGR;16', self.line_length)
        return Image.frombytes(
            'RGB', (self.width, self.height), raw,
            'raw', 'BGR', self.line_length)

    def get_snapshot_png(self, client_etag=None):
        with self._lock:
            raw = self._read_raw()
            raw_hash = hashlib.md5(raw).hexdigest()[:16]
            if client_etag and client_etag == raw_hash:
                return raw_hash, None
            if raw_hash == self._cache_hash and self._cache_png:
                return raw_hash, self._cache_png
            img = self._raw_to_image(raw)
            buf = BytesIO()
            img.save(buf, 'PNG', compress_level=6)
            self._cache_hash = raw_hash
            self._cache_png = buf.getvalue()
            self._cache_jpeg = None
            return raw_hash, self._cache_png

    def get_snapshot_jpeg(self, quality=80):
        with self._lock:
            raw = self._read_raw()
            raw_hash = hashlib.md5(raw).hexdigest()[:16]
            if raw_hash == self._cache_hash and self._cache_jpeg:
                return raw_hash, self._cache_jpeg
            img = self._raw_to_image(raw)
            buf = BytesIO()
            img.save(buf, 'JPEG', quality=quality)
            self._cache_hash = raw_hash
            self._cache_jpeg = buf.getvalue()
            self._cache_png = None
            return raw_hash, self._cache_jpeg

    def close(self):
        if self.mm:
            self.mm.close()
            self.mm = None
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None


class DRMFramebuffer:
    """Capture display output via DRM/KMS for apps that bypass /dev/fb0."""

    def __init__(self, device=None):
        self.device = device
        self.fd = None
        self.width = 0
        self.height = 0
        self.bpp = 0
        self.line_length = 0
        self._crtc_id = None
        self._fb_cache = {}
        self._drm = None
        self._cache_hash = None
        self._cache_jpeg = None
        self._cache_png = None
        self._lock = threading.Lock()

    def open(self):
        try:
            self._drm = ctypes.CDLL('libdrm.so.2')
        except OSError:
            raise RuntimeError(
                "libdrm not found — install with: sudo apt install libdrm2")
        self._setup_libdrm()
        if self.device:
            self._try_open(self.device)
        else:
            for i in range(8):
                path = f'/dev/dri/card{i}'
                if not os.path.exists(path):
                    continue
                try:
                    self._try_open(path)
                    self.device = path
                    return
                except Exception:
                    if self.fd is not None:
                        os.close(self.fd)
                        self.fd = None
            raise RuntimeError(
                "No active DRM display found. "
                "Set drm_device in [remote_display] config.")

    def _setup_libdrm(self):
        d = self._drm
        d.drmModeGetResources.restype = ctypes.POINTER(_DrmModeRes)
        d.drmModeGetResources.argtypes = [ctypes.c_int]
        d.drmModeFreeResources.argtypes = [ctypes.c_void_p]
        d.drmModeGetCrtc.restype = ctypes.POINTER(_DrmModeCrtc)
        d.drmModeGetCrtc.argtypes = [ctypes.c_int, ctypes.c_uint32]
        d.drmModeFreeCrtc.argtypes = [ctypes.c_void_p]
        d.drmModeGetFB.restype = ctypes.POINTER(_DrmModeFB)
        d.drmModeGetFB.argtypes = [ctypes.c_int, ctypes.c_uint32]
        d.drmModeFreeFB.argtypes = [ctypes.c_void_p]

    def _try_open(self, path):
        self.fd = os.open(path, os.O_RDWR)
        res = self._drm.drmModeGetResources(self.fd)
        if not res:
            raise RuntimeError(f"drmModeGetResources failed on {path}")
        try:
            for i in range(res.contents.count_crtcs):
                crtc_id = res.contents.crtcs[i]
                crtc = self._drm.drmModeGetCrtc(self.fd, crtc_id)
                if not crtc:
                    continue
                try:
                    if crtc.contents.buffer_id and crtc.contents.mode_valid:
                        self._crtc_id = crtc_id
                        self.width = crtc.contents.width
                        self.height = crtc.contents.height
                        fb = self._drm.drmModeGetFB(
                            self.fd, crtc.contents.buffer_id)
                        if fb:
                            self.bpp = fb.contents.bpp
                            self.line_length = fb.contents.pitch
                            if not fb.contents.handle:
                                self._drm.drmModeFreeFB(fb)
                                raise RuntimeError(
                                    "DRM buffer handle not accessible. "
                                    "Grant CAP_SYS_ADMIN to Klipper: "
                                    "add 'AmbientCapabilities="
                                    "CAP_SYS_ADMIN' to "
                                    "[Service] in klipper.service")
                            self._drm.drmModeFreeFB(fb)
                        return
                finally:
                    self._drm.drmModeFreeCrtc(crtc)
        finally:
            self._drm.drmModeFreeResources(res)
        raise RuntimeError(f"No active CRTC on {path}")

    def _get_fb_buffer(self):
        crtc = self._drm.drmModeGetCrtc(self.fd, self._crtc_id)
        if not crtc:
            raise RuntimeError("Failed to get CRTC")
        try:
            fb_id = crtc.contents.buffer_id
        finally:
            self._drm.drmModeFreeCrtc(crtc)
        if not fb_id:
            raise RuntimeError("No framebuffer on CRTC")
        if fb_id in self._fb_cache:
            entry = self._fb_cache[fb_id]
            self.line_length = entry[1]
            self.bpp = entry[2]
            return entry[0], entry[3]
        fb = self._drm.drmModeGetFB(self.fd, fb_id)
        if not fb:
            raise RuntimeError("Failed to get FB")
        try:
            handle = fb.contents.handle
            pitch = fb.contents.pitch
            h = fb.contents.height
            bpp = fb.contents.bpp
            size = pitch * h
            if not handle:
                raise RuntimeError("DRM buffer handle=0 (no permission)")
            req = _DrmMapDumb()
            req.handle = handle
            req.pad = 0
            req.offset = 0
            fcntl.ioctl(self.fd, _DRM_IOCTL_MAP_DUMB, req)
            mm = mmap.mmap(self.fd, size, mmap.MAP_SHARED, mmap.PROT_READ,
                           offset=req.offset)
            self._fb_cache[fb_id] = (mm, pitch, bpp, size, handle)
            self.line_length = pitch
            self.bpp = bpp
            if len(self._fb_cache) > 4:
                old_id = next(iter(self._fb_cache))
                old = self._fb_cache.pop(old_id)
                try:
                    old[0].close()
                except Exception:
                    pass
                self._close_gem(old[4])
            return mm, size
        finally:
            self._drm.drmModeFreeFB(fb)

    def _close_gem(self, handle):
        try:
            req = _DrmGemClose()
            req.handle = handle
            req.pad = 0
            fcntl.ioctl(self.fd, _DRM_IOCTL_GEM_CLOSE, req)
        except Exception:
            pass

    def _read_raw(self):
        mm, size = self._get_fb_buffer()
        mm.seek(0)
        return mm.read(size)

    def _raw_to_image(self, raw):
        try:
            from PIL import Image
        except ImportError:
            raise RuntimeError(
                "remote_display requires Pillow. "
                "Install with: pip install Pillow")
        if self.bpp == 32:
            img = Image.frombytes(
                'RGBA', (self.width, self.height), raw,
                'raw', 'BGRA', self.line_length)
            return img.convert('RGB')
        elif self.bpp == 16:
            return Image.frombytes(
                'RGB', (self.width, self.height), raw,
                'raw', 'BGR;16', self.line_length)
        return Image.frombytes(
            'RGB', (self.width, self.height), raw,
            'raw', 'BGR', self.line_length)

    def get_snapshot_png(self, client_etag=None):
        with self._lock:
            raw = self._read_raw()
            raw_hash = hashlib.md5(raw).hexdigest()[:16]
            if client_etag and client_etag == raw_hash:
                return raw_hash, None
            if raw_hash == self._cache_hash and self._cache_png:
                return raw_hash, self._cache_png
            img = self._raw_to_image(raw)
            buf = BytesIO()
            img.save(buf, 'PNG', compress_level=6)
            self._cache_hash = raw_hash
            self._cache_png = buf.getvalue()
            self._cache_jpeg = None
            return raw_hash, self._cache_png

    def get_snapshot_jpeg(self, quality=80):
        with self._lock:
            raw = self._read_raw()
            raw_hash = hashlib.md5(raw).hexdigest()[:16]
            if raw_hash == self._cache_hash and self._cache_jpeg:
                return raw_hash, self._cache_jpeg
            img = self._raw_to_image(raw)
            buf = BytesIO()
            img.save(buf, 'JPEG', quality=quality)
            self._cache_hash = raw_hash
            self._cache_jpeg = buf.getvalue()
            self._cache_png = None
            return raw_hash, self._cache_jpeg

    def close(self):
        for entry in self._fb_cache.values():
            try:
                entry[0].close()
            except Exception:
                pass
            self._close_gem(entry[4])
        self._fb_cache.clear()
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None


class TouchInput:
    def __init__(self, device, fb_width, fb_height):
        self.device = device
        self.fd = None
        self.fb_width = fb_width
        self.fb_height = fb_height
        self.touch_max_x = fb_width
        self.touch_max_y = fb_height

    def open(self):
        # Prefer O_RDWR (some stacks want it), fall back to O_WRONLY.
        last_err = None
        for flags in (os.O_RDWR, os.O_WRONLY):
            try:
                self.fd = os.open(self.device, flags)
                last_err = None
                break
            except OSError as e:
                last_err = e
        if last_err is not None:
            raise last_err
        self._detect_range()

    def _detect_range(self):
        def eviocgabs(axis):
            return 0x80184540 + axis
        try:
            buf = bytearray(24)
            fcntl.ioctl(self.fd, eviocgabs(ABS_MT_POSITION_X), buf)
            self.touch_max_x = struct.unpack('iiiii', buf[:20])[2]
            fcntl.ioctl(self.fd, eviocgabs(ABS_MT_POSITION_Y), buf)
            self.touch_max_y = struct.unpack('iiiii', buf[:20])[2]
        except OSError:
            try:
                buf = bytearray(24)
                fcntl.ioctl(self.fd, eviocgabs(ABS_X), buf)
                self.touch_max_x = struct.unpack('iiiii', buf[:20])[2]
                fcntl.ioctl(self.fd, eviocgabs(ABS_Y), buf)
                self.touch_max_y = struct.unpack('iiiii', buf[:20])[2]
            except OSError:
                pass

    def _write_event(self, ev_type, code, value):
        if self.fd is None:
            return
        tv_sec = int(time.time())
        tv_usec = int((time.time() % 1) * 1_000_000)
        os.write(self.fd, struct.pack('llHHi', tv_sec, tv_usec,
                                      ev_type, code, value))

    def _scale(self, x, y):
        return (int(x * self.touch_max_x / self.fb_width),
                int(y * self.touch_max_y / self.fb_height))

    def tap(self, x, y):
        if self.fd is None:
            return
        tx, ty = self._scale(x, y)
        self._write_event(EV_ABS, ABS_MT_SLOT, 0)
        self._write_event(EV_ABS, ABS_MT_TRACKING_ID, 1)
        self._write_event(EV_ABS, ABS_MT_POSITION_X, tx)
        self._write_event(EV_ABS, ABS_MT_POSITION_Y, ty)
        self._write_event(EV_KEY, BTN_TOUCH, 1)
        self._write_event(EV_SYN, SYN_REPORT, 0)
        time.sleep(0.05)
        self._write_event(EV_ABS, ABS_MT_TRACKING_ID, -1)
        self._write_event(EV_KEY, BTN_TOUCH, 0)
        self._write_event(EV_SYN, SYN_REPORT, 0)

    def touch_down(self, x, y):
        if self.fd is None:
            return
        tx, ty = self._scale(x, y)
        self._write_event(EV_ABS, ABS_MT_SLOT, 0)
        self._write_event(EV_ABS, ABS_MT_TRACKING_ID, 1)
        self._write_event(EV_ABS, ABS_MT_POSITION_X, tx)
        self._write_event(EV_ABS, ABS_MT_POSITION_Y, ty)
        self._write_event(EV_KEY, BTN_TOUCH, 1)
        self._write_event(EV_SYN, SYN_REPORT, 0)

    def touch_move(self, x, y):
        if self.fd is None:
            return
        tx, ty = self._scale(x, y)
        self._write_event(EV_ABS, ABS_MT_POSITION_X, tx)
        self._write_event(EV_ABS, ABS_MT_POSITION_Y, ty)
        self._write_event(EV_SYN, SYN_REPORT, 0)

    def touch_up(self):
        if self.fd is None:
            return
        self._write_event(EV_ABS, ABS_MT_TRACKING_ID, -1)
        self._write_event(EV_KEY, BTN_TOUCH, 0)
        self._write_event(EV_SYN, SYN_REPORT, 0)

    def close(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None


def _make_handler(fb, touch, stream_fps, logger):
    class DisplayHandler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            pass

        def do_GET(self):
            path = urlparse(self.path).path
            if path == '/snapshot' or path == '/snapshot.png':
                self._handle_snapshot_png()
            elif path == '/snapshot.jpg':
                self._handle_snapshot_jpg()
            elif path == '/stream' or path == '/stream.mjpg':
                self._handle_mjpeg_stream()
            elif path == '/' or path == '/viewer':
                self._handle_viewer()
            else:
                self.send_error(404)

        def do_POST(self):
            parsed = urlparse(self.path)
            if parsed.path == '/touch':
                self._handle_touch(parsed.query)
            else:
                self.send_error(404)

        def _handle_snapshot_png(self):
            try:
                client_etag = self.headers.get('If-None-Match', '').strip('"')
                etag, data = fb.get_snapshot_png(client_etag)
                if data is None:
                    self.send_response(304)
                    self.send_header('ETag', f'"{etag}"')
                    self.end_headers()
                    return
                self.send_response(200)
                self.send_header('Content-Type', 'image/png')
                self.send_header('Content-Length', len(data))
                self.send_header('ETag', f'"{etag}"')
                self.send_header('Cache-Control', 'no-cache')
                self.end_headers()
                self.wfile.write(data)
            except Exception as e:
                logger.debug(f"Snapshot error: {e}")
                self.send_error(500)

        def _handle_snapshot_jpg(self):
            try:
                _, data = fb.get_snapshot_jpeg()
                self.send_response(200)
                self.send_header('Content-Type', 'image/jpeg')
                self.send_header('Content-Length', len(data))
                self.send_header('Cache-Control',
                                 'no-cache, no-store, must-revalidate')
                self.end_headers()
                self.wfile.write(data)
            except Exception as e:
                logger.debug(f"JPEG snapshot error: {e}")
                self.send_error(500)

        def _handle_mjpeg_stream(self):
            self.send_response(200)
            self.send_header(
                'Content-Type',
                'multipart/x-mixed-replace; boundary=' +
                MJPEG_BOUNDARY.decode())
            self.send_header('Cache-Control',
                             'no-cache, no-store, must-revalidate')
            self.end_headers()
            interval = 1.0 / max(stream_fps, 1)
            last_hash = None
            try:
                while True:
                    etag, data = fb.get_snapshot_jpeg(quality=70)
                    if data is not None and etag != last_hash:
                        header = (
                            b"--" + MJPEG_BOUNDARY + b"\r\n"
                            b"Content-Type: image/jpeg\r\n"
                            b"Content-Length: " +
                            str(len(data)).encode() + b"\r\n\r\n"
                        )
                        self.wfile.write(header)
                        self.wfile.write(data)
                        self.wfile.write(b"\r\n")
                        self.wfile.flush()
                        last_hash = etag
                    time.sleep(interval)
            except (BrokenPipeError, ConnectionResetError):
                pass
            except Exception as e:
                logger.debug(f"MJPEG stream ended: {e}")

        def _handle_touch(self, query):
            if touch is None or touch.fd is None:
                resp = b'{"status":"error","message":"touch not available"}'
                self.send_response(503)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', len(resp))
                self.end_headers()
                self.wfile.write(resp)
                return
            try:
                params = parse_qs(query)
                action = params.get('a', ['tap'])[0]
                x = int(params.get('x', [0])[0])
                y = int(params.get('y', [0])[0])
                if action == 'down':
                    touch.touch_down(x, y)
                elif action == 'move':
                    touch.touch_move(x, y)
                elif action == 'up':
                    touch.touch_up()
                else:
                    touch.tap(x, y)
                resp = (f'{{"status":"ok","a":"{action}",'
                        f'"x":{x},"y":{y}}}').encode()
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', len(resp))
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(resp)
            except Exception as e:
                resp = f'{{"status":"error","message":"{e}"}}'.encode()
                self.send_response(500)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', len(resp))
                self.end_headers()
                self.wfile.write(resp)

        def _handle_viewer(self):
            html = VIEWER_HTML.replace(
                '{{WIDTH}}', str(fb.width)).replace(
                '{{HEIGHT}}', str(fb.height))
            data = html.encode()
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', len(data))
            self.end_headers()
            self.wfile.write(data)

    return DisplayHandler


VIEWER_HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Remote Display</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{background:#1a1a1a;color:#eee;font-family:system-ui,sans-serif;
  display:flex;flex-direction:column;align-items:center;height:100vh;
  overflow:hidden}
#bar{width:100%;padding:6px 16px;background:#222;display:flex;
  align-items:center;justify-content:space-between;font-size:13px;
  border-bottom:1px solid #333;flex-shrink:0}
#bar .title{font-weight:600;font-size:15px}
#bar .status{color:#888}
#bar .status.connected{color:#4caf50}
body.iframe #bar{display:none}
#wrap{flex:1;display:flex;align-items:center;justify-content:center;
  width:100%;padding:8px;overflow:hidden}
body.iframe #wrap{padding:0}
#screen{cursor:crosshair;max-width:100%;max-height:100%;
  image-rendering:auto;border:1px solid #333;display:block}
body.iframe #screen{border:none;width:100%;height:100%;object-fit:contain}
#touch-indicator{position:fixed;width:24px;height:24px;border-radius:50%;
  background:rgba(255,100,100,0.6);border:2px solid rgba(255,100,100,0.9);
  pointer-events:none;transform:translate(-50%,-50%);display:none;z-index:99}
</style>
</head>
<body>
<div id="bar">
  <span class="title">Remote Display</span>
  <span id="info">{{WIDTH}}&times;{{HEIGHT}}</span>
  <span id="status" class="status">connecting&hellip;</span>
</div>
<div id="wrap">
  <img id="screen" draggable="false">
</div>
<div id="touch-indicator"></div>
<script>
(function(){
  var inIframe = false;
  try { inIframe = window.self !== window.top; } catch(e) { inIframe = true; }
  if (inIframe || window.location.search.indexOf('embed') !== -1) document.body.classList.add('iframe');

  var base = '';
  var loc = window.location.pathname;
  if (loc && loc.length > 1) {
    base = loc.replace(/\/[^\/]*$/, '') + '/';
    if (base === '/') base = '';
  }

  const img = document.getElementById('screen');
  const status = document.getElementById('status');
  const indicator = document.getElementById('touch-indicator');
  const FB_W = parseInt('{{WIDTH}}') || 1024;
  const FB_H = parseInt('{{HEIGHT}}') || 600;
  let etag = '';
  let polling = true;
  let errorCount = 0;
  let dragging = false;

  function setStatus(text, cls) {
    status.textContent = text;
    status.className = 'status ' + (cls || '');
  }

  function poll() {
    if (!polling) return;
    const headers = {};
    if (etag) headers['If-None-Match'] = '"' + etag + '"';
    fetch('/snapshot', { headers })
      .then(resp => {
        if (resp.status === 304) {
          errorCount = 0;
          setStatus('live', 'connected');
          setTimeout(poll, 200);
          return;
        }
        if (!resp.ok) throw new Error(resp.status);
        const newEtag = (resp.headers.get('ETag') || '').replace(/"/g, '');
        if (newEtag) etag = newEtag;
        return resp.blob();
      })
      .then(blob => {
        if (!blob) return;
        const url = URL.createObjectURL(blob);
        const prev = img.src;
        img.src = url;
        if (prev && prev.startsWith('blob:')) URL.revokeObjectURL(prev);
        errorCount = 0;
        setStatus('live', 'connected');
        setTimeout(poll, 100);
      })
      .catch(e => {
        errorCount++;
        setTimeout(poll, Math.min(errorCount * 500, 5000));
      });
  }

  function imgCoords(e) {
    const rect = img.getBoundingClientRect();
    const scaleX = FB_W / rect.width;
    const scaleY = FB_H / rect.height;
    return {
      x: Math.round((e.clientX - rect.left) * scaleX),
      y: Math.round((e.clientY - rect.top) * scaleY)
    };
  }

  function sendTouch(action, x, y) {
    fetch('/touch?a=' + action + '&x=' + x + '&y=' + y, { method: 'POST' })
      .catch(() => {});
  }

  function showIndicator(clientX, clientY) {
    indicator.style.left = clientX + 'px';
    indicator.style.top = clientY + 'px';
    indicator.style.display = 'block';
  }

  function hideIndicator() {
    indicator.style.display = 'none';
  }

  img.addEventListener('mousedown', function(e) {
    e.preventDefault();
    const c = imgCoords(e);
    dragging = true;
    sendTouch('down', c.x, c.y);
    showIndicator(e.clientX, e.clientY);
  });

  img.addEventListener('mousemove', function(e) {
    if (!dragging) return;
    const c = imgCoords(e);
    sendTouch('move', c.x, c.y);
    showIndicator(e.clientX, e.clientY);
  });

  window.addEventListener('mouseup', function(e) {
    if (!dragging) return;
    dragging = false;
    sendTouch('up', 0, 0);
    hideIndicator();
  });

  img.addEventListener('touchstart', function(e) {
    e.preventDefault();
    const t = e.touches[0];
    const c = imgCoords(t);
    dragging = true;
    sendTouch('down', c.x, c.y);
  }, { passive: false });

  img.addEventListener('touchmove', function(e) {
    e.preventDefault();
    if (!dragging) return;
    const t = e.touches[0];
    const c = imgCoords(t);
    sendTouch('move', c.x, c.y);
  }, { passive: false });

  img.addEventListener('touchend', function(e) {
    e.preventDefault();
    if (!dragging) return;
    dragging = false;
    sendTouch('up', 0, 0);
  }, { passive: false });

  poll();
})();
</script>
</body>
</html>
"""


class RemoteDisplay:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.logger = logging.getLogger('remote_display')
        self.port = config.getint('port', 8092)
        self.bind = config.get('bind', '0.0.0.0')
        self.display_backend = config.get('display_backend', 'auto')
        self.fb_device = config.get('framebuffer_device', '/dev/fb0')
        self.drm_device = config.get('drm_device', '')
        # Blank by default: auto-detect the touchscreen by capability. A
        # configured path still wins if it really is a touchscreen.
        self.touch_device = config.get('touch_device', '').strip()
        self.stream_fps = config.getint('stream_fps', 5)
        self._server = None
        self._thread = None
        self._fb = None
        self._touch = None
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command(
            'REMOTE_DISPLAY_STATUS', self.cmd_STATUS,
            desc="Show remote display server status")
        gcode.register_command(
            'REMOTE_DISPLAY_TAP', self.cmd_TAP,
            desc="Simulate a tap on the printer display")
        self.printer.register_event_handler(
            "klippy:ready", self._handle_ready)
        self.printer.register_event_handler(
            "klippy:disconnect", self._handle_disconnect)

    def _open_display(self):
        backend = self.display_backend.lower().strip()
        if backend == 'drm':
            drm_dev = self.drm_device or None
            fb = DRMFramebuffer(drm_dev)
            fb.open()
            self.logger.info(
                "DRM display: %dx%d @ %dbpp (%s)",
                fb.width, fb.height, fb.bpp, fb.device)
            return fb
        if backend == 'fbdev':
            fb = Framebuffer(self.fb_device)
            fb.open()
            self.logger.info(
                "Framebuffer: %dx%d @ %dbpp (%s)",
                fb.width, fb.height, fb.bpp, self.fb_device)
            return fb
        # auto: wait for helixscreen (it may start after Klipper)
        import subprocess
        for attempt in range(15):
            try:
                result = subprocess.run(['pgrep', 'helix-screen'],
                                        capture_output=True, timeout=2)
                if result.returncode == 0:
                    try:
                        drm_dev = self.drm_device or None
                        fb = DRMFramebuffer(drm_dev)
                        fb.open()
                        self.logger.info(
                            "DRM display (auto): %dx%d @ %dbpp (%s)",
                            fb.width, fb.height, fb.bpp, fb.device)
                        return fb
                    except Exception as e:
                        self.logger.info(
                            "DRM not available (%s), trying fbdev", e)
                        break
            except Exception:
                pass
            if attempt < 14:
                self.logger.info(
                    "Waiting for helixscreen (%d/15)...", attempt + 1)
                time.sleep(2)
        self.logger.info(
            "helixscreen not detected after 30s, using fbdev")
        fb = Framebuffer(self.fb_device)
        fb.open()
        self.logger.info(
            "Framebuffer (auto): %dx%d @ %dbpp (%s)",
            fb.width, fb.height, fb.bpp, self.fb_device)
        return fb

    def _handle_ready(self):
        try:
            self._fb = self._open_display()
        except Exception as e:
            self.logger.error("Failed to open display: %s", e)
            return
        # Resolve the touch device: honor a valid configured path, otherwise
        # auto-detect the touchscreen by capability so a renumbered eventN (or
        # a stale config) self-heals.
        resolved = self._resolve_touch_device()
        try:
            self._touch = TouchInput(
                resolved, self._fb.width, self._fb.height)
            self._touch.open()
            self.touch_device = resolved
            self.logger.info(
                "Touch input: %s (range %dx%d)",
                resolved, self._touch.touch_max_x, self._touch.touch_max_y)
        except Exception as e:
            hint = ""
            if isinstance(e, PermissionError):
                hint = (" — Klipper's user lacks write access; add it to the "
                        "'input' group or add a udev rule for the touchscreen")
            self.logger.warning(
                "Touch input unavailable (%s): %s%s — display will be "
                "view-only", resolved, e, hint)
            self._touch = None

        handler_cls = _make_handler(
            self._fb, self._touch, self.stream_fps, self.logger)
        try:
            self._server = ThreadingHTTPServer(
                (self.bind, self.port), handler_cls)
            self._server.daemon_threads = True
            def _serve() -> None:
                # serve_forever is a stdlib bound method, so there is nowhere
                # inside it to name the thread -- wrap it instead.
                try:
                    thread_name = threading.current_thread().name
                    chelper.get_ffi()[1].set_thread_name(thread_name.encode("utf-8"))
                except Exception:
                    pass
                self._server.serve_forever()

            self._thread = threading.Thread(
                target=_serve, daemon=True, name="afc_disp_http")
            self._thread.start()
            self.logger.info(
                "Remote display server on http://%s:%d",
                self.bind, self.port)
            self.logger.info(
                "  Mainsail/Fluidd camera URL: "
                "http://<host>:%d/stream.mjpg", self.port)
            self.logger.info(
                "  Viewer with touch: http://<host>:%d/", self.port)
        except Exception as e:
            self.logger.error("Failed to start HTTP server: %s", e)

    def _resolve_touch_device(self):
        """Pick the touch device. A configured path that really is a
        touchscreen wins (mapped to its stable by-path symlink); otherwise
        auto-detect, so a moved eventN or wrong config corrects itself."""
        configured = self.touch_device
        if configured and os.path.exists(configured):
            ok, _name = _device_is_touchscreen(configured)
            if ok:
                return _prefer_by_path(configured)
            self.logger.warning(
                "Configured touch_device %s is not a touchscreen — "
                "auto-detecting", configured)
        elif configured:
            self.logger.warning(
                "Configured touch_device %s not found — auto-detecting",
                configured)
        found, name = _find_touchscreen()
        if found:
            self.logger.info(
                "Auto-detected touchscreen: %s%s",
                found, (" (%s)" % name) if name else "")
            return found
        self.logger.warning(
            "No touchscreen auto-detected; falling back to %s", configured)
        return configured

    def _handle_disconnect(self):
        if self._server:
            self._server.shutdown()
            self._server = None
        if self._fb:
            self._fb.close()
            self._fb = None
        if self._touch:
            self._touch.close()
            self._touch = None

    def cmd_STATUS(self, gcmd):
        if self._server and self._fb:
            backend = "DRM" if isinstance(self._fb, DRMFramebuffer) else "fbdev"
            device = self._fb.device
            gcmd.respond_info(
                f"Remote display: http://{self.bind}:{self.port}\n"
                f"Backend: {backend} ({device})\n"
                f"Resolution: {self._fb.width}x{self._fb.height} "
                f"@ {self._fb.bpp}bpp\n"
                f"Touch: {('available (' + self._touch.device + ')') if self._touch and self._touch.fd else 'unavailable'}\n"
                f"MJPEG stream: /stream.mjpg ({self.stream_fps} fps)\n"
                f"Snapshot: /snapshot.jpg or /snapshot.png\n"
                f"Viewer: /")
        else:
            gcmd.respond_info("Remote display: not running")

    def cmd_TAP(self, gcmd):
        if not self._touch or self._touch.fd is None:
            gcmd.respond_info("Touch input not available")
            return
        x = gcmd.get_int('X', None)
        y = gcmd.get_int('Y', None)
        if x is None or y is None:
            gcmd.respond_info("Usage: REMOTE_DISPLAY_TAP X=<x> Y=<y>")
            return
        self._touch.tap(x, y)
        gcmd.respond_info(f"Tapped at ({x}, {y})")

    def get_status(self, eventtime):
        running = self._server is not None
        return {
            'running': running,
            'port': self.port,
            'width': self._fb.width if self._fb else 0,
            'height': self._fb.height if self._fb else 0,
            'touch_available': (self._touch is not None
                                and self._touch.fd is not None),
        }


def load_config(config):
    return RemoteDisplay(config)
