# Armored Turtle Automated Filament Control
#
# Copyright (C) 2024-2026 Armored Turtle
#
# This file may be distributed under the terms of the GNU GPLv3 license.
"""The ACE serial log is size-capped so it cannot fill the card: a rotating
handler with a fixed byte cap and exactly one rolled backup."""
from __future__ import annotations

import logging
import logging.handlers
import types

import extras.AFC_ACE as mod


def _make_ace(tmp_path):
    obj = mod.afcACE.__new__(mod.afcACE)
    obj.printer = types.SimpleNamespace(
        start_args={"log_file": str(tmp_path / "klippy.log")})
    return obj


def _clear_singleton():
    lg = logging.getLogger("AFC_ACE_serial_file")
    for h in list(lg.handlers):
        lg.removeHandler(h)
    return lg


class TestSerialLogSizeCap:
    def test_it_rotates_at_10mb_with_one_backup(self, tmp_path):
        _clear_singleton()
        obj = _make_ace(tmp_path)
        logger = obj._create_serial_logger()
        try:
            assert logger is not None
            # The lane to the file is a queue, so the serial thread never
            # blocks on disk I/O.
            assert isinstance(logger.handlers[0],
                              logging.handlers.QueueHandler)
            fh = obj._serial_ql.handlers[0]
            assert isinstance(fh, logging.handlers.RotatingFileHandler)
            # 10 MB cap, ONE rolled backup -> at most ~2x on disk, and the
            # previous chunk survives a rollover.
            assert fh.maxBytes == 10 * 1024 * 1024
            assert fh.backupCount == 1
        finally:
            # The listener is stopped by the atexit hook the code registers;
            # calling stop() here too would double-join it. Just detach the
            # handler from the process-global logger for the next test.
            _clear_singleton()

    def test_no_log_file_arg_yields_no_logger(self, tmp_path):
        _clear_singleton()
        obj = mod.afcACE.__new__(mod.afcACE)
        obj.printer = types.SimpleNamespace(start_args={})
        try:
            assert obj._create_serial_logger() is None
        finally:
            _clear_singleton()
