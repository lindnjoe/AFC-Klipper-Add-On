#!/usr/bin/env python3

import configparser
import logging
import logging.handlers
import os
import runpy
import sys


LOG_FILE = "/oem/printer_data/logs/openrfid.log"
MAX_LOG_BYTES = 10 * 1024 * 1024  # 10 MB
MAX_BACKUPS = 1  # keep 2 total: current + 1 backup


class _OpenRFIDFilter(logging.Filter):
    """Suppress openrfid noise while keeping successful reads and real errors.

    OpenRFID uses colon-separated logger names (rfid_reader:name,
    controller:name) which bypass Python's dot-based logger hierarchy,
    so setLevel on parent loggers has no effect — filtering must happen here.
    """
    def filter(self, record):
        # rfid_reader errors are expected when no tag is present and are
        # retried at a higher level; runtime.py logs WARNING on final failure
        if record.name.startswith("rfid_reader:"):
            return False
        # controller debug/info from moonraker polling
        if record.name.startswith("controller:") and record.levelno < logging.WARNING:
            return False
        # retry-cycle INFO spam from runtime.py
        if "will retry" in record.getMessage():
            return False
        return True


def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <target.cfg> [source.cfg ...]")
        sys.exit(1)

    os.makedirs(os.path.dirname(LOG_FILE), exist_ok=True)

    syslogHandler = logging.handlers.SysLogHandler(address="/dev/log")
    stderrHandler = logging.StreamHandler(sys.stderr)
    fileHandler = logging.handlers.RotatingFileHandler(
        LOG_FILE,
        maxBytes=MAX_LOG_BYTES,
        backupCount=MAX_BACKUPS,
    )

    formatter = logging.Formatter(
        fmt="%(asctime)s %(levelname)s %(name)s: %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )
    for handler in (syslogHandler, stderrHandler, fileHandler):
        handler.setFormatter(formatter)

    logging.root.handlers = [syslogHandler, stderrHandler, fileHandler]
    logging.root.setLevel(logging.INFO)
    logging.root.addFilter(_OpenRFIDFilter())

    # urllib3 uses standard dot-hierarchy so setLevel works
    logging.getLogger("urllib3").setLevel(logging.WARNING)

    target = sys.argv[1]
    sources = sys.argv[2:]

    config = configparser.RawConfigParser()
    config.optionxform = str
    config.read(sources)

    with open(target, "w") as f:
        config.write(f)

    sys.argv = [sys.argv[0], target]
    sys.path.insert(0, "/usr/local/share/openrfid")
    os.chdir("/usr/local/share/openrfid")
    runpy.run_path("main.py", run_name="__main__")


if __name__ == "__main__":
    main()
