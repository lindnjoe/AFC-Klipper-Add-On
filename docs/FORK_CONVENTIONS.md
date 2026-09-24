# Fork Conventions

Rules our own modules (the Bambu AMS bridge, the RFID readers, ACE/ACE2, the
dryer panel, AFC_autocal) follow on top of upstream's AGENTS.md. They
are not upstream's rules; they live here so AGENTS.md stays upstream's file.

- **Log through AFC's logger, and give it a finished message.** Modules take
   it IN `__init__`, with
   `self.logger = self.printer.load_object(config, 'AFC').logger`, the way
   `AFC_lane`, `AFC_buffer` and `AFC_extruder` do. `load_object` CONSTRUCTS
   AFC when your section is reached first, so the old reason for swapping at
   `klippy:ready` ("AFC does not exist yet") does not hold; that swap left a
   window where anything logged reached `klippy.log` only, and the modules
   that logged in it were the ones you most wanted to hear from. Note this
   takes only the LOGGER: where a module uses `self.afc` as its ready marker,
   leave that assignment at `klippy:ready`, a dozen guards read it that way.
   A module that genuinely runs without AFC must NOT use `load_object`,
   which would force AFC onto a printer that has none.
   Prefer this over `logging.getLogger`: it writes `AFC.log` with
   timestamps and call sites and echoes to the g-code console, where anyone
   debugging is already looking. Its signature is not the stdlib's, and the
   differences fail quietly: `info(message, console_only)`,
   `warning(message)`, `debug(message, only_debug, traceback)`,
   `error(message, traceback, stack_name)`, and **no `exception()`**. So
   `logger.info("slot %d", n)` binds `n` to `console_only` and drops it from
   the message, `exc_info=True` is rejected, and `logger.exception(...)` is an
   `AttributeError` on a path that may not run for months. Build the string
   first, f-strings, as above, and use
   `error(msg, traceback=traceback.format_exc())` where the stdlib would have
   used `exception`. `extras/AFC_BambuAMS_bridge.py` is a deliberate
   exception, as are `AFC_ACE`'s serial-file loggers; modules that never reach
   AFC (`remote_display.py`, the `temperature_*` shims) keep the stdlib
   logger because AFC's is not
   available to them, and printf args ARE correct there, so a module moving
   onto AFC's logger has to convert those calls in the same change or they
   become `TypeError`s.
   Test doubles for the logger must match AFC's signature, not the stdlib's
  , a fake that accepts printf args hides exactly the calls the real logger
   would reject.
- **Name every background thread, at the OS level too.** A `threading.Thread`
   gets a `name=` AND its target sets the OS name as its first statement, the
   way upstream's `afc.save_vars` worker does (`chelper.get_ffi()[1].set_thread_name`
   in a try/except). The Python name alone is invisible to `top -H`, `htop` and
   `ps -L`, which is exactly where you look when the host is too busy to feed
   the MCU and Klipper starts throwing Timer Too Close. Names are lowercase
   with underscores, prefixed `afc_`, and **at most 15 characters** ,
   `set_thread_name` is `prctl(PR_SET_NAME)`, so the kernel silently truncates
   anything longer and two threads can end up indistinguishable. When the
   target is a stdlib bound method (`server.serve_forever`), wrap it in a
   local function so there is somewhere to put the call.
- **New g-code commands are prefixed `AFC_`**, every command this project
   registers (`register_command` / `register_mux_command`) starts with
   `AFC_` (e.g. `AFC_BAMBU_SCAN`, `AFC_BT_RFID_STATUS`). Exceptions: names
   upstream or the toolchanger convention already owns (`SELECT_TOOL`,
   `SET_BUFFER_MULTIPLIER`, …) stay as they are for parity, and g-code
   overrides (`M106`/`M107`) must keep their real names to intercept them.
