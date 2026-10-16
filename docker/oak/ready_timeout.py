#!/usr/bin/env python3
"""Run the OAK driver; exit 1 if it is not ready within RAMMP_OAK_READY_TIMEOUT.

When driver.i_ip / driver.i_device_id match no reachable camera, the driver
retries discovery forever ("Ignoring device info: ...") and never exits, so a
supervisor sees a healthy node that publishes nothing. This kills it once the
timeout passes without "Driver ready!" in its log, and exits non-zero so the
node shows as crashed. 0 disables the timeout.

The driver's output passes through unchanged. SIGTERM/SIGINT are forwarded,
since this runs as the container's PID 1.
"""
import os
import signal
import subprocess
import sys
import threading

READY = "Driver ready!"

timeout = float(os.environ.get("RAMMP_OAK_READY_TIMEOUT", "60"))
driver = subprocess.Popen(sys.argv[1:], stdout=subprocess.PIPE,
                          stderr=subprocess.STDOUT, text=True, bufsize=1)
ready = threading.Event()
timed_out = threading.Event()

for sig in (signal.SIGTERM, signal.SIGINT):
    signal.signal(sig, lambda s, _: driver.send_signal(s))


def watchdog():
    if ready.wait(timeout):
        return
    print(f"[rammp] OAK driver not ready after {timeout:g}s: no camera matches "
          "driver.i_ip / driver.i_device_id, or it is unreachable. Exiting.",
          file=sys.stderr, flush=True)
    timed_out.set()
    # SIGKILL: the driver segfaults on SIGTERM while it is still searching.
    driver.kill()


if timeout > 0:
    threading.Thread(target=watchdog, daemon=True).start()

for line in driver.stdout:
    sys.stdout.write(line)
    sys.stdout.flush()
    if not ready.is_set() and READY in line:
        ready.set()

code = driver.wait()
if timed_out.is_set():
    sys.exit(1)
sys.exit(128 - code if code < 0 else code)
