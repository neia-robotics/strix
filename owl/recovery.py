"""Worker recovery policy, independent of process termination by the supervisor."""
import signal

STARTUP_TIMEOUT_S = 60.0
WORKER_TIMEOUT_S = 10.0
CAPTURE_TIMEOUT_S = 15.0
NATIVE_FAILURE_SIGNALS = {signal.SIGABRT, signal.SIGSEGV, signal.SIGBUS, signal.SIGILL, signal.SIGFPE}


def needs_restart(alive, now, last_seen, reported):
    return not alive or now - last_seen > (WORKER_TIMEOUT_S if reported else STARTUP_TIMEOUT_S)


def needs_cpu_fallback(exitcode):
    # Inspect BEFORE the supervisor sends terminate/kill. USB failures and watchdog
    # restarts are not evidence of a damaged CUDA context.
    return exitcode is not None and exitcode < 0 and -exitcode in NATIVE_FAILURE_SIGNALS
