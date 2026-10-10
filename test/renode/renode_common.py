"""Portable artifact paths and bounded Renode processes; no test assertions here."""
import os
import re
import subprocess
import sys
import tempfile
import time


def monitor_errors(text):
    # Deliberately NOT generic 'Error:': printer thermal-safety responses are
    # legitimate assertion input. These phrases are Renode monitor failures.
    return [line for line in text.splitlines() if re.search(
        r'Error E[0-9]+:|Could not tokenize|Could not resolve type|'
        r'There was an error executing command|Unhandled [Ee]xception|'
        r'^\s*FATAL\s*:', line)]


def stop_process(proc):
    if os.name == 'nt':
        subprocess.run(['taskkill', '/PID', str(proc.pid), '/T', '/F'],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    if proc.poll() is None:
        proc.kill()
    proc.wait()


def artifact_path(name):
    root = os.environ.get('TEACUP_LOG_DIR', tempfile.gettempdir())
    os.makedirs(root, exist_ok=True)
    return os.path.join(root, name)


def monitor_command(command):
    # A monitor @ token is a filename, not a shell argument. Quote the whole
    # path (including spaces) and use forward slashes on Windows.
    command = re.sub(r'@([^"\r\n]+)$',
                     lambda m: '"' + m.group(1).replace('\\', '/') + '"', command)
    command = re.sub(r"sys.path.append\('([^']+)'\)",
                     lambda m: "sys.path.append('%s')" % m.group(1).replace('\\', '/'), command)
    return command


def run_renode(executable, script, timeout=None):
    """Keep stdin OPEN: communicate() would send EOF and stop Renode early."""
    timeout = timeout or int(os.environ.get('TEACUP_TEST_TIMEOUT_SEC', '1800'))
    log = artifact_path('renode.log')
    with open(log, 'w', encoding='utf-8') as output:
        # Renode expands a positional .resc argument into `i @<path>` WITHOUT
        # monitor quoting, even when CreateProcess argv was correctly quoted.
        # Execute an explicit quoted include instead; shell quoting alone is
        # insufficient for checkout/results paths containing spaces.
        include = monitor_command('include @' + str(script))
        # Renode v1.17.0 README documents that -e may be repeated. A second
        # startup command closes the monitor if include aborts before the
        # script's own quit. It runs AFTER synchronous script execution, so
        # stdin stays open and successful RunFor commands are not interrupted.
        proc = subprocess.Popen([executable, '--console', '--disable-gui', '-e', include,
                                 '-e', 'quit'],
                                stdin=subprocess.PIPE, stdout=output,
                                stderr=subprocess.STDOUT)
        try:
            deadline = time.monotonic() + timeout
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    stop_process(proc)
                    print('ERROR: Renode timeout after %s seconds; log: %s' % (timeout, log), file=sys.stderr)
                    raise SystemExit(124)
                try:
                    code = proc.wait(timeout=min(0.5, remaining))
                    break
                except subprocess.TimeoutExpired:
                    # Include failures can abort the startup command list
                    # before the second -e quit, leaving a live prompt. Read
                    # bounded tail bytes, avoiding repeated full log reads.
                    with open(log, 'rb') as current:
                        current.seek(0, os.SEEK_END)
                        current.seek(max(0, current.tell() - 16384))
                        errors = monitor_errors(current.read().decode('utf-8', errors='replace'))
                    if errors:
                        stop_process(proc)
                        print('ERROR: Renode monitor/script failure; log: %s' % log, file=sys.stderr)
                        print('\n'.join(errors[:8])[:3000], file=sys.stderr)
                        raise SystemExit(1)
        finally:
            proc.stdin.close()
    if code:
        print('ERROR: Renode exited %s; log: %s' % (code, log), file=sys.stderr)
        raise SystemExit(1)
    with open(log, encoding='utf-8', errors='replace') as output:
        text = output.read()
    # The monitor can report a command error but still exit zero after quit.
    # Treat its explicit parser/execution failures as test infrastructure
    # failures, not empty successful emulator output.
    errors = monitor_errors(text)
    if errors:
        print('ERROR: Renode monitor/script failure; log: %s' % log, file=sys.stderr)
        print('\n'.join(errors[:8])[:3000], file=sys.stderr)
        raise SystemExit(1)
    return text
