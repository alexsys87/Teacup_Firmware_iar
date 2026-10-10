"""Portable artifact paths and bounded Renode processes; no test assertions here."""
import os
import re
import subprocess
import sys
import tempfile


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
        # Renode v1.16.0 README documents that -e may be repeated. A second
        # startup command closes the monitor if include aborts before the
        # script's own quit. It runs AFTER synchronous script execution, so
        # stdin stays open and successful RunFor commands are not interrupted.
        proc = subprocess.Popen([executable, '--console', '--disable-gui', '-e', include,
                                 '-e', 'quit'],
                                stdin=subprocess.PIPE, stdout=output,
                                stderr=subprocess.STDOUT)
        try:
            code = proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            if os.name == 'nt':
                subprocess.run(['taskkill', '/PID', str(proc.pid), '/T', '/F'],
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            proc.kill()
            proc.wait()
            print('ERROR: Renode timeout after %s seconds; log: %s' % (timeout, log), file=sys.stderr)
            raise SystemExit(124)
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
    errors = [line for line in text.splitlines() if re.search(
        r'Could not tokenize here:|There was an error executing command|'
        r'Unhandled [Ee]xception|^\s*FATAL\s*:', line)]
    if errors:
        print('ERROR: Renode monitor/script failure; log: %s' % log, file=sys.stderr)
        print('\n'.join(errors[:8])[:3000], file=sys.stderr)
        raise SystemExit(1)
    return text
