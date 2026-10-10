"""Build and run the Windows-portable suites. Invoked by run-renode.cmd."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys

HERE = Path(__file__).resolve().parent
TEST = HERE.parent


def plan(suite, chip, port, part):
    allowed_parts = {'all', 'basic', 'safety', 'proto', 'cmds', 'settings', 'slicer', 'perf', 'tune', 'dfu'}
    if not set(part.split(',')) <= allowed_parts:
        raise ValueError('Unknown baseline part: ' + part)
    manifest = json.loads((HERE / 'windows-suites.json').read_text())
    chips = ['F401', 'F411'] if chip == 'All' else [chip]
    ports = ['uart', 'usb'] if port == 'All' else [port]
    cases = []
    if suite in ('Baseline', 'All'):
        for c in chips:
            for p in ports:
                cases.append(dict(name='baseline-' + p, script='run_tests.py', test=1,
                                  extra='', chip=c, port=p, args=[part], repl='dma'))
            cases.append(dict(name='usb-routing', script='run_usb.py', test=1,
                              extra='', chip=c, port='uart', args=[], repl='dma'))
    for entry in manifest:
        if suite != 'All' and entry['group'] != suite:
            continue
        # Feature suites containing fixed F401 platforms and assumptions are
        # deliberately run only on F401, never mislabeled as F411 coverage.
        if 'F401' not in chips:
            continue
        cases.append(dict(entry, chip='F401', port='uart'))
    if not cases:
        raise ValueError('No suites for this selection. P3Steel/Features target F401; use -Chip F401 or All.')
    return cases


def logged(command, path, cwd, timeout, env):
    print('+ ' + subprocess.list2cmdline([str(x) for x in command]), flush=True)
    with open(path, 'w', encoding='utf-8') as log:
        process = subprocess.Popen(command, cwd=cwd, env=env, stdout=log, stderr=subprocess.STDOUT)
        try:
            code = process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            if os.name == 'nt':
                subprocess.run(['taskkill', '/PID', str(process.pid), '/T', '/F'],
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            process.kill()
            process.wait()
            code = 124
    # Don't dump multi-megabyte compiler/monitor logs; assertions are compact.
    if path.name == 'checks.log' or code:
        print(path.read_text(encoding='utf-8', errors='replace')[-16000:], flush=True)
    return code


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--suite', choices=['Baseline', 'P3Steel', 'Features', 'All'], default='Baseline')
    parser.add_argument('--chip', choices=['All', 'F401', 'F411'], default='All')
    parser.add_argument('--port', choices=['All', 'uart', 'usb'], default='All')
    parser.add_argument('--part', default='all')
    parser.add_argument('--renode', required=True)
    parser.add_argument('--make', default='make')
    parser.add_argument('--jobs', type=int, default=4)
    parser.add_argument('--timeout', type=int, default=1800)
    parser.add_argument('--list', action='store_true')
    args = parser.parse_args()
    cases = plan(args.suite, args.chip, args.port, args.part)
    if args.list:
        print(json.dumps(cases, indent=2)); return 0
    results = TEST / 'results'
    results.mkdir(exist_ok=True)
    built = {}
    summary = []
    for case in cases:
        label = case['chip'] + '-' + case['name']
        directory = results / label
        directory.mkdir(exist_ok=True)
        env = dict(os.environ, TEACUP_PORT=case['port'], TEACUP_LOG_DIR=str(directory),
                   TEACUP_TEST_TIMEOUT_SEC=str(args.timeout))
        key = (case['chip'], case['test'], case['extra'])
        # Reuse only images built in THIS invocation with identical defines.
        # Separate directories and -B avoid all stale configuration objects.
        suffix = '_windows_' + str(len(built)) if key not in built else built[key][0]
        if key not in built:
            command = [args.make, 'SHELL=sh.exe' if os.name == 'nt' else 'SHELL=sh', '-B',
                       '-j' + str(args.jobs), 'CHIP=' + case['chip'], 'TEST=' + str(case['test']),
                       'BUILD_SUFFIX=' + suffix, 'EXTRA=' + case['extra']]
            code = logged(command, directory / 'build.log', TEST / 'gcc', args.timeout, env)
            built[key] = (suffix, code)
        suffix, code = built[key]
        if not code:
            elf = TEST / 'gcc' / ('build_%s_%s%s' % (case['chip'], case['test'], suffix)) / 'teacup.elf'
            repl = HERE / ('stm32%s_%s.repl' % (case['chip'].lower(), case['repl']))
            script_args = [args.renode, str(elf), str(repl)] + case['args']
            if 'kind' in case:
                script_args = [args.renode, case['kind'], str(elf)]
            elif case['script'] == 'run_p3steel.py':
                script_args.append('84000000')
            # Embedded Python ignores PYTHONPATH and the script directory;
            # explicitly add it instead of modifying the shared cached Python.
            bootstrap = "import sys,runpy;sys.path.insert(0,%r);runpy.run_path(sys.argv.pop(1),run_name='__main__')" % str(HERE)
            command = [sys.executable, '-u', '-c', bootstrap, str(HERE / case['script'])] + script_args
            code = logged(command, directory / 'checks.log', HERE, args.timeout + 15, env)
        summary.append(dict(case=label, exit_code=code))
        (results / 'summary.json').write_text(json.dumps(summary, indent=2))
        print('%s: %s' % (label, 'PASS' if not code else 'FAIL (exit %s)' % code), flush=True)
    print('Logs: ' + str(results))
    return 124 if any(x['exit_code'] == 124 for x in summary) else int(any(x['exit_code'] for x in summary))


if __name__ == '__main__':
    sys.exit(main())
