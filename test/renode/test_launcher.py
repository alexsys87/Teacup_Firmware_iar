"""Self-tests: suite selection, paths with spaces, real process exits/timeouts."""
import contextlib
import ast
import json
import ntpath
import os
from pathlib import Path
import runpy
import shutil
import subprocess
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import sys
import warnings

from launch_tests import logged, plan
from renode_common import artifact_path, monitor_command, run_renode


class LauncherTests(unittest.TestCase):
    def test_baseline_both_chips_and_ports(self):
        cases = plan('Baseline', 'All', 'All', 'all')
        self.assertEqual(len(cases), 6)
        self.assertEqual({x['chip'] for x in cases}, {'F401', 'F411'})
        self.assertEqual(sum(x['script'] == 'run_usb.py' for x in cases), 2)
        self.assertTrue(all(x['test'] == 1 and not x['extra'] for x in cases))

    def test_all_manifest_suites_selected(self):
        manifest = json.loads((Path(__file__).parent / 'windows-suites.json').read_text())
        cases = plan('All', 'All', 'All', 'all')
        self.assertEqual(len(cases), len(manifest) + 6)
        self.assertTrue(all(x['chip'] == 'F401' for x in cases if x['script'] != 'run_tests.py' and x['script'] != 'run_usb.py'))
        self.assertIn('-DBLTOUCH', next(x['extra'] for x in cases if x['name'] == 'features'))
        self.assertEqual(next(x['test'] for x in cases if x['name'] == 'mpc'), 1)
        with self.assertRaises(ValueError):
            plan('Features', 'F411', 'uart', 'all')

    def test_monitor_paths(self):
        self.assertEqual(monitor_command(r'include @C:\My Folder\model.cs'), 'include "C:/My Folder/model.cs"')
        self.assertEqual(monitor_command('echo "@@MARK basic"'), 'echo "@@MARK basic"')
        self.assertEqual(monitor_command("python \"import sys; sys.path.append('C:\\My Folder'); import p3_model\""),
                         "python \"import sys; sys.path.append('C:/My Folder'); import p3_model\"")

    def test_windows_emulator_arguments_remain_separate(self):
        # Shell argv quoting does NOT quote Renode's internal positional
        # `i @path` expansion. Explicit -e uses monitor-level quoting instead.
        executable = r'C:\Portable tools\Renode\renode.exe'
        script = r'C:\Teacup source with spaces\test\results\run.resc'
        with tempfile.TemporaryDirectory(prefix='teacup argv spaces ') as root, \
             patch.dict(os.environ, TEACUP_LOG_DIR=root), \
             patch('renode_common.subprocess.Popen') as start:
            start.return_value.wait.return_value = 0
            self.assertEqual(run_renode(executable, script, 10), '')
            self.assertEqual(start.call_args.args[0],
                             [executable, '--console', '--disable-gui', '-e',
                              'include "C:/Teacup source with spaces/test/results/run.resc"'])
            self.assertFalse(start.call_args.kwargs.get('shell', False))
            start.return_value.wait.assert_called_once_with(timeout=10)
            start.return_value.stdin.close.assert_called_once()

    def test_process_exit_and_timeout(self):
        # Python acts as a fake Renode executable; --console/--disable-gui are
        # ignored by this tiny shim. This exercises actual OS child processes.
        with tempfile.TemporaryDirectory(prefix='teacup launcher spaces ') as root:
            with patch.dict(os.environ, TEACUP_LOG_DIR=root):
                fake = Path(root) / 'fake.py'
                fake.write_text('import sys,time,shlex\nprint("fake output",flush=True)\n'
                                'command=sys.argv[sys.argv.index("-e")+1]\n'
                                'script=shlex.split(command)[1]\n'
                                'if script=="timeout": time.sleep(30)\n'
                                'sys.exit(7 if script=="fail" else 0)\n')
                # run_renode normally passes the executable directly; intercept
                # only argv to prepend the interpreter to the fake script.
                import subprocess
                popen = subprocess.Popen
                def start(argv, **kwargs):
                    return popen([sys.executable, str(fake)] + argv[1:], **kwargs)
                with patch('renode_common.subprocess.Popen', side_effect=start):
                    self.assertIn('fake output', run_renode('fake', 'pass', 10))
                    with self.assertRaises(SystemExit) as failure:
                        run_renode('fake', 'fail', 10)
                    self.assertEqual(failure.exception.code, 1)
                    with self.assertRaises(SystemExit) as timeout:
                        run_renode('fake', 'timeout', 1)
                    self.assertEqual(timeout.exception.code, 124)
                self.assertTrue(Path(artifact_path('renode.log')).is_file())

    def test_monitor_startup_include_with_space_path(self):
        # Model the concrete Windows CI failure at the second parser boundary:
        # Renode's positional CLI argument became an unquoted `i @D:\...`.
        # This real child process rejects that expansion, parses -e using
        # quoted monitor string syntax, and opens the actual generated file.
        with tempfile.TemporaryDirectory(prefix='teacup monitor spaces ') as root, \
             patch.dict(os.environ, TEACUP_LOG_DIR=root):
            script = Path(root) / 'script with spaces.resc'
            script.write_text('echo "startup reached"\nquit\n')
            fake = Path(root) / 'monitor.py'
            fake.write_text('import sys,shlex,pathlib\n'
                            'if "-e" not in sys.argv:\n'
                            ' print("Could not tokenize here: i @"+sys.argv[-1]);sys.exit(9)\n'
                            'command=sys.argv[sys.argv.index("-e")+1]\n'
                            'tokens=shlex.split(command)\n'
                            'assert len(tokens)==2 and tokens[0]=="include"\n'
                            'print(pathlib.Path(tokens[1]).read_text())\n')
            popen = subprocess.Popen
            def start(argv, **kwargs):
                return popen([sys.executable, str(fake)] + argv[1:], **kwargs)
            with patch('renode_common.subprocess.Popen', side_effect=start):
                self.assertIn('startup reached', run_renode('fake', str(script), 10))
            # Prove the fixture would reject the previously published argv.
            old = subprocess.run([sys.executable, str(fake), '--console',
                                  '--disable-gui', str(script)], capture_output=True, text=True)
            self.assertEqual(old.returncode, 9)
            self.assertIn('Could not tokenize here', old.stdout)

    def test_outer_process_failure_and_timeout(self):
        with tempfile.TemporaryDirectory(prefix='teacup outer spaces ') as root:
            log = Path(root) / 'checks.log'
            self.assertEqual(logged([sys.executable, '-c', 'raise SystemExit(7)'],
                                    log, root, 10, dict(os.environ)), 7)
            self.assertEqual(logged([sys.executable, '-c', 'import time; time.sleep(30)'],
                                    log, root, 1, dict(os.environ)), 124)

    def test_every_selected_script_runs_bounded_renode(self):
        # Catch accidental loss of the emulator invocation during portability
        # edits, including scripts that originally streamed to their own log.
        here = Path(__file__).parent
        for script in {case['script'] for case in plan('All', 'All', 'All', 'all')}:
            with self.subTest(script=script):
                tree = ast.parse((here / script).read_text(encoding='utf-8'))
                calls = [node for node in ast.walk(tree) if isinstance(node, ast.Call)]
                self.assertTrue(any(isinstance(node.func, ast.Name) and
                                    node.func.id == 'run_renode' for node in calls))
                self.assertTrue(any(isinstance(node.func, ast.Attribute) and
                                    isinstance(node.func.value, ast.Name) and
                                    node.func.value.id == 'sys' and node.func.attr == 'exit'
                                    for node in calls))

    def test_selected_artifacts_use_windows_results_directory(self):
        # Evaluate the actual artifact assignments using Windows path rules,
        # even on Linux. Merely importing artifact_path did not catch the old
        # os.path.join('/tmp', ...) left in the two baseline scripts.
        import renode_common
        root = r'C:\Teacup source with spaces\test\results\F411-baseline-usb'
        windows_os = SimpleNamespace(path=ntpath, getpid=lambda: 1234,
                                     environ={'TEACUP_LOG_DIR': root},
                                     makedirs=lambda *a, **k: None)
        here = Path(__file__).parent
        with patch.object(renode_common, 'os', windows_os):
            for name in {case['script'] for case in plan('All', 'All', 'All', 'all')}:
                tree = ast.parse((here / name).read_text(encoding='utf-8'))
                assignments = [node for node in ast.walk(tree)
                               if isinstance(node, ast.Assign) and
                               any(isinstance(t, ast.Name) and t.id in ('script', 'REPL', 'repl2')
                                   for t in node.targets) and
                               any(isinstance(n, ast.Constant) and isinstance(n.value, str) and
                                   n.value.endswith(('.resc', '.repl')) for n in ast.walk(node.value))]
                resc_count = 0
                for node in assignments:
                    # ELF/platform input defaults are not generated artifacts.
                    if isinstance(node.value, ast.IfExp):
                        continue
                    if not any(isinstance(n, ast.Constant) and isinstance(n.value, str) and
                               n.value.endswith('.resc') for n in ast.walk(node.value)) and not any(
                            isinstance(n, ast.Call) and isinstance(n.func, ast.Name) and
                            n.func.id == 'artifact_path' for n in ast.walk(node.value)):
                        continue
                    with self.subTest(script=name, expression=ast.unparse(node.value)):
                        value = eval(compile(ast.Expression(node.value), name, 'eval'),
                                     {'os': windows_os, 'artifact_path': artifact_path, 'KIND': 'laser'})
                        self.assertEqual(ntpath.dirname(value), root)
                        self.assertNotIn('/tmp', value)
                        resc_count += value.endswith('.resc')
                self.assertGreater(resc_count, 0, name)

    def test_actual_suite_generation_with_space_paths(self):
        # Execute real suite setup and .resc writes, stopping exactly at the
        # emulator boundary. This tests paths, not firmware assertions; fake
        # nm data is only the setup fixture, never used by actual test runs.
        class AtEmulator(Exception):
            pass

        here = Path(__file__).parent
        with tempfile.TemporaryDirectory(prefix='teacup source with spaces ') as root:
            copied = Path(root) / 'test' / 'renode'
            shutil.copytree(here, copied, ignore=shutil.ignore_patterns('__pycache__'))
            symbols = ['adc_buffer', 'dda_start', 'temp_dummy_plant',
                       'temp_dummy_force', 'temp_dummy_fan_loss', 'status_msg']
            # Pick up additional symbols needed only by individual suites.
            for name in {case['script'] for case in plan('All', 'All', 'All', 'all')}:
                tree = ast.parse((copied / name).read_text(encoding='utf-8'))
                symbols.extend(node.args[0].value for node in ast.walk(tree)
                               if isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and
                               node.func.id == 'sym' and node.args and
                               isinstance(node.args[0], ast.Constant))
            def nm(argv, **kwargs):
                self.assertEqual(argv[0], 'arm-none-eabi-nm')
                middle = '00000020 B' if '-S' in argv else 'B'
                return subprocess.CompletedProcess(argv, 0, stdout=''.join(
                    '20000000 %s %s\n' % (middle, symbol) for symbol in symbols))

            for case in plan('All', 'All', 'All', 'all'):
                with self.subTest(chip=case['chip'], case=case['name']):
                    output = Path(root) / 'results' / (case['chip'] + '-' + case['name'])
                    elf = str(Path(root) / 'firmware with spaces' / 'teacup.elf')
                    repl = str(copied / ('stm32%s_%s.repl' % (case['chip'].lower(), case['repl'])))
                    arguments = ['fake renode.exe', elf, repl] + case['args']
                    if 'kind' in case:
                        arguments = ['fake renode.exe', case['kind'], elf]
                    elif case['script'] == 'run_p3steel.py':
                        arguments.append('84000000')
                    seen = []
                    def stop(executable, script, *a, **k):
                        self.assertEqual(executable, 'fake renode.exe')
                        path = Path(script)
                        self.assertEqual(path.parent, output)
                        self.assertTrue(path.is_file())
                        text = path.read_text()
                        self.assertIn('sysbus LoadELF "%s"' % elf.replace('\\', '/'), text)
                        self.assertNotIn('include @', text)
                        self.assertNotIn('LoadPlatformDescription @', text)
                        self.assertIn('quit', text)
                        seen.append(script)
                        raise AtEmulator()
                    with patch.dict(os.environ, TEACUP_LOG_DIR=str(output), TEACUP_PORT=case['port']), \
                         patch.object(sys, 'argv', [case['script']] + arguments), \
                         patch('renode_common.run_renode', side_effect=stop), \
                         patch('subprocess.run', side_effect=nm), \
                         self.assertRaises(AtEmulator), warnings.catch_warnings():
                        # Existing suites use short-lived open(...).write(...)
                        # expressions. Avoid unrelated unittest resource noise.
                        warnings.simplefilter('ignore', ResourceWarning)
                        runpy.run_path(str(copied / case['script']), run_name='__main__')
                    self.assertEqual(len(seen), 1)


if __name__ == '__main__':
    unittest.main(verbosity=2)
