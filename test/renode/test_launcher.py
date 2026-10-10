"""Self-tests: suite selection, paths with spaces, real process exits/timeouts."""
import contextlib
import ast
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import sys

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

    def test_process_exit_and_timeout(self):
        # Python acts as a fake Renode executable; --console/--disable-gui are
        # ignored by this tiny shim. This exercises actual OS child processes.
        with tempfile.TemporaryDirectory(prefix='teacup launcher spaces ') as root:
            with patch.dict(os.environ, TEACUP_LOG_DIR=root):
                fake = Path(root) / 'fake.py'
                fake.write_text('import sys,time\nprint("fake output",flush=True)\n'
                                'if sys.argv[-1]=="timeout": time.sleep(30)\n'
                                'sys.exit(7 if sys.argv[-1]=="fail" else 0)\n')
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


if __name__ == '__main__':
    unittest.main(verbosity=2)
