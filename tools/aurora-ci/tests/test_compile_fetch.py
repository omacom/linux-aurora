"""Exercise the compile workflow's history step without compiling a kernel."""
import os
from pathlib import Path
import re
import subprocess
import tempfile
import unittest


class CompileFetchTests(unittest.TestCase):
    def test_named_merge_ref_contains_both_sides_and_missing_head_refuses(self):
        workflow = (Path(__file__).resolve().parents[3] / '.github/workflows/kernel-ci-jobs.yml').read_text()
        compile_job = workflow.split('\n  compile:\n', 1)[1].split('\n  devicetree:\n', 1)[0]
        checkout = compile_job.split('- name: Fetch history', 1)[0]
        self.assertNotIn('ref: ${{ needs.plan.outputs.head_sha }}', checkout)
        match = re.search(r'- name: Fetch history[^\n]*\n        run: \|\n((?:          .*\n)+)', compile_job)
        self.assertIsNotNone(match)
        assert match is not None
        script = match[1]
        script = '\n'.join(line[10:] for line in script.splitlines())
        self.assertNotIn('origin "$BASE_SHA"', script)
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            source = root / 'source'
            source.mkdir()

            def git(*args, cwd=source):
                return subprocess.check_output(['git', *args], cwd=cwd, text=True, stderr=subprocess.DEVNULL).strip()

            git('init', '-q', '-b', 'main')
            git('config', 'user.name', 'CI test')
            git('config', 'user.email', 'ci@example.invalid')
            git('config', 'commit.gpgsign', 'false')
            git('config', 'core.hooksPath', '/dev/null')
            git('commit', '-q', '--allow-empty', '-m', 'common')
            git('checkout', '-qb', 'change')
            git('commit', '-q', '--allow-empty', '-m', 'change')
            head = git('rev-parse', 'HEAD')
            git('checkout', '-q', 'main')
            git('commit', '-q', '--allow-empty', '-m', 'base')
            base = git('rev-parse', 'HEAD')
            git('merge', '-q', '--no-ff', 'change', '-m', 'merge')
            event_ref = 'refs/pull/166/merge'
            git('update-ref', event_ref, 'HEAD')
            clone = root / 'clone'
            clone.mkdir()
            git('init', '-q', cwd=clone)
            git('remote', 'add', 'origin', source.as_uri(), cwd=clone)
            git('fetch', '-q', '--depth=1', 'origin', event_ref, cwd=clone)
            git('checkout', '-q', 'FETCH_HEAD', cwd=clone)
            env = dict(os.environ, BASE_SHA=base, HEAD_SHA=head, GITHUB_REF=event_ref)
            result = subprocess.run(['bash', '-euo', 'pipefail', '-c', script], cwd=clone, env=env, capture_output=True)
            self.assertEqual(result.returncode, 0, result.stderr.decode())
            self.assertTrue(git('merge-base', base, head, cwd=clone))
            env['HEAD_SHA'] = '0' * 40
            result = subprocess.run(['bash', '-euo', 'pipefail', '-c', script], cwd=clone, env=env, capture_output=True)
            self.assertNotEqual(result.returncode, 0)


if __name__ == '__main__':
    unittest.main()
