"""Run all available stage regression tests against the bundled models."""
import os
from pathlib import Path
import subprocess
import sys
ROOT=Path(__file__).resolve().parent
ENV=dict(os.environ,PYTHONPATH=str(ROOT/'components')+os.pathsep+os.environ.get('PYTHONPATH',''))
for test in sorted((ROOT/'tests').glob('test_step_*.py')):
    print(f'\n--- {test.name} ---',flush=True)
    subprocess.run([sys.executable,str(test)],env=ENV,check=True)
