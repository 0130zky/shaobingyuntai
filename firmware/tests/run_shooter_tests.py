"""Run shooter regressions without serial hardware or optional GUI packages."""
from pathlib import Path
import subprocess
import sys

directory = Path(__file__).resolve().parent
for name in (
    'test_booster_baseline.py',
    'test_sbus_decode.py',
    'test_can_dispatch.py',
    'test_motor_transport.py',
    'test_dial_jam.py',
    'test_dial_dm_control.py',
    'test_friction_sync.py',
    'test_shooter_integration.py',
    'test_gimbal_control.py',
):
    print(f'Running {name}', flush=True)
    subprocess.run([sys.executable, '-X', 'utf8', str(directory / name)], check=True)
