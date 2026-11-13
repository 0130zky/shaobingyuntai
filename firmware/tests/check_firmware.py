"""Verify built profiles contain only the fitted hardware and intended feature flags."""
import argparse
import json
import re
from pathlib import Path
import subprocess

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('build_directory', type=Path)
args = parser.parse_args()
directory = args.build_directory.resolve()
commands = json.loads((directory / 'compile_commands.json').read_text(encoding='utf-8'))
sources = [Path(entry['file']).as_posix() for entry in commands]
assert any(name.endswith('/dial_port_dm.c') for name in sources)
assert not any('/legacy/' in name or name.endswith('/dial_port_rm.c') for name in sources)
assert not any(name.endswith(('/gimbal_task.c','/ins_task.c','/master_task.c')) for name in sources)
math_command = next(entry['command'] for entry in commands if entry['file'].endswith('controller.c'))
assert '-DSTM32' not in math_command and '-DUSE_HAL_DRIVER' not in math_command, 'PID must be hardware independent'
cache = (directory / 'CMakeCache.txt').read_text(encoding='utf-8')
profile = re.search(r'^FIRMWARE_PROFILE:STRING=(\w+)$', cache, re.M)[1]
rc_only, calibration = {'Shooter': (1, 0), 'Bench': (0, 0), 'Calibration': (0, 1)}[profile]
for entry in commands:
    if '/modules/shooter/' in Path(entry['file']).as_posix():
        assert f'-DBOOSTER_RC_ONLY={rc_only}' in entry['command']
        assert f'-DDIAL_DM_CALIB_FREE={calibration}' in entry['command']
symbols = subprocess.check_output(['arm-none-eabi-nm', str(directory / 'robot_template.elf')], text=True)
for name in ('BMI088_init', 'GimbalTask', 'INS_Task', 'MasterLink_Task', 'Dial_Port_RMSlot', 'Get_RM_MotorInfo_total', 'DM4310_RangePitch'):
    assert name not in symbols, name
print('PASS: firmware hardware/dependency boundaries:', directory.name)
