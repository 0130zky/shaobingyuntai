"""Check selected assembly and absence of motor output in a built minimal image."""
import json
import shlex
from pathlib import Path
import re
import subprocess
import sys
build = Path(sys.argv[1]).resolve()
cache = (build / 'CMakeCache.txt').read_text()
robot = re.search(r'^ROBOT:STRING=(\w+)$', cache, re.M)[1]
commands = json.loads((build / 'compile_commands.json').read_text())
sources = [Path(x['file']).as_posix() for x in commands]
assert any(x.endswith(f'/robots/{robot}/robot.c') for x in sources) or 'DM_MC02' in cache
assert not any('/robots/' in x and f'/robots/{robot}/' not in x for x in sources)
compiler = shlex.split(commands[0]['command'])[0]
suffix = '.exe' if compiler.endswith('.exe') else ''
nm = str(Path(compiler).with_name('arm-none-eabi-nm' + suffix))
board_h7 = 'DM_MC02' in cache
artifact = 'sentry_dm_mc02.elf' if board_h7 else 'robot_template.elf'
symbols = subprocess.check_output([nm, str(build / artifact)], text=True)
if board_h7:
    assert all('-mcpu=cortex-m7' in x['command'] and '-mfpu=fpv5-d16' in x['command'] for x in commands)
    assert not any('/Core/' in x and '/boards/dm_mc02/' not in x for x in sources)
    assert not any('STM32F4' in x or 'startup_stm32f407' in x for x in sources)
    for required in ('Board_Init', 'UART5_IRQHandler', 'FDCAN1_IT0_IRQHandler', 'Sentry_Control', 'Sentry_Save'):
        assert required in symbols, required
    for forbidden in ('HAL_CAN_AddTxMessage', 'Booster_ControlStep', 'Gimbal_ControlStep'):
        assert forbidden not in symbols, forbidden
    assert 'LENGTH = 896K' in (Path(__file__).resolve().parents[1] / 'boards/dm_mc02/STM32H723VGTX_FLASH.ld').read_text()
    print('PASS: MC02 Cortex-M7/H723 assembly, H7 CAN/UART/USB and reserved Flash boundaries')
    sys.exit(0)
if robot == 'minimal':
    assert not any('/modules/' in x or x.endswith(('/app_DM4310.c', '/app_RM_Motor.c', '/booster_task.c')) for x in sources)
    for forbidden in ('DM4310_SendFrame', 'Set_RM_MotorCurrent', 'Booster_ControlStep', 'Gimbal_ControlStep'):
        assert forbidden not in symbols, forbidden
elif robot == 'sentry':
    for required in ('Sentry_Control', 'Sentry_Save', 'Robot_UsbRx'):
        assert required in symbols, required
    for forbidden in ('Booster_ControlStep', 'Gimbal_ControlStep', 'Set_RM_MotorCurrent'):
        assert forbidden not in symbols, forbidden
    assert not any('/modules/shooter/' in x or '/modules/gimbal/' in x for x in sources)
else:
    for required in ('Booster_ControlStep', 'Gimbal_ControlStep', 'Robot_UsbRx'):
        assert required in symbols, required
print('PASS: robot assembly and motor-output boundaries:', robot)
