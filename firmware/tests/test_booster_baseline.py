#!/usr/bin/env python3
"""Run real baseline RC mapping/feed admission with host-side state stubs."""
from pathlib import Path
import re
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
rc = (root / 'robots/example/remote_mapping.c').read_text()
booster = (root / 'Application/booster_task.c').read_text()
header = (root / 'Application/RC_task.h').read_text()


def function(source, signature):
    start = source.index(signature)
    return source[start:source.index('\n}', start)+2]


constants = '\n'.join(line for line in header.splitlines()
                      if re.match(r'#define RC_(FRI_|CH8_RAW_)', line))
code = r'''
#include <stdint.h>
#include <assert.h>
#include <stdio.h>
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
#define check_yse 0
static int online=1,enabled=1;
uint8_t RC_LinkFresh(void){return online;}
uint8_t RC_MasterEnabled(void){return enabled;}
struct {struct {uint16_t SD_CH8;} ET07_DATA;} RC_Data;
struct {struct {int Dail_check_state,Dail_stuck_count,Dial_Jam_Retry;} Dail_state;
        int16_t Dial_Shoot;} BOOSTER;
static uint8_t s_ready_tracking,s_wheels_ready;
uint8_t Booster_DialJammed(void){return BOOSTER.Dail_state.Dail_check_state!=0;}
'''
code += constants + '\n'
code += function(rc, 'int16_t RC_FrictionWheelRpm(void)') + '\n'
code += r'''
int main(void){
  RC_Data.ET07_DATA.SD_CH8=RC_CH8_RAW_MID;assert(RC_FrictionWheelRpm()==5560);
  RC_Data.ET07_DATA.SD_CH8=RC_CH8_RAW_MAX;assert(RC_FrictionWheelRpm()==10000);
  RC_Data.ET07_DATA.SD_CH8=RC_CH8_RAW_MIN;assert(RC_FrictionWheelRpm()==0);
  enabled=0;assert(RC_FrictionWheelRpm()==0);
  enabled=1;online=0;assert(RC_FrictionWheelRpm()==0);
  puts("PASS: midpoint5560, endpoints, master/link stop, RC mapping (feed bounds covered by test_dial_jam.py)");
}
'''
with tempfile.TemporaryDirectory(prefix='baseline-test-') as temp:
    path = Path(temp) / 'test.c'
    path.write_text(code)
    exe = Path(temp) / 'test'
    subprocess.run(['cc', '-std=c11', str(path), '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
