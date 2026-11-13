#!/usr/bin/env python3
"""Run the real SBUS decoder on the host with a mocked HAL clock."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory(prefix='sbus-test-') as tmp:
    temp = Path(tmp)
    (temp / 'main.h').write_text('''#pragma once
#include <stdint.h>
uint32_t HAL_GetTick(void);
''')
    (temp / 'test.c').write_text(r'''
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "app_et07.h"
volatile uint32_t pack_time;
uint32_t HAL_GetTick(void) { return 1234; }

int main(void)
{
    uint8_t frame[25] = {0x0f};
    const uint16_t values[10] = {353, 1024, 1696, 900, 353, 1696, 1696, 1024, 500, 800};
    for (unsigned ch = 0; ch < 10; ++ch)
        for (unsigned bit = 0; bit < 11; ++bit)
            if (values[ch] & (1u << bit))
                frame[1 + (ch * 11 + bit) / 8] |= 1u << ((ch * 11 + bit) % 8);
    ET07_Data_t data = {0};
    Get_ET07_DataDecode(&data, frame);
    assert(pack_time == 1234);
    assert(data.RightTransverse_CH1 == 353 && data.RightDirection_CH2 == 1024);
    assert(data.LeftDirection_CH3 == 1696 && data.LeftTransverse_CH4 == 900);
    assert(data.SA_CH5 == 353 && data.SB_CH6 == 1696 && data.SC_CH7 == 1696);
    assert(data.SD_CH8 == 1024 && data.V1_CH9 == 500 && data.V2_CH10 == 800);

    /* Digital SBUS channels are valid; only lost-frame/failsafe bits stop control. */
    frame[23] = 0x03;
    Get_ET07_DataDecode(&data, frame);
    assert(pack_time == 1234 && data.ET07_Flag == 0x03);
    ET07_Data_t previous = data;
    const uint8_t errors[] = {0x04, 0x08, 0x0c, 0x0f};
    for (unsigned i = 0; i < sizeof(errors); ++i) {
        pack_time = 1234;
        frame[23] = errors[i];
        frame[1] ^= 0xff;
        Get_ET07_DataDecode(&data, frame);
        assert(pack_time == ((errors[i] & 0x08) ? 0u : 1234u));
        assert(memcmp(&data, &previous, sizeof(data)) == 0);
    }
    frame[23] = 0;
    frame[0] = 0;
    pack_time = 1234;
    Get_ET07_DataDecode(&data, frame);
    assert(pack_time == 1234 && memcmp(&data, &previous, sizeof(data)) == 0);
    frame[0] = 0x0f;
    frame[24] = 0xff;
    pack_time = 1234;
    Get_ET07_DataDecode(&data, frame);
    assert(pack_time == 1234 && memcmp(&data, &previous, sizeof(data)) == 0);
    assert(g_sbus_frame_lost_count == 1);
    assert(g_sbus_failsafe_count == 3);
    assert(g_sbus_invalid_count == 2);
    frame[24] = 0;
    Get_ET07_DataDecode(&data, frame);
    assert(pack_time == 1234);  /* Valid reception recovers. */
    puts("SBUS: channel decode, digital flags, frame loss, failsafe, invalid header/tail, recovery passed");
}
''')
    subprocess.run([
        'cc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-I', str(temp),
        '-I', str(ROOT / 'Components/Devices'),
        str(ROOT / 'Components/Devices/app_et07.c'), str(temp / 'test.c'),
        '-o', str(temp / 'test'),
    ], check=True)
    subprocess.run([str(temp / 'test')], check=True)
