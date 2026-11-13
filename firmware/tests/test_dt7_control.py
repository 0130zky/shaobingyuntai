"""Protocol vectors from the supplied DT7 bench report and sender fail-safe."""
import sys
import time
import unittest
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from dt7_control import packet, DT7Control

class DT7Test(unittest.TestCase):
    def test_document_vectors(self):
        self.assertEqual(packet().hex(), '5a100136000000200000000000000000000000004aea')
        self.assertEqual(packet(1).hex(), '5a10013600000020000000000000000001000000f1f6')

    def test_mouse_signed_and_buttons(self):
        self.assertEqual(packet(8, (-2, 32767, -32768), (1, 0))[8:18].hex(),
                         'feffff7f008001000800')

    def test_panel_timeout_disconnect_and_wire_heartbeat(self):
        class Port:
            frames = []
            def write(self, data):
                self.frames.append(data)
                return len(data)
            def close(self):
                pass
        port = Port()
        with patch('dt7_control.SerialPort', return_value=port):
            relay = DT7Control()
            try:
                relay.connect('fake')
                relay.update(1)
                time.sleep(.19)
                keys = [int.from_bytes(f[16:18], 'little') for f in port.frames]
                self.assertIn(0x4001, keys)
                self.assertIn(0xc001, keys)
                time.sleep(.15)
                self.assertEqual(port.frames[-1], packet())
                self.assertEqual(relay.snapshot()['keys'], 0)
                relay.update(4)
            finally:
                relay.disconnect()
            self.assertEqual(port.frames[-3:], [packet()] * 3)
            self.assertFalse(relay.snapshot()['connected'])

if __name__ == '__main__':
    unittest.main()
