"""Pitch auxiliary framing: fragmentation, corruption and coexistence with replies."""
import pathlib,sys,struct,unittest
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]/'tools'))
from telemetry_protocol import AuxiliaryDecoder

def packet(kind,payload):
 b=bytearray(b'\xa6\x5a')+bytes([kind])+payload
 checksum=0
 for x in b:checksum^=x
 return bytes(b)+bytes([checksum])
class PitchTelemetryTest(unittest.TestCase):
 def test_fragmented_and_calibration(self):
  b=packet(0xc1,struct.pack('<5f',26000,-2.5,0,2,0))+packet(0xb1,struct.pack('<h',-2500))
  d=AuxiliaryDecoder();frames=[]
  for x in b:frames.extend(d.feed(bytes([x])))
  self.assertEqual(len(frames),2);self.assertEqual(frames[0][1]['raw'],26000)
  self.assertAlmostEqual(frames[0][1]['position_deg'],-143.23945,places=4)
  self.assertEqual(frames[1],(0xb1,-2500))
 def test_exit_event(self):
  b=packet(0xc2,bytes([5,13]))
  d=AuxiliaryDecoder()
  self.assertEqual(d.feed(b[:4]),[])
  self.assertEqual(d.feed(b[4:]),[(0xc2,(5,13))])
 def test_corrupt_then_valid(self):
  good=packet(0xc1,struct.pack('<5f',30000,0.1,0,1,0));bad=bytearray(good);bad[-1]^=1
  result=AuxiliaryDecoder().feed(b'noise'+bad+good)
  self.assertEqual(len(result),1);self.assertEqual(result[0][1]['raw'],30000)
 def test_reject_nonfinite_and_invalid_raw(self):
  for raw,rad in [(float('nan'),0),(1,float('nan')),(-1,0),(65536,0)]:
   self.assertEqual(AuxiliaryDecoder().feed(packet(0xc1,struct.pack('<5f',raw,rad,0,0,0))),[])
if __name__=='__main__':unittest.main()
