"""Read-only SWD verification of DT7 relay with S1 disabled. No motor enable."""
import json, re, struct, subprocess, sys, threading, time
from pathlib import Path
root=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(root/'firmware/tools'))
from dt7_control import DT7Control
out=Path(__file__).resolve().parent
cmd=['openocd','-f','interface/cmsis-dap.cfg','-c','cmsis_dap_backend usb_bulk',
 '-c','transport select swd','-f','target/stm32h7x.cfg','-c','adapter speed 1000','-c','init',
 '-c','echo REMOTE=[read_memory 0x20007de8 8 36]',
 '-c','echo STATUS=[read_memory 0x200082a4 32 47]','-c','shutdown']
logs=[]
def sample(label):
 r=subprocess.run(cmd,capture_output=True,text=True,timeout=10)
 text=r.stdout+r.stderr;logs.append(text)
 match=re.search(r'REMOTE=([^\n]+)',text)
 if not match:raise RuntimeError(text)
 data=bytes(int(x,0) for x in match[1].split())
 status=[int(x,0) for x in re.search(r'STATUS=([^\n]+)',text)[1].split()]
 result=dict(label=label,channels=list(struct.unpack_from('<4H',data)),s1=data[8],s2=data[9],seen=data[10],
             tick=struct.unpack_from('<I',data,12)[0],frames=struct.unpack_from('<I',data,16)[0],
             mouse=list(struct.unpack_from('<3h',data,24)),keys=struct.unpack_from('<H',data,30)[0],
             fold_raw=status[2],pitch_raw=status[3],armed=status[24],fault=status[25],master=status[29])
 # Offsets: status mode/valid/armed/fault at words22/23/24/25, master word29.
 if result['s1']!=1:raise RuntimeError('S1 no longer disabled; aborting')
 return result
records=[sample('before')]
relay=DT7Control();stop=threading.Event();held=0
try:
 relay.connect('/dev/cu.usbmodem0001A00000001')
 def refresh():
  while not stop.is_set():relay.update(held);stop.wait(.05)
 worker=threading.Thread(target=refresh);worker.start()
 for label,mask in [('idle',0),('W',1),('S',2),('A',4),('D',8),('release',0)]:
  held=mask;time.sleep(.3)
  for i in range(3):records.append(sample(label))
 held=0
finally:
 stop.set()
 if 'worker' in locals():worker.join()
 relay.disconnect()
 time.sleep(.15)
records.append(sample('disconnected'))
(out/'link_samples.json').write_text(json.dumps(records,ensure_ascii=False,indent=2))
(out/'link_samples.log').write_text('\n'.join(logs))
for label,mask in [('idle',0),('W',1),('S',2),('A',4),('D',8),('release',0),('disconnected',0)]:
 rows=[r for r in records if r['label']==label]
 assert any(r['keys']&15==mask for r in rows),(label,rows)
assert all(r['master']==0 for r in records),records
assert max(r['fold_raw'] for r in records)-min(r['fold_raw'] for r in records)<32
assert max(r['pitch_raw'] for r in records)-min(r['pitch_raw'] for r in records)<32
print('PASS live DT7 W/S/A/D press, release and disconnect; S1 up throughout; no enabled motor test.')
print([(r['label'],hex(r['keys'])) for r in records])
