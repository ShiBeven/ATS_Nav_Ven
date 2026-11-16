"""Execute actual C snapshot functions; check stable publication and wire layout.
Does not emulate the USB controller, mechanics or physical CAN transmission.
"""
from pathlib import Path
import contextlib,importlib.util,io,json,re,struct
from pycparser import c_parser,c_ast as A
ROOT=Path(__file__).resolve().parents[2]
spec=importlib.util.spec_from_file_location('calibration_verifier',ROOT/'验证/小Yaw校准/verify_calibration.py')
v=importlib.util.module_from_spec(spec)
with contextlib.redirect_stdout(io.StringIO()):spec.loader.exec_module(v)
source=(ROOT/'application/gimbal_task.c').read_bytes().decode('latin1')
def extract(name):
    m=re.search(r'^(?:static )?void '+name+r'\([^;]*?\)\s*\{',source,re.M)
    assert m,name
    opening=source.index('{',m.start());end=opening+1;depth=1
    while depth:
        depth+=(source[end]=='{')-(source[end]=='}');end+=1
    return source[m.start():end]
code='\n'.join(extract(n) for n in ['gimbal_publish_debug_snapshot','gimbal_get_debug_snapshot'])
tree=c_parser.CParser().parse('typedef unsigned int uint32_t;\n'+re.sub(r'/\*.*?\*/|//[^\n]*','',code.replace('\r',''),flags=re.S))
x=v.Machine();x.funcs.update({n.decl.name:n for n in tree.ext if isinstance(n,A.FuncDef)})
x.g.update(GIMBAL_DEBUG_CHANNELS=20,gimbal_debug_values=[0.]*20,gimbal_behaviour=1,
           pitch_can_set_current=-16000,yaw2_can_set_current=1234,yaw_can_set_current=-4321,
           dm_gyro_online=1,dm_gyro_age_ms=7)
x.now=12345
for name,base in [('gimbal_pitch_motor',10),('gimbal_yaw2_motor',20),('gimbal_yaw_motor',30)]:
    x.g['gimbal_control'][name].update(relative_angle_set=base+.1,relative_angle=base+.2,
        absolute_angle=base+.3,motor_gyro_set=base+.4,motor_gyro=base+.5)
expected=[12.345,1,10.1,10.2,10.3,10.4,10.5,-16000,20.1,20.2,20.4,20.5,1234,30.1,30.2,30.4,30.5,-4321,1,7]
x.call('gimbal_publish_debug_snapshot')
for got,wanted in zip(x.g['gimbal_debug_values'],expected):assert abs(got-wanted)<1e-8
# A reader must see the published cycle, even if live control fields have changed.
x.g['gimbal_control']['gimbal_pitch_motor']['relative_angle']=99.
x.g['pitch_can_set_current']=0;x.g['gimbal_behaviour']=0
out=[None]*20;x.call('gimbal_get_debug_snapshot',out)
assert out==x.g['gimbal_debug_values'] and out[1]==1 and out[3]==10.2 and out[7]==-16000
x.call('gimbal_publish_debug_snapshot');x.call('gimbal_get_debug_snapshot',out)
assert out[1]==0 and out[3]==99. and out[7]==0
cfg=json.loads((ROOT/'SerialPlotter-当前三轴20通道.json').read_text(encoding='utf-8'))
assert cfg['channelsCount']=='20' and len(cfg['channels'])==20
assert cfg['headerHex']=='AA 55' and cfg['footerHex']=='0D 0A' and cfg['dataType']=='float32' and cfg['endianness']=='little'
packet=b'\xaa\x55'+struct.pack('<20f',*expected)+b'\r\n'
assert len(packet)==84 and struct.unpack('<20f',packet[2:-2])[7]==-16000
usb=(ROOT/'application/usb_task.c').read_bytes().decode('latin1')
assert 'packet[2U + GIMBAL_DEBUG_CHANNELS * sizeof(float) + 2U]' in usb
assert 'packet[sizeof(packet) - 2U] = 0x0D;' in usb and 'cdc->TxState != 0U' in usb
print(json.dumps({'calibration_and_init_groups':len(v.PASSED),'snapshot_checks':['all 20 channels match the wire contract','live changes cannot tear the published sample','next publication advances the sample','84-byte float32 frame and configuration agree'], 'limitations':__doc__},ensure_ascii=False,indent=2))
