"""Interpret the actual calibration C function bodies for logic/geometry checks.

Uses pycparser and a deliberately small C interpreter, without hardware access.
Floating-point expressions use Python precision; Keil separately compiles target C.
This does not model mechanics, interrupt scheduling, CAN transmission, or Flash wear.
"""
from pathlib import Path
import re, math, copy, json, hashlib
from pycparser import c_parser, c_ast as A

ROOT = Path(__file__).resolve().parents[2]
SOURCE = (ROOT/'application/gimbal_task.c').read_bytes().decode('latin1')
IMPL = SOURCE[SOURCE.index('/* Pitch and limited small-Yaw calibration.'):SOURCE.index('\nuint16_t b;')]
PRELUDE = '''
typedef unsigned char uint8_t; typedef unsigned short uint16_t;
typedef unsigned int uint32_t; typedef short int16_t; typedef int int32_t;
typedef float fp32; typedef int bool_t;
typedef struct {uint16_t ecd; int16_t speed_rpm;} motor_measure_t;
typedef struct {fp32 max_yaw,min_yaw,max_pitch,min_pitch;
uint16_t max_yaw_ecd,min_yaw_ecd,max_pitch_ecd,min_pitch_ecd;uint8_t step;} gimbal_step_cali_t;
'''
def extract_function(name):
    match=re.search(r'^\s*void '+name+r'\([^;]*?\)\s*\{',SOURCE,re.M)
    assert match,name
    opening=SOURCE.index('{',match.start());depth=1;end=opening+1
    while depth:
        if SOURCE[end]=='{':depth+=1
        elif SOURCE[end]=='}':depth-=1
        end+=1
    return SOURCE[match.start():end]
IMPL += '\n'+extract_function('GetgimbalMode')+'\n'+extract_function('Gimbal_Zero_Force')+'\n'+extract_function('Gimbal_Init')
PRELUDE += 'typedef int gimbal_behaviour_e;\n'

TREE = c_parser.CParser().parse(PRELUDE + re.sub(r'/\*.*?\*/|//[^\n]*', '', IMPL.replace('\r',''), flags=re.S))

class Ref:
    def __init__(self, obj, key): self.obj, self.key = obj, key
    def get(self): return self.obj[self.key]
    def set(self, value): self.obj[self.key] = value
class Returned(Exception):
    def __init__(self, value): self.value = value

class Machine:
    def __init__(self):
        self.types = {n.name:n.type for n in TREE.ext if isinstance(n,A.Typedef)}
        self.funcs = {n.decl.name:n for n in TREE.ext if isinstance(n,A.FuncDef)}
        self.now, self.age, self.seen = 1000, 0, True
        self.down, self.rc_error, self.board_error = True, False, False
        self.motor = [{'ecd':2000,'speed_rpm':0}, {'ecd':7773,'speed_rpm':0}, {'ecd':7900,'speed_rpm':0}]
        self.g = {'NULL':0, 'PI':math.pi, 'MOTOR_ECD_TO_RAD':0.000766990394,
                  'PITCH_TURN':1, 'YAW2_TURN':0, 'GIMBAL_MOTOR_RAW':0, 'GIMBAL_ZERO_FORCE':0,
                  'GIMBAL_CALI_PITCH_MAX_STEP':1,'GIMBAL_CALI_PITCH_MIN_STEP':2,
                  'GIMBAL_CALI_YAW_MAX_STEP':3,'GIMBAL_CALI_YAW_MIN_STEP':4,
                  'GIMBAL_CALI_START_STEP':1,'GIMBAL_CALI_END_STEP':5,
                  'GIMBAL_YAW_OFFSET_ECD':7773,
                  'dm_fault_latched':0,'dm_gyro_online':1, 'BOARD_GYRO_TOE':8,
                  'add_pitch_angle':0.,'add_yaw_angle':0.,'add_yaw2_angle':0.,
                  'pitch_can_set_current':0,'yaw_can_set_current':0,'yaw2_can_set_current':0,
                  'gimbal_behaviour':0}
        header = (ROOT/'application/gimbal_task.h').read_bytes().decode('latin1')
        for name, val in re.findall(r'^#define\s+(GIMBAL_CALI_\w+)\s+([\d.]+)[fFUu]*\s*$', header, re.M):
            self.g[name] = float(val) if '.' in val else int(val)
        self.g['gimbal_control'] = {
            'gimbal_pitch_motor':self.motor_control(), 'gimbal_yaw_motor':self.motor_control(),
            'gimbal_yaw2_motor':self.motor_control(), 'gimbal_rc_ctrl':{'rc':{'s':[2,2]}},
            'gimbal_cali':self.zero(self.types['gimbal_step_cali_t'])}
        for n in TREE.ext:
            if isinstance(n,A.Decl) and not isinstance(n.type,A.FuncDecl):
                self.g[n.name] = self.val(n.init,{}) if n.init else self.zero(n.type)
        self.stub = {'HAL_GetTick':lambda:self.now, '__get_PRIMASK':lambda:0,
                     '__disable_irq':lambda:None,'__set_PRIMASK':lambda value:None,
                     'gimbal_total_pid_clear':lambda ptr:None,
                     'RC_data_is_error':lambda:self.rc_error,
                     'switch_is_down':lambda position:self.down,
                     'toe_is_error':lambda toe:self.board_error,
                     'fabsf':abs,'gimbal_get_motor_snapshot':self.snapshot}
    @staticmethod
    def motor_control():
        return dict(gimbal_motor_measure=1, raw_cmd_current=0.,current_set=0.,given_current=0,
                    gimbal_motor_mode=0,motor_gyro=0.,absolute_angle=0.,offset_ecd=0,
                    max_relative_angle=0.,min_relative_angle=0.)
    def zero(self, typ):
        if isinstance(typ,A.TypeDecl): return self.zero(typ.type)
        if isinstance(typ,A.IdentifierType):
            return self.zero(self.types[typ.names[0]]) if typ.names[0] in self.types else 0
        if isinstance(typ,A.Struct): return {d.name:self.zero(d.type) for d in typ.decls}
        if isinstance(typ,A.PtrDecl): return 0
        raise AssertionError(type(typ))
    def snapshot(self, index, out, age):
        out.set(copy.deepcopy(self.motor[index])); age.set(self.age)
        return self.seen
    def ref(self,n,s):
        if isinstance(n,A.ID): return Ref(s if n.name in s else self.g,n.name)
        if isinstance(n,A.UnaryOp) and n.op=='*': return self.val(n.expr,s)
        if isinstance(n,A.StructRef):
            obj=self.val(n.name,s)
            if n.type=='->' and isinstance(obj,Ref): obj=obj.get()
            return Ref(obj,n.field.name)
        if isinstance(n,A.ArrayRef): return Ref(self.val(n.name,s),self.val(n.subscript,s))
        raise AssertionError(type(n))
    def val(self,n,s):
        if n is None: return None
        if isinstance(n,A.Constant):
            raw=re.sub(r'[uUlLfF]+$' if n.type in ('float','double') else r'[uUlL]+$','',n.value)
            return float(raw) if n.type in ('float','double') else int(raw,0)
        if isinstance(n,(A.ID,A.StructRef,A.ArrayRef)): return self.ref(n,s).get()
        if isinstance(n,A.UnaryOp):
            if n.op=='&': return self.ref(n.expr,s)
            if n.op=='*': return self.val(n.expr,s).get()
            v=self.val(n.expr,s)
            if n.op=='!': return int(not v)
            if n.op=='-': return -v
            if n.op=='+': return v
            raise AssertionError(n.op)
        if isinstance(n,A.Cast):
            v=self.val(n.expr,s); names=n.to_type.type.type.names
            return float(v) if names[0] in ('fp32','float') else int(v)
        if isinstance(n,A.BinaryOp):
            a=self.val(n.left,s)
            if n.op=='&&': return int(bool(a) and bool(self.val(n.right,s)))
            if n.op=='||': return int(bool(a) or bool(self.val(n.right,s)))
            b=self.val(n.right,s)
            return self.binary(n.op,a,b)
        if isinstance(n,A.TernaryOp): return self.val(n.iftrue if self.val(n.cond,s) else n.iffalse,s)
        if isinstance(n,A.FuncCall):
            args=[self.val(x,s) for x in n.args.exprs] if n.args else []
            return self.call(n.name.name,*args)
        if isinstance(n,A.Assignment):
            ref=self.ref(n.lvalue,s); value=self.val(n.rvalue,s)
            if n.op!='=': value=self.binary(n.op[:-1],ref.get(),value)
            ref.set(value);return value
        raise AssertionError(type(n))
    @staticmethod
    def binary(op,a,b):
        if op=='+':return a+b
        if op=='-':return a-b
        if op=='*':return a*b
        if op=='/':return int(a/b) if isinstance(a,int) and isinstance(b,int) else a/b
        if op=='%':return a-int(a/b)*b
        if op=='<':return int(a<b)
        if op=='>':return int(a>b)
        if op=='<=':return int(a<=b)
        if op=='>=':return int(a>=b)
        if op=='==':return int(a==b)
        if op=='!=':return int(a!=b)
        raise AssertionError(op)
    def run(self,n,s):
        if n is None:return
        if isinstance(n,A.Compound):
            for x in n.block_items or []:self.run(x,s)
        elif isinstance(n,A.Decl):s[n.name]=self.val(n.init,s) if n.init else self.zero(n.type)
        elif isinstance(n,A.If):self.run(n.iftrue if self.val(n.cond,s) else n.iffalse,s)
        elif isinstance(n,A.Return):raise Returned(self.val(n.expr,s))
        elif isinstance(n,A.While):
            count=0
            while self.val(n.cond,s):
                self.run(n.stmt,s);count+=1
                assert count<10000
        else:self.val(n,s)
    def call(self,name,*args):
        if hasattr(self,'stub') and name in self.stub:return self.stub[name](*args)
        f=self.funcs[name]
        params=f.decl.type.args.params if f.decl.type.args else []
        params=[p for p in params if isinstance(p,A.Decl)]
        assert len(params)==len(args),(name,len(params),args)
        s={p.name:v for p,v in zip(params,args)}
        try:self.run(f.body,s)
        except Returned as r:return r.value
    def outputs(self):
        return [self.g['gimbal_control'][name]['given_current'] for name in
                ('gimbal_pitch_motor','gimbal_yaw_motor','gimbal_yaw2_motor')]
    def pointers(self):
        self.g['test_outputs']=[0]*6
        return [Ref(self.g['test_outputs'],i) for i in range(6)]

PASSED=[]
def ok(name):PASSED.append(name)

# Real C math: wrap at both ends, integer midpoint, 2 independent direction macros.
for turn in (0,1):
    m=Machine();m.g['YAW2_TURN']=turn
    sign=-1 if turn else 1
    for zero in (0,100,4096,7900,8191):
        for span in (600,2000,5000,8000):
            maximum=(zero+sign*(span//2))%8192
            minimum=(maximum-sign*span)%8192
            m.g['gimbal_cali_yaw2_span_counts']=span
            data={'max_yaw_ecd':maximum,'min_yaw_ecd':minimum,
                  'max_pitch_ecd':1478,'min_pitch_ecd':2522,'max_pitch':.4,'min_pitch':-.4}
            m.g['case']=data
            args=m.pointers()
            assert m.call('calc_gimbal_cali',Ref(m.g,'case'),*args)==1
            out=m.g['test_outputs']
            assert abs(m.call('cali_count_delta',out[0],zero))<=1
            assert out[2]>0>out[3] and out[4]>0>out[5]
            assert abs((out[2]-out[3])-(span*m.g['MOTOR_ECD_TO_RAD']-.2))<.002
ok('40 geometry cases: encoder wrap, >180 degree travel, both small-Yaw direction signs')

# Full four-step run: predetermined physical motion and hard stops; run actual C.
m=Machine(); ptrs=m.pointers(); assert m.call('cmd_cali_gimbal_hook',*ptrs)==0
pos={0:0.,2:0.};zero={0:2000,2:7900};prev_step=0
for iteration in range(6000):
    step=m.g['gimbal_control']['gimbal_cali']['step']
    if step==5:break
    assert step in (1,2,3,4)
    if step!=prev_step:
        began=m.now;index=0 if step<=2 else 2;origin=pos[index];prev_step=step
        target={1:.4,2:-.4,3:.6,4:-.6}[step]
        duration=1000 if step in (1,3) else 2000
    elapsed=m.now-began
    pos[index]=origin+(target-origin)*min(1.,elapsed/duration)
    raw_sign=-1 if index==0 else 1
    m.motor[index]['ecd']=round(zero[index]+raw_sign*pos[index]/m.g['MOTOR_ECD_TO_RAD'])%8192
    m.motor[index]['speed_rpm']=round(raw_sign*(target-origin)/(duration/1000)*60/(2*math.pi)) if elapsed<duration else 0
    m.g['gimbal_control']['gimbal_pitch_motor']['absolute_angle']=pos[0]
    m.g['gimbal_control']['gimbal_pitch_motor']['motor_gyro']=(target-origin)/(duration/1000) if index==0 and elapsed<duration else 0.
    m.call('Gimbal_Cali')
    out=m.outputs(); assert out[1]==0
    assert out[2 if step<=2 else 0]==0
    assert m.g['gimbal_cali_status']==1
    m.now+=10
else:raise AssertionError('Four-step run did not terminate')
assert m.outputs()==[0,0,0]
assert m.call('cmd_cali_gimbal_hook',*ptrs)==1
assert m.g['gimbal_cali_status']==2 and m.g['gimbal_cali_valid']==1
assert abs(m.g['gimbal_control']['gimbal_yaw2_motor']['offset_ecd']-7900)<=1
saved=m.g['test_outputs'][:]
ok('Four steps -> validated completion, small-Yaw zero applied, big-Yaw drive always zero')

loaded=Machine();loaded.call('set_cali_gimbal_hook',*saved)
assert loaded.g['gimbal_cali_status']==4 and loaded.g['gimbal_cali_valid']==1
assert loaded.g['gimbal_control']['gimbal_yaw2_motor']['offset_ecd']==saved[0]
ok('Saved values reload into small-Yaw; continuous big-Yaw retains fixed 7773')

def active(step=3):
    x=Machine();x.g['gimbal_cali_status']=1;x.g['gimbal_control']['gimbal_cali']['step']=step
    return x
for kind,error in [('abort',5),('switch',1),('stale',3),('missing',3)]:
    x=active()
    if kind=='abort':x.g['gimbal_cali_abort']=1
    if kind=='switch':x.down=False
    if kind=='stale':x.age=51
    if kind=='missing':x.seen=False
    x.call('Gimbal_Cali')
    assert x.g['gimbal_cali_status']==3 and x.g['gimbal_cali_error']==error and x.outputs()==[0,0,0]
ok('Abort/switch change/stale or missing CAN all stop')

x=active();x.motor[2]['speed_rpm']=-10;x.call('Gimbal_Cali');x.now+=12000;x.call('Gimbal_Cali')
assert x.g['gimbal_cali_status']==3 and x.g['gimbal_cali_error']==2 and x.outputs()==[0,0,0]
ok('Negative rotation does not count as stopped; 12 second stage timeout stops')

x=active(2);x.g['gimbal_control']['gimbal_cali']['max_pitch']=0.
for t in (0,300,2300):x.now=1000+t;x.call('Gimbal_Cali')
assert x.g['gimbal_cali_error']==4 and x.g['gimbal_cali_status']==3 and x.outputs()==[0,0,0]
ok('No actual Pitch travel fails before small-Yaw is driven')

for bad in ((7900,2000,-.1,.1,.3,-.3),(7900,2000,.3,-.3,-.1,.1),(9000,2000,.3,-.3,.3,-.3)):
    x=Machine();x.call('set_cali_gimbal_hook',*bad);assert x.g['gimbal_cali_valid']==0
ok('Inverted bounds or out-of-range encoder zero are rejected on load')

x=active();x.call('Gimbal_Cali')
x.now+=1300;x.call('Gimbal_Cali');assert x.g['gimbal_control']['gimbal_cali']['step']==3
x.motor[2]['ecd']+=10;x.call('Gimbal_Cali')
x.now+=1300;x.call('Gimbal_Cali');assert x.g['gimbal_control']['gimbal_cali']['step']==3
x.now+=800;x.call('Gimbal_Cali');assert x.g['gimbal_control']['gimbal_cali']['step']==4
ok('Motion resets the still timer: separated short pauses cannot accumulate a false stop')

for span in (0,-1000,8192,80):
    x=Machine();x.g['gimbal_cali_yaw2_span_counts']=span
    x.g['case']={'max_yaw_ecd':489,'min_yaw_ecd':7118,
                 'max_pitch_ecd':1478,'min_pitch_ecd':2522,'max_pitch':.4,'min_pitch':-.4}
    assert x.call('calc_gimbal_cali',Ref(x.g,'case'),*x.pointers())==0
ok('Zero/reversed/full-turn/too-narrow small-Yaw travel is rejected')

x=Machine();x.g['gimbal_cali_yaw2_span_counts']=1564
x.g['case']={'max_yaw_ecd':489,'min_yaw_ecd':7118,
             'max_pitch_ecd':1739,'min_pitch_ecd':2261,'max_pitch':.4,'min_pitch':-.4}
assert x.call('calc_gimbal_cali',Ref(x.g,'case'),*x.pointers())==0
ok('Incompatible Pitch encoder/IMU angular model is rejected instead of saved')


def mode_case(status, valid, step):
    x=Machine()
    x.g.update(GIMBAL_CALI=2,GIMBAL_INIT=1,GIMBAL_ABSOLUTE_ANGLE=3,
               GIMBAL_VISION=7,AUTO_AIM=8,RC_Right=0,
               gimbal_cali_status=status,gimbal_cali_valid=valid,gimbal_behaviour=2)
    x.stub.update(UseVision=lambda:0,switch_is_mid=lambda value:False,
                  switch_is_up=lambda value:False,PID_clear=lambda ptr:None,fabs=abs)
    x.g['gimbal_control']['gimbal_cali']['step']=step
    x.g['gimbal_control']['gimbal_yaw_motor']['given_current']=-8000
    x.g['gimbal_control']['gimbal_yaw_motor']['current_set']=-8000.
    x.g['add_yaw_angle']=-8000.;x.g['yaw_can_set_current']=-8000
    return x

x=mode_case(2,1,0)
x.call('GetgimbalMode');assert x.g['gimbal_behaviour']==0
x.call('Gimbal_Zero_Force');assert x.outputs()==[0,0,0]
ok('Screenshot regression: completed step0 with stale CALI/-8000 exits to ZERO and clears drive')

x=mode_case(1,0,5)
x.call('GetgimbalMode');assert x.g['gimbal_behaviour']==0
x.call('Gimbal_Zero_Force');assert x.outputs()==[0,0,0]
assert x.g['gimbal_control']['gimbal_cali']['step']==5
ok('END waits with zero drive without resetting step before the save callback')

x=mode_case(1,0,4)
x.g['gimbal_control']['gimbal_cali']['max_yaw_ecd']=7900
for elapsed in (0,300,2300):
    x.now=1000+elapsed;x.call('Gimbal_Cali')
assert x.g['gimbal_control']['gimbal_cali']['step']==5
assert x.outputs()==[0,0,0]
assert x.g['yaw_can_set_current']==0 and x.g['yaw2_can_set_current']==0
ok('Final reverse endpoint clears both old big-Yaw and small-Yaw CAN commands in that cycle')

for status,valid,step in ((0,0,0),(3,0,0),(1,0,0)):
    x=mode_case(status,valid,step)
    x.call('GetgimbalMode');assert x.g['gimbal_behaviour']==0
    x.call('Gimbal_Zero_Force');assert x.outputs()==[0,0,0]
ok('Uncalibrated/failed/start-transition states do not continue the old negative command')


for step, name, axis in ((1,'gimbal_cali_pitch_command',0),(3,'gimbal_cali_yaw2_command',2)):
    for command, accepted in ((8000,True),(8001,False)):
        x=active(step);x.g[name]=command;x.call('Gimbal_Cali')
        if accepted:
            assert x.g['gimbal_cali_status']==1 and x.outputs()[axis]==8000
            assert x.outputs()[1]==0
        else:
            assert x.g['gimbal_cali_status']==3 and x.g['gimbal_cali_error']==1 and x.outputs()==[0,0,0]
ok('Restored 8000 command accepted on both axes; 8001 still rejected; big-Yaw stays zero')


# A failure raised by gimbal_task must survive the next calibrate_task callback.
# Run the real C callback repeatedly after the fault, with feedback healthy again.
for kind,error in [('abort',5),('switch',1),('stale',3),('missing',3),('timeout',2),('geometry',4)]:
    x=active(2 if kind=='geometry' else 1); ptrs=x.pointers()
    failed_step=x.g['gimbal_control']['gimbal_cali']['step']
    if kind=='abort': x.g['gimbal_cali_abort']=1
    if kind=='switch': x.down=False
    if kind=='stale': x.age=51
    if kind=='missing': x.seen=False
    if kind=='timeout':
        x.motor[0]['speed_rpm']=10; x.call('Gimbal_Cali'); x.now+=12000
    if kind=='geometry':
        for t in (0,300): x.now=1000+t; x.call('Gimbal_Cali')
        x.now=3300
    x.call('Gimbal_Cali')
    assert x.g['gimbal_cali_status']==3 and x.g['gimbal_cali_error']==error,kind
    frozen_timers=(x.g['gimbal_cali_still_ms'],x.g['gimbal_cali_step_elapsed_ms'])
    x.down=True; x.age=0; x.seen=True; x.motor[0]['speed_rpm']=0
    for retry in range(20):
        x.now+=10
        assert x.call('cmd_cali_gimbal_hook',*ptrs)==0
        assert x.g['gimbal_cali_status']==3 and x.g['gimbal_cali_error']==error, ('FAILURE_AUTOMATICALLY_RESTARTED',kind)
        assert x.g['gimbal_cali_error_step']==failed_step
        assert x.g['gimbal_control']['gimbal_cali']['step']==0 and x.outputs()==[0,0,0]
        assert (x.g['gimbal_cali_still_ms'],x.g['gimbal_cali_step_elapsed_ms'])==frozen_timers
    assert x.g['test_outputs']==[0]*6
    x.call('gimbal_cali_fail',1 if error!=1 else 5)
    assert x.g['gimbal_cali_error']==error and x.g['gimbal_cali_error_step']==failed_step
    # Only a new explicit RC calibration request rearms a failed run.
    x.g['gimbal_cali_status']=0
    assert x.call('cmd_cali_gimbal_hook',*ptrs)==0
    assert x.g['gimbal_cali_status']==1 and x.g['gimbal_control']['gimbal_cali']['step']==1
    assert x.g['gimbal_cali_error']==0 and x.g['gimbal_cali_error_step']==0
ok('Fault between tasks stays latched across 20 callbacks for all 6 remaining fault paths; only explicit rearm starts again')


# The recorded -22 RPM startup must continue rather than abort with error 6.
# Rotation in either direction must still prevent false endpoint completion,
# and a motor that never stops must still be shut down by the stage timeout.
for step in (1,2,3,4):
    for rpm in (-22,22):
        x=active(step); axis=0 if step<=2 else 2
        x.motor[axis]['speed_rpm']=rpm
        began=x.now
        for elapsed in (0,300,2300,11999):
            x.now=began+elapsed; x.call('Gimbal_Cali')
            assert x.g['gimbal_cali_status']==1, ('CALIBRATION_ABORTED_AT_22_RPM',step,rpm)
            assert x.g['gimbal_cali_error']==0
            assert x.g['gimbal_cali_motor_rpm']==rpm
            assert x.g['gimbal_control']['gimbal_cali']['step']==step
            assert x.g['gimbal_cali_still_ms']==0
            assert x.outputs()[axis]==(8000 if step in (1,3) else -8000)
            assert x.outputs()[1]==0
        x.now=began+12000; x.call('Gimbal_Cali')
        assert x.g['gimbal_cali_status']==3 and x.g['gimbal_cali_error']==2
        assert x.g['gimbal_cali_error_step']==step and x.outputs()==[0,0,0]
ok('Recorded +/-22 RPM runs on all 4 stages; spinning never counts as an endpoint; 12s timeout still clears output')


# Execute the real INIT function with stubbed PIDs to isolate target continuity.
# Stale targets model returning from calibration with a changed encoder zero.
init_header=(ROOT/'application/gimbal_task.h').read_bytes().decode('latin1')
for relative,absolute,stale in ((-.45,-.43,1.2),(.38,.4,-.9),(-.2,-.22,.7),(.1,.12,-1.1)):
    x=Machine()
    x.g['GIMBAL_MOTOR_ENCONDE']=2
    for name,val in re.findall(r'^#define\s+((?:GIMBAL_INIT_\w+|INIT_PITCH_SET|INIT_YAW_SET))\s+([\d.]+)[fFUu]*\s*$',init_header,re.M):
        x.g[name]=float(val) if '.' in val else int(val)
    for name in ('gimbal_pitch_motor','gimbal_yaw_motor','gimbal_yaw2_motor'):
        x.g['gimbal_control'][name].update(relative_angle=.2,relative_angle_set=-.8,
            gimbal_motor_relative_angle_pid=0,gimbal_motor_gyro_pid=0)
    x.g.update(add_init_pitch_angle=0.,add_init_yaw_angle=0.,add_init_yaw2_angle=0.)
    x.stub.update(fabs=abs,gimbal_PID_calc=lambda ptr,get,set,delta:set-get,
                  PID_calc=lambda ptr,get,set:set-get)
    pitch=x.g['gimbal_control']['gimbal_pitch_motor']
    pitch.update(relative_angle=relative,absolute_angle=absolute,relative_angle_set=stale)
    increment=(x.g['INIT_PITCH_SET']-absolute)*x.g['GIMBAL_INIT_PITCH_SPEED']
    x.call('Gimbal_Init')
    assert abs(pitch['relative_angle_set']-(relative+increment))<1e-9, ('PITCH_INIT_STALE_TARGET',relative,absolute,stale,pitch['relative_angle_set'])
    assert abs(pitch['motor_gyro_set'])<.005
    assert pitch['gimbal_motor_mode']==2
    # Next cycle must retain the advancing target rather than resetting it.
    x.call('Gimbal_Init')
    assert abs(pitch['relative_angle_set']-(relative+2*increment))<1e-9
    # Re-entry after ZERO/calibration must sync again to the new current angle.
    pitch.update(gimbal_motor_mode=0,relative_angle=relative+.15,relative_angle_set=stale)
    x.call('Gimbal_Init')
    assert abs(pitch['relative_angle_set']-(relative+.15+increment))<1e-9
ok('Actual INIT: 4 stale-target/encoder-zero cases sync Pitch on entry, advance normally, resync on re-entry; PIDs stubbed')

cal=(ROOT/'application/calibrate_task.c').read_bytes().decode('latin1')
assert '"G2P"' in cal and 'memcmp(cali_sensor[i].name, cali_name[i], 3U)' in cal
assert 'else if (i == CALI_GIMBAL && gimbal_cali_status == 3U)' in cal
assert 'if (!gimbal_cali_valid)\r\n        init->gimbal_yaw2_motor.offset_ecd' in SOURCE
manifest=json.loads((ROOT/'小Yaw校准修改记录.json').read_text(encoding='utf-8'))
for rel, hashes in manifest.items():
    assert hashlib.sha256((Path('D:/code-ai/双板通信的云台-修改备份/20261022-小Yaw校准前-163713')/rel).read_bytes()).hexdigest()==hashes['original_sha256']
    assert hashlib.sha256((ROOT/rel).read_bytes()).hexdigest()==hashes['new_sha256']
ok('Flash version rejection, no failed save, startup offset preservation, backup preserves every original source file')

print(json.dumps({'passed':PASSED,'count':len(PASSED),'limitations':__doc__},ensure_ascii=False,indent=2))
