"""Exercise real RX/task functions with RC, USB and clock hardware stubs."""
from pathlib import Path
import subprocess,json,re,hashlib
OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
v2=(ROOT/'验证/跟随发射分离第2版/verify_tracking.py').read_text(encoding='utf-8')
ns={'__file__':str(__file__)}
# Reuse the historical parser cases plus version 2's policy/time boundary cases.
exec(v2.split('old_source=')[0],ns)
scope=ns['scope'];parts=ns['parts'];tests=ns['tests'];integration=ns['integration']
header=(ROOT/'application/usb_task.h').read_text(encoding='utf-8')
enabled=int(re.search(r'#define USB_ATS_LEGACY_RC_AIM_COMPAT\s+(\d)',header)[1])
stubs=r'''
#define USB_ATS_LEGACY_RC_AIM_COMPAT COMPAT_SETTING
#define RC_Left 1
#define RC_Right 0
#define switch_is_mid(x) ((x)==3)
#define switch_is_down(x) ((x)==2)
typedef struct { struct { uint8_t s[2]; } rc; } RC_ctrl_t;
static RC_ctrl_t test_rc={{{3,2}}};
static uint8_t remote_error;
static const RC_ctrl_t *get_remote_control_point(void) { return &test_rc; }
static uint8_t RC_data_is_error(void) { return remote_error; }
static volatile uint8_t usb_nav_compat_active;
static uint8_t nav54_stamp_seen,nav54_compat_angles_valid;
static uint32_t nav54_sender_stamp,nav54_stamp_changed_at;
'''
parts.insert(2,stubs)
idx=next(i for i,p in enumerate(parts) if 'static void unified_nav_apply_ats_gcc_54(' in p)
parts.insert(idx,scope['function']('usb_nav_compat_allowed'))
more=r'''
  tick=10000; vision_last_rx=tick-200;
  test_rc.rc.s[0]=2; test_rc.rc.s[1]=3;
#if USB_ATS_LEGACY_RC_AIM_COMPAT
  /* Ten seconds of tracking=0/fire=0 with fresh commands must keep aiming. */
  for(i=0;i<500;++i) {
    tick+=20; unified54(nav54,0,3,0,1,0);
    memcpy(nav54+4,&tick,4); field=0.2f+i*0.001f; memcpy(nav54+41,&field,4);
    checksum(nav54,54); feed(nav54,54); run_task_once();
    CHECK(usb_nav_compat_active==1 && nav_override==1 && ReceivedPacketVision.mode==1);
    CHECK(ReceivedPacketVision.pitch==field && RECEIVE_ROBOT_CMD_DATA.data.tracking.tracking==0);
  }
  for(i=0;i<20;++i) {
    tick+=20; unified54(nav54,0,(uint8_t)(i%7),(uint8_t)(i&1),1,0);
    memcpy(nav54+4,&tick,4);checksum(nav54,54);feed(nav54,54);run_task_once();
    CHECK(usb_nav_compat_active==1 && nav_override==1 && ReceivedPacketVision.mode==((i&1)?2:1));
  }
  /* All switch positions, not posture or fire, determine local authorization. */
  for(int right=1;right<=3;++right) for(int left=1;left<=3;++left) {
    test_rc.rc.s[0]=(uint8_t)right;test_rc.rc.s[1]=(uint8_t)left;
    tick+=20;unified54(nav54,0,1,1,1,0);memcpy(nav54+4,&tick,4);checksum(nav54,54);feed(nav54,54);run_task_once();
    CHECK(usb_nav_compat_active==((right==2&&left==3)?1:0));
    CHECK(ReceivedPacketVision.mode==((right==2&&left==3)?2:0));
  }
  test_rc.rc.s[0]=2;test_rc.rc.s[1]=3;
  tick+=20;memcpy(nav54+4,&tick,4);checksum(nav54,54);feed(nav54,54);
  CHECK(usb_nav_compat_active==1);
  /* Exit is processed without waiting for the next packet. */
  test_rc.rc.s[0]=3;++tick;run_task_once();
  CHECK(usb_nav_compat_active==0 && nav_override==0 && ReceivedPacketVision.mode==0);
  test_rc.rc.s[0]=2;run_task_once();CHECK(nav_override==0);
  ++tick;memcpy(nav54+4,&tick,4);checksum(nav54,54);feed(nav54,54);
  remote_error=1;++tick;run_task_once();
  CHECK(usb_nav_compat_active==0 && nav_override==0 && ReceivedPacketVision.mode==0);
  feed(nav54,54);CHECK(nav_override==0);remote_error=0;
  ++tick;memcpy(nav54+4,&tick,4);checksum(nav54,54);feed(nav54,54);
  /* Repeated valid frames cannot keep a frozen sender timestamp alive. */
  tick+=100;feed(nav54,54);run_task_once();CHECK(nav_override==1);
  ++tick;feed(nav54,54);run_task_once();CHECK(nav_override==0 && ReceivedPacketVision.mode==0);
  ++tick;memcpy(nav54+4,&tick,4);checksum(nav54,54);feed(nav54,54);CHECK(nav_override==1);
  /* Actual receive loss, independently of frozen/repeated packets. */
  tick+=100;run_task_once();CHECK(nav_override==1);
  ++tick;run_task_once();CHECK(nav_override==0 && usb_nav_compat_active==0);
  /* Non-finite commanded angles cannot enable legacy compatibility. */
  for(i=0;i<3;++i) {
    uint32_t invalid_bits=(i==0?0x7FC00000U:(i==1?0x7F800000U:0xFF800000U));
    ++tick;unified54(nav54,0,3,0,1,0);memcpy(nav54+4,&tick,4);
    memcpy(nav54+41,&invalid_bits,4);checksum(nav54,54);feed(nav54,54);
    CHECK(nav_override==0 && usb_nav_compat_active==0 && ReceivedPacketVision.mode==0);
    ++tick;unified54(nav54,0,3,0,1,0);memcpy(nav54+4,&tick,4);
    memcpy(nav54+45,&invalid_bits,4);checksum(nav54,54);feed(nav54,54);CHECK(nav_override==0);
  }
  ++tick;unified54(nav54,0,3,0,1,0);memcpy(nav54+4,&tick,4);checksum(nav54,54);feed(nav54,54);
  CHECK(nav_override==1 && ReceivedPacketVision.mode==1);
  /* CRC-bad changes cannot refresh timestamp freshness. */
  before=tick;tick+=100;nav54[4]^=1;feed(nav54,54);run_task_once();CHECK(nav_override==1);
  ++tick;run_task_once();CHECK(nav_override==0 && nav_last_rx==before);
  /* A fresh independent vision source resumes when the operator leaves. */
  ++tick;unified54(nav54,0,3,0,1,0);memcpy(nav54+4,&tick,4);checksum(nav54,54);feed(nav54,54);
  legacy(one,1,0.75f);feed(one,28);CHECK(ReceivedPacketVision.yaw==-0.45f);
  test_rc.rc.s[0]=3;++tick;run_task_once();CHECK(nav_override==0 && ReceivedPacketVision.yaw==0.75f);
  /* Switching protocols must clear compatibility ownership metadata. */
  test_rc.rc.s[0]=2;++tick;feed(nav54,54);CHECK(usb_nav_compat_active==1);
  unified(nav52,1,52);nav52[48]=1;checksum(nav52,52);feed(nav52,52);
  CHECK(usb_nav_compat_active==0 && nav_override==1);
  test_rc.rc.s[0]=3;++tick;run_task_once();CHECK(nav_override==1);
  test_rc.rc.s[0]=2;++tick;feed(nav54,54);CHECK(usb_nav_compat_active==1);
  unified(nav50,1,50);nav50[46]=1;checksum(nav50,50);feed(nav50,50);
  CHECK(usb_nav_compat_active==0 && nav_override==1);
  test_rc.rc.s[0]=3;++tick;run_task_once();CHECK(nav_override==1);
  /* Real tracking remains the strict source and clears the legacy flag. */
  test_rc.rc.s[0]=2;++tick;feed(nav54,54);CHECK(usb_nav_compat_active==1);
  nav54[51]=1;checksum(nav54,54);feed(nav54,54);CHECK(usb_nav_compat_active==0 && nav_override==1);
  /* Both the sender and local clock may wrap. */
  unified54(nav54,0,3,0,1,0);tick=0xFFFFFFF0U;vision_last_rx=tick-200;
  memcpy(nav54+4,&tick,4);checksum(nav54,54);feed(nav54,54);CHECK(nav_override==1);
  tick=0x20U;memcpy(nav54+4,&tick,4);checksum(nav54,54);feed(nav54,54);run_task_once();CHECK(nav_override==1);
  tick=0x84U;feed(nav54,54);run_task_once();CHECK(nav_override==1);
  tick=0x85U;feed(nav54,54);run_task_once();CHECK(nav_override==0);
#else
  unified54(nav54,0,3,0,1,0);memcpy(nav54+4,&tick,4);checksum(nav54,54);feed(nav54,54);run_task_once();
  CHECK(usb_nav_compat_active==0 && nav_override==0 && ReceivedPacketVision.mode==0);
#endif
'''
tests=tests.replace('  printf("PASS %d checks:',more+'  printf("PASS %d checks:',1)
body='\n'.join(parts)+integration+tests
results={}
for name,setting,negative in [('v3',enabled,False),('strict_config',0,False),('v2_negative_control',1,True)]:
 code=body.replace('COMPAT_SETTING',str(setting))
 if negative:
  code=code.replace('!RECEIVE_ROBOT_CMD_DATA.data.tracking.tracking && usb_nav_compat_allowed(now);','0U;',1)
 cpp=OUT/(name+'.cpp');exe=OUT/(name+'.exe');cpp.write_text(code,encoding='ascii')
 build=subprocess.run([str(ns['CXX']),'-std=c++11','-Wall','-Wextra',str(cpp),'-o',str(exe)],cwd=OUT,capture_output=True)
 (OUT/(name+'-build.log')).write_bytes(build.stdout+build.stderr)
 assert build.returncode==0,build.stderr.decode(errors='replace')
 run=subprocess.run([str(exe)],cwd=OUT,capture_output=True)
 result=run.stdout.decode(errors='replace')+run.stderr.decode(errors='replace');print(name+': '+result.strip())
 (OUT/(name+'-result.txt')).write_text(result,encoding='utf-8');results[name]={'returncode':run.returncode,'output':result}
assert results['v3']['returncode']==0 and results['strict_config']['returncode']==0
assert results['v2_negative_control']['returncode']!=0
assert 'usb_nav_compat_active==1' in results['v2_negative_control']['output']
before=json.loads((OUT/'修改前文件哈希.json').read_text(encoding='utf-8'))
changed=[rel for rel,h in before.items() if Path(rel).suffix.lower() in {'.c','.h','.s','.uvprojx','.uvoptx'} and hashlib.sha256((ROOT/rel).read_bytes()).hexdigest()!=h]
assert set(changed)=={'application/usb_task.c','application/usb_task.h'},changed
results['changed_firmware_files']=changed;results['hardware_tested']=False
(OUT/'验证结果.json').write_text(json.dumps(results,ensure_ascii=False,indent=2),encoding='utf-8')
