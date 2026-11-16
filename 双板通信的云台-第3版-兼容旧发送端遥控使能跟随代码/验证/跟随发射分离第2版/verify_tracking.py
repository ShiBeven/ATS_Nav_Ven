"""Run the actual firmware parser and task, including the earlier protocol cases."""
from pathlib import Path
import hashlib,json,subprocess,re

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
BASE=Path(r'D:\code-ai\双板通信的云台-第2版修改前完整备份-20261116')
CXX=Path(r'D:\C语言\Dev-Cpp\TDM-GCC-64\bin\g++.exe')
legacy_test=(ROOT/'验证/视觉接口协议对齐/verify_visual_protocol.py').read_text(encoding='utf-8')
# Reuse the actual-function extraction and historical protocol cases, without
# running their obsolete baseline checks or changing the archived test script.
scope={'__file__':str(__file__)}
exec(legacy_test.split('host = OUT /')[0],scope)
parts=scope['parts']
parts.insert(1, '''
enum mode { Attack=1, Defend=2, Move=3, EnhancedAttack=4, EnhancedDefend=5, EnhancedMove=6 };
static uint8_t get_yaw_joint_angles(float *big_joint, float *small_joint) {
  *big_joint=angles[0]; *small_joint=angles[0]; return 1;
}
''')
tests=scope['tests']
old='''  ++tick; feed(nav54_attack, 54);
  CHECK(nav_last_rx == tick && nav_override == 1 && ReceivedPacketVision.mode == 1);
  CHECK(ReceivedPacketVision.pitch == 0.35f && ReceivedPacketVision.yaw == -0.45f);'''
new='''  tick += 200; feed(nav54_attack, 54);
  CHECK(nav_last_rx == tick && nav_override == 0 && ReceivedPacketVision.mode == 0);'''
assert old in tests
tests=tests.replace(old,new,1)

integration=r'''
#include <setjmp.h>
#define USB_ATS_COMPAT_FEEDBACK_TX 1
#define USB_LEGACY_VISION_TX 1
static jmp_buf loop_end;
static void MX_USB_DEVICE_Init(void) {}
static void CAN_CmdBoard3(void *p) { (void)p; }
static void unified_usb_send_ats_feedback(void) {}
static void osDelay(int n) { (void)n; longjmp(loop_end,1); }
'''+scope['function']('usb_task')+r'''
static void run_task_once(void) { if (!setjmp(loop_end)) usb_task(0); }
'''

extra=r'''
  /* Replay the observed Move/tracking=0/fire toggle: fire cannot own aim. */
  tick += 200;
  for (i = 0; i < 8; ++i) {
    unified54(nav54, 0, 3, (uint8_t)(i & 1), 1, 0);
    tick += 20; feed(nav54,54); run_task_once();
    CHECK(nav_override == 0 && ReceivedPacketVision.mode == 0);
    CHECK(Navigation_speed_set.vx == 0.75f);
  }
  /* A fresh valid target tracks across all fire transitions and postures. */
  for (i = 0; i < 20; ++i) {
    unified54(nav54, 0, (uint8_t)(i % 7), (uint8_t)(i & 1), 1, 1);
    field = 0.1f + i * 0.01f; memcpy(nav54+41,&field,4); checksum(nav54,54);
    tick += 20; feed(nav54,54); run_task_once();
    CHECK(nav_override == 1 && ReceivedPacketVision.mode == ((i&1)?2:1));
    CHECK(ReceivedPacketVision.pitch == field && nav_last_rx == tick);
  }
  /* Explicit loss and conflicting fire must release immediately. */
  unified54(nav54,0,1,1,1,0); ++tick; feed(nav54,54);
  CHECK(nav_override == 0 && ReceivedPacketVision.mode == 0);
  /* Fresh independent vision is held while nav owns aim, then restored. */
  unified54(nav54,0,3,0,1,1); ++tick; feed(nav54,54);
  legacy(one,1,0.75f); feed(one,28);
  CHECK(nav_override == 1 && ReceivedPacketVision.yaw == -0.45f);
  unified54(nav54,0,3,0,1,0); feed(nav54,54);
  CHECK(nav_override == 0 && ReceivedPacketVision.mode == 1 && ReceivedPacketVision.yaw == 0.75f);
  /* No stale independent source can silently resume control. */
  tick += 101; feed(nav54,54); run_task_once();
  CHECK(nav_override == 0 && ReceivedPacketVision.mode == 0);
  /* CRC failure may not drop a currently valid target. */
  unified54(nav54,0,3,0,1,1); ++tick; feed(nav54,54);
  before=nav_last_rx; nav54[51]=0; ++tick; feed(nav54,54);
  CHECK(nav_override == 1 && nav_last_rx == before && ReceivedPacketVision.mode == 1);
  /* Real task timeout, including the fresh-vision handover at timeout. */
  unified54(nav54,0,3,0,1,1); ++tick; feed(nav54,54);
  tick += 100; run_task_once(); CHECK(nav_override == 1 && ReceivedPacketVision.mode == 1);
  ++tick; run_task_once(); CHECK(nav_override == 0 && ReceivedPacketVision.mode == 0);
  CHECK(Navigation_speed_set.vx == 0);
  ++tick; feed(nav54,54); tick += 90; legacy(one,1,0.75f); feed(one,28);
  tick += 11; run_task_once();
  CHECK(nav_override == 0 && ReceivedPacketVision.mode == 1 && ReceivedPacketVision.yaw == 0.75f);
  /* 32-bit tick rollover preserves fresh navigation until the boundary. */
  tick=0xFFFFFFF0U; feed(nav54,54); vision_last_rx=tick-200;
  tick=0x20U; run_task_once(); CHECK(nav_override == 1 && ReceivedPacketVision.mode == 1);
  tick=0x55U; run_task_once(); CHECK(nav_override == 0 && ReceivedPacketVision.mode == 0);
'''
needle='  printf("PASS %d checks:'
assert needle in tests
tests=tests.replace(needle,extra+needle,1)
fixed='\n'.join(parts)+integration+tests
old_source=(BASE/'application/usb_task.c').read_text(encoding='utf-8')
old_scope=dict(scope)
old_scope['clean']=re.sub(r'/\*.*?\*/|//[^\n]*','',old_source,flags=re.S)
# function() closes over the scope dictionary; use the original globals briefly.
clean_saved=scope['clean'];scope['clean']=old_scope['clean']
old_func=scope['function']('unified_nav_apply_ats_gcc_54')
scope['clean']=clean_saved
fixed_func=scope['function']('unified_nav_apply_ats_gcc_54')
mutated=fixed.replace(fixed_func,old_func,1)
results={}
for name,body in [('v2',fixed),('v1_policy_negative_control',mutated)]:
 cpp=OUT/(name+'.cpp');exe=OUT/(name+'.exe');cpp.write_text(body,encoding='ascii')
 build=subprocess.run([str(CXX),'-std=c++11','-Wall','-Wextra',str(cpp),'-o',str(exe)],cwd=OUT,capture_output=True)
 (OUT/(name+'-build.log')).write_bytes(build.stdout+build.stderr)
 assert build.returncode==0,build.stderr.decode(errors='replace')
 run=subprocess.run([str(exe)],cwd=OUT,capture_output=True)
 result=run.stdout.decode(errors='replace')+run.stderr.decode(errors='replace')
 (OUT/(name+'-result.txt')).write_text(result,encoding='utf-8')
 results[name]={'returncode':run.returncode,'output':result}
 print(name+': '+result.strip())
assert results['v2']['returncode']==0
assert results['v1_policy_negative_control']['returncode']!=0
assert 'nav_override == 0 && ReceivedPacketVision.mode == 0' in results['v1_policy_negative_control']['output']
# Preserve the timing repair, other receive paths, and all non-USB firmware.
before=json.loads((OUT/'修改前文件哈希.json').read_text(encoding='utf-8'))
suffixes={'.c','.h','.s','.uvprojx','.uvoptx'}
changed=[]
for rel,digest in before.items():
 if Path(rel).suffix.lower() in suffixes and hashlib.sha256((ROOT/rel).read_bytes()).hexdigest()!=digest:
  changed.append(rel)
assert set(changed)=={'application/usb_task.c','application/usb_task.h'},changed
new_source=(ROOT/'application/usb_task.c').read_text(encoding='utf-8')
for name in ['usb_task','unified_nav_apply_50','usb_apply_vision_command','unified_usb_frame','unified_usb_process']:
 current=scope['function'](name);scope['clean']=old_scope['clean'];old_body=scope['function'](name);scope['clean']=clean_saved
 assert current==old_body,name+' unexpectedly changed'
results['changed_firmware_files']=changed
results['sender_updated']=False
results['hardware_tested']=False
(OUT/'验证结果.json').write_text(json.dumps(results,ensure_ascii=False,indent=2),encoding='utf-8')
print('PASS: actual parser/task, legacy protocols, handover, real timeout and policy negative control.')
