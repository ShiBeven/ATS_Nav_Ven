"""Exercise the actual USB scheduler and existing RX/TX regression tests."""
from pathlib import Path
import re
import hashlib
import json

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
original = (ROOT / "验证/视觉接口协议对齐/verify_visual_protocol.py").read_text(encoding="utf-8")
setup = original[:original.index('backup = ROOT / "排查记录/')]
reference = original[original.index("# Independent bitwise reference"):]
header = (ROOT / "application/usb_task.h").read_text(encoding="utf-8")
mode_enum = re.search(r"enum mode \{[^}]+\}", header).group()
setup = setup.replace('#pragma pack(push, 1)', mode_enum + ';\n#pragma pack(push, 1)')
setup = setup.replace('static uint8_t nav_override, Follow_Vision;',
                      'static uint8_t nav_override, Follow_Vision, ats_feedback_tx_phase;')
extra = r'''
  /* Run the actual active scheduler: phases 0/1 must emit no packet. */
  ats_feedback_tx_phase = 0;
  hUsbDeviceFS.dev_state = USBD_STATE_CONFIGURED;
  hUsbDeviceFS.pClassData = &cdc;
  for (i = 0; i < 16; ++i) {
    cdc.TxState = 0; before = send_count;
    unified_usb_send_ats_feedback();
    CHECK(ats_feedback_tx_phase == (uint8_t)((i + 1) % 4));
    if (i % 4 < 2) {
      CHECK(send_count == before);
    } else {
      CHECK(send_count == before + 1);
      CHECK(submitted[0] == 0x5A);
      CHECK(submitted[2] == (i % 4 == 2 ? 0x02 : 0x0C));
      CHECK(submitted_length == (i % 4 == 2 ? 34 : 22));
      CHECK(submitted[3] == unified_crc8(submitted, 3));
      CHECK(Verify_CRC16_Check_Sum(submitted, submitted_length));
    }
  }
  /* An outstanding transfer owns the buffer, including across idle slots. */
  memcpy(saved, submitted, submitted_length);
  before = send_count; cdc.TxState = 1;
  for (i = 0; i < 4; ++i) unified_usb_send_ats_feedback();
  CHECK(send_count == before && memcmp(saved, submitted, submitted_length) == 0);
  hUsbDeviceFS.dev_state = 0; cdc.TxState = 0;
  for (i = 0; i < 4; ++i) unified_usb_send_ats_feedback();
  CHECK(send_count == before);
'''
setup = setup.replace('  printf("PASS %d checks:', extra + '\n  printf("PASS %d checks:')
injection = '''parts.append(function("unified_usb_send_ats_feedback"))
'''
setup = setup.replace('host = OUT / "usb_protocol_host_test.cpp"', injection + 'host = OUT / "usb_protocol_host_test.cpp"')
reference = reference.replace('enum mode { Move = 0, Attack = 1, Defend = 2 }', mode_enum)
reference = reference.replace('legacy_chassis_mode_enum_preserved', 'sentry_posture_enum_2027')
reference = reference.replace('legacy TX preserved, additive ATS feedback and chassis modes',
                              'frame encoders and 2027 posture enum checked; active TX limited to 0x02/0x0C')
reference = reference.replace('print("PASS: existing timeout/CAN forwarding and non-USB source hashes unchanged")', '')
exec(compile(setup + '\nreport = {"host_test": result.strip(), "hardware_tested": False}\n' + reference,
             __file__, 'exec'))

# Check unchanged executable code outside the authorized scheduler edit.
def strip_comments(code):
    return re.sub(r'/\*.*?\*/|//[^\n]*', '', code, flags=re.S)

before = (OUT / "修改前/usb_task.c").read_text(encoding="utf-8")
current = (ROOT / "application/usb_task.c").read_text(encoding="utf-8")
old_clean, new_clean = strip_comments(before), strip_comments(current)
def get_scheduler(code):
    start = code.index('static void unified_usb_send_ats_feedback(void)\n{')
    end = code.index('\n}', start) + 2
    return code[start:end]

assert old_clean.replace(get_scheduler(old_clean), '') == new_clean.replace(get_scheduler(new_clean), '')
assert strip_comments((OUT / "修改前/usb_task.h").read_text(encoding="utf-8")) == strip_comments(header)
assert re.search(r'^#define USB_YAW_DEBUG\s+0\s*$', header, re.M)
assert re.search(r'^#define USB_ATS_COMPAT_FEEDBACK_TX\s+1\s*$', header, re.M)
assert 'usb_send_vision(' not in get_scheduler(new_clean)
assert 'unified_usb_send_state(' not in get_scheduler(new_clean)
baseline = json.loads((OUT / "修改前/源码哈希.json").read_text(encoding="utf-8"))
changed = {}
for relative, old_hash in baseline.items():
    new_hash = hashlib.sha256((ROOT / relative).read_bytes()).hexdigest()
    if new_hash != old_hash:
        changed[relative] = new_hash
assert {p.replace('\\', '/') for p in changed} == {'application/usb_task.c', 'application/usb_task.h'}
(OUT / "修改文件哈希.json").write_text(json.dumps(changed, ensure_ascii=False, indent=2), encoding="utf-8")
report_path = OUT / "验证结果.json"
report = json.loads(report_path.read_text(encoding="utf-8"))
report.pop("legacy_vision_tx_preserved", None)
report.update({"legacy_encoder_unit_tested": True, "active_tx_ids": [2, 12],
               "legacy_42_byte_tx_enabled": False, "id_0e_tx_enabled": False,
               "four_slot_cadence_and_busy_guards_preserved": True,
               "rx_control_referee_code_unchanged": True})
report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
print("PASS: active scheduler sends only 0x02/0x0C; four-slot cadence and busy guards preserved")
print("PASS: RX/control/referee code and configuration unchanged; only scheduler and comments changed")
