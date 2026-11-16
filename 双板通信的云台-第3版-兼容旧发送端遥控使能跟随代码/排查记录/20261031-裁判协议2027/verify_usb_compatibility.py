"""Run existing USB wire tests against current functions, with 2027 posture enum."""
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[2]
original = (ROOT / "验证/视觉接口协议对齐/verify_visual_protocol.py").read_text(encoding="utf-8")
setup = original[:original.index('backup = ROOT / "排查记录/')]
reference = original[original.index("# Independent bitwise reference"):]
setup = setup.replace('OUT = Path(__file__).resolve().parent',
                      'OUT = Path(__file__).resolve().parent / "USB回归"\nOUT.mkdir(exist_ok=True)')
header = (ROOT / "application/usb_task.h").read_text(encoding="utf-8")
mode_enum = re.search(r"enum mode \{[^}]+\}", header).group()
setup = setup.replace('#pragma pack(push, 1)', mode_enum + ';\n#pragma pack(push, 1)')
reference = reference.replace('enum mode { Move = 0, Attack = 1, Defend = 2 }', mode_enum)
reference = reference.replace('legacy_chassis_mode_enum_preserved', 'sentry_posture_enum_2027')
reference = reference.replace('legacy TX preserved, additive ATS feedback and chassis modes',
                              'legacy TX and additive ATS feedback preserved; 2027 sentry posture enum')
reference = reference.replace('print("PASS: existing timeout/CAN forwarding and non-USB source hashes unchanged")', '')
exec(compile(setup + '\nreport = {"host_test": result.strip(), "hardware_tested": False}\n' + reference,
             __file__, 'exec'))

# Check the actual current checkout, rather than absolute paths to older copies.
backup = Path(__file__).resolve().parent / "修改前"
for relative in ("application/usb_task.c", "application/gimbal_task.c", "application/CAN_receive.c",
                 "Src/shoot_task.c", "Src/Vision_Task.c", "Src/Vision.c"):
    assert (ROOT / relative).read_bytes() == (backup / relative).read_bytes(), relative
print("PASS: USB parser, gimbal/PID, CAN serialization and shooting/vision control sources unchanged")
