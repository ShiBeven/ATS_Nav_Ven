"""Re-run the existing time-order regression with compatibility inactive."""
from pathlib import Path
root=Path(__file__).resolve().parents[2]
test=(root/'验证/USB接收超时第1版/verify_timeout.py').read_text(encoding='utf-8')
test=test.replace('static uint8_t nav_override;','''static uint8_t nav_override, usb_nav_compat_active;
static uint8_t usb_nav_compat_allowed(uint32_t now) { (void)now; return 0; }''',1)
exec(compile(test,str(__file__),'exec'),{'__file__':str(__file__)})
