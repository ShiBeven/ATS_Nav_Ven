"""Read-only comparison of ARMCC maps and fromelf listings.

Address normalization is evidence for the selected instruction sequences only.
It is not a proof of whole-program equivalence or hardware timing.
"""
from pathlib import Path
import difflib
import hashlib
import json
import re
import struct

HERE = Path(__file__).resolve().parent
CURRENT = HERE.parents[1]
BASELINE = CURRENT.parent / "双板通信的云台-修改备份" / "20261022-小Yaw校准前-163713"


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read_map(path):
    pattern = re.compile(r"^\s+(\S+)\s+0x([0-9a-fA-F]+)\s+(Thumb Code|ARM Code|Data)\s+(\d+)\s+(.+)$")
    symbols = {}
    for line in path.read_text(errors="replace").splitlines():
        match = pattern.match(line)
        if not match:
            continue
        name, address, kind, size, owner = match.groups()
        address = int(address, 16)
        if "Code" in kind:
            address &= ~1
        item = (address, int(size), kind, owner)
        if name in symbols and symbols[name] != item:
            # Static variables may reuse names across translation units.
            name = f"{owner}::{name}"
        symbols[name] = item
    return symbols


def canonical_address(value, symbols):
    # Resolve only known named intervals. Hardware registers/constants stay exact.
    for name, (address, size, kind, _) in symbols.items():
        if address <= value < address + size:
            return f"{name}+{value-address}"
    return hex(value)


def read_hex(path):
    memory = {}
    upper = 0
    for line in path.read_text(encoding="ascii").splitlines():
        record = bytes.fromhex(line.removeprefix(":"))
        if sum(record) & 255 or len(record) != record[0] + 5:
            raise ValueError("Invalid Intel HEX record")
        count, address, kind = record[0], int.from_bytes(record[1:3], "big"), record[3]
        payload = record[4:4+count]
        if kind == 4:
            upper = int.from_bytes(payload, "big") << 16
        elif kind == 2:
            upper = int.from_bytes(payload, "big") << 4
        elif kind == 0:
            memory.update((upper+address+i, value) for i, value in enumerate(payload))
        elif kind == 1:
            break
    return memory


def read_thread_definitions(root, symbols):
    map_text = (root / "MDK-ARM/OBJ/PROJECT.map").read_text(errors="replace")
    match = re.search(r"(?m)^\s+\.constdata\s+0x([0-9a-fA-F]+)\s+Section\s+280\s+freertos\.o\(\.constdata\)", map_text)
    if not match:
        raise ValueError("Expected ten 28-byte thread definitions")
    address = int(match[1], 16)
    memory = read_hex(root / "MDK-ARM/OBJ/PROJECT.hex")
    definitions = []
    for index in range(10):
        raw = bytes(memory[address+index*28+i] for i in range(28))
        name_pointer, task, priority, instances, stack_size, buffer, control = struct.unpack("<7I", raw)
        name_bytes = []
        for offset in range(64):
            value = memory[name_pointer+offset]
            if not value:
                break
            name_bytes.append(value)
        else:
            raise ValueError("Invalid task name")
        definitions.append({"name": bytes(name_bytes).decode("ascii"),
            "task": canonical_address(task & ~1, symbols),
            "priority_enum": priority if priority < 0x80000000 else priority-0x100000000,
            "instances": instances, "stacksize_field": stack_size,
            "buffer": canonical_address(buffer, symbols), "controlblock": canonical_address(control, symbols)})
    return definitions


def instructions(listing, symbols, name):
    address, size, _, _ = symbols[name]
    rows = []
    for line in listing:
        match = re.match(r"\s+0x([0-9a-fA-F]+):", line)
        if not match or not address <= int(match[1], 16) < address + size:
            continue
        # fromelf prints instruction text at column 43, after machine bytes/ASCII.
        assembly, _, comment = line[43:].partition(";")
        assembly = re.sub(r"\s+", " ", assembly.strip())
        literal = re.search(r"\[0x[0-9a-fA-F]+\] = (0x[0-9a-fA-F]+|\d+)", comment)
        if literal and "[pc," in assembly:
            value = int(literal[1], 0)
            assembly = re.sub(r"\[pc,#[^\]]+\]", f"LITERAL({canonical_address(value, symbols)})", assembly)
        elif assembly.startswith(("B ", "BL ", "BLX ", "BEQ ", "BNE ", "BCC ", "BCS ",
                                  "BHI ", "BLS ", "BGE ", "BGT ", "BLE ", "BLT ",
                                  "BMI ", "BPL ", "CBZ ", "CBNZ ")):
            def branch(match):
                value = int(match[0], 16)
                if address <= value < address + size:
                    return f"THIS+{value-address}"
                return canonical_address(value, symbols)
            assembly = re.sub(r"0x[0-9a-fA-F]+", branch, assembly)
        rows.append(assembly)
    if not rows:
        raise ValueError(f"No instructions for {name}")
    return rows


def main():
    manifest = json.loads((BASELINE / "备份清单.json").read_text(encoding="utf-8-sig"))
    integrity = {name: sha(BASELINE / name) == expected.lower() for name, expected in manifest.items()}
    if not all(integrity.values()):
        raise ValueError("Baseline integrity mismatch")
    versions = {}
    for version, root, filename in (("old", BASELINE, "正常旧固件反汇编.txt"),
                                    ("current", CURRENT, "当前固件反汇编.txt")):
        versions[version] = {
            "symbols": read_map(root / "MDK-ARM/OBJ/PROJECT.map"),
            "listing": (HERE / filename).read_text(errors="replace").splitlines(),
        }
    names = [
        "CAN_SendRcBasic", "CAN_SendRcAux", "CAN_SendRcMouse", "CAN_SendRcFrame",
        "CAN_SendGimbalState", "CAN_CmdBoard3", "rc_put_i16_be", "get_remote_control_point",
        "remote_control_init", "remote_control_can_timeout_check", "USART3_IRQHandler",
        "sbus_to_rc", "RC_data_is_error", "RC_Init", "RC_restart",
        "MX_TIM2_Init", "MX_TIM6_Init", "MX_CAN1_Init", "MX_CAN2_Init",
        "SystemClock_Config", "HAL_TIM_Base_MspInit", "HAL_CAN_MspInit",
        "MX_FREERTOS_Init", "HAL_CAN_AddTxMessage", "imu_send_cmd", "imu_get_gyro_snapshot",
        "gimbal_feedback_update", "big_yaw_follow_control", "big_yaw_output_limit",
        "HAL_TIM_PeriodElapsedCallback", "gimbal_task", "usb_task", "usb_send_yaw_debug",
        "gimbal_init", "Gimbal_Init", "GetgimbalMode", "cali_data_read", "cali_data_write",
        "cali_gyro_hook", "gimbal_get_debug_snapshot", "CAN_cmd_gimbal", "CAN_cmd_gimbal_2",
        "CAN_cmd_gimbal_3",
    ]
    result = {"baseline": str(BASELINE), "current": str(CURRENT), "integrity": integrity,
              "firmware_sha256": {}, "functions": {},
              "limits": "Named addresses and PC literal references are normalized. Matching selected instruction sequences does not establish runtime timing, sensor values, whole-program equivalence or flashed firmware identity."}
    for version, root in (("old", BASELINE), ("current", CURRENT)):
        result["firmware_sha256"][version] = {
            ext: sha(root / f"MDK-ARM/OBJ/PROJECT.{ext}") for ext in ("axf", "hex", "map")}
    result["thread_definitions"] = {v: read_thread_definitions(root, versions[v]["symbols"])
                                    for v, root in (("old", BASELINE), ("current", CURRENT))}
    result["thread_definitions_equal"] = result["thread_definitions"]["old"] == result["thread_definitions"]["current"]
    diffs = []
    for name in names:
        if not all(name in version["symbols"] for version in versions.values()):
            result["functions"][name] = {"status": "not present in both symbol maps"}
            continue
        rows = {v: instructions(data["listing"], data["symbols"], name) for v, data in versions.items()}
        equal = rows["old"] == rows["current"]
        result["functions"][name] = {"normalized_equal": equal,
            "old_code_bytes": versions["old"]["symbols"][name][1],
            "current_code_bytes": versions["current"]["symbols"][name][1]}
        if not equal:
            diffs.extend(difflib.unified_diff(rows["old"], rows["current"],
                         fromfile=f"old/{name}", tofile=f"current/{name}", lineterm=""))
    first_root = BASELINE.parent / "20261022-170546-校准故障锁存前"
    first_symbols = read_map(first_root / "MDK-ARM/OBJ/PROJECT.map")
    first_listing = (HERE / "初次小Yaw校准固件反汇编.txt").read_text(errors="replace").splitlines()
    first_functions = {}
    for name in names:
        if name not in first_symbols or name not in versions["old"]["symbols"]:
            continue
        old_rows = instructions(versions["old"]["listing"], versions["old"]["symbols"], name)
        first_rows = instructions(first_listing, first_symbols, name)
        first_functions[name] = {"normalized_equal": old_rows == first_rows}
    result["first_small_yaw_build"] = {
        "root": str(first_root),
        "sha256": {ext: sha(first_root / f"MDK-ARM/OBJ/PROJECT.{ext}") for ext in ("axf", "hex", "map")},
        "thread_definitions_equal_to_old": read_thread_definitions(first_root, first_symbols) == result["thread_definitions"]["old"],
        "functions_vs_old": first_functions,
    }
    (HERE / "对比结果.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    (HERE / "归一化指令差异.diff").write_text("\n".join(diffs) + "\n", encoding="utf-8")
    print(json.dumps({"thread_definitions_equal": result["thread_definitions_equal"],
                      "selected_equal_functions": [n for n, r in result["functions"].items() if r.get("normalized_equal")],
                      "selected_different_functions": [n for n, r in result["functions"].items() if r.get("normalized_equal") is False],
                      "first_build_timer_callback_equal": first_functions["HAL_TIM_PeriodElapsedCallback"]["normalized_equal"],
                      "first_build_thread_definitions_equal": result["first_small_yaw_build"]["thread_definitions_equal_to_old"]},
                      ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
