"""Independent PDF-offset fixtures against actual ARM firmware C functions."""
from pathlib import Path
import os
import re
import struct
import subprocess
import json

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
VC = Path(r"C:\Program Files (x86)\Microsoft Visual Studio\VC98")


def text(path):
    return (ROOT / path).read_text(encoding="utf-8", errors="replace")


def clean(source):
    source = re.sub(r"/\*.*?\*/|//[^\n]*", "", source, flags=re.S)
    return re.sub(r'^\s*#include[^\n]*', '', source, flags=re.M)


def function(source, name):
    m = re.search(r"^(?:static\s+)?[\w* ]+\b" + name + r"\([^;]*?\)\s*\{", source, re.M)
    assert m, name
    i = source.index("{", m.start()) + 1
    depth = 1
    while depth:
        depth += (source[i] == "{") - (source[i] == "}")
        i += 1
    return source[m.start():i]


def crc(data, initial, poly):
    value = initial
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ (poly if value & 1 else 0)
    return value


def frame(command, payload):
    header = b"\xa5" + struct.pack("<H", len(payload)) + b"\x37"
    packet = header + bytes([crc(header, 0xFF, 0x8C)]) + struct.pack("<H", command) + payload
    return packet + struct.pack("<H", crc(packet, 0xFFFF, 0x8408))


prefix = r'''
#include <stdio.h>
#include <string.h>
#include <stddef.h>
typedef unsigned char uint8_t;
typedef signed char int8_t;
typedef unsigned short uint16_t;
typedef signed short int16_t;
typedef unsigned int uint32_t;
typedef signed int int32_t;
typedef unsigned __int64 uint64_t;
typedef float fp32;
static uint32_t tick = 1000;
static uint32_t HAL_GetTick(void) { return tick; }
typedef struct { int unused; } UART_HandleTypeDef;
static UART_HandleTypeDef huart6;
static uint8_t transmitted[64];
static uint16_t transmitted_length;
static unsigned int transmit_count;
static int HAL_UART_Transmit(UART_HandleTypeDef *, uint8_t *data, uint16_t length, uint32_t) {
    memcpy(transmitted, data, length); transmitted_length = length; ++transmit_count; return 0;
}
typedef struct { uint8_t bytes[4096]; unsigned int head, tail; } fifo_s_t;
static unsigned int fifo_s_used(fifo_s_t *fifo) { return fifo->head - fifo->tail; }
static uint8_t fifo_s_get(fifo_s_t *fifo) { return fifo->bytes[(fifo->tail++) & 4095]; }
static void fifo_s_puts(fifo_s_t *fifo, const char *data, unsigned int length) {
    while (length--) fifo->bytes[(fifo->head++) & 4095] = (uint8_t)*data++;
}
static struct { struct { struct { uint8_t mode; } speed_vector; } data; } RECEIVE_ROBOT_CMD_DATA;
'''
headers = "\n".join(clean(text(p)).replace("__packed", "") for p in (
    "application/protocol/protocol.h", "application/referee.h", "application/referee_usart_task.h"))
parts = [prefix, "#pragma pack(push, 1)", headers, "#pragma pack(pop)",
         clean(text("components/support/CRC8_CRC16.c")), clean(text("application/referee.c")),
         clean(text("Src/referee_lib.c")),
         r'''static fifo_s_t referee_fifo;
static unpack_data_t referee_unpack_obj;
static uint8_t Vision_ID, UI_Seq1, hhg_test[19];
static volatile uint8_t sentry_activate_rune;
''']
task = clean(text("application/referee_usart_task.c"))
for name in ("robot_id", "send_test", "get_sentry_info", "get_event_info", "referee_unpack_fifo_data"):
    parts.append(function(task, name))

# Literal field order and lengths are taken from the PDF, not sizeof(C types).
spec = [
    ("game", 0x0001, 11, "game_state"), ("result", 0x0002, 1, "game_result"),
    ("hp", 0x0003, 24, "game_robot_HP"), ("event", 0x0101, 8, "field_event"),
    ("warning", 0x0104, 3, "referee_warning"), ("dart", 0x0105, 3, "dart_info"),
    ("status", 0x0201, 17, "robot_state"), ("heat", 0x0202, 14, "power_heat_data"),
    ("pos", 0x0203, 12, "game_robot_pos"), ("buff", 0x0204, 7, "buff"),
    ("hurt", 0x0206, 7, "robot_hurt"), ("shoot", 0x0207, 7, "shoot_data"),
    ("ammo", 0x0208, 8, "projectile_allowance"), ("rfid", 0x0209, 6, "robot_rfid"),
    ("dart_client", 0x020A, 6, "dart_client_cmd"), ("ground", 0x020B, 40, "ground_robot_position"),
    ("mark", 0x020C, 2, "radar_mark_data"), ("sentry", 0x020D, 14, "SentryInfo"),
    ("radar", 0x020E, 8, "RadarInfo"), ("map_cmd", 0x0303, 12, "map_command"),
    ("map_robot", 0x0305, 48, "map_robot_data"), ("path", 0x0307, 105, "map_data"),
    ("custom_info", 0x0308, 34, "custom_info"), ("custom_tx", 0x0310, 300, "robot_custom_data_2"),
    ("custom_rx", 0x0311, 60, "robot_custom_data"),
]
payloads = {name: bytes((7 + 13 * i) % 256 for i in range(length)) for name, _, length, _ in spec}
payloads["hp"] = struct.pack("<4Hh7H", 800, 0, 250, 300, -321, 600, 1200, 5000, 1100, 4000, 42, 43)
event_fields = [1, 2, 1, 4, 3, 2, 1, 1, 2, 3, 4, 2, 1, 2]
event_offsets = [0, 1, 3, 5, 8, 11, 14, 17, 18, 20, 22, 25, 28, 30]
event_word = sum(value << shift for value, shift in zip(event_fields, event_offsets))
event2_fields = [1, 2, 420, 6, 3, 2, 1, 2, 3, 1]
event2_offsets = [0, 2, 4, 13, 16, 18, 20, 22, 24, 26]
event2_word = sum(value << shift for value, shift in zip(event2_fields, event2_offsets))
payloads["event"] = struct.pack("<II", event_word, event2_word)
payloads["status"] = struct.pack("<BB5HfB", 107, 3, 800, 900, 40, 240, 120, 30.0, 5)
payloads["heat"] = struct.pack("<HHf3H", 0, 0, 0, 120, 70, 80)
payloads["buff"] = struct.pack("<BHBBH", 10, 0x345, 50, 30, 150)
payloads["hurt"] = struct.pack("<BHhh", 0x53, 0x1234, -23, 66)
payloads["shoot"] = struct.pack("<BBBf", 1, 1, 10, 23.5)
payloads["sentry"] = struct.pack("<IH", 123 | (3 << 11) | (4 << 15) | (1 << 19) | (1 << 20) | (500 << 21),
                                1 | (1000 << 1) | (2 << 12) | (1 << 14) | (1 << 15)) + bytes([10, 20, 30, 0xA5, 40, 50, 60, 0x5A])
payloads["map_cmd"] = struct.pack("<ffBBH", 1.25, -2.5, 65, 103, 0x0167)

declarations = []


def fixture(name, packet):
    declarations.append("static uint8_t %s[] = {%s};" % (name, ",".join("0x%02x" % x for x in packet)))


test_lines = []
for name, command, length, obj in spec:
    data = payloads[name]
    assert len(data) == length
    fixture("f_" + name, frame(command, data))
    fixture("short_" + name, frame(command, data[:-1]))
    fixture("long_" + name, frame(command, data + b"\x00"))
    test_lines.append(f'''
    for (split = 0; split <= sizeof(f_{name}); ++split) {{
        reset_stream(); before = referee_rx_valid_frames; ++tick;
        feed(f_{name}, split); feed(f_{name} + split, sizeof(f_{name}) - split);
        CHECK(referee_rx_valid_frames == before + 1 && referee_last_cmd_id == 0x{command:04x});
        CHECK(referee_last_rx_tick == tick && memcmp(&{obj}, f_{name} + 7, {length}) == 0);
    }}
    memcpy(saved, &{obj}, {length}); before = referee_rx_valid_frames;
    CHECK(referee_data_solve(short_{name}, sizeof(short_{name})) == 0);
    CHECK(referee_data_solve(long_{name}, sizeof(long_{name})) == 0);
    CHECK(referee_rx_valid_frames == before && memcmp(saved, &{obj}, {length}) == 0);
''')

for sub, size in [(0x100, 2), (0x101, 15), (0x102, 30), (0x103, 75), (0x104, 105), (0x110, 45), (0x120, 4), (0x121, 13)]:
    payload = struct.pack("<3H", sub, 3, 0x0103) + bytes([0x33] * size)
    fixture(f"sub_{sub:x}", frame(0x0301, payload))
    fixture(f"sub_bad_{sub:x}", frame(0x0301, payload[:-1]))
    test_lines.append(f'''
    reset_stream(); before = referee_rx_valid_frames; feed(sub_{sub:x}, sizeof(sub_{sub:x}));
    CHECK(referee_rx_valid_frames == before + 1 && robot_interaction_user_data_length == {size});
    CHECK(robot_interaction_data.data_cmd_id == 0x{sub:x});
    CHECK(referee_data_solve(sub_bad_{sub:x}, sizeof(sub_bad_{sub:x})) == 0);
''')
for size in [0, 1, 112]:
    fixture(f"interaction_{size}", frame(0x0301, struct.pack("<3H", 0x0201, 3, 4) + bytes([0x77] * size)))
fixture("interaction_too_long", frame(0x0301, struct.pack("<3H", 0x0201, 3, 4) + bytes(113)))
fixture("interaction_unknown", frame(0x0301, struct.pack("<3H", 0x1234, 3, 4)))
for old in [0x0102, 0x0103, 0x0205, 0x0302, 0x0304, 0x0306, 0x0309]:
    fixture(f"removed_{old:x}", frame(old, b"\0" * 30))
    test_lines.append(f"    CHECK(referee_data_solve(removed_{old:x}, sizeof(removed_{old:x})) == 0);")

parts += declarations
parts.append(r'''
static unsigned int checks;
#define CHECK(condition) do { ++checks; if (!(condition)) { printf("FAIL line %d: %s\n", __LINE__, #condition); return 1; } } while (0)
static void reset_stream(void) { memset(&referee_unpack_obj, 0, sizeof(referee_unpack_obj)); memset(&referee_fifo, 0, sizeof(referee_fifo)); }
static void feed(const uint8_t *data, unsigned int length) { fifo_s_puts(&referee_fifo, (const char *)data, length); referee_unpack_fifo_data(); }
int main(void) {
    unsigned int split, i, before;
    uint8_t saved[300], bad[320], stream[500];
    sentry_fh sent;
    sentry_info_extracted_t sentry;
    event_data_parsed_t event;
    FILE *capture;
    init_referee_struct_data();
    CHECK(offsetof(robot_status_t, bullet_speed_limit) == 12);
    CHECK(offsetof(buff_t, cooling_buff) == 1 && offsetof(buff_t, attack_buff) == 5);
    CHECK(offsetof(sentry_info_t, sentry_info_3) == 6);
    CHECK(offsetof(rfid_status_t, rfid_status_2) == 4);
    CHECK(offsetof(map_command_t, cmd_source) == 10);
''')
parts += test_lines
parts.append(r'''
    CHECK(game_robot_HP.damage_difference == -321 && game_robot_HP.outpost_B_HP == 1100);
    CHECK(game_robot_HP.ally_control_degree == 42 && game_robot_HP.enemy_control_degree == 43);
    CHECK(robot_state.bullet_speed_limit == 30.0f && robot_state.current_HP == 800);
    CHECK(robot_state.power_management_gimbal_output == 1 && robot_state.power_management_chassis_output == 0 && robot_state.power_management_shooter_output == 1);
    CHECK(buff.cooling_buff == 0x345 && buff.defence_buff == 50 && buff.attack_buff == 150);
    CHECK(robot_hurt.armor_id == 3 && robot_hurt.HP_deduction_reason == 5 && robot_hurt.hit_sequence == 0x1234);
    CHECK(robot_hurt.hit_position_x == -23 && robot_hurt.hit_position_y == 66);
    CHECK(map_command.cmd_source == 0x0167 && map_command.target_position_y == -2.5f);
    CHECK(get_robot_remain_HP() == 800 && get_robot_max_HP() == 900);
    CHECK(get_chassis_buffer_energy() == 120 && Chassis_Power_Limit() == 120);
    CHECK(get_shooter_cooling_value() == 40 && get_shooter_heat_limit() == 240);
    CHECK(get_shooter_17mm_heat() == 70 && get_shooter_42mm_heat() == 80);
    CHECK(get_bullet_initial_speed() == 23.5f && get_bullet_launching_frequency() == 10 && get_bullet_speed_limit() == 30.0f);

    /* Restore sentry fixture after the sub-ID decoding tests. */
    CHECK(referee_data_solve(f_sentry, sizeof(f_sentry)) == 1);
    get_sentry_info(&sentry);
    CHECK(sentry.exchanged_bullets == 123 && sentry.remote_bullet_cnt == 3 && sentry.remote_heal_cnt == 4);
    CHECK(sentry.can_free_respawn == 1 && sentry.can_pay_respawn == 1 && sentry.respawn_gold_cost == 500);
    CHECK(sentry.current_posture == 2 && sentry.is_enhanced_posture == 1 && sentry.can_activate_rune == 1);
    CHECK(sentry.team_17mm_remaining == 1000 && sentry.is_out_of_combat == 1);
    CHECK(sentry.attack_remaining_time == 10 && sentry.defend_remaining_time == 20 && sentry.move_remaining_time == 30);
    CHECK(sentry.enhanced_attack_remaining_time == 40 && sentry.enhanced_defend_remaining_time == 50 && sentry.enhanced_move_remaining_time == 60);
    get_event_info(&event);
''')
event_members = ["supply_zone_activated", "small_rune_status", "big_rune_status", "r_fortress_gain", "b_fortress_gain", "r_base_gain", "b_base_gain", "allied_high_ground_gain", "r_central_highland_gain", "b_central_highland_gain", "r_outpost_gain", "b_outpost_gain", "r_outpost_control", "b_outpost_control"]
event2_members = ["r_base_control", "b_base_control", "enemy_dart_last_hit_time", "enemy_dart_last_hit_target", "r_highland_activation", "r_highland_control", "b_highland_activation", "b_highland_control", "central_activation", "central_control"]
for member, expected in zip(event_members + event2_members, event_fields + event2_fields):
    parts.append(f"    CHECK(event.{member} == {expected});")
parts.append(r'''
    /* Packet limits and clearing tails of variable-length interaction data. */
    CHECK(referee_data_solve(interaction_112, sizeof(interaction_112)) == 1);
    CHECK(robot_interaction_user_data_length == 112 && robot_interaction_data.user_data[111] == 0x77);
    CHECK(referee_data_solve(interaction_1, sizeof(interaction_1)) == 1);
    for (i = 1; i < 112; ++i) CHECK(robot_interaction_data.user_data[i] == 0);
    CHECK(referee_data_solve(interaction_0, sizeof(interaction_0)) == 1 && robot_interaction_user_data_length == 0);
    CHECK(referee_data_solve(interaction_too_long, sizeof(interaction_too_long)) == 0);
    CHECK(referee_data_solve(interaction_unknown, sizeof(interaction_unknown)) == 0);

    memcpy(bad, f_status, sizeof(f_status)); bad[7] ^= 1;
    before = referee_rx_valid_frames;
    CHECK(referee_data_solve(bad, sizeof(f_status)) == 0 && referee_rx_valid_frames == before);
    memcpy(bad, f_status, sizeof(f_status)); bad[4] ^= 1;
    CHECK(referee_data_solve(bad, sizeof(f_status)) == 0 && referee_rx_valid_frames == before);
    CHECK(referee_data_solve(f_status, sizeof(f_status) - 1) == 0);
    CHECK(referee_data_solve(NULL, 0) == 0 && referee_data_solve(f_status, 4) == 0);
    /* Noise, a bad packet, a following good packet, and glued packets. */
    reset_stream(); memset(stream, 0xFF, 20); memcpy(stream + 20, bad, sizeof(f_status));
    memcpy(stream + 20 + sizeof(f_status), f_status, sizeof(f_status));
    before = referee_rx_valid_frames; feed(stream, 20 + 2 * sizeof(f_status));
    CHECK(referee_rx_valid_frames == before + 1);
    reset_stream(); memcpy(stream, f_status, sizeof(f_status)); memcpy(stream + sizeof(f_status), f_hp, sizeof(f_hp));
    before = referee_rx_valid_frames; feed(stream, sizeof(f_status) + sizeof(f_hp));
    CHECK(referee_rx_valid_frames == before + 2);
    reset_stream(); stream[0] = 0xA5; stream[1] = 0xFF; stream[2] = 0xFF;
    memcpy(stream + 3, f_status, sizeof(f_status)); before = referee_rx_valid_frames;
    feed(stream, 3 + sizeof(f_status)); CHECK(referee_rx_valid_frames == before + 1);

    CHECK(referee_team_color(1) == 0 && referee_team_color(11) == 0);
    CHECK(referee_team_color(101) == 1 && referee_team_color(107) == 1 && referee_team_color(111) == 1);
    CHECK(referee_team_color(2) == 0xFF && referee_team_color(102) == 0xFF && referee_team_color(17) == 0xFF);
    robot_state.robot_id = 101; robot_id(); CHECK(Vision_ID == 0 && MyColour() == 1);
    robot_state.robot_id = 1; robot_id(); CHECK(Vision_ID == 1 && MyColour() == 0);
    robot_state.robot_id = 0; robot_id(); CHECK(Vision_ID == 0xFF && MyColour() == -1);
    before = transmit_count; send_test(&sent); CHECK(transmit_count == before);
    robot_state.robot_id = 3; send_test(&sent); CHECK(transmit_count == before);
    send_test(NULL); CHECK(transmit_count == before);
    robot_state.robot_id = 107;
    UI_Seq1 = 0;
    capture = fopen("sentry-tx.bin", "wb"); CHECK(capture != NULL);
    for (i = 1; i <= 6; ++i) {
        RECEIVE_ROBOT_CMD_DATA.data.speed_vector.mode = (uint8_t)i;
        sentry_activate_rune = 1;
        send_test(&sent);
        CHECK(transmitted_length == 19 && transmitted[1] == 10 && transmitted[2] == 0);
        CHECK(transmitted[5] == 1 && transmitted[6] == 3 && transmitted[7] == 0x20 && transmitted[8] == 1);
        CHECK(transmitted[9] == 107 && transmitted[11] == 0x80 && transmitted[12] == 0x80);
        CHECK(verify_CRC8_check_sum(transmitted, 5) && verify_CRC16_check_sum(transmitted, 19));
        CHECK(((sent.datahead.SENTRY.sentry_cmd >> 21) & 7) == i);
        CHECK((sent.datahead.SENTRY.sentry_cmd & (1UL << 24)) && !(sent.datahead.SENTRY.sentry_cmd & (0x7FUL << 25)));
        CHECK(fwrite(transmitted, 1, 19, capture) == 19);
    }
    fclose(capture);
    sentry_activate_rune = 0; RECEIVE_ROBOT_CMD_DATA.data.speed_vector.mode = 0;
    send_test(&sent); CHECK(((sent.datahead.SENTRY.sentry_cmd >> 21) & 7) == 3);
    CHECK(!(sent.datahead.SENTRY.sentry_cmd & (1UL << 24)));
    init_referee_struct_data();
    CHECK(referee_rx_valid_frames == 0 && referee_last_rx_tick == 0);
    CHECK(robot_custom_data.data[59] == 0 && robot_custom_data_2.data[299] == 0 && map_command.cmd_source == 0);
    printf("PASS %u checks: 2027 layouts, independent wire fixtures, CRC, split/glued RX, invalid lengths, interaction limits, sentry bits and IDs\n", checks);
    return 0;
}
''')
host = OUT / "referee_host_test.cpp"
host.write_text("\n".join(parts), encoding="ascii")
env = os.environ.copy()
env["PATH"] = str(VC / "Bin") + os.pathsep + str(VC.parent / "Common/MSDev98/Bin") + os.pathsep + env.get("PATH", "")
env["INCLUDE"] = str(VC / "Include")
env["LIB"] = str(VC / "Lib")
build = subprocess.run([str(VC / "Bin/CL.EXE"), "/nologo", "/TP", "/MLd", "/W3", str(host), "/Fereferee_host_test.exe"], cwd=OUT, env=env, capture_output=True)
log = (build.stdout + build.stderr).decode("mbcs", errors="replace")
(OUT / "host-build.log").write_text(log, encoding="utf-8")
if build.returncode:
    raise SystemExit(log)
run = subprocess.run([str(OUT / "referee_host_test.exe")], cwd=OUT, capture_output=True)
result = (run.stdout + run.stderr).decode("mbcs", errors="replace").strip()
(OUT / "host-test.log").write_text(result, encoding="utf-8")
print(result)
if run.returncode:
    raise SystemExit(run.returncode)

# Verify exact sentry TX bytes using Python bitwise CRC, not firmware CRC tables.
capture = (OUT / "sentry-tx.bin").read_bytes()
for posture in range(1, 7):
    payload = struct.pack("<3HI", 0x0120, 107, 0x8080, 1 | (posture << 21) | (1 << 24))
    header = b"\xa5\x0a\x00" + bytes([posture - 1])
    packet = header + bytes([crc(header, 0xFF, 0x8C)]) + b"\x01\x03" + payload
    packet += struct.pack("<H", crc(packet, 0xFFFF, 0x8408))
    assert capture[(posture - 1) * 19:posture * 19] == packet, posture
print("PASS: all 6 sentry postures and rune bit match independent 19-byte wire reference")
(OUT / "验证结果.json").write_text(json.dumps({"host_test": result, "independent_sentry_tx": True,
    "fixed_commands_tested": len(spec), "hardware_tested": False}, ensure_ascii=False, indent=2), encoding="utf-8")
