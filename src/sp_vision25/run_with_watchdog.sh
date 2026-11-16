#!/usr/bin/env bash
# 鏂囦欢鍚? run_with_watchdog.sh
# 鍔熻兘:
# 1. 鍦ㄦ柊缁堢涓惎鍔ㄤ竴涓€滄瘮璧涜繍琛屽瀷鈥濈▼搴忥紙濡?standard_mpc / sentry锛?# 2. 绋嬪簭閫€鍑哄悗鑷姩閲嶅惎
# 3. 涓嶉檮甯︽祻瑙堝櫒 / Web 璋冭瘯绛夊緟閫昏緫锛屽敖閲忓噺灏戞棤鍏冲紑閿€
#
# 閫傜敤鍦烘櫙:
# - standard_mpc
# - sentry
# - standard
# - mt_standard
# - 鍏朵粬姣旇禌涓诲叆鍙ｇ▼搴?
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

BIN_NAME="${1:-auto_aim_debug_mpc}"
BIN_PATH="${ROOT_DIR}/../install/sp_vision25/lib/sp_vision25/${BIN_NAME}"

# 绗簩涓弬鏁板厑璁告樉寮忎紶閰嶇疆鏂囦欢锛涙湭鎻愪緵鏃舵寜涓荤▼搴忕粰甯哥敤榛樿鍊?DEFAULT_CONFIG="${ROOT_DIR}/../install/sp_vision25/share/sp_vision25/configs/standard3.yaml"
CONFIG_PATH="${2:-${DEFAULT_CONFIG}}"

RESTART_DELAY="${RESTART_DELAY:-2}"

start_program_in_terminal() {
    local title="${BIN_NAME}"
    local inner_cmd="cd \"${ROOT_DIR}\" && \"${BIN_PATH}\" \"${CONFIG_PATH}\"; echo '绋嬪簭宸查€€鍑猴紝绐楀彛灏嗗叧闂?; sleep 2"

    if command -v gnome-terminal >/dev/null 2>&1; then
        gnome-terminal --title="${title}" -- bash -c "${inner_cmd}" &
    elif command -v xfce4-terminal >/dev/null 2>&1; then
        xfce4-terminal --title="${title}" --command="bash -c \"${inner_cmd}\"" &
    elif command -v konsole >/dev/null 2>&1; then
        konsole --hold -p tabtitle="${title}" -e bash -c "${inner_cmd}" &
    elif command -v xterm >/dev/null 2>&1; then
        xterm -T "${title}" -e bash -c "${inner_cmd}" &
    else
        echo "[姣旇禌鐪嬮棬鐙梋 閿欒: 鎵句笉鍒板彲鐢ㄧ殑缁堢妯℃嫙鍣?
        exit 1
    fi

    sleep 2
    local pid
    pid=$(pgrep -f "${BIN_PATH}" | head -1 || true)
    echo "${pid}"
}

main() {
    [[ ! -x "${BIN_PATH}" ]] && { echo "閿欒: ${BIN_PATH} 涓嶅彲鎵ц"; exit 1; }

    echo "=========================================="
    echo "姣旇禌鐪嬮棬鐙楀凡鍚姩 (鎸?Ctrl+C 褰诲簳閫€鍑?"
    echo "绋嬪簭: ${BIN_PATH}"
    echo "閰嶇疆: ${CONFIG_PATH}"
    echo "閲嶅惎寤惰繜: ${RESTART_DELAY} 绉?
    echo "宸ヤ綔鐩綍: ${ROOT_DIR}"
    echo "=========================================="

    while true; do
        echo "[$(date '+%H:%M:%S')] 鍚姩姣旇禌绋嬪簭..."

        local prog_pid
        prog_pid=$(start_program_in_terminal)
        if [[ -z "${prog_pid}" ]]; then
            echo "[姣旇禌鐪嬮棬鐙梋 璀﹀憡: 鏃犳硶鑾峰彇绋嬪簭 PID锛屽彲鑳藉惎鍔ㄥけ璐ワ紝绛夊緟鍚庨噸璇?
            sleep "${RESTART_DELAY}"
            continue
        fi

        while kill -0 "${prog_pid}" 2>/dev/null; do
            sleep 1
        done

        echo "[$(date '+%H:%M:%S')] 绋嬪簭宸查€€鍑猴紝${RESTART_DELAY} 绉掑悗閲嶅惎..."
        sleep "${RESTART_DELAY}"
    done
}

main
