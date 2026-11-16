#!/usr/bin/env bash
# 鏂囦欢鍚? debug_mpc_watchdog.sh
# 鍔熻兘:
# 1. 鍦ㄦ柊缁堢涓惎鍔ㄤ竴涓€滃彲瑙嗗寲璋冭瘯鍨嬧€濈▼搴忥紙榛樿 auto_aim_debug_mpc锛?# 2. 绋嬪簭閫€鍑哄悗鑷姩閲嶅惎
# 3. 鑻ョ▼搴忓惎鐢ㄤ簡 WebDebugger锛屽垯鑷姩绛夊緟缃戦〉鏈嶅姟灏辩华骞舵墦寮€娴忚鍣?#
# 閫傜敤鍦烘櫙:
# - auto_aim_debug_mpc
# - auto_debug
# - sentry_debug
# - 鍏朵粬甯︾綉椤?/ 鍙鍖栬皟璇曡兘鍔涚殑 debug 绋嬪簭

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 榛樿涓荤▼搴忎负 auto_aim_debug_mpc锛屼篃鍏佽閫氳繃绗竴涓弬鏁拌鐩?BIN_NAME="${1:-auto_aim_debug_mpc}"
BIN_PATH="${ROOT_DIR}/../install/sp_vision25/lib/sp_vision25/${BIN_NAME}"

# 绗簩涓弬鏁颁负閰嶇疆鏂囦欢璺緞锛涙湭鎻愪緵鏃舵寜甯歌 debug 绋嬪簭閫夋嫨榛樿閰嶇疆
DEFAULT_CONFIG="${ROOT_DIR}/../install/sp_vision25/share/sp_vision25/configs/standard3.yaml"
CONFIG_PATH="${2:-${DEFAULT_CONFIG}}"

RESTART_DELAY="${RESTART_DELAY:-2}"
WEB_READY_TIMEOUT="${WEB_READY_TIMEOUT:-10}"

# 浠庨厤缃枃浠惰鍙?web 璁剧疆锛堢畝鍗曡В鏋愶級
WEB_HOST="0.0.0.0"
WEB_PORT="8090"
DISABLE_WEB="false"

yaml_value() {
    local key="$1" default="$2" value=""
    [[ -f "${CONFIG_PATH}" ]] && value=$(awk -F ':' -v key="$key" '$0 !~ /^[[:space:]]*#/ && $1 ~ ("^[[:space:]]*" key "[[:space:]]*$") {sub(/^[^:]*:[[:space:]]*/, ""); gsub(/^[[:space:]]+|[[:space:]]+$/, ""); gsub(/^"/, ""); gsub(/"$/, ""); print; exit}' "${CONFIG_PATH}" 2>/dev/null)
    echo "${value:-${default}}"
}

resolve_web_settings() {
    WEB_HOST=$(yaml_value web_host "0.0.0.0")
    WEB_PORT=$(yaml_value web_port "8090")
    DISABLE_WEB=$(yaml_value disable_web "false")
}

wait_for_web() {
    local url="http://127.0.0.1:${WEB_PORT}/healthz"
    local deadline=$((SECONDS + WEB_READY_TIMEOUT))
    echo "[璋冭瘯鐪嬮棬鐙梋 绛夊緟 Web 鏈嶅姟灏辩华 (${url}) ..."
    while [[ ${SECONDS} -lt ${deadline} ]]; do
        if command -v curl >/dev/null 2>&1 && curl -fsS --max-time 1 "${url}" >/dev/null 2>&1; then
            echo "[璋冭瘯鐪嬮棬鐙梋 Web 鏈嶅姟宸插氨缁?
            return 0
        fi
        if command -v wget >/dev/null 2>&1 && wget -q -T 1 -O - "${url}" >/dev/null 2>&1; then
            echo "[璋冭瘯鐪嬮棬鐙梋 Web 鏈嶅姟宸插氨缁?
            return 0
        fi
        sleep 1
    done
    echo "[璋冭瘯鐪嬮棬鐙梋 璀﹀憡: Web 鏈嶅姟鏈氨缁紝浠嶅皾璇曟墦寮€娴忚鍣?
    return 1
}

open_browser() {
    local url="http://127.0.0.1:${WEB_PORT}/"
    echo "[璋冭瘯鐪嬮棬鐙梋 鎵撳紑娴忚鍣? ${url}"
    if command -v firefox >/dev/null 2>&1; then
        nohup firefox "${url}" >/dev/null 2>&1 &
    elif command -v xdg-open >/dev/null 2>&1; then
        nohup xdg-open "${url}" >/dev/null 2>&1 &
    elif command -v python3 >/dev/null 2>&1; then
        nohup python3 -m webbrowser "${url}" >/dev/null 2>&1 &
    else
        echo "[璋冭瘯鐪嬮棬鐙梋 鏃犳硶鑷姩鎵撳紑娴忚鍣紝璇锋墜鍔ㄨ闂? ${url}"
    fi
}

# 鍦ㄦ柊缁堢涓惎鍔ㄧ▼搴忥紝骞惰繑鍥炵▼搴?PID
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
        echo "[璋冭瘯鐪嬮棬鐙梋 閿欒: 鎵句笉鍒板彲鐢ㄧ殑缁堢妯℃嫙鍣?
        exit 1
    fi

    sleep 2
    local pid
    pid=$(pgrep -f "${BIN_PATH}" | head -1 || true)
    echo "${pid}"
}

is_truthy() {
    case "${1,,}" in
        1|true|yes|on) return 0 ;;
        *) return 1 ;;
    esac
}

main() {
    [[ ! -x "${BIN_PATH}" ]] && { echo "閿欒: ${BIN_PATH} 涓嶅彲鎵ц"; exit 1; }
    resolve_web_settings

    echo "=========================================="
    echo "璋冭瘯鐪嬮棬鐙楀凡鍚姩 (鎸?Ctrl+C 褰诲簳閫€鍑?"
    echo "绋嬪簭: ${BIN_PATH}"
    echo "閰嶇疆: ${CONFIG_PATH}"
    echo "Web: http://${WEB_HOST}:${WEB_PORT}/"
    echo "閲嶅惎寤惰繜: ${RESTART_DELAY} 绉?
    echo "宸ヤ綔鐩綍: ${ROOT_DIR}"
    echo "=========================================="

    local browser_opened=0

    while true; do
        echo "[$(date '+%H:%M:%S')] 鍚姩璋冭瘯绋嬪簭..."

        local prog_pid
        prog_pid=$(start_program_in_terminal)
        if [[ -z "${prog_pid}" ]]; then
            echo "[璋冭瘯鐪嬮棬鐙梋 璀﹀憡: 鏃犳硶鑾峰彇绋嬪簭 PID锛屽彲鑳藉惎鍔ㄥけ璐ワ紝绛夊緟鍚庨噸璇?
            sleep "${RESTART_DELAY}"
            continue
        fi

        if [[ ${browser_opened} -eq 0 ]] && ! is_truthy "${DISABLE_WEB}"; then
            wait_for_web
            open_browser
            browser_opened=1
        fi

        while kill -0 "${prog_pid}" 2>/dev/null; do
            sleep 1
        done

        echo "[$(date '+%H:%M:%S')] 绋嬪簭宸查€€鍑猴紝${RESTART_DELAY} 绉掑悗閲嶅惎..."
        sleep "${RESTART_DELAY}"
        browser_opened=0
    done
}

main
