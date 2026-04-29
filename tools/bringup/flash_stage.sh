#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BUILD_SCRIPT="$ROOT_DIR/tools/bringup/build_stage.sh"

usage() {
    cat <<'USAGE'
Usage:
  tools/bringup/flash_stage.sh [flash options] <stage> [build options]

Flash options:
  --method auto|openocd|stm32prog|st-flash
  --openocd-bin PATH
  --openocd-scripts DIR
  --cmsis-dap-vid-pid VID:PID   default: 0x303a:0x40ff for ICWorkshop PowerDebugger

Examples:
  tools/bringup/flash_stage.sh board
  tools/bringup/flash_stage.sh --method openocd board
  tools/bringup/flash_stage.sh --method openocd go_leg_hold --leg FL
  tools/bringup/flash_stage.sh m3508_jog --wheel FL --speed 0.3
USAGE
}

if [[ $# -lt 1 ]]; then
    usage
    echo
    "$BUILD_SCRIPT" --help
    exit 2
fi

method="auto"
openocd_bin="${OPENOCD_BIN:-}"
openocd_scripts="${OPENOCD_SCRIPTS:-}"
cmsis_dap_vid_pid="${OPENOCD_CMSIS_DAP_VID_PID:-0x303a:0x40ff}"
build_args=()

while [[ $# -gt 0 ]]; do
    case "$1" in
        --method)
            [[ $# -ge 2 ]] || { echo "--method needs a value" >&2; exit 2; }
            method="$2"
            shift 2
            ;;
        --openocd-bin)
            [[ $# -ge 2 ]] || { echo "--openocd-bin needs a value" >&2; exit 2; }
            openocd_bin="$2"
            shift 2
            ;;
        --openocd-scripts)
            [[ $# -ge 2 ]] || { echo "--openocd-scripts needs a value" >&2; exit 2; }
            openocd_scripts="$2"
            shift 2
            ;;
        --cmsis-dap-vid-pid)
            [[ $# -ge 2 ]] || { echo "--cmsis-dap-vid-pid needs a value" >&2; exit 2; }
            cmsis_dap_vid_pid="$2"
            shift 2
            ;;
        -h|--help)
            usage
            echo
            "$BUILD_SCRIPT" --help
            exit 0
            ;;
        *)
            build_args+=("$1")
            shift
            ;;
    esac
done

case "$method" in
    auto|openocd|stm32prog|st-flash) ;;
    *) echo "Unknown flash method: $method" >&2; exit 2 ;;
esac

if [[ ${#build_args[@]} -lt 1 ]]; then
    usage
    exit 2
fi

stage_arg="${build_args[0]}"
"$BUILD_SCRIPT" "${build_args[@]}"

case "$stage_arg" in
    0|board|board_only) stage_name="00_board" ;;
    1|bus_zero|bus-zero) stage_name="01_bus_zero" ;;
    2|m3508_zero|m3508-zero) stage_name="02_m3508_zero" ;;
    3|go_zero|go-zero) stage_name="03_go_zero" ;;
    4|m3508_jog|m3508-jog) stage_name="04_m3508_jog" ;;
    5|go_leg_hold|go-leg-hold) stage_name="05_go_leg_hold" ;;
    6|stand_low|stand-low) stage_name="06_stand_low" ;;
    7|trot_low|trot-low) stage_name="07_trot_low" ;;
    100|normal) stage_name="99_normal" ;;
    *) echo "Unknown stage after build: $stage_arg" >&2; exit 2 ;;
esac

build_dir="$ROOT_DIR/build/bringup/$stage_name"
elf="$build_dir/FengMH.elf"
bin="$build_dir/FengMH.bin"

find_openocd() {
    if [[ -n "$openocd_bin" ]]; then
        [[ -x "$openocd_bin" ]] || return 1
        echo "$openocd_bin"
        return 0
    fi

    if command -v openocd >/dev/null 2>&1; then
        command -v openocd
        return 0
    fi

    local candidates=(
        "/opt/homebrew/bin/openocd"
        "/Applications/STM32CubeIDE.app/Contents/Eclipse/plugins/com.st.stm32cube.ide.mcu.externaltools.openocd.macosaarch64_1.0.0.202601242230/tools/bin/openocd"
    )

    for p in "${candidates[@]}"; do
        if [[ -x "$p" ]]; then
            echo "$p"
            return 0
        fi
    done

    return 1
}

find_openocd_scripts() {
    if [[ -n "$openocd_scripts" ]]; then
        [[ -d "$openocd_scripts" ]] || return 1
        echo "$openocd_scripts"
        return 0
    fi

    local candidates=(
        "/opt/homebrew/share/openocd/scripts"
        "/usr/local/share/openocd/scripts"
    )

    for p in "${candidates[@]}"; do
        if [[ -d "$p" ]]; then
            echo "$p"
            return 0
        fi
    done

    return 1
}

find_stm32_programmer() {
    if command -v STM32_Programmer_CLI >/dev/null 2>&1; then
        command -v STM32_Programmer_CLI
        return 0
    fi

    local candidates=(
        "/opt/ST/STM32CubeCLT_1.18.0/STM32CubeProgrammer/bin/STM32_Programmer_CLI"
        "/Applications/STMicroelectronics/STM32Cube/STM32CubeProgrammer/STM32CubeProgrammer.app/Contents/MacOs/bin/STM32_Programmer_CLI"
        "/Applications/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI"
    )

    for p in "${candidates[@]}"; do
        if [[ -x "$p" ]]; then
            echo "$p"
            return 0
        fi
    done

    return 1
}

flash_openocd() {
    local ocd scripts
    ocd="$(find_openocd)" || return 1
    scripts="$(find_openocd_scripts)" || return 1

    local vid_pid_args=()
    if [[ -n "$cmsis_dap_vid_pid" && "$cmsis_dap_vid_pid" != "auto" ]]; then
        if [[ "$cmsis_dap_vid_pid" != *:* ]]; then
            echo "--cmsis-dap-vid-pid must be VID:PID, or 'auto'" >&2
            return 2
        fi
        local vid="${cmsis_dap_vid_pid%%:*}"
        local pid="${cmsis_dap_vid_pid##*:}"
        vid_pid_args=(-c "cmsis_dap_vid_pid $vid $pid")
    fi

    "$ocd" \
        -s "$scripts" \
        -f interface/cmsis-dap.cfg \
        "${vid_pid_args[@]}" \
        -c "transport select swd" \
        -c "adapter speed 1000" \
        -f target/stm32h7x.cfg \
        -c "program $elf verify reset exit"
}

flash_stm32prog() {
    local programmer
    programmer="$(find_stm32_programmer)" || return 1
    "$programmer" -c port=SWD mode=UR reset=HWrst -w "$elf" -v -rst
}

flash_stflash() {
    command -v st-flash >/dev/null 2>&1 || return 1
    st-flash --connect-under-reset write "$bin" 0x08000000
}

try_method() {
    case "$1" in
        openocd) flash_openocd ;;
        stm32prog) flash_stm32prog ;;
        st-flash) flash_stflash ;;
    esac
}

if [[ "$method" == "auto" ]]; then
    for m in openocd stm32prog st-flash; do
        echo "Trying flash method: $m"
        if try_method "$m"; then
            exit 0
        fi
        echo "Flash method failed or unavailable: $m" >&2
    done
else
    try_method "$method"
    exit $?
fi

cat >&2 <<EOF
No supported flash method succeeded.

Built firmware is still available at:
  $elf
  $bin
EOF
exit 1
