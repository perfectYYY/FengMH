#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

usage() {
    cat <<'USAGE'
Usage:
  tools/bringup/build_stage.sh <stage> [--leg FL|FR|RL|RR|all|none] [--wheel FL|FR|RL|RR|all|none] [--speed RAD_S] [--clean]

Stages:
  board          0   MCU + USB only, no motor TX
  bus_zero       1   GO disabled frames + M3508 zero-current frames
  m3508_zero     2   M3508 zero-current frames only
  go_zero        3   GO disabled frames only
  m3508_jog      4   selected M3508 wheel low-speed jog
  go_leg_hold    5   selected leg GO low-gain stand hold
  stand_low      6   all legs low-gain stand hold
  trot_low       7   all legs low-gain, tiny trot allowed by USB command
  usb_cdc        8   USB CDC ping/pong + telemetry, no motor/IMU init
  imu            9   BMI088 read + USB telemetry, no motor task
  motor_test    10   one-shot fixed GO joint + M3508 wheel motion test
  normal       100   normal firmware behavior

Examples:
  tools/bringup/build_stage.sh board
  tools/bringup/build_stage.sh m3508_jog --wheel FL --speed 0.3
  tools/bringup/build_stage.sh go_leg_hold --leg RL
  tools/bringup/build_stage.sh motor_test --leg all --wheel all --speed 0.3
USAGE
}

mask_for_name() {
    case "$1" in
        FL|fl) echo "0x01" ;;
        FR|fr) echo "0x02" ;;
        RL|rl) echo "0x04" ;;
        RR|rr) echo "0x08" ;;
        all|ALL) echo "0x0F" ;;
        none|NONE) echo "0x00" ;;
        0x*|0X*) echo "$1" ;;
        *) echo "unknown" ;;
    esac
}

if [[ $# -lt 1 ]]; then
    usage
    exit 2
fi

stage_arg="$1"
shift

case "$stage_arg" in
    0|board|board_only) stage=0; stage_name="00_board" ;;
    1|bus_zero|bus-zero) stage=1; stage_name="01_bus_zero" ;;
    2|m3508_zero|m3508-zero) stage=2; stage_name="02_m3508_zero" ;;
    3|go_zero|go-zero) stage=3; stage_name="03_go_zero" ;;
    4|m3508_jog|m3508-jog) stage=4; stage_name="04_m3508_jog" ;;
    5|go_leg_hold|go-leg-hold) stage=5; stage_name="05_go_leg_hold" ;;
    6|stand_low|stand-low) stage=6; stage_name="06_stand_low" ;;
    7|trot_low|trot-low) stage=7; stage_name="07_trot_low" ;;
    8|usb_cdc|usb-cdc|cdc|usb) stage=8; stage_name="08_usb_cdc_test" ;;
    9|imu|imu_test|imu-test) stage=9; stage_name="09_imu_test" ;;
    10|motor_test|motor-test|motors) stage=10; stage_name="10_motor_fixed_test" ;;
    100|normal) stage=100; stage_name="99_normal" ;;
    -h|--help|help) usage; exit 0 ;;
    *) echo "Unknown stage: $stage_arg" >&2; usage; exit 2 ;;
esac

leg_mask="0x01"
wheel_mask="0x01"
wheel_speed="0.5"
clean=0

while [[ $# -gt 0 ]]; do
    case "$1" in
        --leg)
            [[ $# -ge 2 ]] || { echo "--leg needs a value" >&2; exit 2; }
            leg_mask="$(mask_for_name "$2")"
            shift 2
            ;;
        --wheel)
            [[ $# -ge 2 ]] || { echo "--wheel needs a value" >&2; exit 2; }
            wheel_mask="$(mask_for_name "$2")"
            shift 2
            ;;
        --speed)
            [[ $# -ge 2 ]] || { echo "--speed needs a value" >&2; exit 2; }
            wheel_speed="$2"
            shift 2
            ;;
        --clean)
            clean=1
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Unknown option: $1" >&2
            usage
            exit 2
            ;;
    esac
done

if [[ "$leg_mask" == "unknown" || "$wheel_mask" == "unknown" ]]; then
    echo "Unknown leg or wheel name. Use FL, FR, RL, RR, all, none, or a hex mask." >&2
    exit 2
fi

case "$wheel_speed" in
    *f) wheel_speed_macro="$wheel_speed" ;;
    *) wheel_speed_macro="${wheel_speed}f" ;;
esac

build_dir="$ROOT_DIR/build/bringup/$stage_name"
if [[ "$clean" -eq 1 ]]; then
    rm -rf "$build_dir"
fi

cmake -S "$ROOT_DIR" -B "$build_dir" \
    -DCMAKE_BUILD_TYPE=Debug \
    -DAPP_BRINGUP_STAGE="$stage" \
    -DAPP_BRINGUP_LEG_MASK="$leg_mask" \
    -DAPP_BRINGUP_WHEEL_MASK="$wheel_mask" \
    -DAPP_BRINGUP_WHEEL_JOG_RAD_S="$wheel_speed_macro"

cmake --build "$build_dir" --target FengMH.elf

echo
echo "Built stage $stage ($stage_name)"
echo "  leg_mask=$leg_mask wheel_mask=$wheel_mask wheel_jog=$wheel_speed_macro"
echo "  ELF: $build_dir/FengMH.elf"
echo "  HEX: $build_dir/FengMH.hex"
echo "  BIN: $build_dir/FengMH.bin"
