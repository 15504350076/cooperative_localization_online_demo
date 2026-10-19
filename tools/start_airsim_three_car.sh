#!/usr/bin/env bash
set -euo pipefail

# 功能：使用本工程三车配置启动 AirSimAG Unreal 场景。
# 默认打开图形窗口；加 --headless 仅用于无显卡 RPC 冒烟测试。

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
AIRSIM_ROOT="${AIRSIM_ROOT:-/home/pdq/Code/AirSimAG}"
HEADLESS=0

for arg in "$@"; do
  case "$arg" in
    --headless) HEADLESS=1 ;;
    *) printf '未知参数：%s\n' "$arg" >&2; exit 2 ;;
  esac
done

START_SIM="$AIRSIM_ROOT/deployment/start_sim.sh"
SETTINGS="$PROJECT_ROOT/ros2/zju_coop_gazebo/config/airsim_three_ugv_settings.json"
test -x "$START_SIM" || { printf '找不到 AirSimAG 启动脚本：%s\n' "$START_SIM" >&2; exit 1; }
test -f "$SETTINGS" || { printf '找不到三车配置：%s\n' "$SETTINGS" >&2; exit 1; }

cd "$AIRSIM_ROOT"
HEADLESS="$HEADLESS" "$START_SIM" --settings "$SETTINGS"
