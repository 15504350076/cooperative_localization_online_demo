#!/usr/bin/env bash
# Minimal ROS 2 graph/data smoke check for an AIBrainBox.
# UWB payload fields are deliberately not parsed: its formal type is not frozen.

set -u
set -o pipefail

readonly NODE_STATE_TOPIC="/cooperative_localization/node_state"
readonly NODE_STATE_TYPE="cooperative_localization_msgs/msg/NodeState"
readonly UWB_TOPIC="/uwb/range"
readonly POSES_TOPIC="/cooperative_localization/poses_2d"
readonly POSES_TYPE="cooperative_localization_msgs/msg/CooperativePose2DArray"
readonly IMU_TYPE="sensor_msgs/msg/Imu"

WAIT_SECONDS=0
SPIN_TIME=2
IMU_TOPIC=""
EXPECT_ARCH=""
EXPECT_RMW=""
FAILURES=0

usage() {
  cat <<'EOF'
Usage: zju_box_smoke_check [options]

Checks the ROS 2 environment and discoverable types/endpoint summary for
NodeState, UWB range and cooperative result.  Add --imu-topic to check one
vehicle IMU.  Add --wait N to require one message on every checked topic.

Options:
  --imu-topic TOPIC    Also check TOPIC as sensor_msgs/msg/Imu.
  --wait SECONDS       Require one message on every checked topic.
  --spin-time SECONDS  ROS 2 discovery wait per graph query (default: 2).
  --expect-arch ARCH   Fail if uname -m differs (e.g. aarch64).
  --expect-rmw RMW     Fail if RMW_IMPLEMENTATION differs.
  -h, --help           Show this help.

Exit status: 0=pass, 1=check failure, 2=invalid arguments/tool unavailable.
EOF
}

fail() {
  printf '[FAIL] %s\n' "$*" >&2
  FAILURES=$((FAILURES + 1))
}

pass() {
  printf '[PASS] %s\n' "$*"
}

warn() {
  printf '[WARN] %s\n' "$*" >&2
}

invalid_args() {
  printf '[ERROR] %s\n' "$*" >&2
  usage >&2
  exit 2
}

is_positive_integer() {
  [[ "$1" =~ ^[1-9][0-9]*$ ]]
}

normalize_topic() {
  case "$1" in
    /*) printf '%s' "$1" ;;
    *) printf '/%s' "$1" ;;
  esac
}

while (($# > 0)); do
  case "$1" in
    --imu-topic)
      (($# >= 2)) || invalid_args "--imu-topic requires a topic name"
      IMU_TOPIC="$(normalize_topic "$2")"
      shift 2
      ;;
    --wait)
      (($# >= 2)) || invalid_args "--wait requires a positive integer"
      is_positive_integer "$2" || invalid_args "--wait must be a positive integer"
      WAIT_SECONDS="$2"
      shift 2
      ;;
    --spin-time)
      (($# >= 2)) || invalid_args "--spin-time requires a positive integer"
      is_positive_integer "$2" || invalid_args "--spin-time must be a positive integer"
      SPIN_TIME="$2"
      shift 2
      ;;
    --expect-arch)
      (($# >= 2)) || invalid_args "--expect-arch requires a value"
      EXPECT_ARCH="$2"
      shift 2
      ;;
    --expect-rmw)
      (($# >= 2)) || invalid_args "--expect-rmw requires a value"
      EXPECT_RMW="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      invalid_args "unknown option: $1"
      ;;
  esac
done

printf '%s\n' 'zju_box_smoke_check: minimal ROS 2 box smoke check'

if ! command -v ros2 >/dev/null 2>&1; then
  printf '[ERROR] ros2 command is unavailable; source the target ROS 2 setup.bash\n' >&2
  exit 2
fi
if ((WAIT_SECONDS > 0)) && ! command -v timeout >/dev/null 2>&1; then
  printf '[ERROR] --wait requires the standard timeout command\n' >&2
  exit 2
fi

if [[ "${ROS_VERSION-}" == "2" ]]; then
  pass "ROS_VERSION=2"
else
  fail "ROS_VERSION must be 2 (got '${ROS_VERSION-<unset>}')"
fi
if [[ -n "${ROS_DISTRO-}" ]]; then
  pass "ROS_DISTRO=${ROS_DISTRO}"
else
  fail 'ROS_DISTRO is unset; source the target ROS 2 setup.bash'
fi
if [[ -n "${ROS_DOMAIN_ID-}" && "${ROS_DOMAIN_ID}" =~ ^[0-9]+$ ]]; then
  pass "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
else
  fail 'ROS_DOMAIN_ID must be an explicitly set non-negative integer'
fi
if [[ -n "${RMW_IMPLEMENTATION-}" ]]; then
  if [[ -n "$EXPECT_RMW" && "$RMW_IMPLEMENTATION" != "$EXPECT_RMW" ]]; then
    fail "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION} (expected ${EXPECT_RMW})"
  else
    pass "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
  fi
else
  fail 'RMW_IMPLEMENTATION is unset; set the same RMW on all boxes'
fi
if [[ "${ROS_LOCALHOST_ONLY-0}" == "1" ]]; then
  fail 'ROS_LOCALHOST_ONLY=1 blocks cross-box DDS; set it to 0'
else
  pass 'ROS_LOCALHOST_ONLY allows cross-box discovery'
fi

MACHINE="$(uname -m 2>/dev/null || true)"
case "$MACHINE" in
  aarch64|arm64|x86_64|amd64)
    if [[ -n "$EXPECT_ARCH" && "$MACHINE" != "$EXPECT_ARCH" ]]; then
      fail "architecture=${MACHINE} (expected ${EXPECT_ARCH})"
    else
      pass "architecture=${MACHINE}"
    fi
    ;;
  *)
    fail "unrecognized architecture '${MACHINE:-<unknown>}'"
    ;;
esac

# --no-daemon avoids stale discovery state after a box process is restarted.
ros2_topic_type() {
  ros2 topic type --no-daemon --spin-time "$SPIN_TIME" "$1" 2>/dev/null
}

ros2_topic_info() {
  ros2 topic info --no-daemon --spin-time "$SPIN_TIME" --verbose "$1" 2>/dev/null
}

qos_values() {
  local pattern="$1"
  local info="$2"
  local values
  values="$(printf '%s\n' "$info" \
    | sed -n -E "s/^[[:space:]]*${pattern}:[[:space:]]*//p" \
    | sort -u | tr '\n' ',' | sed 's/,$//')"
  printf '%s' "${values:-unknown}"
}

check_topic_graph() {
  local label="$1"
  local topic="$2"
  local expected_type="$3"
  local type_output type info publisher_count subscriber_count reliability durability

  type_output="$(ros2_topic_type "$topic")"
  type="$(printf '%s\n' "$type_output" | sed '/^[[:space:]]*$/d' | head -n 1 | tr -d '\r')"
  if [[ -z "$type" ]]; then
    fail "$label: ${topic} is not discoverable or has no visible type"
    return
  fi
  if [[ -n "$expected_type" ]] && ! printf '%s\n' "$type_output" | grep -Fqx "$expected_type"; then
    fail "$label: ${topic} type=${type}, expected ${expected_type}"
    return
  fi

  info="$(ros2_topic_info "$topic")"
  if [[ -z "$info" ]]; then
    fail "$label: unable to read topic graph for ${topic}"
    return
  fi
  publisher_count="$(printf '%s\n' "$info" | sed -n -E 's/^[[:space:]]*Publisher count:[[:space:]]*//p' | head -n 1)"
  subscriber_count="$(printf '%s\n' "$info" | sed -n -E 's/^[[:space:]]*Subscription count:[[:space:]]*//p' | head -n 1)"
  publisher_count="${publisher_count:-?}"
  subscriber_count="${subscriber_count:-?}"
  reliability="$(qos_values 'Reliability' "$info")"
  durability="$(qos_values 'Durability' "$info")"
  pass "$label: ${topic} type=${type} publishers=${publisher_count} subscribers=${subscriber_count} reliability=${reliability} durability=${durability}"
  if [[ "$publisher_count" == "0" ]]; then
    warn "$label: ${topic} has no discovered publisher yet"
  fi
}

check_topic_message() {
  local label="$1"
  local topic="$2"
  # Best-effort subscription can match both best-effort and reliable publishers.
  if timeout "${WAIT_SECONDS}s" ros2 topic echo --no-daemon --once \
      --qos-reliability best_effort --qos-durability volatile \
      "$topic" >/dev/null 2>&1; then
    pass "$label: received one message on ${topic} within ${WAIT_SECONDS}s"
  else
    fail "$label: no message received on ${topic} within ${WAIT_SECONDS}s"
  fi
}

check_topic_graph 'NodeState' "$NODE_STATE_TOPIC" "$NODE_STATE_TYPE"
# Do not assume the platform team's not-yet-frozen UWB package/type.
check_topic_graph 'UWB range' "$UWB_TOPIC" ''
check_topic_graph 'Cooperative result' "$POSES_TOPIC" "$POSES_TYPE"
if [[ -n "$IMU_TOPIC" ]]; then
  check_topic_graph 'IMU' "$IMU_TOPIC" "$IMU_TYPE"
fi

if ((WAIT_SECONDS > 0)); then
  check_topic_message 'NodeState' "$NODE_STATE_TOPIC"
  check_topic_message 'UWB range' "$UWB_TOPIC"
  check_topic_message 'Cooperative result' "$POSES_TOPIC"
  if [[ -n "$IMU_TOPIC" ]]; then
    check_topic_message 'IMU' "$IMU_TOPIC"
  fi
fi

if ((FAILURES == 0)); then
  if ((WAIT_SECONDS > 0)); then
    printf '%s\n' 'SMOKE PASS (graph + one-message checks)'
  else
    printf '%s\n' 'SMOKE PASS (graph checks; use --wait N for data checks)'
  fi
  exit 0
fi

printf 'SMOKE FAIL (failures=%d)\n' "$FAILURES" >&2
exit 1
