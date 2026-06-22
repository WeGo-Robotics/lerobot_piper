#!/usr/bin/env bash
set -euo pipefail

# Record a LeRobot dataset while a separate PC relay process teleoperates PiPER.
#
# Topology:
#   leader   -> USB-CAN A -> can0 -> PC
#   follower -> USB-CAN B -> can1 -> PC
#
# The relay process sends commands to the follower. lerobot-record only reads
# follower observations/cameras and stores follower state as action.

PIPER_ROOT="${PIPER_ROOT:-/home/liyuqi/Documents/Piper}"
LEROBOT_DIR="${LEROBOT_DIR:-${PIPER_ROOT}/lerobot_piper-piper}"
CONDA_ENV="${CONDA_ENV:-lerobot}"

ROBOT_CAMERAS_FROM_ENV="${ROBOT_CAMERAS-}"
ROBOT_CAMERAS_WAS_SET=0
if [[ -n "${ROBOT_CAMERAS+x}" ]]; then
  ROBOT_CAMERAS_WAS_SET=1
fi

if [[ -f "${LEROBOT_DIR}/camera_config_realsense.env" ]]; then
  # Provides WRIST_REALSENSE_SERIAL, GLOBAL_REALSENSE_SERIAL, and a default ROBOT_CAMERAS.
  source "${LEROBOT_DIR}/camera_config_realsense.env"
fi
if [[ "${ROBOT_CAMERAS_WAS_SET}" == "1" ]]; then
  ROBOT_CAMERAS="${ROBOT_CAMERAS_FROM_ENV}"
fi

LEADER_CAN="${LEADER_CAN:-can0}"
FOLLOWER_CAN="${FOLLOWER_CAN:-can1}"

RELAY_HZ="${RELAY_HZ:-20}"
RELAY_SPEED="${RELAY_SPEED:-100}"
RELAY_MAX_STEP_DEG="${RELAY_MAX_STEP_DEG:-10}"
RELAY_SOURCE="${RELAY_SOURCE:-control}"
RELAY_INCLUDE_GRIPPER="${RELAY_INCLUDE_GRIPPER:-1}"
RELAY_HIGH_FOLLOW="${RELAY_HIGH_FOLLOW:-1}"
RELAY_MODE_COMMAND_PERIOD="${RELAY_MODE_COMMAND_PERIOD:-1.0}"
RELAY_GRIPPER_HZ="${RELAY_GRIPPER_HZ:-5}"
RELAY_PRINT_PERIOD="${RELAY_PRINT_PERIOD:-0}"
RELAY_PAUSE_FILE="${RELAY_PAUSE_FILE:-/tmp/piper_pc_relay.pause}"

DATASET_BASE_DIR="${DATASET_BASE_DIR:-${DATASET_ROOT:-${PIPER_ROOT}/datasets}}"
DATASET_NAME="${DATASET_NAME:-piper_pc_relay_smoke}"
DATASET_FPS="${DATASET_FPS:-30}"
NUM_EPISODES="${NUM_EPISODES:-5}"
EPISODE_TIME_S="${EPISODE_TIME_S:-20}"
RESET_TIME_S="${RESET_TIME_S:-10}"
TASK="${TASK:-Teleoperate the follower PiPER with the leader PiPER}"
DISPLAY_DATA="${DISPLAY_DATA:-false}"
PUSH_TO_HUB="${PUSH_TO_HUB:-false}"
PLAY_SOUNDS="${PLAY_SOUNDS:-false}"
PHASE_BEEP="${PHASE_BEEP:-true}"
VOICE_PROMPT_DIR="${VOICE_PROMPT_DIR:-${LEROBOT_DIR}/audio_prompts/en-US-AriaNeural}"
AUTO_SUFFIX_DATASET_ROOT="${AUTO_SUFFIX_DATASET_ROOT:-1}"
MANUAL_STEP="${MANUAL_STEP:-true}"
AUTO_RESET_ARMS="${AUTO_RESET_ARMS:-true}"
AUTO_RESET_LEADER="${AUTO_RESET_LEADER:-false}"
RESET_SPEED="${RESET_SPEED:-50}"
RESET_DURATION="${RESET_DURATION:-5}"
RESET_HZ="${RESET_HZ:-10}"
RESET_TARGET_DEG="${RESET_TARGET_DEG:-0,0,0,0,0,0}"

# Default is a no-camera smoke test. For VLA data, override ROBOT_CAMERAS.
# Example:
# ROBOT_CAMERAS="{ wrist: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}, global: {type: opencv, index_or_path: 2, width: 640, height: 480, fps: 30} }"
if [[ -z "${ROBOT_CAMERAS+x}" ]]; then
  ROBOT_CAMERAS="{}"
fi

if [[ -n "${WRIST_REALSENSE_SERIAL:-}" ]]; then
  ROBOT_CAMERAS="${ROBOT_CAMERAS//WRIST_SERIAL/${WRIST_REALSENSE_SERIAL}}"
fi
if [[ -n "${GLOBAL_REALSENSE_SERIAL:-}" ]]; then
  ROBOT_CAMERAS="${ROBOT_CAMERAS//GLOBAL_SERIAL/${GLOBAL_REALSENSE_SERIAL}}"
fi

if [[ "${ROBOT_CAMERAS}" == *"WRIST_SERIAL"* || "${ROBOT_CAMERAS}" == *"GLOBAL_SERIAL"* ]]; then
  echo "[record] ERROR: ROBOT_CAMERAS still contains placeholder serials:" >&2
  echo "${ROBOT_CAMERAS}" >&2
  echo "[record] Source camera_config_realsense.env or replace WRIST_SERIAL/GLOBAL_SERIAL with actual serial numbers." >&2
  exit 1
fi

HF_USER_DEFAULT="$(hf auth whoami 2>/dev/null | head -n 1 || true)"
HF_USER="${HF_USER:-${HF_USER_DEFAULT:-local}}"
DATASET_REPO_ID="${DATASET_REPO_ID:-${HF_USER}/${DATASET_NAME}}"
DATASET_OUTPUT_ROOT="${DATASET_OUTPUT_ROOT:-${DATASET_BASE_DIR}/${DATASET_REPO_ID}}"

if [[ -e "${DATASET_OUTPUT_ROOT}" && "${AUTO_SUFFIX_DATASET_ROOT}" == "1" ]]; then
  DATASET_OUTPUT_ROOT="${DATASET_OUTPUT_ROOT}_$(date +%Y%m%d_%H%M%S)"
fi

if [[ -e "${DATASET_OUTPUT_ROOT}" ]]; then
  echo "[record] ERROR: dataset output root already exists: ${DATASET_OUTPUT_ROOT}" >&2
  echo "[record] Set DATASET_OUTPUT_ROOT to a new directory, or set AUTO_SUFFIX_DATASET_ROOT=1." >&2
  exit 1
fi

relay_cmd=(
  "${PIPER_ROOT}/piper_sdk/piper/bin/python"
  "${LEROBOT_DIR}/scripts/piper_pc_relay_teleop.py"
  --leader "${LEADER_CAN}"
  --follower "${FOLLOWER_CAN}"
  --source "${RELAY_SOURCE}"
  --hz "${RELAY_HZ}"
  --speed "${RELAY_SPEED}"
  --max-step-deg "${RELAY_MAX_STEP_DEG}"
  --mode-command-period "${RELAY_MODE_COMMAND_PERIOD}"
  --gripper-hz "${RELAY_GRIPPER_HZ}"
  --print-period "${RELAY_PRINT_PERIOD}"
  --pause-file "${RELAY_PAUSE_FILE}"
  --execute
)

if [[ "${RELAY_HIGH_FOLLOW}" == "1" ]]; then
  relay_cmd+=(--high-follow)
fi

if [[ "${RELAY_INCLUDE_GRIPPER}" == "1" ]]; then
  relay_cmd+=(--include-gripper)
fi

cleanup() {
  rm -f "${RELAY_PAUSE_FILE}" 2>/dev/null || true
  if [[ -n "${RELAY_PID:-}" ]] && kill -0 "${RELAY_PID}" 2>/dev/null; then
    kill "${RELAY_PID}" 2>/dev/null || true
    wait "${RELAY_PID}" 2>/dev/null || true
  fi
}
source "${HOME}/miniconda3/etc/profile.d/conda.sh"
conda activate "${CONDA_ENV}"
cd "${LEROBOT_DIR}"

ROBOT_CAMERAS="$(ROBOT_CAMERAS="${ROBOT_CAMERAS}" DATASET_FPS="${DATASET_FPS}" python - <<'PY'
import json
import os
import yaml

camera_cfg = yaml.safe_load(os.environ["ROBOT_CAMERAS"])
dataset_fps = int(os.environ["DATASET_FPS"])
if isinstance(camera_cfg, dict):
    for cfg in camera_cfg.values():
        if isinstance(cfg, dict) and "fps" in cfg:
            cfg["fps"] = dataset_fps
print(json.dumps(camera_cfg))
PY
)"

echo "[record] repo_id=${DATASET_REPO_ID}"
echo "[record] base_dir=${DATASET_BASE_DIR}"
echo "[record] output_root=${DATASET_OUTPUT_ROOT}"
echo "[record] cameras=${ROBOT_CAMERAS}"
echo "[record] keyboard controls: right=finish current episode/reset phase, left=rerecord episode, esc=stop recording"
echo "[record] play_sounds=${PLAY_SOUNDS}"
echo "[record] phase_beep=${PHASE_BEEP}"
echo "[record] voice_prompt_dir=${VOICE_PROMPT_DIR}"
echo "[record] manual_step=${MANUAL_STEP}"
echo "[record] auto_reset_arms=${AUTO_RESET_ARMS}"

ROBOT_CAMERAS="${ROBOT_CAMERAS}" DATASET_FPS="${DATASET_FPS}" python - <<'PY'
import os
import yaml

camera_cfg = yaml.safe_load(os.environ["ROBOT_CAMERAS"])
dataset_fps = int(os.environ["DATASET_FPS"])
if isinstance(camera_cfg, dict):
    mismatches = {
        name: cfg.get("fps")
        for name, cfg in camera_cfg.items()
        if isinstance(cfg, dict) and cfg.get("fps") is not None and int(cfg.get("fps")) != dataset_fps
    }
    if mismatches:
        raise SystemExit(
            "[record] ERROR: camera fps must match DATASET_FPS. "
            f"DATASET_FPS={dataset_fps}, mismatches={mismatches}"
        )
print("[record] camera config parse ok")
PY

trap cleanup EXIT INT TERM
rm -f "${RELAY_PAUSE_FILE}" 2>/dev/null || true

echo "[relay] ${relay_cmd[*]}"
"${relay_cmd[@]}" &
RELAY_PID="$!"
sleep 2

if [[ "${PHASE_BEEP}" == "1" || "${PHASE_BEEP}" == "true" ]]; then
  export LEROBOT_PHASE_BEEP=1
else
  export LEROBOT_PHASE_BEEP=0
fi
export LEROBOT_VOICE_PROMPT_DIR="${VOICE_PROMPT_DIR}"
if [[ "${AUTO_RESET_ARMS}" == "1" || "${AUTO_RESET_ARMS}" == "true" ]]; then
  reset_ports=("${FOLLOWER_CAN}")
  master_home_args=()
  if [[ "${AUTO_RESET_LEADER}" == "1" || "${AUTO_RESET_LEADER}" == "true" ]]; then
    reset_ports=("${LEADER_CAN}" "${FOLLOWER_CAN}")
    master_home_args=(--master-home-ports "${LEADER_CAN}")
  fi
  export LEROBOT_RESET_COMMAND="${PIPER_ROOT}/piper_sdk/piper/bin/python ${LEROBOT_DIR}/scripts/piper_reset_to_initial.py --ports ${reset_ports[*]} ${master_home_args[*]} --target-deg ${RESET_TARGET_DEG} --speed ${RESET_SPEED} --hz ${RESET_HZ} --duration ${RESET_DURATION} --high-follow"
  export LEROBOT_RELAY_PAUSE_FILE="${RELAY_PAUSE_FILE}"
  echo "[record] reset_command=${LEROBOT_RESET_COMMAND}"
  echo "[record] relay_pause_file=${LEROBOT_RELAY_PAUSE_FILE}"
else
  unset LEROBOT_RESET_COMMAND
  unset LEROBOT_RELAY_PAUSE_FILE
fi

lerobot-record \
  --robot.type=piper_follower \
  --robot.port="${FOLLOWER_CAN}" \
  --robot.cameras="${ROBOT_CAMERAS}" \
  --robot.id=pc_relay_follower \
  --dataset.root="${DATASET_OUTPUT_ROOT}" \
  --dataset.repo_id="${DATASET_REPO_ID}" \
  --dataset.fps="${DATASET_FPS}" \
  --dataset.num_episodes="${NUM_EPISODES}" \
  --dataset.episode_time_s="${EPISODE_TIME_S}" \
  --dataset.reset_time_s="${RESET_TIME_S}" \
  --dataset.single_task="${TASK}" \
  --dataset.push_to_hub="${PUSH_TO_HUB}" \
  --display_data="${DISPLAY_DATA}" \
  --play_sounds="${PLAY_SOUNDS}" \
  --manual_step="${MANUAL_STEP}"
