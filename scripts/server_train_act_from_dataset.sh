#!/usr/bin/env bash
set -euo pipefail

# Train ACT from a transferred local LeRobot dataset.
# Run this on the training server inside the lerobot conda environment.

PIPER_ROOT="${PIPER_ROOT:-/root/autodl-tmp/Piper}"
LEROBOT_DIR="${LEROBOT_DIR:-${PIPER_ROOT}/lerobot_piper-piper}"
DATASET_DIR="${DATASET_DIR:-}"
DATASET_NAME="${DATASET_NAME:-}"
DATASET_REPO_ID="${DATASET_REPO_ID:-}"
OUTPUT_BASE="${OUTPUT_BASE:-${PIPER_ROOT}/outputs/train}"
JOB_NAME="${JOB_NAME:-piper_act}"
OUTPUT_DIR="${OUTPUT_DIR:-${OUTPUT_BASE}/${JOB_NAME}_$(date +%Y%m%d_%H%M%S)}"

POLICY_DEVICE="${POLICY_DEVICE:-cuda}"
BATCH_SIZE="${BATCH_SIZE:-8}"
STEPS="${STEPS:-20000}"
LOG_FREQ="${LOG_FREQ:-100}"
SAVE_FREQ="${SAVE_FREQ:-5000}"
EVAL_FREQ="${EVAL_FREQ:-0}"
NUM_WORKERS="${NUM_WORKERS:-4}"
CHUNK_SIZE="${CHUNK_SIZE:-30}"
N_ACTION_STEPS="${N_ACTION_STEPS:-30}"
VIDEO_BACKEND="${VIDEO_BACKEND:-pyav}"
PRETRAINED_BACKBONE_WEIGHTS="${PRETRAINED_BACKBONE_WEIGHTS:-ResNet18_Weights.IMAGENET1K_V1}"

if [[ -z "${DATASET_DIR}" ]]; then
  echo "[error] DATASET_DIR is required and must contain data/, meta/, videos/." >&2
  exit 1
fi

DATASET_DIR="$(readlink -f "${DATASET_DIR}")"
if [[ -z "${DATASET_NAME}" ]]; then
  DATASET_NAME="$(basename "${DATASET_DIR}")"
fi
if [[ -z "${DATASET_REPO_ID}" ]]; then
  DATASET_REPO_ID="local/${DATASET_NAME}"
fi

echo "[env] PIPER_ROOT=${PIPER_ROOT}"
echo "[env] LEROBOT_DIR=${LEROBOT_DIR}"
echo "[env] DATASET_DIR=${DATASET_DIR}"
echo "[env] DATASET_REPO_ID=${DATASET_REPO_ID}"
echo "[env] OUTPUT_DIR=${OUTPUT_DIR}"
echo "[env] POLICY_DEVICE=${POLICY_DEVICE}"
echo "[env] STEPS=${STEPS}"
echo "[env] BATCH_SIZE=${BATCH_SIZE}"
echo "[env] VIDEO_BACKEND=${VIDEO_BACKEND}"
echo "[env] PRETRAINED_BACKBONE_WEIGHTS=${PRETRAINED_BACKBONE_WEIGHTS}"

if [[ ! -d "${LEROBOT_DIR}" ]]; then
  echo "[error] LEROBOT_DIR does not exist: ${LEROBOT_DIR}" >&2
  exit 1
fi
if [[ ! -f "${DATASET_DIR}/meta/info.json" ]]; then
  echo "[error] Dataset meta not found: ${DATASET_DIR}/meta/info.json" >&2
  exit 1
fi
if ! command -v lerobot-train >/dev/null 2>&1; then
  echo "[error] lerobot-train not found. Activate the lerobot conda env first." >&2
  exit 1
fi

cd "${LEROBOT_DIR}"
mkdir -p "${OUTPUT_BASE}"

echo "[check] dataset load"
python - <<PY
from lerobot.datasets.lerobot_dataset import LeRobotDataset

ds = LeRobotDataset(
    repo_id="${DATASET_REPO_ID}",
    root="${DATASET_DIR}",
    video_backend="${VIDEO_BACKEND}",
)
print("num_frames:", ds.num_frames)
print("num_episodes:", ds.num_episodes)
print("fps:", ds.fps)
sample = ds[0]
print("action:", tuple(sample["action"].shape))
print("state:", tuple(sample["observation.state"].shape))
for key in sorted(k for k in sample if k.startswith("observation.images.")):
    print(key + ":", tuple(sample[key].shape))
PY

ARGS=(
  --policy.type=act
  --policy.device="${POLICY_DEVICE}"
  --policy.push_to_hub=false
  --policy.chunk_size="${CHUNK_SIZE}"
  --policy.n_action_steps="${N_ACTION_STEPS}"
  --dataset.repo_id="${DATASET_REPO_ID}"
  --dataset.root="${DATASET_DIR}"
  --dataset.video_backend="${VIDEO_BACKEND}"
  --batch_size="${BATCH_SIZE}"
  --steps="${STEPS}"
  --log_freq="${LOG_FREQ}"
  --save_freq="${SAVE_FREQ}"
  --eval_freq="${EVAL_FREQ}"
  --num_workers="${NUM_WORKERS}"
  --output_dir="${OUTPUT_DIR}"
  --job_name="${JOB_NAME}"
  --wandb.enable=false
)

if [[ "${PRETRAINED_BACKBONE_WEIGHTS}" == "null" || "${PRETRAINED_BACKBONE_WEIGHTS}" == "None" || "${PRETRAINED_BACKBONE_WEIGHTS}" == "none" ]]; then
  ARGS+=(--policy.pretrained_backbone_weights=null)
else
  ARGS+=(--policy.pretrained_backbone_weights="${PRETRAINED_BACKBONE_WEIGHTS}")
fi

echo "[train] lerobot-train ${ARGS[*]}"
lerobot-train "${ARGS[@]}"

echo "[done] output=${OUTPUT_DIR}"
echo "[done] latest checkpoints:"
find "${OUTPUT_DIR}/checkpoints" -maxdepth 3 -type f | sort | tail -50
