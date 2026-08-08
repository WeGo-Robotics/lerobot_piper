#!/usr/bin/env bash
set -euo pipefail

# Package one local LeRobot dataset directory for transfer to a training server.
#
# Usage:
#   DATASET_DIR=/home/liyuqi/Documents/Piper/datasets/local/my_dataset ./tools/package_lerobot_dataset.sh

DATASET_DIR="${DATASET_DIR:-}"
OUT_DIR="${OUT_DIR:-/home/liyuqi/Documents/Piper/exported_datasets}"

if [[ -z "${DATASET_DIR}" ]]; then
  echo "[error] DATASET_DIR is required." >&2
  echo "Example:" >&2
  echo "  DATASET_DIR=/home/liyuqi/Documents/Piper/datasets/local/piper_task_v1 $0" >&2
  exit 1
fi

DATASET_DIR="$(readlink -f "${DATASET_DIR}")"
DATASET_NAME="$(basename "${DATASET_DIR}")"
ARCHIVE="${OUT_DIR}/${DATASET_NAME}.tar.gz"
SHA_FILE="${ARCHIVE}.sha256"

if [[ ! -f "${DATASET_DIR}/meta/info.json" ]]; then
  echo "[error] Missing ${DATASET_DIR}/meta/info.json" >&2
  exit 1
fi
if [[ ! -d "${DATASET_DIR}/data" || ! -d "${DATASET_DIR}/meta" ]]; then
  echo "[error] Dataset must contain data/ and meta/." >&2
  exit 1
fi

mkdir -p "${OUT_DIR}"

echo "[package] dataset=${DATASET_DIR}"
echo "[package] output=${ARCHIVE}"
tar -C "$(dirname "${DATASET_DIR}")" -czf "${ARCHIVE}" "${DATASET_NAME}"

sha256sum "${ARCHIVE}" > "${SHA_FILE}"
du -h "${ARCHIVE}"
cat "${SHA_FILE}"
echo "[done] ${ARCHIVE}"
