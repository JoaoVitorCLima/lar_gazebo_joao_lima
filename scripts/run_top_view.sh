#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
export USER_UID="$(id -u)"
export USER_GID="$(id -g)"
export USER_NAME="${USER:-ros}"
xhost +local:docker >/dev/null 2>&1 || true
docker compose run --rm -e LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-0}" lar_gazebo roslaunch lar_gazebo top_view.launch "$@"
