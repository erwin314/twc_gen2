#!/usr/bin/env bash

# This script is used to run the ESPHome command in a container.
# It is used to avoid having to install ESPHome on the host machine.
# It is also used to avoid having to install the dependencies on the host machine.

# Example to compile the TWC component: 
# ./esphome.sh compile twctest.yaml

# Fail fast: 
# -e exit on error, 
# -u error on unset vars, 
# -o pipefail causes a pipeline to fail if any command fails (not just the last)
set -euo pipefail

# The ESPHome Docker image
IMAGE="ghcr.io/esphome/esphome"

# Use interactive TTY only when attached to a terminal
TTY_ARGS=""
if [ -t 0 ] && [ -t 1 ]; then
  TTY_ARGS="-it"
fi

# Resolve repository root based on script location (this script lives in dev/)
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

# Run the ESPHome command in a container
docker run --rm ${TTY_ARGS} \
  -v "${REPO_ROOT}/dev:/config" \
  -v "${REPO_ROOT}/components:/components" \
  "${IMAGE}" "$@"
