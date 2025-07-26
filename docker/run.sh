#!/usr/bin/env bash
set -euo pipefail

# ------------------------------------------------------------------------------
# Image Configuration
# ------------------------------------------------------------------------------

IMAGE="software-learning-period:latest"  # Default Docker image name/tag

# ------------------------------------------------------------------------------
# Platform Detection (Optional)
# ------------------------------------------------------------------------------
# While 'docker run' doesn't require this unless you do platform-specific logic,
# we log it for transparency.
# ------------------------------------------------------------------------------

ARCHITECTURE="$(uname -m)"
if [[ "$ARCHITECTURE" == "arm64" || "$ARCHITECTURE" == "aarch64" ]]; then
    PLATFORM="arm64"
elif [[ "$ARCHITECTURE" == "x86_64" ]]; then
    PLATFORM="amd64"
else
    echo "Unsupported architecture: $ARCHITECTURE" >&2
    exit 1
fi

# ------------------------------------------------------------------------------
# Locate Script Directory and Workspace Root
# ------------------------------------------------------------------------------

SCRIPT_DIR="$(dirname "$(realpath "$0")")"
WORKSPACE="$(realpath "$SCRIPT_DIR/../../..")" # A bit cursed, but should work

# ------------------------------------------------------------------------------
# Run Information
# ------------------------------------------------------------------------------

echo "======================================================================"
echo " Running Container"
echo "   • IMAGE:          $IMAGE"
echo "   • PLATFORM:       $PLATFORM"
echo "   • MOUNT:          $WORKSPACE  →  /ros2_ws"
echo "======================================================================"
echo ""

# ------------------------------------------------------------------------------
# Run Docker Container
# ------------------------------------------------------------------------------

docker run -it --rm \
    --user "$(id -u):$(id -g)" \
    --privileged \
    --network host \
    --ipc=host \
    -v "$WORKSPACE":/ros2_ws \
    -w /ros2_ws \
    "$IMAGE" \
    /bin/bash
