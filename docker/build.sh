#!/usr/bin/env bash
set -euo pipefail

# ------------------------------------------------------------------------------
# Environment Variables
# ------------------------------------------------------------------------------

export IMAGE="software-learning-period:latest"      # Docker image name/tag
export BASE_IMAGE="ros:humble"                      # Base image for Docker builds

# ------------------------------------------------------------------------------
# Platform Detection
# ------------------------------------------------------------------------------

ARCHITECTURE="$(uname -m)"
if [[ "$ARCHITECTURE" == "arm64" || "$ARCHITECTURE" == "aarch64" ]]; then
    PLATFORM="linux/arm64"
elif [[ "$ARCHITECTURE" == "x86_64" ]]; then
    PLATFORM="amd64"
else
    echo "Unsupported architecture: $ARCHITECTURE" >&2
    exit 1
fi

# ------------------------------------------------------------------------------
# Validate Required Variables
# ------------------------------------------------------------------------------

: "${BASE_IMAGE:?Environment variable BASE_IMAGE is required}"
: "${IMAGE:?Environment variable IMAGE is required}"

# ------------------------------------------------------------------------------
# Locate Script and Workspace Root
# ------------------------------------------------------------------------------

SCRIPT_DIR="$(dirname "$(realpath "$0")")"
# Use the parent of the docker folder as the workspace root
WORKSPACE="$(realpath "$SCRIPT_DIR/..")"

# ------------------------------------------------------------------------------
# Build Start Info
# ------------------------------------------------------------------------------

echo "======================================================================"
echo " Building Docker Image"
echo "   • PLATFORM:       $PLATFORM"
echo "   • BASE_IMAGE:     $BASE_IMAGE"
echo "   • IMAGE:          $IMAGE"
echo "   • BUILD CONTEXT:  $WORKSPACE"
echo "======================================================================"
echo ""

# ------------------------------------------------------------------------------
# Build Docker Image with Buildx
# ------------------------------------------------------------------------------

docker buildx build \
    --platform "$PLATFORM" \
    --build-arg BASE_IMAGE="$BASE_IMAGE" \
    --build-arg USER_ID="$(id -u)" \
    --build-arg GROUP_ID="$(id -g)" \
    --tag "$IMAGE" \
    --file "$SCRIPT_DIR/Dockerfile" \
    --load \
    "$WORKSPACE"

# ------------------------------------------------------------------------------
# Success Message
# ------------------------------------------------------------------------------

echo ""
echo "======================================================================"
echo " Successfully built image '$IMAGE' for platform '$PLATFORM'"
echo "======================================================================"
