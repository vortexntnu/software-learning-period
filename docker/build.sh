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
# On macOS (Darwin), force 'linux/arm64' (for Docker Desktop on Mac).
# Otherwise, default to the host architecture.
# NOTE: This is hardcoded for simplicity.
# ------------------------------------------------------------------------------

if [[ "$(uname)" == "Darwin" ]]; then
    export PLATFORM="linux/arm64"
else
    export PLATFORM="linux/amd64"
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
WORKSPACE="$(realpath "$SCRIPT_DIR/../../..")"  # A bit cursed, but should work

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
