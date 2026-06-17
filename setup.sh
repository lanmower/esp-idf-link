#!/bin/bash
set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
IMAGE_NAME="esp-idf-link"
IMAGE_TAG="latest"

echo "=== ESP-IDF Link Docker Setup ==="
echo "Project directory: $SCRIPT_DIR"

if ! command -v docker &> /dev/null; then
    echo "Error: Docker is not installed"
    echo "Please install Docker from https://www.docker.com/"
    exit 1
fi

echo "Checking Docker daemon..."
if ! docker info &> /dev/null; then
    echo "Error: Docker daemon is not running"
    echo "Please start Docker and try again"
    exit 1
fi

echo "Building Docker image: $IMAGE_NAME:$IMAGE_TAG"
docker build --no-cache -t "$IMAGE_NAME:$IMAGE_TAG" "$SCRIPT_DIR"

echo ""
echo "Initializing git submodules..."
docker run --rm \
  -v "$SCRIPT_DIR":/project \
  -w /project \
  "$IMAGE_NAME:$IMAGE_TAG" \
  bash -c "git submodule update --init --recursive"

echo ""
echo "[OK] Setup complete!"
echo ""
echo "To build the project:"
echo "  bash build.sh"
echo ""
echo "To flash to device:"
echo "  docker run --rm --device=/dev/ttyUSB0 -v \"$SCRIPT_DIR\":/project \"$IMAGE_NAME:$IMAGE_TAG\" bash -c 'source /opt/esp/idf/export.sh && idf.py -p /dev/ttyUSB0 flash'"
echo ""
