#!/bin/bash
set -e

CLEAN="${1:-}"
IMAGE_NAME="${IMAGE_NAME:-esp-idf-link}"
IMAGE_TAG="${IMAGE_TAG:-latest}"
IMAGE="$IMAGE_NAME:$IMAGE_TAG"

echo "Building ESP-IDF firmware..."

if [ "$CLEAN" = "clean" ]; then
    echo "Full clean build..."
    rm -rf build
fi

if ! docker image inspect "$IMAGE" > /dev/null 2>&1; then
    echo "Error: Docker image $IMAGE not found"
    echo "Run 'bash setup.sh' once to build it"
    exit 1
fi

docker run --rm \
    -v "$(pwd)":/project \
    -w /project \
    -e IDF_TARGET=esp32 \
    "$IMAGE" \
    bash -c "source /opt/esp/idf/export.sh && idf.py build"
echo ""
echo "[OK] Build complete!"
ls -lh build/link-idf-example.bin
