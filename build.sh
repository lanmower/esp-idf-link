#!/bin/bash
set -e

CLEAN="${1:-}"

echo "Building ESP-IDF firmware..."

if [ "$CLEAN" = "clean" ]; then
    echo "Full clean build..."
    rm -rf build
fi

docker compose up build
echo ""
echo "✓ Build complete!"
ls -lh build/link-idf-example.bin
