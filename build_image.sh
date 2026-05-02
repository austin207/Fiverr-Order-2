#!/bin/bash
set -e
REPO=$(cd "$(dirname "$0")" && pwd)
docker build -f "$REPO/navigation/.devcontainer/Dockerfile" -t ros2_benchmark:humble "$REPO/navigation/.devcontainer/"
