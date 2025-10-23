#!/bin/bash

# ==== Configuration ====
REPO_NAME="Croissant"
DESCRIPTION="goes well with coffee"
MAINTAINER="Gulin534 (gerbilen@constructor.university)"
ARCH="amd64"
DISTRO="daffy"
DOCKER_REGISTRY="docker.io"
IMAGE_NAME="duckietown/croissant:demo-amd64"

# ==== Build ====
echo "🔧 Building Docker image: $IMAGE_NAME"
docker build \
  --build-arg ARCH=$ARCH \
  --build-arg DISTRO=$DISTRO \
  --build-arg DOCKER_REGISTRY=$DOCKER_REGISTRY \
  --build-arg REPO_NAME="$REPO_NAME" \
  --build-arg DESCRIPTION="$DESCRIPTION" \
  --build-arg MAINTAINER="$MAINTAINER" \
  -t $IMAGE_NAME .

# ==== Result ====
if [ $? -eq 0 ]; then
  echo "✅ Build completed: $IMAGE_NAME"
else
  echo "❌ Build failed"
fi
