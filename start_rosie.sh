#!/bin/bash

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# 1. Allow access to X server
xhost +local: > /dev/null

IMAGE_NAME="rosie_image"
# Find the ID of any container (running or stopped) using our image
CONTAINER_ID=$(docker ps -aq -f ancestor=$IMAGE_NAME | head -n 1)

# 2. If a container using this image exists
if [ ! -z "$CONTAINER_ID" ]; then
    
    # 3. Check if it's currently running
    IS_RUNNING=$(docker ps -q -f id=$CONTAINER_ID)
    
    if [ -z "$IS_RUNNING" ]; then
        echo "🔄 Resuming your existing Rosie container ($CONTAINER_ID)..."
        docker start $CONTAINER_ID
    fi

    echo "🤖 Entering active Rosie container..."
    docker exec -it $CONTAINER_ID /bin/bash
else
    # 4. If no container exists for this image, create a fresh one
    echo "🚀 No existing containers found. Starting fresh..."
    docker run -it \
      --privileged=true \
      --net=host \
      --env="DISPLAY=$DISPLAY" \
      --env="QT_X11_NO_MITSHM=1" \
      --env="LIBGL_ALWAYS_SOFTWARE=1" \
      --env ROS_DOMAIN_ID=0 \
      --security-opt seccomp=unconfined \
      -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
      -v /dev/shm:/dev/shm \
      -v /dev/input:/dev/input \
      -v /dev/video0:/dev/video0 \
      -v "$SCRIPT_DIR":/workspace/rosie \
      $IMAGE_NAME /bin/bash
fi
