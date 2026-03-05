#!/bin/bash

xhost +local:root
xhost +local:docker

# 获取当前DISPLAY
XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth
touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

# 获取 GVINS 工程的绝对路径
GVINS_DIR=$(cd .. && pwd)

# 检查是否存在镜像
if ! docker images | grep -q "gvins.*latest"; then
    echo "Error: Docker image 'gvins:latest' not found."
    echo "Please build or pull the image first."
    exit 1
fi

echo "启动 GVINS Docker 容器（后台模式）..."
echo "GVINS 目录: ${GVINS_DIR}"

# 删除已存在的同名容器
docker rm -f gvins-test 2>/dev/null || true

# 后台启动容器
docker run -d \
  --name gvins-test \
  --net=host \
  --privileged \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --env="XAUTHORITY=$XAUTH" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="$XSOCK:$XSOCK:rw" \
  --volume="$XAUTH:$XAUTH:rw" \
  --volume="${GVINS_DIR}:/root/catkin_ws/src/GVINS" \
  --volume="/home/serena/gvins_output:/root/data" \
  --volume="/home/serena/lab/bags:/root/bags" \
  --device=/dev/dri:/dev/dri \
  gvins:latest \
  tail -f /dev/null

echo "容器已启动！"
echo "现在可以从任意终端使用以下命令进入容器："
echo "  docker exec -it gvins-test /bin/bash"