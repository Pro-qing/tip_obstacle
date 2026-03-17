# 使用官方 Noetic 基础镜像
FROM osrf/ros:noetic-desktop-full

# 设置环境变量，避免安装时的交互提示
ENV DEBIAN_FRONTEND=noninteractive

# 1. 更换国内镜像源（中科大源）
RUN sed -i 's/archive.ubuntu.com/mirrors.ustc.edu.cn/g' /etc/apt/sources.list

# 2. 安装基础开发工具及依赖
RUN apt-get update && apt-get install -y \
    python3-catkin-tools \
    python3-colcon-common-extensions \
    build-essential \
    cmake \
    git \
    vim \
    tree \
    && rm -rf /var/lib/apt/lists/*

# 4. 设置工作空间
WORKDIR /home/ros_ws

# 6. 自动加载 ROS 环境
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# 7. 设置启动入口
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]