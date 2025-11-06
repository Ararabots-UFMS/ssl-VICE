# docker/Dockerfile.dev
FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive

# Install common tools + colcon + Python deps
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    iproute2 \
    net-tools \
    iputils-ping \
    && rm -rf /var/lib/apt/lists/*

RUN pip install --upgrade pip setuptools==58.2.0 wheel packaging scikit-build protobuf==3.20.* flask flask-socketio pyserial numpy pytest pytest-mock pytest-cov

# ROS environment automatically sourced
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

WORKDIR /root/ssl-VICE
CMD ["bash"]

