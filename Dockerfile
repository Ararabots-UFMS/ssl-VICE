FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    iproute2 \
    net-tools \
    iputils-ping \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root/ssl-VICE
COPY . .

# Instala dependências Python
RUN pip install --upgrade pip setuptools wheel packaging scikit-build
RUN pip install -r requirements.txt
RUN pip install protobuf==3.20.*

# Build do workspace
RUN rm -rf build/ install/ log/ && \
    . /opt/ros/humble/setup.sh && colcon build

# Variável de ambiente do projeto
ENV ARARA_VICE_PATH=/root/ssl-VICE

# Source automático do ROS2 + ambiente
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

CMD ["bash"]
