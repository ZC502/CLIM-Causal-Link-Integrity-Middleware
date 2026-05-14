FROM osrf/ros:humble-desktop

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y \
    sudo \
    git \
    curl \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-humble-diagnostic-msgs \
    ros-humble-tf2-ros \
    && rm -rf /var/lib/apt/lists/*

RUN if ! id -u vscode >/dev/null 2>&1; then \
      useradd -m -s /bin/bash vscode && \
      echo "vscode ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/vscode; \
    fi

RUN rosdep init 2>/dev/null || true
RUN rosdep update || true

USER vscode

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
