FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    locales curl software-properties-common wget gnupg2 lsb-release \
    git cmake build-essential python3-pip python3-opencv \
    rapidjson-dev \
    && locale-gen en_US en_US.UTF-8
ENV LANG=en_US.UTF-8

RUN add-apt-repository universe \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN curl -sSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable noble main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

RUN apt-get update && apt-get install -y \
    ros-jazzy-desktop \
    ros-jazzy-ros-gz \
    ros-jazzy-cv-bridge \
    ros-jazzy-vision-msgs \
    gz-harmonic \
    libgz-sim8-dev \
    libgz-cmake3-dev \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-libav \
    gstreamer1.0-gl \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt
RUN git clone https://github.com/ArduPilot/ardupilot_gazebo.git \
    && cd ardupilot_gazebo \
    && mkdir build \
    && cd build \
    && export GZ_VERSION=harmonic \
    && cmake .. -DCMAKE_BUILD_TYPE=Release \
    && make -j$(nproc) \
    && make install

RUN pip3 install --break-system-packages --no-cache-dir \
        pymavlink torch torchvision --extra-index-url https://download.pytorch.org/whl/cpu

RUN echo "numpy==1.26.4" > /etc/pip-constraints.txt
ENV PIP_CONSTRAINT=/etc/pip-constraints.txt

RUN pip3 install --break-system-packages --no-cache-dir --ignore-installed "pillow>=10.3.0" \
    && pip3 install --break-system-packages --no-cache-dir \
        gitpython pandas seaborn tqdm scipy "requests>=2.32.2" packaging \
        cloudpickle polars nvidia-ml-py ultralytics-thop ultralytics-platform \
    && pip3 install --break-system-packages --no-cache-dir --no-deps ultralytics

ENV YOLOv5_AUTOINSTALL=false

ENV GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ardupilot_gazebo/build:/usr/local/lib
ENV GZ_SIM_RESOURCE_PATH=/opt/ardupilot_gazebo/models:/opt/ardupilot_gazebo/worlds

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

WORKDIR /workspace
CMD ["/bin/bash"]