FROM ros:humble

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-opencv \
        python3-pip \
        ros-humble-rviz2 \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir torch==2.4.1 ros2_numpy==0.0.5

WORKDIR /ws
COPY src src
RUN . /opt/ros/humble/setup.sh && colcon build
