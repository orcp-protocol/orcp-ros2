# Dev / runtime image for the ORCP ROS 2 driver.
#
# Base: official ROS 2 Jazzy (Ubuntu 24.04), which already includes rclpy,
# the standard message packages, colcon, and rosdep. We add the keyboard
# teleop app (used to prove the stack), pip, and the runtime dependencies of
# the ORCP Python library + simulator — so the editable installs done at
# container start need no network.
FROM ros:jazzy-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-teleop-twist-keyboard \
        ros-jazzy-foxglove-bridge \
        ros-jazzy-robot-state-publisher \
        python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Ubuntu 24.04 marks its system Python as "externally managed" (PEP 668);
# inside a throwaway container it is safe to install into it directly.
RUN pip install --no-cache-dir --break-system-packages \
        "pyserial>=3.5" "websockets>=11"

# The ORCP client is pinned to a FLOOR, not merely "latest": 0.1.x returned
# partial lines from the serial layer, which corrupts command responses on a
# node that streams continuously. See src/orcp_ros2/setup.py.
RUN pip install --no-cache-dir --break-system-packages "orcp>=0.2.0"

# Source ROS 2 in every interactive shell.
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc

WORKDIR /work/orcp-ros2
