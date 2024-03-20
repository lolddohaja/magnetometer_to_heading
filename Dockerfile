FROM ros:iron-ros-base

ARG NETRC

SHELL ["bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-dev-tools \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root/ros2_ws

COPY magnetometer_to_heading src/magnetometer_to_heading

RUN rosdep update --rosdistro $ROS_DISTRO

RUN apt update && rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y

RUN . /opt/ros/$ROS_DISTRO/setup.sh \
  && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release


RUN sed -i '$isource "/root/ros2_ws/install/setup.bash"' /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]