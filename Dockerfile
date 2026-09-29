FROM ros:humble-ros-base-jammy AS dependencies
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake python3-numpy \
    ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

FROM dependencies AS build
WORKDIR /ws
COPY cpp /ws/cpp
COPY config /ws/config
COPY ros2 /ws/ros2
RUN source /opt/ros/humble/setup.bash && \
    colcon build --base-paths ros2 --merge-install \
      --cmake-args -DCMAKE_BUILD_TYPE=Release

FROM dependencies AS runtime
WORKDIR /ws
COPY --from=build /ws/install /ws/install
COPY tools /ws/tools
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "run", "metropilot_ros", "obstacle_detector"]
