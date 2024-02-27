FROM osrf/ros:noetic-desktop-full

# Update and install build tools and dependencies
RUN apt-get update && apt-get install -y \
    git \
    build-essential \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    python3-catkin-tools \
    libgtest-dev \
    ros-noetic-ackermann-msgs \
    ros-noetic-geometry2 \
    ros-noetic-hector-gazebo \
    ros-noetic-hector-models \
    ros-noetic-jsk-rviz-plugins \
    ros-noetic-ros-control \
    ros-noetic-ros-controllers \
    ros-noetic-velodyne-simulator \
    && rm -rf /var/lib/apt/lists/*

# Clone the required ROS packages into the workspace src directory
RUN git clone https://github.com/karsafar/catkin_ws.git


# Initialize rosdep and update
RUN if [ ! -e /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi && rosdep update
# Skip any packages that cannot be found
#RUN apt-get update && \
#    rosdep install --from-paths . --ignore-src -r -y || echo "Skipping packages that could not be installed."
# Build the catkin workspace

# Copy the Dockerfile into the image
COPY Dockerfile /catkin_ws/Dockerfile

WORKDIR /catkin_ws
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash && catkin build'
# Source the workspace setup file on each new shell
RUN echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc
# Set the default command to bash
CMD ["bash"]
