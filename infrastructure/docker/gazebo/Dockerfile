# Base image with ROS 2 Iron
FROM ros:iron-ros-base
# version: iron

# Set environment variables
ENV GAZEBO_VERSION=11.0 \
    CMAKE_VERSION=3.25 \
    WORKSPACE_DIR=/workspace \
    DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=iron \
    NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=all

# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Build tools
    build-essential \
    cmake=${CMAKE_VERSION}* \
    gcc-11 \
    g++-11 \
    git \
    # Gazebo and dependencies
    gazebo-dev=${GAZEBO_VERSION}* \
    libgazebo-dev=${GAZEBO_VERSION}* \
    libgazebo${GAZEBO_VERSION}-dev \
    # ROS 2 Gazebo packages
    ros-${ROS_DISTRO}-gazebo-ros \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-gazebo-plugins \
    # Additional dependencies
    libeigen3-dev \
    libopencv-dev \
    python3-pip \
    # Security packages
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# Create non-root user for security
RUN groupadd -r gazebo && \
    useradd -r -g gazebo -s /bin/bash gazebo && \
    mkdir -p ${WORKSPACE_DIR} && \
    chown -R gazebo:gazebo ${WORKSPACE_DIR}

# Set up workspace
WORKDIR ${WORKSPACE_DIR}

# Copy simulation assets
COPY --chown=gazebo:gazebo src/backend/src/gazebo/simulation/environment.world ${WORKSPACE_DIR}/worlds/
COPY --chown=gazebo:gazebo src/backend/src/gazebo/models/drone.sdf ${WORKSPACE_DIR}/models/
COPY --chown=gazebo:gazebo src/backend/src/gazebo/models/groundRobot.sdf ${WORKSPACE_DIR}/models/
COPY --chown=gazebo:gazebo src/backend/src/gazebo/plugins/robotController.cpp ${WORKSPACE_DIR}/plugins/

# Build Gazebo plugins with optimizations
RUN mkdir -p ${WORKSPACE_DIR}/build && \
    cd ${WORKSPACE_DIR}/build && \
    cmake \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_CXX_FLAGS="-O3 -march=native -flto" \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        ${WORKSPACE_DIR}/plugins && \
    make -j$(nproc) && \
    make install && \
    rm -rf ${WORKSPACE_DIR}/build

# Set up Gazebo environment
ENV GAZEBO_MODEL_PATH=${WORKSPACE_DIR}/models \
    GAZEBO_RESOURCE_PATH=${WORKSPACE_DIR} \
    GAZEBO_PLUGIN_PATH=/usr/local/lib/gazebo-${GAZEBO_VERSION}/plugins \
    GAZEBO_MODEL_DATABASE_URI="" \
    IGN_IP=127.0.0.1

# Configure resource limits
ENV GAZEBO_MAX_THREADS=$(nproc) \
    GAZEBO_MAX_CAMERA_RESOLUTION="1920x1080" \
    GAZEBO_PHYSICS_UPDATE_RATE="1000" \
    GAZEBO_SENSOR_UPDATE_RATE="100"

# Setup ROS 2 environment
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/gazebo/.bashrc

# Create required directories with proper permissions
RUN mkdir -p /var/log/gazebo && \
    chown -R gazebo:gazebo /var/log/gazebo && \
    chmod 755 /var/log/gazebo

# Switch to non-root user
USER gazebo

# Health check
HEALTHCHECK --interval=30s --timeout=10s --start-period=5s --retries=3 \
    CMD gz topic -l || exit 1

# Expose Gazebo ports
EXPOSE 11345/tcp

# Set entrypoint
ENTRYPOINT ["gzserver"]
CMD ["--verbose", "${WORKSPACE_DIR}/worlds/environment.world"]