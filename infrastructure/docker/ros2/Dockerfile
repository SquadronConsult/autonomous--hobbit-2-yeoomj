# Stage 1: Base ROS 2 JAZZY with real-time kernel
FROM ros:jazzy-ros-base AS base
# Version: ros:jazzy-ros-base - latest
# Purpose: Provides ROS 2 JAZZY base installation with real-time kernel

# Set environment variables for ROS 2
ENV ROS_DISTRO=jazzy \
    WORKSPACE_DIR=/opt/ros2_ws \
    NODE_PATH=/opt/ros2_ws/src/nodes \
    NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=compute,utility \
    DDS_CONFIG_PATH=/opt/ros2_ws/config/dds \
    FASTRTPS_DEFAULT_PROFILES_FILE=/opt/ros2_ws/config/dds/fastdds_profile.xml

# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    python3-pip \
    python3-colcon-common-extensions \
    libasio-dev \
    libtinyxml2-dev \
    libcunit1-dev \
    && rm -rf /var/lib/apt/lists/*

# Stage 2: Build environment with CUDA support
FROM nvidia/cuda:11.8.0-devel-ubuntu22.04 AS builder
# Version: nvidia/cuda:11.8.0-devel-ubuntu22.04
# Purpose: Compiles ROS 2 nodes with GPU support and optimization flags

# Copy ROS 2 environment setup from base
COPY --from=base /opt/ros /opt/ros

# Install build dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    python3-pip \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Set up workspace
WORKDIR ${WORKSPACE_DIR}
RUN mkdir -p src/nodes src/launch config/dds

# Copy source files
COPY src/backend/src/ros2/nodes/ src/nodes/
COPY src/backend/src/ros2/launch/ src/launch/
COPY src/backend/src/ros2/msgs/ src/msgs/

# Install FastDDS v2.10.1
RUN git clone --branch v2.10.1 --depth 1 https://github.com/eProsima/Fast-DDS.git \
    && cd Fast-DDS \
    && mkdir build && cd build \
    && cmake -DCMAKE_BUILD_TYPE=Release .. \
    && make -j$(nproc) \
    && make install

# Configure DDS for low latency
COPY config/dds/fastdds_profile.xml ${DDS_CONFIG_PATH}/

# Build workspace with optimization flags
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build \
        --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_CXX_FLAGS="-O3 -march=native -flto" \
        -DCMAKE_CUDA_FLAGS="-O3 -use_fast_math"

# Stage 3: Final runtime image
FROM nvidia/cuda:11.8.0-base-ubuntu22.04
# Version: nvidia/cuda:11.8.0-base-ubuntu22.04
# Purpose: Optimized production runtime with minimal footprint

# Copy ROS 2 and built workspace
COPY --from=base /opt/ros /opt/ros
COPY --from=builder ${WORKSPACE_DIR} ${WORKSPACE_DIR}

# Install runtime dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-minimal \
    libpython3.10 \
    libtinyxml2-dev \
    && rm -rf /var/lib/apt/lists/*

# Set up environment
ENV LD_LIBRARY_PATH=/usr/local/cuda/lib64:${LD_LIBRARY_PATH}
ENV PATH=/usr/local/cuda/bin:${PATH}

# Create non-root user for security
RUN useradd -r -s /bin/bash ros \
    && mkdir -p /home/ros \
    && chown -R ros:ros /home/ros ${WORKSPACE_DIR}

# Set up entrypoint
COPY ros2_entrypoint.sh /
RUN chmod +x /ros2_entrypoint.sh

# Configure resource limits
ENV NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=compute,utility \
    RCUTILS_LOGGING_USE_STDOUT=1 \
    RCUTILS_LOGGING_BUFFERED_STREAM=1 \
    RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{time}] [{name}]: {message}"

# Set up health check
HEALTHCHECK --interval=30s --timeout=30s --start-period=5s --retries=3 \
    CMD ros2 node list || exit 1

# Switch to non-root user
USER ros
WORKDIR ${WORKSPACE_DIR}

# Source ROS 2 environment
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc \
    && echo "source ${WORKSPACE_DIR}/install/setup.bash" >> ~/.bashrc

ENTRYPOINT ["/ros2_entrypoint.sh"]
CMD ["ros2", "launch", "agricultural_robotics", "system.launch.py"]