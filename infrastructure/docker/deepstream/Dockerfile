# Stage 1: Base image with NVIDIA DeepStream SDK
FROM nvcr.io/nvidia/deepstream-l4t:6.2 AS base

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive \
    PATH=/opt/nvidia/deepstream/bin:$PATH \
    NVIDIA_DRIVER_CAPABILITIES=compute,utility,video \
    NVIDIA_VISIBLE_DEVICES=all \
    MAX_BATCH_SIZE=8 \
    MAX_STREAMS=8 \
    CUDA_BATCH_MEMORY_SIZE=1073741824

# Stage 2: Builder stage for compiling DeepStream backend components
FROM base AS builder

# Install build dependencies with version pinning
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential=12.9 \
    cmake=3.18.4-2 \
    cuda-toolkit-11-4=11.4.0-1 \
    libgstreamer1.0-dev=1.16.3-0 \
    libgstreamer-plugins-base1.0-dev=1.16.3-0 \
    && rm -rf /var/lib/apt/lists/*

# Copy source files and CMake configuration
COPY src/backend/src/deepstream /opt/nvidia/deepstream/sources/
WORKDIR /opt/nvidia/deepstream/sources

# Configure CMake with optimization flags
RUN cmake -B build \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CUDA_ARCHITECTURES=87 \
    -DCMAKE_CXX_FLAGS="-O3 -march=native -mtune=native -ffast-math -flto -DNDEBUG" \
    -DCMAKE_CUDA_FLAGS="-O3 --use_fast_math --relocatable-device-code=true"

# Build with parallel compilation
RUN cmake --build build -j$(nproc) --target install

# Stage 3: Final runtime image
FROM base AS final

# Create non-root user for security
RUN groupadd -r deepstream -g 1000 && \
    useradd -r -g deepstream -u 1000 -s /sbin/nologin deepstream && \
    mkdir -p /opt/nvidia/deepstream/{models,config,output} && \
    chown -R deepstream:deepstream /opt/nvidia/deepstream

# Copy compiled artifacts from builder
COPY --from=builder /usr/local/lib/libdeepstream_backend.so* /usr/local/lib/
COPY --from=builder /usr/local/include/deepstream_backend /usr/local/include/deepstream_backend

# Install runtime dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    libgstreamer1.0-0=1.16.3-0 \
    libgstreamer-plugins-base1.0-0=1.16.3-0 \
    && rm -rf /var/lib/apt/lists/* \
    && ldconfig

# Copy configuration files
COPY src/backend/src/config/deepstream.ts /opt/nvidia/deepstream/config/
COPY src/backend/src/config/tao.ts /opt/nvidia/deepstream/config/

# Set up secure volumes
VOLUME ["/opt/nvidia/deepstream/models", "/opt/nvidia/deepstream/config", "/opt/nvidia/deepstream/output"]

# Configure resource limits
ENV NVIDIA_DEEPSTREAM_MEMORY_POOL_SIZE=32 \
    NVIDIA_DEEPSTREAM_MAX_PROCESSING_LATENCY_MS=100 \
    GST_DEBUG=3

# Set up health check
HEALTHCHECK --interval=30s --timeout=10s --start-period=30s --retries=3 \
    CMD pgrep -f "deepstream-app" || exit 1

# Switch to non-root user
USER deepstream

# Expose ports for metrics and RTSP
EXPOSE 8000/tcp
EXPOSE 8554/tcp

# Set default command
CMD ["deepstream-app", \
     "--config-file", "/opt/nvidia/deepstream/config/pipeline_config.txt", \
     "--init-config", "/opt/nvidia/deepstream/config/initialization_config.txt"]

# Apply security labels
LABEL vendor="NVIDIA" \
      com.nvidia.deepstream.version="6.2" \
      maintainer="Agricultural System Team" \
      description="DeepStream container for agricultural monitoring" \
      security.capabilities="cap_net_bind_service,cap_sys_ptrace" \
      seccomp="default.json"