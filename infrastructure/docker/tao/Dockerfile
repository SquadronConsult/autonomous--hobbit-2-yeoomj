# Stage 1: Base image with CUDA and system dependencies
FROM nvidia/cuda:11.4.2-cudnn8-devel-ubuntu20.04 as base
LABEL maintainer="Agricultural Management System Team" \
      version="1.0.0" \
      description="NVIDIA TAO Toolkit container for agricultural pest detection"

# Set noninteractive installation
ENV DEBIAN_FRONTEND=noninteractive

# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3.8 \
    python3.8-dev \
    python3-pip \
    python3-setuptools \
    wget \
    curl \
    git \
    libgl1-mesa-glx \
    libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

# Create non-root user and group
RUN groupadd -g 10000 tao-group && \
    useradd -u 10000 -g tao-group -s /bin/bash -m tao-user

# Stage 2: Builder stage for installing Python packages
FROM base as builder

# Set working directory
WORKDIR /tmp/build

# Install Python dependencies with version pinning
COPY requirements.txt .
RUN python3 -m pip install --no-cache-dir -U pip setuptools wheel && \
    python3 -m pip install --no-cache-dir -r requirements.txt && \
    python3 -m pip install --no-cache-dir \
    torch==1.13.0 \
    tensorrt==8.5.0 \
    numpy==1.23.5 \
    pycuda==2022.1 \
    nvidia-pyindex \
    nvidia-tao-toolkit==4.0.0

# Stage 3: Final runtime image
FROM base as final

# Copy installed packages from builder
COPY --from=builder /usr/local/lib/python3.8/dist-packages /usr/local/lib/python3.8/dist-packages

# Set working directory
WORKDIR /app

# Create necessary directories with proper permissions
RUN mkdir -p /app/models /app/src/backend/src /app/config && \
    chown -R tao-user:tao-group /app && \
    chmod 755 /app && \
    chmod 750 /app/models

# Copy application code and configurations
COPY --chown=tao-user:tao-group src/backend/src/tao /app/src/backend/src/tao
COPY --chown=tao-user:tao-group src/backend/src/tao/models/pestDetection/config.yaml /app/config/pest_detection_config.yaml
COPY --chown=tao-user:tao-group src/backend/src/tao/inference/inferenceEngine.py /app/src/backend/src/tao/inference/inferenceEngine.py

# Set environment variables
ENV NVIDIA_DRIVER_CAPABILITIES=compute,utility,video \
    NVIDIA_VISIBLE_DEVICES=all \
    PYTHONPATH=/app/src/backend/src \
    MODEL_STORE_PATH=/app/models \
    TRT_OPTIMIZATION_LEVEL=3 \
    CUDA_CACHE_PATH=/tmp/cuda-cache \
    PYTHONUNBUFFERED=1

# Set resource limits
ENV NVIDIA_DRIVER_CAPABILITIES=compute,utility \
    NVIDIA_VISIBLE_DEVICES=all \
    CUDA_DEVICE_MAX_CONNECTIONS=8

# Switch to non-root user
USER tao-user

# Set security options
RUN mkdir -p /tmp/cuda-cache && \
    chmod 700 /tmp/cuda-cache

# Health check
HEALTHCHECK --interval=30s --timeout=10s --start-period=60s --retries=3 \
    CMD python3 -c 'import torch; assert torch.cuda.is_available()' && \
        python3 -c 'import tensorrt; assert tensorrt.Builder(tensorrt.Logger()).platform_has_fast_fp16'

# Configure logging
LABEL org.opencontainers.image.logging.driver=json-file \
      org.opencontainers.image.logging.options.max-size=100m \
      org.opencontainers.image.logging.options.max-file=5

# Set entrypoint and default command
ENTRYPOINT ["python3"]
CMD ["-m", "tao.inference.inferenceEngine"]

# Security configurations
LABEL org.opencontainers.image.security.capabilities=CAP_NET_BIND_SERVICE \
      org.opencontainers.image.security.seccomp=unconfined

# Resource limits
LABEL nvidia.resource.gpu.memory=8G \
      nvidia.resource.gpu.compute=4 \
      nvidia.resource.shared-memory=1G \
      nvidia.resource.fdlimit=4096

# Documentation labels
LABEL org.opencontainers.image.title="TAO Toolkit for Agricultural Pest Detection" \
      org.opencontainers.image.description="Optimized container for agricultural pest detection using NVIDIA TAO Toolkit" \
      org.opencontainers.image.version="1.0.0" \
      org.opencontainers.image.vendor="Agricultural Management System" \
      org.opencontainers.image.licenses="Proprietary"