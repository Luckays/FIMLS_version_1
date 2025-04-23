# ============================
# Stage 1: Build C++ tools
# ============================
FROM python:3.10-slim AS builder

# Set environment and timezone (no prompts during install)
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

# Install build dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    wget \
    libpcl-dev \
    libboost-all-dev \
    libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

# Copy project
COPY . /app
WORKDIR /app/scripts/data_processing

# Build C++ tools
RUN cmake -S . -B build && \
    cmake --build build -j$(nproc) && \
    mkdir -p /app/bin && \
    cp build/clear_and_filter /app/bin/ && \
    cp build/tile_pointcloud /app/bin/

# ============================
# Stage 2: Runtime container
# ============================
FROM python:3.10-slim

# Set environment and timezone
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC
ENV PYTHONUNBUFFERED=1

# Install runtime dependencies
RUN apt-get update && apt-get install -y \
    libpcl-dev \
    libboost-all-dev \
    libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
COPY requirements.txt /tmp/
RUN pip install --no-cache-dir --upgrade pip && \
    pip install -r /tmp/requirements.txt

# Set working directory
WORKDIR /app

# Copy everything from builder stage
COPY --from=builder /app /app

# Default command
CMD ["python", "scripts/data_processing/pipeline.py", "--config", "config/parameters_processing.yaml"]