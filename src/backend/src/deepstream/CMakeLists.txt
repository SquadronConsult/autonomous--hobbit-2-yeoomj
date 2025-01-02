# Minimum CMake version required for modern CUDA support
cmake_minimum_required(VERSION 3.15)

# Project definition with C++17 support
project(deepstream_backend CUDA CXX)

# Set C++ standard to 17 for modern features
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Set build type to Release for optimizations
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Set CUDA architecture for Jetson Orin (Ampere)
set(CMAKE_CUDA_ARCHITECTURES 87)

# Optimization flags for Release build
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native -mtune=native -ffast-math -flto -DNDEBUG")
set(CMAKE_CUDA_FLAGS_RELEASE "-O3 --use_fast_math --relocatable-device-code=true")

# Find required packages
find_package(CUDA 11.4 REQUIRED)
find_package(TensorRT 8.5 REQUIRED)
find_package(GStreamer 1.0 REQUIRED COMPONENTS gstreamer-1.0 gstreamer-app-1.0)

# DeepStream SDK path (adjust if needed)
set(DEEPSTREAM_SDK_ROOT "/opt/nvidia/deepstream/deepstream" CACHE PATH "DeepStream SDK root directory")

# Source files
set(SOURCES
    pipeline.cpp
    processors/objectDetection.cpp
    processors/videoAnalytics.cpp
    utils/tensorrt.cpp
)

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CUDA_INCLUDE_DIRS}
    ${TensorRT_INCLUDE_DIRS}
    ${GSTREAMER_INCLUDE_DIRS}
    ${DEEPSTREAM_SDK_ROOT}/sources/includes
)

# Create shared library
add_library(deepstream_backend SHARED ${SOURCES})

# Link dependencies
target_link_libraries(deepstream_backend
    PRIVATE
    ${CUDA_LIBRARIES}
    ${CUDA_CUDA_LIBRARY}
    ${CUDA_NVRTC_LIBRARY}
    ${TensorRT_LIBRARIES}
    ${GSTREAMER_LIBRARIES}
    # DeepStream libraries
    nvds_meta
    nvds_utils
    nvds_inferutils
    nvbufsurface
    nvbufsurftransform
    nvdsgst_meta
    nvds_batch_meta
)

# Set library properties
set_target_properties(deepstream_backend PROPERTIES
    CUDA_SEPARABLE_COMPILATION ON
    CUDA_RESOLVE_DEVICE_SYMBOLS ON
    POSITION_INDEPENDENT_CODE ON
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
)

# Installation rules
install(TARGETS deepstream_backend
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

# Install headers
install(FILES
    pipeline.hpp
    processors/objectDetection.hpp
    processors/videoAnalytics.hpp
    utils/tensorrt.hpp
    DESTINATION include/deepstream_backend
)

# Configure rpath for installed library
set_target_properties(deepstream_backend PROPERTIES
    INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib"
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH_USE_LINK_PATH TRUE
)

# Add compile definitions
target_compile_definitions(deepstream_backend
    PRIVATE
    MAX_BATCH_SIZE=8
    MAX_STREAMS=8
    CUDA_BATCH_MEMORY_SIZE=1073741824  # 1GB
    MAX_PROCESSING_LATENCY_MS=100
)

# Enable LTO for Release builds
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    include(CheckIPOSupported)
    check_ipo_supported(RESULT ipo_supported)
    if(ipo_supported)
        set_target_properties(deepstream_backend PROPERTIES
            INTERPROCEDURAL_OPTIMIZATION TRUE
        )
    endif()
endif()

# Add version information
set_target_properties(deepstream_backend PROPERTIES
    VERSION 1.0.0
    SOVERSION 1
)

# Generate and install pkg-config file
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/deepstream_backend.pc.in
    ${CMAKE_CURRENT_BINARY_DIR}/deepstream_backend.pc
    @ONLY
)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/deepstream_backend.pc
    DESTINATION lib/pkgconfig
)