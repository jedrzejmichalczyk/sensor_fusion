cmake_minimum_required(VERSION 3.15)
project(SensorFusion VERSION 1.0.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Options for cross-compilation
option(BUILD_TESTS "Build test programs" ON)
option(BUILD_EXAMPLES "Build example programs" ON)
option(BUILD_SHARED_LIBS "Build shared libraries" OFF)

# Compiler flags
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -O3)
    if(CMAKE_BUILD_TYPE MATCHES Debug)
        add_compile_options(-g -O0)
    endif()
endif()

# For embedded systems, optimize for size
if(EMBEDDED_BUILD)
    add_compile_options(-Os -ffunction-sections -fdata-sections)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--gc-sections")
endif()

# Include directories
include_directories(
    ${PROJECT_SOURCE_DIR}/include
)

# Matrix library - using custom lightweight implementation
# Eigen3 not required for this implementation
message(STATUS "Using built-in matrix operations (no external dependencies)")

# Source files
set(SENSOR_SOURCES
    src/sensors/imu.cpp
    src/sensors/gps.cpp
    src/sensors/barometer.cpp
    src/sensors/magnetometer.cpp
)

set(FUSION_SOURCES
    src/fusion/ekf.cpp
    src/fusion/state.cpp
    src/fusion/measurement.cpp
)

set(UTILS_SOURCES
    src/utils/math_utils.cpp
    src/utils/coordinate_transform.cpp
    src/utils/data_logger.cpp
)

# Library
add_library(sensor_fusion
    ${SENSOR_SOURCES}
    ${FUSION_SOURCES}
    ${UTILS_SOURCES}
)

target_include_directories(sensor_fusion PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Examples
if(BUILD_EXAMPLES)
    add_executable(simulation_example examples/simulation_example.cpp)
    target_link_libraries(simulation_example sensor_fusion)

    add_executable(imu_gps_fusion examples/imu_gps_fusion.cpp)
    target_link_libraries(imu_gps_fusion sensor_fusion)

    add_executable(ekf_debug examples/ekf_debug.cpp)
    target_link_libraries(ekf_debug sensor_fusion)

    add_executable(ekf_debug2 examples/ekf_debug2.cpp)
    target_link_libraries(ekf_debug2 sensor_fusion)
endif()

# Tests
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# Installation
install(TARGETS sensor_fusion
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)

# Export package (optional, commented out for minimal build)
# Uncomment if CMake 3.0+ package export is needed
# include(CMakePackageConfigHelper)
# write_basic_package_version_file(
#     "${CMAKE_CURRENT_BINARY_DIR}/SensorFusionConfigVersion.cmake"
#     VERSION ${PROJECT_VERSION}
#     COMPATIBILITY AnyNewerVersion
# )
#
# install(FILES
#     "${CMAKE_CURRENT_BINARY_DIR}/SensorFusionConfigVersion.cmake"
#     DESTINATION lib/cmake/SensorFusion
# )
