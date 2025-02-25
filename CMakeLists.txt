cmake_minimum_required(VERSION 3.21)
cmake_policy(SET CMP0048 NEW)

project(rope_rrt VERSION 1.0.0 LANGUAGES CXX CUDA)

# Catkin specific configuration
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  pcl_ros
  pcl_conversions
)

# Generate catkin package configuration
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs
  DEPENDS EIGEN3 CUDA OMPL
)

# Compiler options
add_compile_options(-Wall -Wextra -O3)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
endif()

# C++17 setup
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Find required packages
set(glog_INCLUDE_DIRS "/usr/include")  # Default include path for glog
set(glog_LIBRARIES "/usr/lib/x86_64-linux-gnu/libglog.a")  # Path to libglog.a
find_package(gflags REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(CUDAToolkit REQUIRED)
enable_language(CUDA)

# CUDA architecture
set(CMAKE_CUDA_ARCHITECTURES 86)

# OMPL configuration
set(OMPL_INCLUDE_DIRS "/usr/local/include/ompl-1.6")
set(OMPL_LIBRARIES "/usr/local/lib/libompl.so")

# nvblox configuration
# Manually specify nvblox paths
set(NVBLOX_DIR "/home/hakim/test_blox/nvblox/build")
set(NVBLOX_INCLUDE_DIRS "${NVBLOX_DIR}/include")
set(NVBLOX_LIBRARIES "${NVBLOX_DIR}/nvblox/libnvblox_lib.so")

# Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${glog_INCLUDE_DIRS}
  ${gflags_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  ${NVBLOX_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
)

# Add executable
add_executable(tether_planner_node
  src/tether_planner_main.cpp
  src/publishers.cpp
  src/helper_functions.cpp
  src/global_vars.cpp
  src/my_motion_validator.cpp
  src/tether_planner.cpp
  src/collision_checker.cpp
)

# Set properties for CUDA compilation
set_target_properties(tether_planner_node PROPERTIES
  CUDA_SEPARABLE_COMPILATION ON
)

# Link libraries
target_link_libraries(tether_planner_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OMPL_LIBRARIES}
  ${glog_LIBRARIES}
  ${gflags_LIBRARIES}
  CUDA::cudart
  ${NVBLOX_LIBRARIES}
  -lunwind  # Explicitly link libunwind

)

# Install targets
install(TARGETS tether_planner_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
