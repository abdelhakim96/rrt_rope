cmake_minimum_required(VERSION 2.8.3)

# Set policy CMP0048 to NEW to allow VERSION to be specified
cmake_policy(SET CMP0048 NEW)

# Set the project version
project(rope_rrt VERSION 1.0.0)

# Default C++14 setup
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Find catkin packages and other dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  roslib
  ompl

)

# Find Eigen3
find_package(ompl REQUIRED)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS message_runtime
)

# Include directories for project
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}

  ${PROJECT_SOURCE_DIR}/include
)

# Add the rope_rrt executable
add_executable(rope_rrt src/rope_rrt.cpp)

# Link necessary libraries for rope_rrt
target_link_libraries(rope_rrt ${OMPL_LIBRARIES} ${catkin_LIBRARIES})

# Install targets
install(TARGETS
  rope_rrt
  DESTINATION lib/${PROJECT_NAME}
)
