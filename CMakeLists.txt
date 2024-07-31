cmake_minimum_required(VERSION 3.22.1)
project(collision)

# By default we build for the native cuda architecture. Customize by passing
# '-DCMAKE_CUDA_ARCHITECTURES=89;75;72' to cmake.
if (DEFINED CMAKE_CUDA_ARCHITECTURES)
  set(CMAKE_CUDA_ARCHITECTURES_SET_EXTERNALLY TRUE)
else()
  set(CMAKE_CUDA_ARCHITECTURES_SET_EXTERNALLY FALSE)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(std_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Find CUDA
find_package(CUDAToolkit REQUIRED)

# Find nvblox
set(NVBLOX_DIR "${CMAKE_CURRENT_SOURCE_DIR}/nvblox/nvblox")  # Path to the nvblox directory
list(APPEND CMAKE_MODULE_PATH "${NVBLOX_DIR}/cmake")
find_package(nvblox REQUIRED PATHS ${NVBLOX_DIR})

# Include nvblox headers
include_directories(
  ${CUDA_INCLUDE_DIRS}  
  ${nvblox_INCLUDE_DIRS}
)

add_executable(talker src/main.cpp)

# Link against nvblox
target_link_libraries(talker 
  nvblox::nvblox_lib
  CUDA::cudart
)

ament_target_dependencies(talker rclcpp std_msgs nav_msgs sensor_msgs)

install(TARGETS
  talker
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
