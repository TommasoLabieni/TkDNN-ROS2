cmake_minimum_required(VERSION 3.5)
project(tkdnn_ros2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(OpenCV REQUIRED)
add_definitions(-DOPENCV)
add_definitions(-O4 -g)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(mmr_base REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)

# find_package(message_runtime REQUIRED)

include_directories(
                ${OpenCV_INCLUDE_DIRS}
                tkDNN/include
                include
)

add_subdirectory(tkDNN)

include_directories(/usr/local/cuda/include)
link_directories(/usr/local/cuda/lib64)

add_executable(camera_node src/camera_node.cpp)
add_library(camera_detection src/camera_detection.cpp)

ament_target_dependencies(camera_detection
  rclcpp
  sensor_msgs
  mmr_base
  cv_bridge
  message_filters
)

# target tkDNN library
target_link_libraries(camera_detection
  tkDNN
)

ament_target_dependencies(camera_node
  rclcpp
  mmr_base
)

target_link_libraries(camera_node camera_detection)

# Install target
install(TARGETS
  camera_node
  camera_detection
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch directory
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config directory
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
