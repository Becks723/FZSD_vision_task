cmake_minimum_required(VERSION 3.8)
project(armor_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(fmt REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(armor_interface REQUIRED)

include_directories(include)

# 声明armor_detector可执行文件
add_executable(armor_detector 
  src/armordetectornode.cpp 
  src/detector.cpp 
  src/pnpsolver.cpp
)

ament_target_dependencies(armor_detector
  rclcpp
  image_transport
  sensor_msgs
  cv_bridge
  tf2
  tf2_ros
  tf2_geometry_msgs
  armor_interface
)

target_include_directories(armor_detector PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(armor_detector ${OpenCV_LIBS} fmt::fmt)

# 部署可执行文件
install(
  TARGETS armor_detector
  DESTINATION lib/${PROJECT_NAME}
)

# 部署零散文件
install(
  FILES tiny_resnet.onnx
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
