cmake_minimum_required(VERSION 3.8)
project(vision_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(camera_node src/camera_node.cpp)
ament_target_dependencies(camera_node
	rclcpp
  sensor_msgs
  cv_bridge
)

add_executable(camera_subscriber src/camera_subscriber.cpp)
ament_target_dependencies(camera_subscriber
	rclcpp
  sensor_msgs
  cv_bridge
)

target_include_directories(camera_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_compile_features(camera_node PRIVATE cxx_std_17)
target_link_libraries(camera_node ${OpenCV_LIBS})

target_include_directories(camera_subscriber PRIVATE ${OpenCV_INCLUDE_DIRS})
target_compile_features(camera_subscriber PRIVATE cxx_std_17)
target_link_libraries(camera_subscriber ${OpenCV_LIBS})

install(TARGETS
	camera_node
	DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
	camera_subscriber
	DESTINATION lib/${PROJECT_NAME}
)

ament_package()
