cmake_minimum_required(VERSION 3.8)
project(composition_analyzer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(portrait_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(OpenCV REQUIRED)

add_library(composition_rules
  src/composition_rules.cpp
)

target_include_directories(composition_rules PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(composition_rules PUBLIC cxx_std_17)

add_executable(composition_analyzer_node
  src/composition_analyzer_node.cpp
)

target_include_directories(composition_analyzer_node PRIVATE
  include
)

ament_target_dependencies(composition_analyzer_node
  rclcpp
  std_msgs
  portrait_interfaces
)

target_link_libraries(composition_analyzer_node
  composition_rules
)

install(TARGETS
  composition_rules
  composition_analyzer_node
  EXPORT export_composition_rules
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

ament_export_targets(export_composition_rules HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

ament_package()
