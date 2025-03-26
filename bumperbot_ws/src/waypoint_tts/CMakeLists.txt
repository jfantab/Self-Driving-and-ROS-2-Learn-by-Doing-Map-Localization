cmake_minimum_required(VERSION 3.8)
project(waypoint_tts)

# Default to C++14
set(CMAKE_CXX_STANDARD 14)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(lifecycle_msgs REQUIRED)

include_directories(include)

set(library_name ${PROJECT_NAME}_plugin)

set(dependencies
 rclcpp
 rclcpp_lifecycle
 visualization_msgs
 nav2_util
 nav2_msgs
 nav_msgs
 geometry_msgs
 nav2_core
 pluginlib
)

add_library(${library_name} SHARED src/plugins/tts_at_waypoint.cpp)

target_compile_definitions(${library_name} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

# Ensure correct plugin registration
pluginlib_export_plugin_description_file(waypoint_tts plugins.xml)
ament_target_dependencies(${library_name} ${dependencies})

# install(TARGETS ${library_name}
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION lib
# )

install(TARGETS ${library_name}
  DESTINATION lib
)

install(DIRECTORY include/ DESTINATION include/)

install(FILES plugins.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_package()
