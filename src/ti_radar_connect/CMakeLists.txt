cmake_minimum_required(VERSION 3.8)
project(ti_radar_connect)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

#add subdirectories
add_subdirectory(include)

#following instructions from: following instructions from: https://docs.ros.org/en/rolling/How-To-Guides/Ament-CMake-Documentation.html
#declare TIRadarCOnnectNode library
add_library(TIRadarConnectNode src/TIRadarConnectNode.cpp)
target_link_libraries(TIRadarConnectNode PUBLIC Runner Utilities nlohmann_json::nlohmann_json)
ament_target_dependencies(TIRadarConnectNode PUBLIC rclcpp sensor_msgs std_msgs)
target_include_directories(TIRadarConnectNode
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
install(
  DIRECTORY include/ti_radar_connect #look to potentially remove this
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS TIRadarConnectNode
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp sensor_msgs std_msgs nlohmann_json Utilities CLIController SerialStreamer DCA1000Handler DCA1000Runner Runner TLVProcessing DCA1000Commands)
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(TIRadarConnectNode)


#declare the executable
add_executable(ti_radar_connect src/ti_radar_connect.cpp)
target_link_libraries(ti_radar_connect PUBLIC TIRadarConnectNode)
ament_target_dependencies(ti_radar_connect PUBLIC rclcpp sensor_msgs)
install(TARGETS
  ti_radar_connect
  DESTINATION lib/${PROJECT_NAME})

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

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
ament_package()
