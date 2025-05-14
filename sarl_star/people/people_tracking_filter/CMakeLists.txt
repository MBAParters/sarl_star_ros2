cmake_minimum_required(VERSION 3.8)
project(people_tracking_filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(people_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(BFL REQUIRED orocos-bfl)

include_directories(${BFL_INCLUDE_DIRS} include)
link_directories(${BFL_LIBRARY_DIRS})

add_library(${PROJECT_NAME} SHARED
            src/uniform_vector.cpp
            src/gaussian_vector.cpp
            src/gaussian_pos_vel.cpp
            src/mcpdf_pos_vel.cpp
            src/mcpdf_vector.cpp
            src/sysmodel_pos_vel.cpp
            src/sysmodel_vector.cpp
            src/measmodel_pos.cpp
            src/measmodel_vector.cpp
	          src/tracker_particle.cpp
	          src/tracker_kalman.cpp
	          src/detector_particle.cpp
)
ament_target_dependencies(${PROJECT_NAME} PUBLIC rclcpp geometry_msgs message_filters people_msgs sensor_msgs std_msgs tf2 tf2_geometry_msgs )
target_link_libraries(${PROJECT_NAME} PUBLIC ${BFL_LIBRARIES}
)

ament_export_include_directories(include)
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)


install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
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
