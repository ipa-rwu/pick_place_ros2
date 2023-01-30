cmake_minimum_required(VERSION 3.8)
project(pick_place_app)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosparam_shortcuts REQUIRED)
find_package(pick_place_msgs REQUIRED)
find_package(ur_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# System dependencies are found with CMake's conventions
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

set(PROJECT_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/include/pick_place_app)

include_directories(include)

add_subdirectory(src/skills)
add_subdirectory(src/app)

##########
# Build pick place with object pose in param ##
##########

set(PICK_PLACE_TASK_DEPS
	ament_cmake
	rclcpp
	rosparam_shortcuts
	moveit_core
	moveit_ros_planning_interface
	moveit_ros_planning
)

set(pick_place_task pick_place_task_demo)
add_executable(${pick_place_task} src/main_pick_place_task.cpp)
target_compile_options(${pick_place_task} PUBLIC -g -O0)
target_link_libraries(${pick_place_task}
  ${PROJECT_NAME}_app
)

target_include_directories(${pick_place_task}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS
  ${pick_place_task}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(
  ${PROJECT_NAME}_app
  ${PROJECT_NAME}_skills
)

ament_export_dependencies(
  ${APP_DEPS}
  ${SKILL_DEPS})

ament_package()
