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
find_package(ur_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(aruco_msgs REQUIRED)

# System dependencies are found with CMake's conventions
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

set(PROJECT_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/include/pick_place_app)

include_directories(include)

add_subdirectory(src/skills)
add_subdirectory(src/app)

##########
# Build pick place with object detection ##
##########
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


##########
# Build point to  point ##
##########
set(point_to_point_task point_to_point_demo)
add_executable(${point_to_point_task} src/main_point_to_point.cpp)
target_compile_options(${point_to_point_task} PUBLIC -g -O0)
target_link_libraries(${point_to_point_task}
  ${PROJECT_NAME}_app
)

target_include_directories(${point_to_point_task}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

##########
# Build pick place with object pose in param ##
##########
set(pick_place_static_demo pick_place_static_demo)
add_executable(${pick_place_static_demo} src/main_pick_place_static.cpp)
target_compile_options(${pick_place_static_demo} PUBLIC -g -O0)
target_link_libraries(${pick_place_static_demo}
  ${PROJECT_NAME}_app
)

target_include_directories(${pick_place_static_demo}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)


##########
# Build test planning pipeline monitor demo ##
##########
set(test_psm_demo test_psm_demo)
add_executable(${test_psm_demo} src/main_test_psm.cpp)
target_compile_options(${test_psm_demo} PUBLIC -g -O0)
target_link_libraries(${test_psm_demo}
  ${PROJECT_NAME}_app
)

target_include_directories(${test_psm_demo}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)


install(TARGETS
  ${pick_place_task}
  ${point_to_point_task}
  ${pick_place_static_demo}
  ${test_psm_demo}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

# ##########
# # Mock IO Server ##
# ##########
add_executable(mock_io_server src/mock_io_server.cpp)
ament_target_dependencies(mock_io_server
rclcpp ur_msgs)
install(TARGETS
mock_io_server
  DESTINATION lib/${PROJECT_NAME})

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
