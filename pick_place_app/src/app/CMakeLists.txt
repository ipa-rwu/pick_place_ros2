add_library(${PROJECT_NAME}_app SHARED
	${PROJECT_INCLUDE}/app/pick_place_task.h
	${PROJECT_INCLUDE}/app/point_to_point.h
	${PROJECT_INCLUDE}/app/pick_place_static_task.h


	pick_place_task.cpp
	point_to_point.cpp
	pick_place_static_task.cpp
)

set(APP_DEPS
	ament_cmake
	rclcpp
	rclcpp_components
	rosparam_shortcuts
	moveit_core
	moveit_ros_planning_interface
	moveit_ros_planning
)

ament_target_dependencies(${PROJECT_NAME}_app
	${APP_DEPS}
)

target_link_libraries(${PROJECT_NAME}_app
  ${PROJECT_NAME}_skills
)

install(TARGETS ${PROJECT_NAME}_app
        EXPORT ${PROJECT_NAME}APP
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib)
