add_library(${PROJECT_NAME}_skills SHARED
	${PROJECT_INCLUDE}/skills/utils.h
	${PROJECT_INCLUDE}/skills/execute_trajectory_skill.h
	${PROJECT_INCLUDE}/skills/io_gripper_with_ur_skill.h
	${PROJECT_INCLUDE}/skills/modify_planning_scene_skill.h
	${PROJECT_INCLUDE}/skills/detect_aruco_marker_skill.h
	${PROJECT_INCLUDE}/skills/compute_path_with_moveitcpp_skill.h


	utils.cpp
	execute_trajectory_skill.cpp
	io_gripper_with_ur_skill.cpp
	modify_planning_scene_skill.cpp
	detect_aruco_marker_skill.cpp
	compute_path_with_moveitcpp_skill.cpp
)

set(SKILL_DEPS
	ament_cmake
	rclcpp
	pluginlib
	rclcpp_components
	rosparam_shortcuts
	moveit_core
	moveit_ros_planning_interface
	moveit_ros_planning
	ur_msgs
	aruco_msgs
	yaml_cpp_vendor
)

ament_target_dependencies(${PROJECT_NAME}_skills
	${SKILL_DEPS}
)

install(TARGETS ${PROJECT_NAME}_skills
        EXPORT ${PROJECT_NAME}SKILLS
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib)
