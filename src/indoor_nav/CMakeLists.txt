cmake_minimum_required(VERSION 3.0.2)
project(indoor_nav)
find_package(catkin REQUIRED COMPONENTS
  rospy geometry_msgs nav_msgs std_msgs sensor_msgs std_srvs mavros_msgs
  nlink_parser message_generation)
catkin_python_setup()
add_message_files(FILES LocalizationStatus.msg SafetyStatus.msg)
generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS rospy geometry_msgs nav_msgs std_msgs sensor_msgs
               std_srvs mavros_msgs nlink_parser message_runtime)
catkin_install_python(PROGRAMS
  scripts/world_estimator.py scripts/mock_uwb_node.py
  scripts/mission_manager.py scripts/safety_supervisor.py
  scripts/relative_mission_manager.py scripts/v0_nav_health.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config launch docs DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
