cmake_minimum_required(VERSION 3.0.2)
project(lar_gazebo)

find_package(catkin REQUIRED COMPONENTS)

catkin_python_setup()
catkin_package()

catkin_install_python(PROGRAMS scripts/top_view_node.py scripts/visual_patrol.py scripts/evaluate_top_view.py scripts/report_top_view.py scripts/record_top_view.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Mark other files for installation (e.g. launch and bag files, etc.)
install(
  DIRECTORY launch maps models worlds config husky_urdf_extras
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch/top_view.launch)
  roslaunch_add_file_check(launch/hector_slam.launch)
  roslaunch_add_file_check(launch/lar_husky.launch)
  roslaunch_add_file_check(launch/lar_world.launch)
endif()
