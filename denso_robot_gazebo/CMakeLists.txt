cmake_minimum_required(VERSION 2.8.3)
project(denso_robot_gazebo)

find_package(catkin REQUIRED)

catkin_package()

add_subdirectory(models)

#############
## Install ##
#############

# other files
foreach(dir launch worlds)
   install(DIRECTORY ${dir}
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
endforeach()

install(PROGRAMS
  script/spawn_objects
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)