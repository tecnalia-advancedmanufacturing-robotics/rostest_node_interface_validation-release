cmake_minimum_required(VERSION 2.8.3)
project(rostest_node_interface_validation)

find_package(catkin REQUIRED COMPONENTS
    rospy
    rospy_message_converter
)

catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    CATKIN_DEPENDS
        rospy
        rospy_message_converter
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

catkin_install_python(PROGRAMS
   nodes/test_service
   nodes/test_filter
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############
if (CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)
  find_package(catkin REQUIRED COMPONENTS rostest std_srvs)

  roslint_python()
  add_rostest(test/service_call.test)
  add_rostest(test/msg_filter.test)
endif()
