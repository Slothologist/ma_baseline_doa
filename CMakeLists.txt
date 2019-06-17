cmake_minimum_required(VERSION 2.8.3)
project(esiaf_doa)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
        rospy
        esiaf_ros
        moveit_ros_planning_interface
        )



## Set compiler flags
set (CMAKE_CXX_FLAGS "-O2")

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
        #INCLUDE_DIRS
        #LIBRARIES ${PROJECT_NAME}
)

###########
## Build ##
###########


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
       # ${catkin_INCLUDE_DIRS}
       # ${Boost_INCLUDE_DIRS}
       # ${PYTHON_INCLUDE_DIRS}
)




#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

CATKIN_INSTALL_PYTHON(PROGRAMS ${CMAKE_CURRENT_SOURCE_DIR}/scripts/start_doa.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
INSTALL(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/launch" DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
INSTALL(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/config" DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
