cmake_minimum_required(VERSION 3.0)
project(trajectory_generation)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
##find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
     std_msgs message_generation
     sensor_msgs
     geometry_msgs
     rqt_gui_cpp
     rqt_gui

  )
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

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
#  INCLUDE_DIRS include
#  LIBRARIES testpack
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
/opt/ros/kinetic/include
/home/cast/humanoid/surena4/devel/include

)
## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/testpack.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/testpack_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_testpack.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
#SET(CMAKE_AUTOUIC ON)
include_directories(include ${catkin_INCLUDE_DIRS})
find_package(Qt5 COMPONENTS Core Gui Widgets PrintSupport)

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})

include_directories(
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Gui_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
    ${Qt5PrintSupport_INCLUDE_DIRS}
    )

add_definitions( -std=c++11 -fPIC)
##############################################################################
# Sources
##############################################################################

set( QT_SOURCES
    src/MinimumJerkInterpolation.cpp
    src/taskspaceoffline.cpp
    src/LinkM.cpp
    src/Robot.cpp
   # src/TaskSpace.cpp
    src/pidcontroller.cpp
    src/qcgenerator.cpp
    src/right_hand.cpp
    src/left_hand.cpp
    src/taskspaceofflineRamp.cpp
    src/taskspaceonline2.cpp
    src/taskspaceonline3.cpp
    src/surenastateestimation.cpp
    src/taskspaceturning
#    src/hand_gui.cpp

    )
set ( UIS
# src/hand_gui.ui
)
add_executable(trajectory_generation src/trajectory_generation.cpp ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(trajectory_generation
    ${catkin_LIBRARIES}
    Qt5::Core
    Qt5::Widgets
    Qt5::PrintSupport
)
install(TARGETS trajectory_generation RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#add_executable(trajectory_generation_with_foot_corr src/trajectory_generation_with_foot_corr.cpp ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
#target_link_libraries(trajectory_generation_with_foot_corr
#    ${catkin_LIBRARIES}
#    Qt5::Core
#    Qt5::Widgets
#    Qt5::PrintSupport
#)
#install(TARGETS trajectory_generation_with_foot_corr RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#add_executable(ankle_controller src/ankle_controller.cpp ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
#target_link_libraries(ankle_controller
#    ${catkin_LIBRARIES}
#    Qt5::Core
#    Qt5::Widgets
#    Qt5::PrintSupport
#)
#install(TARGETS ankle_controller RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#add_executable(bumpsensor src/bumpsensor.cpp ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
#target_link_libraries(bumpsensor
#    ${catkin_LIBRARIES}
#    Qt5::Core
#    Qt5::Widgets
#    Qt5::PrintSupport
#)
#install(TARGETS bumpsensor RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#add_executable(trajectory_generationOnlineAdaptation src/trajectory_generationOnlineAdaptation.cpp ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
#target_link_libraries(trajectory_generationOnlineAdaptation
#    ${catkin_LIBRARIES}
#    Qt5::Core
#    Qt5::Widgets
#    Qt5::PrintSupport
#)
#install(TARGETS trajectory_generationOnlineAdaptation RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
add_executable(AnkleOrientation src/AnkleOrientation.cpp ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(AnkleOrientation
    ${catkin_LIBRARIES}
    Qt5::Core
    Qt5::Widgets
    Qt5::PrintSupport
)
install(TARGETS AnkleOrientation RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


add_executable(trajectory_generationOfflinePublisher src/trajectory_generationOfflinePublisher.cpp ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(trajectory_generationOfflinePublisher
    ${catkin_LIBRARIES}
    Qt5::Core
    Qt5::Widgets
    Qt5::PrintSupport
)
install(TARGETS trajectory_generationOfflinePublisher RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(trajectory_generationOnlineAdaptation2 src/trajectory_generationOnlineAdaptation2.cpp ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(trajectory_generationOnlineAdaptation2
    ${catkin_LIBRARIES}
    Qt5::Core
    Qt5::Widgets
    Qt5::PrintSupport
)
install(TARGETS trajectory_generationOnlineAdaptation2 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(trajectory_generationDynamicWalk src/trajectory_generationDynamicWalk.cpp ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(trajectory_generationDynamicWalk
    ${catkin_LIBRARIES}
    Qt5::Core
    Qt5::Widgets
    Qt5::PrintSupport
)
install(TARGETS trajectory_generationDynamicWalk RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(trajectory_generation_march_walk src/trajectory_generation_march_walk.cpp ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(trajectory_generation_march_walk
    ${catkin_LIBRARIES}
    Qt5::Core
    Qt5::Widgets
    Qt5::PrintSupport
)
install(TARGETS trajectory_generation_march_walk RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(hand_trajectory src/hand_trajectory.cpp ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(hand_trajectory
    ${catkin_LIBRARIES}
    Qt5::Core
    Qt5::Widgets
    Qt5::PrintSupport
)
install(TARGETS hand_trajectory RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(hand_test src/hand_test.cpp ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(hand_test
    ${catkin_LIBRARIES}
    Qt5::Core
    Qt5::Widgets
    Qt5::PrintSupport
)
install(TARGETS hand_test RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


#add_executable(hand_trajectory_shaking src/hand_trajectory_shaking.cpp ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
#target_link_libraries(hand_trajectory_shaking
#    ${catkin_LIBRARIES}
#    Qt5::Core
#    Qt5::Widgets
#    Qt5::PrintSupport
#)
#install(TARGETS hand_trajectory_shaking RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#add_executable(hand_trajectory_waving src/hand_trajectory_waving.cpp ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
#target_link_libraries(hand_trajectory_waving
#    ${catkin_LIBRARIES}
#    Qt5::Core
#    Qt5::Widgets
#    Qt5::PrintSupport
#)
#install(TARGETS hand_trajectory_waving RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(hands_interaction src/hands_interaction.cpp ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(hands_interaction
    ${catkin_LIBRARIES}
    Qt5::Core
    Qt5::Widgets
    Qt5::PrintSupport
)
install(TARGETS hands_interaction RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(hand_trajectory_pick_and_place src/hand_trajectory_pick_and_place.cpp ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(hand_trajectory_pick_and_place
    ${catkin_LIBRARIES}
    Qt5::Core
    Qt5::Widgets
    Qt5::PrintSupport
)
install(TARGETS hand_trajectory_pick_and_place RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(hand_trajectory_random src/hand_trajectory_random.cpp ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(hand_trajectory_random
    ${catkin_LIBRARIES}
    Qt5::Core
    Qt5::Widgets
    Qt5::PrintSupport
)
install(TARGETS hand_trajectory_random RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#add_executable(trajectory_generation_ramp src/trajectory_generation_ramp.cpp ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
#target_link_libraries(trajectory_generation_ramp
#    ${catkin_LIBRARIES}
#    Qt5::Core
#    Qt5::Widgets
#    Qt5::PrintSupport
#)
#install(TARGETS trajectory_generation_ramp RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#add_executable(trajectory_generation_imu_pushrecovery src/trajectory_generation_imu_pushrecovery.cpp ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
#target_link_libraries(trajectory_generation_imu_pushrecovery
#    ${catkin_LIBRARIES}
#    Qt5::Core
#    Qt5::Widgets
#    Qt5::PrintSupport
#)
#install(TARGETS trajectory_generation_imu_pushrecovery RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(home_legs src/home_legs.cpp ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(home_legs
    ${catkin_LIBRARIES}
    Qt5::Core
    Qt5::Widgets
    Qt5::PrintSupport
)
install(TARGETS home_legs RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#add_executable(deflection_measure src/deflection_measure.cpp ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
#target_link_libraries(deflection_measure
#    ${catkin_LIBRARIES}
#    Qt5::Core
#    Qt5::Widgets
#    Qt5::PrintSupport
#)
#install(TARGETS deflection_measure RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#add_executable(onlinetrajectorygeneration src/onlinetrajectorygeneration.cpp ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
#target_link_libraries(onlinetrajectorygeneration
#    ${catkin_LIBRARIES}
#    Qt5::Core
#    Qt5::Widgets
#    Qt5::PrintSupport
#)
#install(TARGETS onlinetrajectorygeneration RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#add_executable(hand_gui src/hand_gui.cpp ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
#target_link_libraries(hand_gui
#    ${catkin_LIBRARIES}
#    Qt5::Core
#    Qt5::Widgets
#    Qt5::PrintSupport
#)
#install(TARGETS hand_gui RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(key_control_of_legs src/key_control_of_legs.cpp ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(key_control_of_legs
    ${catkin_LIBRARIES}
    Qt5::Core
    Qt5::Widgets
    Qt5::PrintSupport
)
install(TARGETS key_control_of_legs RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(hand_trajectory_func_path src/hand_trajectory_func_path.cpp ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(hand_trajectory_func_path
    ${catkin_LIBRARIES}
    Qt5::Core
    Qt5::Widgets
    Qt5::PrintSupport
)
install(TARGETS hand_trajectory_func_path RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#add_executable(trajectory_generation_turning src/trajectory_generation_turning.cpp ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
#target_link_libraries(trajectory_generation_turning
#    ${catkin_LIBRARIES}
#    Qt5::Core
#    Qt5::Widgets
#    Qt5::PrintSupport
#)
#install(TARGETS trajectory_generation_turning RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


#add_executable(random_driver src/myNode.cpp )
#target_link_libraries(random_driver ${catkin_LIBRARIES})

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
