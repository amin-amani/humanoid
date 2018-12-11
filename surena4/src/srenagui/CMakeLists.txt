## BEGIN_TUTORIAL
## This CMakeLists.txt file for rviz_plugin_tutorials builds both the
## TeleopPanel tutorial and the ImuDisplay tutorial.
##
## First start with some standard catkin stuff.
cmake_minimum_required(VERSION 2.8.3)
project(surenagui)
find_package(catkin REQUIRED COMPONENTS rviz roscpp)
catkin_package()
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

## This setting causes Qt's "MOC" generation to happen automatically.
set(CMAKE_AUTOMOC ON)

## This plugin includes Qt widgets, so we must include Qt.
## We'll use the version that rviz used so they are compatible.
if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  ## pull in all required include dirs, define QT_LIBRARIES, etc.
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  set(QT_LIBRARIES Qt5::Widgets)
endif()

## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

## Here we specify the list of source files.
## The generated MOC files are included automatically as headers.
set(SRC_FILES
  src/myviz.cpp
  src/surenagui.cpp
)

## Add the "myviz" executable and specify the list of source files we
## collected above in ``${SRC_FILES}``.
add_executable(surenagui ${SRC_FILES})

## Link the myviz executable with whatever Qt libraries have been defined by
## the ``find_package(Qt4 ...)`` line above, or by the
## ``set(QT_LIBRARIES Qt5::Widgets)``, and with whatever libraries
## catkin has included.
target_link_libraries(surenagui ${QT_LIBRARIES} ${catkin_LIBRARIES})
## END_TUTORIAL

## Install
install(TARGETS surenagui DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


###############################################################################
## CMake
###############################################################################

#cmake_minimum_required(VERSION 2.8.0)
#project(surenagui)

###############################################################################
## Catkin
###############################################################################

## qt_build provides the qt cmake glue, roscpp the comms for a default talker
#find_package(catkin REQUIRED COMPONENTS  roscpp sensor_msgs interactive_markers  visualization_msgs  tf   message_generation message_runtime)
#set(LIBUSB_1_LIBRARIES "/usr/lib/x86_64-linux-gnu/libusb-1.0.so")
#include_directories(include ${catkin_INCLUDE_DIRS})
## Use this to define what the package will export (e.g. libs, headers).
## Since the default here is to produce only a binary, we don't worry about
## exporting anything.

##add_message_files(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/msg FILES num.msg)
#add_service_files(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/srv FILES active_csp.srv)
#add_service_files(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/srv FILES reset_node.srv)
#add_service_files(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/srv FILES home.srv)
#### Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )



#catkin_package()
#message(${CMAKE_CURRENT_SOURCE_DIR})
###############################################################################
## Qt Environment
###############################################################################

#set(CMAKE_INCLUDE_CURRENT_DIR ON)
#set(CMAKE_AUTOMOC ON)
##SET(CMAKE_AUTOUIC ON)

#find_package(Qt5 COMPONENTS Core Gui Widgets PrintSupport)

###############################################################################
## Sections
###############################################################################

#file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
#file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)


#QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
#QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
### This plugin includes Qt widgets, so we must include Qt.
### We'll use the version that rviz used so they are compatible.
#if(rviz_QT_VERSION VERSION_LESS "5")
#  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
#  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
#  ## pull in all required include dirs, define QT_LIBRARIES, etc.
#  include(${QT_USE_FILE})
#else()
#  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
#  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
#  ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
#  set(QT_LIBRARIES Qt5::Widgets)
#endif()

#include_directories(
#    ${Qt5Core_INCLUDE_DIRS}
#    ${Qt5Gui_INCLUDE_DIRS}
#    ${Qt5Widgets_INCLUDE_DIRS}
#    ${Qt5PrintSupport_INCLUDE_DIRS}
#    )

#add_definitions( -std=c++11 -fPIC)
###############################################################################
## Sources
###############################################################################

#set( QT_SOURCES
#    src/surenagui.cpp
#    src/myviz.cpp


#    )
###############################################################################
## Binaries
###############################################################################

#add_executable(surenagui ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
#target_link_libraries(surenagui
#    ${catkin_LIBRARIES}
#    Qt5::Core
#    Qt5::Widgets
#    Qt5::PrintSupport
#    ${LIBUSB_1_LIBRARIES}
#)
#install(TARGETS surenagui RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


