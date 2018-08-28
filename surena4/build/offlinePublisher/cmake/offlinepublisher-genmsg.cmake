# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "offlinepublisher: 1 messages, 1 services")

set(MSG_I_FLAGS "-Iofflinepublisher:/home/amin/humanoid/surea4/src/offlinePublisher/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(offlinepublisher_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/amin/humanoid/surea4/src/offlinePublisher/msg/num.msg" NAME_WE)
add_custom_target(_offlinepublisher_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "offlinepublisher" "/home/amin/humanoid/surea4/src/offlinePublisher/msg/num.msg" ""
)

get_filename_component(_filename "/home/amin/humanoid/surea4/src/offlinePublisher/srv/srv1.srv" NAME_WE)
add_custom_target(_offlinepublisher_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "offlinepublisher" "/home/amin/humanoid/surea4/src/offlinePublisher/srv/srv1.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(offlinepublisher
  "/home/amin/humanoid/surea4/src/offlinePublisher/msg/num.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/offlinepublisher
)

### Generating Services
_generate_srv_cpp(offlinepublisher
  "/home/amin/humanoid/surea4/src/offlinePublisher/srv/srv1.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/offlinepublisher
)

### Generating Module File
_generate_module_cpp(offlinepublisher
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/offlinepublisher
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(offlinepublisher_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(offlinepublisher_generate_messages offlinepublisher_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/amin/humanoid/surea4/src/offlinePublisher/msg/num.msg" NAME_WE)
add_dependencies(offlinepublisher_generate_messages_cpp _offlinepublisher_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amin/humanoid/surea4/src/offlinePublisher/srv/srv1.srv" NAME_WE)
add_dependencies(offlinepublisher_generate_messages_cpp _offlinepublisher_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(offlinepublisher_gencpp)
add_dependencies(offlinepublisher_gencpp offlinepublisher_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS offlinepublisher_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(offlinepublisher
  "/home/amin/humanoid/surea4/src/offlinePublisher/msg/num.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/offlinepublisher
)

### Generating Services
_generate_srv_eus(offlinepublisher
  "/home/amin/humanoid/surea4/src/offlinePublisher/srv/srv1.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/offlinepublisher
)

### Generating Module File
_generate_module_eus(offlinepublisher
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/offlinepublisher
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(offlinepublisher_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(offlinepublisher_generate_messages offlinepublisher_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/amin/humanoid/surea4/src/offlinePublisher/msg/num.msg" NAME_WE)
add_dependencies(offlinepublisher_generate_messages_eus _offlinepublisher_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amin/humanoid/surea4/src/offlinePublisher/srv/srv1.srv" NAME_WE)
add_dependencies(offlinepublisher_generate_messages_eus _offlinepublisher_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(offlinepublisher_geneus)
add_dependencies(offlinepublisher_geneus offlinepublisher_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS offlinepublisher_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(offlinepublisher
  "/home/amin/humanoid/surea4/src/offlinePublisher/msg/num.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/offlinepublisher
)

### Generating Services
_generate_srv_lisp(offlinepublisher
  "/home/amin/humanoid/surea4/src/offlinePublisher/srv/srv1.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/offlinepublisher
)

### Generating Module File
_generate_module_lisp(offlinepublisher
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/offlinepublisher
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(offlinepublisher_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(offlinepublisher_generate_messages offlinepublisher_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/amin/humanoid/surea4/src/offlinePublisher/msg/num.msg" NAME_WE)
add_dependencies(offlinepublisher_generate_messages_lisp _offlinepublisher_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amin/humanoid/surea4/src/offlinePublisher/srv/srv1.srv" NAME_WE)
add_dependencies(offlinepublisher_generate_messages_lisp _offlinepublisher_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(offlinepublisher_genlisp)
add_dependencies(offlinepublisher_genlisp offlinepublisher_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS offlinepublisher_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(offlinepublisher
  "/home/amin/humanoid/surea4/src/offlinePublisher/msg/num.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/offlinepublisher
)

### Generating Services
_generate_srv_nodejs(offlinepublisher
  "/home/amin/humanoid/surea4/src/offlinePublisher/srv/srv1.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/offlinepublisher
)

### Generating Module File
_generate_module_nodejs(offlinepublisher
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/offlinepublisher
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(offlinepublisher_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(offlinepublisher_generate_messages offlinepublisher_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/amin/humanoid/surea4/src/offlinePublisher/msg/num.msg" NAME_WE)
add_dependencies(offlinepublisher_generate_messages_nodejs _offlinepublisher_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amin/humanoid/surea4/src/offlinePublisher/srv/srv1.srv" NAME_WE)
add_dependencies(offlinepublisher_generate_messages_nodejs _offlinepublisher_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(offlinepublisher_gennodejs)
add_dependencies(offlinepublisher_gennodejs offlinepublisher_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS offlinepublisher_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(offlinepublisher
  "/home/amin/humanoid/surea4/src/offlinePublisher/msg/num.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/offlinepublisher
)

### Generating Services
_generate_srv_py(offlinepublisher
  "/home/amin/humanoid/surea4/src/offlinePublisher/srv/srv1.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/offlinepublisher
)

### Generating Module File
_generate_module_py(offlinepublisher
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/offlinepublisher
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(offlinepublisher_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(offlinepublisher_generate_messages offlinepublisher_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/amin/humanoid/surea4/src/offlinePublisher/msg/num.msg" NAME_WE)
add_dependencies(offlinepublisher_generate_messages_py _offlinepublisher_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amin/humanoid/surea4/src/offlinePublisher/srv/srv1.srv" NAME_WE)
add_dependencies(offlinepublisher_generate_messages_py _offlinepublisher_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(offlinepublisher_genpy)
add_dependencies(offlinepublisher_genpy offlinepublisher_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS offlinepublisher_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/offlinepublisher)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/offlinepublisher
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(offlinepublisher_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/offlinepublisher)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/offlinepublisher
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(offlinepublisher_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/offlinepublisher)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/offlinepublisher
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(offlinepublisher_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/offlinepublisher)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/offlinepublisher
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(offlinepublisher_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/offlinepublisher)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/offlinepublisher\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/offlinepublisher
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(offlinepublisher_generate_messages_py std_msgs_generate_messages_py)
endif()
