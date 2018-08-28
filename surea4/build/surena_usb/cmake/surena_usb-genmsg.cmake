# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "surena_usb: 0 messages, 2 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(surena_usb_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/amin/humanoid/surea4/src/surena_usb/srv/reset_node.srv" NAME_WE)
add_custom_target(_surena_usb_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "surena_usb" "/home/amin/humanoid/surea4/src/surena_usb/srv/reset_node.srv" ""
)

get_filename_component(_filename "/home/amin/humanoid/surea4/src/surena_usb/srv/active_csp.srv" NAME_WE)
add_custom_target(_surena_usb_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "surena_usb" "/home/amin/humanoid/surea4/src/surena_usb/srv/active_csp.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(surena_usb
  "/home/amin/humanoid/surea4/src/surena_usb/srv/reset_node.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/surena_usb
)
_generate_srv_cpp(surena_usb
  "/home/amin/humanoid/surea4/src/surena_usb/srv/active_csp.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/surena_usb
)

### Generating Module File
_generate_module_cpp(surena_usb
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/surena_usb
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(surena_usb_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(surena_usb_generate_messages surena_usb_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/amin/humanoid/surea4/src/surena_usb/srv/reset_node.srv" NAME_WE)
add_dependencies(surena_usb_generate_messages_cpp _surena_usb_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amin/humanoid/surea4/src/surena_usb/srv/active_csp.srv" NAME_WE)
add_dependencies(surena_usb_generate_messages_cpp _surena_usb_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(surena_usb_gencpp)
add_dependencies(surena_usb_gencpp surena_usb_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS surena_usb_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(surena_usb
  "/home/amin/humanoid/surea4/src/surena_usb/srv/reset_node.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/surena_usb
)
_generate_srv_eus(surena_usb
  "/home/amin/humanoid/surea4/src/surena_usb/srv/active_csp.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/surena_usb
)

### Generating Module File
_generate_module_eus(surena_usb
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/surena_usb
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(surena_usb_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(surena_usb_generate_messages surena_usb_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/amin/humanoid/surea4/src/surena_usb/srv/reset_node.srv" NAME_WE)
add_dependencies(surena_usb_generate_messages_eus _surena_usb_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amin/humanoid/surea4/src/surena_usb/srv/active_csp.srv" NAME_WE)
add_dependencies(surena_usb_generate_messages_eus _surena_usb_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(surena_usb_geneus)
add_dependencies(surena_usb_geneus surena_usb_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS surena_usb_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(surena_usb
  "/home/amin/humanoid/surea4/src/surena_usb/srv/reset_node.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/surena_usb
)
_generate_srv_lisp(surena_usb
  "/home/amin/humanoid/surea4/src/surena_usb/srv/active_csp.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/surena_usb
)

### Generating Module File
_generate_module_lisp(surena_usb
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/surena_usb
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(surena_usb_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(surena_usb_generate_messages surena_usb_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/amin/humanoid/surea4/src/surena_usb/srv/reset_node.srv" NAME_WE)
add_dependencies(surena_usb_generate_messages_lisp _surena_usb_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amin/humanoid/surea4/src/surena_usb/srv/active_csp.srv" NAME_WE)
add_dependencies(surena_usb_generate_messages_lisp _surena_usb_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(surena_usb_genlisp)
add_dependencies(surena_usb_genlisp surena_usb_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS surena_usb_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(surena_usb
  "/home/amin/humanoid/surea4/src/surena_usb/srv/reset_node.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/surena_usb
)
_generate_srv_nodejs(surena_usb
  "/home/amin/humanoid/surea4/src/surena_usb/srv/active_csp.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/surena_usb
)

### Generating Module File
_generate_module_nodejs(surena_usb
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/surena_usb
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(surena_usb_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(surena_usb_generate_messages surena_usb_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/amin/humanoid/surea4/src/surena_usb/srv/reset_node.srv" NAME_WE)
add_dependencies(surena_usb_generate_messages_nodejs _surena_usb_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amin/humanoid/surea4/src/surena_usb/srv/active_csp.srv" NAME_WE)
add_dependencies(surena_usb_generate_messages_nodejs _surena_usb_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(surena_usb_gennodejs)
add_dependencies(surena_usb_gennodejs surena_usb_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS surena_usb_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(surena_usb
  "/home/amin/humanoid/surea4/src/surena_usb/srv/reset_node.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/surena_usb
)
_generate_srv_py(surena_usb
  "/home/amin/humanoid/surea4/src/surena_usb/srv/active_csp.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/surena_usb
)

### Generating Module File
_generate_module_py(surena_usb
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/surena_usb
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(surena_usb_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(surena_usb_generate_messages surena_usb_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/amin/humanoid/surea4/src/surena_usb/srv/reset_node.srv" NAME_WE)
add_dependencies(surena_usb_generate_messages_py _surena_usb_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/amin/humanoid/surea4/src/surena_usb/srv/active_csp.srv" NAME_WE)
add_dependencies(surena_usb_generate_messages_py _surena_usb_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(surena_usb_genpy)
add_dependencies(surena_usb_genpy surena_usb_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS surena_usb_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/surena_usb)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/surena_usb
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(surena_usb_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/surena_usb)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/surena_usb
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(surena_usb_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/surena_usb)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/surena_usb
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(surena_usb_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/surena_usb)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/surena_usb
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(surena_usb_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/surena_usb)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/surena_usb\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/surena_usb
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(surena_usb_generate_messages_py std_msgs_generate_messages_py)
endif()
