# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "remote_control: 1 messages, 0 services")

set(MSG_I_FLAGS "-Iremote_control:/home/crow/Documents/SDU-UAS-TX/remotecontrol_ws/src/remote_control/msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(remote_control_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/crow/Documents/SDU-UAS-TX/remotecontrol_ws/src/remote_control/msg/set_controller.msg" NAME_WE)
add_custom_target(_remote_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "remote_control" "/home/crow/Documents/SDU-UAS-TX/remotecontrol_ws/src/remote_control/msg/set_controller.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(remote_control
  "/home/crow/Documents/SDU-UAS-TX/remotecontrol_ws/src/remote_control/msg/set_controller.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/remote_control
)

### Generating Services

### Generating Module File
_generate_module_cpp(remote_control
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/remote_control
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(remote_control_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(remote_control_generate_messages remote_control_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/crow/Documents/SDU-UAS-TX/remotecontrol_ws/src/remote_control/msg/set_controller.msg" NAME_WE)
add_dependencies(remote_control_generate_messages_cpp _remote_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(remote_control_gencpp)
add_dependencies(remote_control_gencpp remote_control_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS remote_control_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(remote_control
  "/home/crow/Documents/SDU-UAS-TX/remotecontrol_ws/src/remote_control/msg/set_controller.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/remote_control
)

### Generating Services

### Generating Module File
_generate_module_eus(remote_control
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/remote_control
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(remote_control_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(remote_control_generate_messages remote_control_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/crow/Documents/SDU-UAS-TX/remotecontrol_ws/src/remote_control/msg/set_controller.msg" NAME_WE)
add_dependencies(remote_control_generate_messages_eus _remote_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(remote_control_geneus)
add_dependencies(remote_control_geneus remote_control_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS remote_control_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(remote_control
  "/home/crow/Documents/SDU-UAS-TX/remotecontrol_ws/src/remote_control/msg/set_controller.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/remote_control
)

### Generating Services

### Generating Module File
_generate_module_lisp(remote_control
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/remote_control
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(remote_control_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(remote_control_generate_messages remote_control_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/crow/Documents/SDU-UAS-TX/remotecontrol_ws/src/remote_control/msg/set_controller.msg" NAME_WE)
add_dependencies(remote_control_generate_messages_lisp _remote_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(remote_control_genlisp)
add_dependencies(remote_control_genlisp remote_control_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS remote_control_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(remote_control
  "/home/crow/Documents/SDU-UAS-TX/remotecontrol_ws/src/remote_control/msg/set_controller.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/remote_control
)

### Generating Services

### Generating Module File
_generate_module_nodejs(remote_control
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/remote_control
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(remote_control_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(remote_control_generate_messages remote_control_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/crow/Documents/SDU-UAS-TX/remotecontrol_ws/src/remote_control/msg/set_controller.msg" NAME_WE)
add_dependencies(remote_control_generate_messages_nodejs _remote_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(remote_control_gennodejs)
add_dependencies(remote_control_gennodejs remote_control_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS remote_control_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(remote_control
  "/home/crow/Documents/SDU-UAS-TX/remotecontrol_ws/src/remote_control/msg/set_controller.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/remote_control
)

### Generating Services

### Generating Module File
_generate_module_py(remote_control
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/remote_control
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(remote_control_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(remote_control_generate_messages remote_control_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/crow/Documents/SDU-UAS-TX/remotecontrol_ws/src/remote_control/msg/set_controller.msg" NAME_WE)
add_dependencies(remote_control_generate_messages_py _remote_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(remote_control_genpy)
add_dependencies(remote_control_genpy remote_control_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS remote_control_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/remote_control)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/remote_control
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/remote_control)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/remote_control
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/remote_control)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/remote_control
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/remote_control)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/remote_control
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/remote_control)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/remote_control\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/remote_control
    DESTINATION ${genpy_INSTALL_DIR}
    # skip all init files
    PATTERN "__init__.py" EXCLUDE
    PATTERN "__init__.pyc" EXCLUDE
  )
  # install init files which are not in the root folder of the generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/remote_control
    DESTINATION ${genpy_INSTALL_DIR}
    FILES_MATCHING
    REGEX "${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/remote_control/.+/__init__.pyc?$"
  )
endif()
