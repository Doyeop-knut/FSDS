# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "fs_msgs: 6 messages, 1 services")

set(MSG_I_FLAGS "-Ifs_msgs:/home/user/FSDS/src/fs_msgs/msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(fs_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/user/FSDS/src/fs_msgs/msg/ControlCommand.msg" NAME_WE)
add_custom_target(_fs_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fs_msgs" "/home/user/FSDS/src/fs_msgs/msg/ControlCommand.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/user/FSDS/src/fs_msgs/msg/FinishedSignal.msg" NAME_WE)
add_custom_target(_fs_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fs_msgs" "/home/user/FSDS/src/fs_msgs/msg/FinishedSignal.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/user/FSDS/src/fs_msgs/msg/GoSignal.msg" NAME_WE)
add_custom_target(_fs_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fs_msgs" "/home/user/FSDS/src/fs_msgs/msg/GoSignal.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/user/FSDS/src/fs_msgs/msg/Track.msg" NAME_WE)
add_custom_target(_fs_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fs_msgs" "/home/user/FSDS/src/fs_msgs/msg/Track.msg" "fs_msgs/Cone:geometry_msgs/Point"
)

get_filename_component(_filename "/home/user/FSDS/src/fs_msgs/msg/Cone.msg" NAME_WE)
add_custom_target(_fs_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fs_msgs" "/home/user/FSDS/src/fs_msgs/msg/Cone.msg" "geometry_msgs/Point"
)

get_filename_component(_filename "/home/user/FSDS/src/fs_msgs/msg/ExtraInfo.msg" NAME_WE)
add_custom_target(_fs_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fs_msgs" "/home/user/FSDS/src/fs_msgs/msg/ExtraInfo.msg" ""
)

get_filename_component(_filename "/home/user/FSDS/src/fs_msgs/srv/Reset.srv" NAME_WE)
add_custom_target(_fs_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fs_msgs" "/home/user/FSDS/src/fs_msgs/srv/Reset.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(fs_msgs
  "/home/user/FSDS/src/fs_msgs/msg/ControlCommand.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fs_msgs
)
_generate_msg_cpp(fs_msgs
  "/home/user/FSDS/src/fs_msgs/msg/FinishedSignal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fs_msgs
)
_generate_msg_cpp(fs_msgs
  "/home/user/FSDS/src/fs_msgs/msg/GoSignal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fs_msgs
)
_generate_msg_cpp(fs_msgs
  "/home/user/FSDS/src/fs_msgs/msg/Track.msg"
  "${MSG_I_FLAGS}"
  "/home/user/FSDS/src/fs_msgs/msg/Cone.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fs_msgs
)
_generate_msg_cpp(fs_msgs
  "/home/user/FSDS/src/fs_msgs/msg/Cone.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fs_msgs
)
_generate_msg_cpp(fs_msgs
  "/home/user/FSDS/src/fs_msgs/msg/ExtraInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fs_msgs
)

### Generating Services
_generate_srv_cpp(fs_msgs
  "/home/user/FSDS/src/fs_msgs/srv/Reset.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fs_msgs
)

### Generating Module File
_generate_module_cpp(fs_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fs_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(fs_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(fs_msgs_generate_messages fs_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/user/FSDS/src/fs_msgs/msg/ControlCommand.msg" NAME_WE)
add_dependencies(fs_msgs_generate_messages_cpp _fs_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/FSDS/src/fs_msgs/msg/FinishedSignal.msg" NAME_WE)
add_dependencies(fs_msgs_generate_messages_cpp _fs_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/FSDS/src/fs_msgs/msg/GoSignal.msg" NAME_WE)
add_dependencies(fs_msgs_generate_messages_cpp _fs_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/FSDS/src/fs_msgs/msg/Track.msg" NAME_WE)
add_dependencies(fs_msgs_generate_messages_cpp _fs_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/FSDS/src/fs_msgs/msg/Cone.msg" NAME_WE)
add_dependencies(fs_msgs_generate_messages_cpp _fs_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/FSDS/src/fs_msgs/msg/ExtraInfo.msg" NAME_WE)
add_dependencies(fs_msgs_generate_messages_cpp _fs_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/FSDS/src/fs_msgs/srv/Reset.srv" NAME_WE)
add_dependencies(fs_msgs_generate_messages_cpp _fs_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fs_msgs_gencpp)
add_dependencies(fs_msgs_gencpp fs_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fs_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(fs_msgs
  "/home/user/FSDS/src/fs_msgs/msg/ControlCommand.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fs_msgs
)
_generate_msg_eus(fs_msgs
  "/home/user/FSDS/src/fs_msgs/msg/FinishedSignal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fs_msgs
)
_generate_msg_eus(fs_msgs
  "/home/user/FSDS/src/fs_msgs/msg/GoSignal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fs_msgs
)
_generate_msg_eus(fs_msgs
  "/home/user/FSDS/src/fs_msgs/msg/Track.msg"
  "${MSG_I_FLAGS}"
  "/home/user/FSDS/src/fs_msgs/msg/Cone.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fs_msgs
)
_generate_msg_eus(fs_msgs
  "/home/user/FSDS/src/fs_msgs/msg/Cone.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fs_msgs
)
_generate_msg_eus(fs_msgs
  "/home/user/FSDS/src/fs_msgs/msg/ExtraInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fs_msgs
)

### Generating Services
_generate_srv_eus(fs_msgs
  "/home/user/FSDS/src/fs_msgs/srv/Reset.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fs_msgs
)

### Generating Module File
_generate_module_eus(fs_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fs_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(fs_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(fs_msgs_generate_messages fs_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/user/FSDS/src/fs_msgs/msg/ControlCommand.msg" NAME_WE)
add_dependencies(fs_msgs_generate_messages_eus _fs_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/FSDS/src/fs_msgs/msg/FinishedSignal.msg" NAME_WE)
add_dependencies(fs_msgs_generate_messages_eus _fs_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/FSDS/src/fs_msgs/msg/GoSignal.msg" NAME_WE)
add_dependencies(fs_msgs_generate_messages_eus _fs_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/FSDS/src/fs_msgs/msg/Track.msg" NAME_WE)
add_dependencies(fs_msgs_generate_messages_eus _fs_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/FSDS/src/fs_msgs/msg/Cone.msg" NAME_WE)
add_dependencies(fs_msgs_generate_messages_eus _fs_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/FSDS/src/fs_msgs/msg/ExtraInfo.msg" NAME_WE)
add_dependencies(fs_msgs_generate_messages_eus _fs_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/FSDS/src/fs_msgs/srv/Reset.srv" NAME_WE)
add_dependencies(fs_msgs_generate_messages_eus _fs_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fs_msgs_geneus)
add_dependencies(fs_msgs_geneus fs_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fs_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(fs_msgs
  "/home/user/FSDS/src/fs_msgs/msg/ControlCommand.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fs_msgs
)
_generate_msg_lisp(fs_msgs
  "/home/user/FSDS/src/fs_msgs/msg/FinishedSignal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fs_msgs
)
_generate_msg_lisp(fs_msgs
  "/home/user/FSDS/src/fs_msgs/msg/GoSignal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fs_msgs
)
_generate_msg_lisp(fs_msgs
  "/home/user/FSDS/src/fs_msgs/msg/Track.msg"
  "${MSG_I_FLAGS}"
  "/home/user/FSDS/src/fs_msgs/msg/Cone.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fs_msgs
)
_generate_msg_lisp(fs_msgs
  "/home/user/FSDS/src/fs_msgs/msg/Cone.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fs_msgs
)
_generate_msg_lisp(fs_msgs
  "/home/user/FSDS/src/fs_msgs/msg/ExtraInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fs_msgs
)

### Generating Services
_generate_srv_lisp(fs_msgs
  "/home/user/FSDS/src/fs_msgs/srv/Reset.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fs_msgs
)

### Generating Module File
_generate_module_lisp(fs_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fs_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(fs_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(fs_msgs_generate_messages fs_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/user/FSDS/src/fs_msgs/msg/ControlCommand.msg" NAME_WE)
add_dependencies(fs_msgs_generate_messages_lisp _fs_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/FSDS/src/fs_msgs/msg/FinishedSignal.msg" NAME_WE)
add_dependencies(fs_msgs_generate_messages_lisp _fs_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/FSDS/src/fs_msgs/msg/GoSignal.msg" NAME_WE)
add_dependencies(fs_msgs_generate_messages_lisp _fs_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/FSDS/src/fs_msgs/msg/Track.msg" NAME_WE)
add_dependencies(fs_msgs_generate_messages_lisp _fs_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/FSDS/src/fs_msgs/msg/Cone.msg" NAME_WE)
add_dependencies(fs_msgs_generate_messages_lisp _fs_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/FSDS/src/fs_msgs/msg/ExtraInfo.msg" NAME_WE)
add_dependencies(fs_msgs_generate_messages_lisp _fs_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/FSDS/src/fs_msgs/srv/Reset.srv" NAME_WE)
add_dependencies(fs_msgs_generate_messages_lisp _fs_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fs_msgs_genlisp)
add_dependencies(fs_msgs_genlisp fs_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fs_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(fs_msgs
  "/home/user/FSDS/src/fs_msgs/msg/ControlCommand.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fs_msgs
)
_generate_msg_nodejs(fs_msgs
  "/home/user/FSDS/src/fs_msgs/msg/FinishedSignal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fs_msgs
)
_generate_msg_nodejs(fs_msgs
  "/home/user/FSDS/src/fs_msgs/msg/GoSignal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fs_msgs
)
_generate_msg_nodejs(fs_msgs
  "/home/user/FSDS/src/fs_msgs/msg/Track.msg"
  "${MSG_I_FLAGS}"
  "/home/user/FSDS/src/fs_msgs/msg/Cone.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fs_msgs
)
_generate_msg_nodejs(fs_msgs
  "/home/user/FSDS/src/fs_msgs/msg/Cone.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fs_msgs
)
_generate_msg_nodejs(fs_msgs
  "/home/user/FSDS/src/fs_msgs/msg/ExtraInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fs_msgs
)

### Generating Services
_generate_srv_nodejs(fs_msgs
  "/home/user/FSDS/src/fs_msgs/srv/Reset.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fs_msgs
)

### Generating Module File
_generate_module_nodejs(fs_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fs_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(fs_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(fs_msgs_generate_messages fs_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/user/FSDS/src/fs_msgs/msg/ControlCommand.msg" NAME_WE)
add_dependencies(fs_msgs_generate_messages_nodejs _fs_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/FSDS/src/fs_msgs/msg/FinishedSignal.msg" NAME_WE)
add_dependencies(fs_msgs_generate_messages_nodejs _fs_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/FSDS/src/fs_msgs/msg/GoSignal.msg" NAME_WE)
add_dependencies(fs_msgs_generate_messages_nodejs _fs_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/FSDS/src/fs_msgs/msg/Track.msg" NAME_WE)
add_dependencies(fs_msgs_generate_messages_nodejs _fs_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/FSDS/src/fs_msgs/msg/Cone.msg" NAME_WE)
add_dependencies(fs_msgs_generate_messages_nodejs _fs_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/FSDS/src/fs_msgs/msg/ExtraInfo.msg" NAME_WE)
add_dependencies(fs_msgs_generate_messages_nodejs _fs_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/FSDS/src/fs_msgs/srv/Reset.srv" NAME_WE)
add_dependencies(fs_msgs_generate_messages_nodejs _fs_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fs_msgs_gennodejs)
add_dependencies(fs_msgs_gennodejs fs_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fs_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(fs_msgs
  "/home/user/FSDS/src/fs_msgs/msg/ControlCommand.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fs_msgs
)
_generate_msg_py(fs_msgs
  "/home/user/FSDS/src/fs_msgs/msg/FinishedSignal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fs_msgs
)
_generate_msg_py(fs_msgs
  "/home/user/FSDS/src/fs_msgs/msg/GoSignal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fs_msgs
)
_generate_msg_py(fs_msgs
  "/home/user/FSDS/src/fs_msgs/msg/Track.msg"
  "${MSG_I_FLAGS}"
  "/home/user/FSDS/src/fs_msgs/msg/Cone.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fs_msgs
)
_generate_msg_py(fs_msgs
  "/home/user/FSDS/src/fs_msgs/msg/Cone.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fs_msgs
)
_generate_msg_py(fs_msgs
  "/home/user/FSDS/src/fs_msgs/msg/ExtraInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fs_msgs
)

### Generating Services
_generate_srv_py(fs_msgs
  "/home/user/FSDS/src/fs_msgs/srv/Reset.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fs_msgs
)

### Generating Module File
_generate_module_py(fs_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fs_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(fs_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(fs_msgs_generate_messages fs_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/user/FSDS/src/fs_msgs/msg/ControlCommand.msg" NAME_WE)
add_dependencies(fs_msgs_generate_messages_py _fs_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/FSDS/src/fs_msgs/msg/FinishedSignal.msg" NAME_WE)
add_dependencies(fs_msgs_generate_messages_py _fs_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/FSDS/src/fs_msgs/msg/GoSignal.msg" NAME_WE)
add_dependencies(fs_msgs_generate_messages_py _fs_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/FSDS/src/fs_msgs/msg/Track.msg" NAME_WE)
add_dependencies(fs_msgs_generate_messages_py _fs_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/FSDS/src/fs_msgs/msg/Cone.msg" NAME_WE)
add_dependencies(fs_msgs_generate_messages_py _fs_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/FSDS/src/fs_msgs/msg/ExtraInfo.msg" NAME_WE)
add_dependencies(fs_msgs_generate_messages_py _fs_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/user/FSDS/src/fs_msgs/srv/Reset.srv" NAME_WE)
add_dependencies(fs_msgs_generate_messages_py _fs_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fs_msgs_genpy)
add_dependencies(fs_msgs_genpy fs_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fs_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fs_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fs_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(fs_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fs_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fs_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(fs_msgs_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fs_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fs_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(fs_msgs_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fs_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fs_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(fs_msgs_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fs_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fs_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fs_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(fs_msgs_generate_messages_py geometry_msgs_generate_messages_py)
endif()
