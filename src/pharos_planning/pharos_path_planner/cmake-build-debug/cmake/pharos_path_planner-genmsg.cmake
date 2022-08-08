# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "pharos_path_planner: 5 messages, 0 services")

set(MSG_I_FLAGS "-Ipharos_path_planner:/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg;-Itf:/opt/ros/kinetic/share/tf/cmake/../msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/kinetic/share/sensor_msgs/cmake/../msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(pharos_path_planner_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/ReferencePath.msg" NAME_WE)
add_custom_target(_pharos_path_planner_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pharos_path_planner" "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/ReferencePath.msg" "pharos_path_planner/Position:pharos_path_planner/RoadInfo"
)

get_filename_component(_filename "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/ReferencePath2.msg" NAME_WE)
add_custom_target(_pharos_path_planner_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pharos_path_planner" "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/ReferencePath2.msg" "pharos_path_planner/Position:pharos_path_planner/RoadInfo:pharos_path_planner/LaneInfo"
)

get_filename_component(_filename "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/Position.msg" NAME_WE)
add_custom_target(_pharos_path_planner_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pharos_path_planner" "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/Position.msg" ""
)

get_filename_component(_filename "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/RoadInfo.msg" NAME_WE)
add_custom_target(_pharos_path_planner_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pharos_path_planner" "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/RoadInfo.msg" "pharos_path_planner/Position"
)

get_filename_component(_filename "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/LaneInfo.msg" NAME_WE)
add_custom_target(_pharos_path_planner_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pharos_path_planner" "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/LaneInfo.msg" "pharos_path_planner/Position:pharos_path_planner/RoadInfo"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(pharos_path_planner
  "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/ReferencePath.msg"
  "${MSG_I_FLAGS}"
  "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/Position.msg;/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/RoadInfo.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pharos_path_planner
)
_generate_msg_cpp(pharos_path_planner
  "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/ReferencePath2.msg"
  "${MSG_I_FLAGS}"
  "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/Position.msg;/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/RoadInfo.msg;/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/LaneInfo.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pharos_path_planner
)
_generate_msg_cpp(pharos_path_planner
  "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/Position.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pharos_path_planner
)
_generate_msg_cpp(pharos_path_planner
  "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/RoadInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/Position.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pharos_path_planner
)
_generate_msg_cpp(pharos_path_planner
  "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/LaneInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/Position.msg;/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/RoadInfo.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pharos_path_planner
)

### Generating Services

### Generating Module File
_generate_module_cpp(pharos_path_planner
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pharos_path_planner
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(pharos_path_planner_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(pharos_path_planner_generate_messages pharos_path_planner_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/ReferencePath.msg" NAME_WE)
add_dependencies(pharos_path_planner_generate_messages_cpp _pharos_path_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/ReferencePath2.msg" NAME_WE)
add_dependencies(pharos_path_planner_generate_messages_cpp _pharos_path_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/Position.msg" NAME_WE)
add_dependencies(pharos_path_planner_generate_messages_cpp _pharos_path_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/RoadInfo.msg" NAME_WE)
add_dependencies(pharos_path_planner_generate_messages_cpp _pharos_path_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/LaneInfo.msg" NAME_WE)
add_dependencies(pharos_path_planner_generate_messages_cpp _pharos_path_planner_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(pharos_path_planner_gencpp)
add_dependencies(pharos_path_planner_gencpp pharos_path_planner_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS pharos_path_planner_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(pharos_path_planner
  "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/ReferencePath.msg"
  "${MSG_I_FLAGS}"
  "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/Position.msg;/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/RoadInfo.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pharos_path_planner
)
_generate_msg_eus(pharos_path_planner
  "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/ReferencePath2.msg"
  "${MSG_I_FLAGS}"
  "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/Position.msg;/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/RoadInfo.msg;/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/LaneInfo.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pharos_path_planner
)
_generate_msg_eus(pharos_path_planner
  "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/Position.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pharos_path_planner
)
_generate_msg_eus(pharos_path_planner
  "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/RoadInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/Position.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pharos_path_planner
)
_generate_msg_eus(pharos_path_planner
  "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/LaneInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/Position.msg;/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/RoadInfo.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pharos_path_planner
)

### Generating Services

### Generating Module File
_generate_module_eus(pharos_path_planner
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pharos_path_planner
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(pharos_path_planner_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(pharos_path_planner_generate_messages pharos_path_planner_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/ReferencePath.msg" NAME_WE)
add_dependencies(pharos_path_planner_generate_messages_eus _pharos_path_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/ReferencePath2.msg" NAME_WE)
add_dependencies(pharos_path_planner_generate_messages_eus _pharos_path_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/Position.msg" NAME_WE)
add_dependencies(pharos_path_planner_generate_messages_eus _pharos_path_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/RoadInfo.msg" NAME_WE)
add_dependencies(pharos_path_planner_generate_messages_eus _pharos_path_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/LaneInfo.msg" NAME_WE)
add_dependencies(pharos_path_planner_generate_messages_eus _pharos_path_planner_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(pharos_path_planner_geneus)
add_dependencies(pharos_path_planner_geneus pharos_path_planner_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS pharos_path_planner_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(pharos_path_planner
  "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/ReferencePath.msg"
  "${MSG_I_FLAGS}"
  "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/Position.msg;/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/RoadInfo.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pharos_path_planner
)
_generate_msg_lisp(pharos_path_planner
  "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/ReferencePath2.msg"
  "${MSG_I_FLAGS}"
  "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/Position.msg;/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/RoadInfo.msg;/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/LaneInfo.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pharos_path_planner
)
_generate_msg_lisp(pharos_path_planner
  "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/Position.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pharos_path_planner
)
_generate_msg_lisp(pharos_path_planner
  "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/RoadInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/Position.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pharos_path_planner
)
_generate_msg_lisp(pharos_path_planner
  "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/LaneInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/Position.msg;/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/RoadInfo.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pharos_path_planner
)

### Generating Services

### Generating Module File
_generate_module_lisp(pharos_path_planner
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pharos_path_planner
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(pharos_path_planner_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(pharos_path_planner_generate_messages pharos_path_planner_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/ReferencePath.msg" NAME_WE)
add_dependencies(pharos_path_planner_generate_messages_lisp _pharos_path_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/ReferencePath2.msg" NAME_WE)
add_dependencies(pharos_path_planner_generate_messages_lisp _pharos_path_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/Position.msg" NAME_WE)
add_dependencies(pharos_path_planner_generate_messages_lisp _pharos_path_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/RoadInfo.msg" NAME_WE)
add_dependencies(pharos_path_planner_generate_messages_lisp _pharos_path_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/LaneInfo.msg" NAME_WE)
add_dependencies(pharos_path_planner_generate_messages_lisp _pharos_path_planner_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(pharos_path_planner_genlisp)
add_dependencies(pharos_path_planner_genlisp pharos_path_planner_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS pharos_path_planner_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(pharos_path_planner
  "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/ReferencePath.msg"
  "${MSG_I_FLAGS}"
  "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/Position.msg;/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/RoadInfo.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pharos_path_planner
)
_generate_msg_nodejs(pharos_path_planner
  "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/ReferencePath2.msg"
  "${MSG_I_FLAGS}"
  "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/Position.msg;/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/RoadInfo.msg;/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/LaneInfo.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pharos_path_planner
)
_generate_msg_nodejs(pharos_path_planner
  "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/Position.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pharos_path_planner
)
_generate_msg_nodejs(pharos_path_planner
  "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/RoadInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/Position.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pharos_path_planner
)
_generate_msg_nodejs(pharos_path_planner
  "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/LaneInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/Position.msg;/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/RoadInfo.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pharos_path_planner
)

### Generating Services

### Generating Module File
_generate_module_nodejs(pharos_path_planner
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pharos_path_planner
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(pharos_path_planner_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(pharos_path_planner_generate_messages pharos_path_planner_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/ReferencePath.msg" NAME_WE)
add_dependencies(pharos_path_planner_generate_messages_nodejs _pharos_path_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/ReferencePath2.msg" NAME_WE)
add_dependencies(pharos_path_planner_generate_messages_nodejs _pharos_path_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/Position.msg" NAME_WE)
add_dependencies(pharos_path_planner_generate_messages_nodejs _pharos_path_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/RoadInfo.msg" NAME_WE)
add_dependencies(pharos_path_planner_generate_messages_nodejs _pharos_path_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/LaneInfo.msg" NAME_WE)
add_dependencies(pharos_path_planner_generate_messages_nodejs _pharos_path_planner_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(pharos_path_planner_gennodejs)
add_dependencies(pharos_path_planner_gennodejs pharos_path_planner_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS pharos_path_planner_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(pharos_path_planner
  "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/ReferencePath.msg"
  "${MSG_I_FLAGS}"
  "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/Position.msg;/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/RoadInfo.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pharos_path_planner
)
_generate_msg_py(pharos_path_planner
  "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/ReferencePath2.msg"
  "${MSG_I_FLAGS}"
  "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/Position.msg;/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/RoadInfo.msg;/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/LaneInfo.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pharos_path_planner
)
_generate_msg_py(pharos_path_planner
  "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/Position.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pharos_path_planner
)
_generate_msg_py(pharos_path_planner
  "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/RoadInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/Position.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pharos_path_planner
)
_generate_msg_py(pharos_path_planner
  "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/LaneInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/Position.msg;/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/RoadInfo.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pharos_path_planner
)

### Generating Services

### Generating Module File
_generate_module_py(pharos_path_planner
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pharos_path_planner
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(pharos_path_planner_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(pharos_path_planner_generate_messages pharos_path_planner_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/ReferencePath.msg" NAME_WE)
add_dependencies(pharos_path_planner_generate_messages_py _pharos_path_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/ReferencePath2.msg" NAME_WE)
add_dependencies(pharos_path_planner_generate_messages_py _pharos_path_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/Position.msg" NAME_WE)
add_dependencies(pharos_path_planner_generate_messages_py _pharos_path_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/RoadInfo.msg" NAME_WE)
add_dependencies(pharos_path_planner_generate_messages_py _pharos_path_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pharos-main/i30_ws/src/pharos/pharos_path_planner/msg/LaneInfo.msg" NAME_WE)
add_dependencies(pharos_path_planner_generate_messages_py _pharos_path_planner_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(pharos_path_planner_genpy)
add_dependencies(pharos_path_planner_genpy pharos_path_planner_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS pharos_path_planner_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pharos_path_planner)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pharos_path_planner
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET tf_generate_messages_cpp)
  add_dependencies(pharos_path_planner_generate_messages_cpp tf_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pharos_path_planner)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pharos_path_planner
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET tf_generate_messages_eus)
  add_dependencies(pharos_path_planner_generate_messages_eus tf_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pharos_path_planner)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pharos_path_planner
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET tf_generate_messages_lisp)
  add_dependencies(pharos_path_planner_generate_messages_lisp tf_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pharos_path_planner)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pharos_path_planner
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET tf_generate_messages_nodejs)
  add_dependencies(pharos_path_planner_generate_messages_nodejs tf_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pharos_path_planner)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pharos_path_planner\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pharos_path_planner
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET tf_generate_messages_py)
  add_dependencies(pharos_path_planner_generate_messages_py tf_generate_messages_py)
endif()
