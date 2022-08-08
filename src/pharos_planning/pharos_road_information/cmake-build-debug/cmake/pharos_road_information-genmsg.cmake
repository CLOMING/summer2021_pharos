# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "pharos_road_information: 7 messages, 0 services")

set(MSG_I_FLAGS "-Ipharos_road_information:/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg;-Inav_msgs:/opt/ros/kinetic/share/nav_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(pharos_road_information_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Workzone_xy.msg" NAME_WE)
add_custom_target(_pharos_road_information_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pharos_road_information" "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Workzone_xy.msg" ""
)

get_filename_component(_filename "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Workzone_list.msg" NAME_WE)
add_custom_target(_pharos_road_information_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pharos_road_information" "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Workzone_list.msg" "pharos_road_information/Workzone_xy"
)

get_filename_component(_filename "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Road.msg" NAME_WE)
add_custom_target(_pharos_road_information_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pharos_road_information" "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Road.msg" "pharos_road_information/Lane:pharos_road_information/Waypoint"
)

get_filename_component(_filename "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Waypoint.msg" NAME_WE)
add_custom_target(_pharos_road_information_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pharos_road_information" "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Waypoint.msg" ""
)

get_filename_component(_filename "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Lane.msg" NAME_WE)
add_custom_target(_pharos_road_information_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pharos_road_information" "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Lane.msg" "pharos_road_information/Waypoint"
)

get_filename_component(_filename "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Lanes.msg" NAME_WE)
add_custom_target(_pharos_road_information_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pharos_road_information" "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Lanes.msg" "pharos_road_information/Lane:pharos_road_information/Waypoint"
)

get_filename_component(_filename "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/RoadNetworks.msg" NAME_WE)
add_custom_target(_pharos_road_information_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pharos_road_information" "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/RoadNetworks.msg" "pharos_road_information/Road:pharos_road_information/Lane:pharos_road_information/Waypoint"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(pharos_road_information
  "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Workzone_xy.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pharos_road_information
)
_generate_msg_cpp(pharos_road_information
  "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Workzone_list.msg"
  "${MSG_I_FLAGS}"
  "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Workzone_xy.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pharos_road_information
)
_generate_msg_cpp(pharos_road_information
  "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Road.msg"
  "${MSG_I_FLAGS}"
  "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Lane.msg;/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Waypoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pharos_road_information
)
_generate_msg_cpp(pharos_road_information
  "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Waypoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pharos_road_information
)
_generate_msg_cpp(pharos_road_information
  "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Lane.msg"
  "${MSG_I_FLAGS}"
  "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Waypoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pharos_road_information
)
_generate_msg_cpp(pharos_road_information
  "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Lanes.msg"
  "${MSG_I_FLAGS}"
  "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Lane.msg;/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Waypoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pharos_road_information
)
_generate_msg_cpp(pharos_road_information
  "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/RoadNetworks.msg"
  "${MSG_I_FLAGS}"
  "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Road.msg;/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Lane.msg;/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Waypoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pharos_road_information
)

### Generating Services

### Generating Module File
_generate_module_cpp(pharos_road_information
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pharos_road_information
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(pharos_road_information_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(pharos_road_information_generate_messages pharos_road_information_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Workzone_xy.msg" NAME_WE)
add_dependencies(pharos_road_information_generate_messages_cpp _pharos_road_information_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Workzone_list.msg" NAME_WE)
add_dependencies(pharos_road_information_generate_messages_cpp _pharos_road_information_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Road.msg" NAME_WE)
add_dependencies(pharos_road_information_generate_messages_cpp _pharos_road_information_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Waypoint.msg" NAME_WE)
add_dependencies(pharos_road_information_generate_messages_cpp _pharos_road_information_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Lane.msg" NAME_WE)
add_dependencies(pharos_road_information_generate_messages_cpp _pharos_road_information_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Lanes.msg" NAME_WE)
add_dependencies(pharos_road_information_generate_messages_cpp _pharos_road_information_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/RoadNetworks.msg" NAME_WE)
add_dependencies(pharos_road_information_generate_messages_cpp _pharos_road_information_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(pharos_road_information_gencpp)
add_dependencies(pharos_road_information_gencpp pharos_road_information_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS pharos_road_information_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(pharos_road_information
  "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Workzone_xy.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pharos_road_information
)
_generate_msg_eus(pharos_road_information
  "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Workzone_list.msg"
  "${MSG_I_FLAGS}"
  "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Workzone_xy.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pharos_road_information
)
_generate_msg_eus(pharos_road_information
  "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Road.msg"
  "${MSG_I_FLAGS}"
  "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Lane.msg;/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Waypoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pharos_road_information
)
_generate_msg_eus(pharos_road_information
  "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Waypoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pharos_road_information
)
_generate_msg_eus(pharos_road_information
  "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Lane.msg"
  "${MSG_I_FLAGS}"
  "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Waypoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pharos_road_information
)
_generate_msg_eus(pharos_road_information
  "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Lanes.msg"
  "${MSG_I_FLAGS}"
  "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Lane.msg;/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Waypoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pharos_road_information
)
_generate_msg_eus(pharos_road_information
  "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/RoadNetworks.msg"
  "${MSG_I_FLAGS}"
  "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Road.msg;/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Lane.msg;/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Waypoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pharos_road_information
)

### Generating Services

### Generating Module File
_generate_module_eus(pharos_road_information
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pharos_road_information
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(pharos_road_information_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(pharos_road_information_generate_messages pharos_road_information_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Workzone_xy.msg" NAME_WE)
add_dependencies(pharos_road_information_generate_messages_eus _pharos_road_information_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Workzone_list.msg" NAME_WE)
add_dependencies(pharos_road_information_generate_messages_eus _pharos_road_information_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Road.msg" NAME_WE)
add_dependencies(pharos_road_information_generate_messages_eus _pharos_road_information_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Waypoint.msg" NAME_WE)
add_dependencies(pharos_road_information_generate_messages_eus _pharos_road_information_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Lane.msg" NAME_WE)
add_dependencies(pharos_road_information_generate_messages_eus _pharos_road_information_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Lanes.msg" NAME_WE)
add_dependencies(pharos_road_information_generate_messages_eus _pharos_road_information_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/RoadNetworks.msg" NAME_WE)
add_dependencies(pharos_road_information_generate_messages_eus _pharos_road_information_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(pharos_road_information_geneus)
add_dependencies(pharos_road_information_geneus pharos_road_information_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS pharos_road_information_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(pharos_road_information
  "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Workzone_xy.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pharos_road_information
)
_generate_msg_lisp(pharos_road_information
  "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Workzone_list.msg"
  "${MSG_I_FLAGS}"
  "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Workzone_xy.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pharos_road_information
)
_generate_msg_lisp(pharos_road_information
  "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Road.msg"
  "${MSG_I_FLAGS}"
  "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Lane.msg;/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Waypoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pharos_road_information
)
_generate_msg_lisp(pharos_road_information
  "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Waypoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pharos_road_information
)
_generate_msg_lisp(pharos_road_information
  "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Lane.msg"
  "${MSG_I_FLAGS}"
  "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Waypoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pharos_road_information
)
_generate_msg_lisp(pharos_road_information
  "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Lanes.msg"
  "${MSG_I_FLAGS}"
  "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Lane.msg;/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Waypoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pharos_road_information
)
_generate_msg_lisp(pharos_road_information
  "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/RoadNetworks.msg"
  "${MSG_I_FLAGS}"
  "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Road.msg;/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Lane.msg;/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Waypoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pharos_road_information
)

### Generating Services

### Generating Module File
_generate_module_lisp(pharos_road_information
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pharos_road_information
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(pharos_road_information_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(pharos_road_information_generate_messages pharos_road_information_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Workzone_xy.msg" NAME_WE)
add_dependencies(pharos_road_information_generate_messages_lisp _pharos_road_information_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Workzone_list.msg" NAME_WE)
add_dependencies(pharos_road_information_generate_messages_lisp _pharos_road_information_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Road.msg" NAME_WE)
add_dependencies(pharos_road_information_generate_messages_lisp _pharos_road_information_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Waypoint.msg" NAME_WE)
add_dependencies(pharos_road_information_generate_messages_lisp _pharos_road_information_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Lane.msg" NAME_WE)
add_dependencies(pharos_road_information_generate_messages_lisp _pharos_road_information_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Lanes.msg" NAME_WE)
add_dependencies(pharos_road_information_generate_messages_lisp _pharos_road_information_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/RoadNetworks.msg" NAME_WE)
add_dependencies(pharos_road_information_generate_messages_lisp _pharos_road_information_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(pharos_road_information_genlisp)
add_dependencies(pharos_road_information_genlisp pharos_road_information_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS pharos_road_information_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(pharos_road_information
  "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Workzone_xy.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pharos_road_information
)
_generate_msg_nodejs(pharos_road_information
  "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Workzone_list.msg"
  "${MSG_I_FLAGS}"
  "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Workzone_xy.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pharos_road_information
)
_generate_msg_nodejs(pharos_road_information
  "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Road.msg"
  "${MSG_I_FLAGS}"
  "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Lane.msg;/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Waypoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pharos_road_information
)
_generate_msg_nodejs(pharos_road_information
  "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Waypoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pharos_road_information
)
_generate_msg_nodejs(pharos_road_information
  "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Lane.msg"
  "${MSG_I_FLAGS}"
  "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Waypoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pharos_road_information
)
_generate_msg_nodejs(pharos_road_information
  "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Lanes.msg"
  "${MSG_I_FLAGS}"
  "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Lane.msg;/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Waypoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pharos_road_information
)
_generate_msg_nodejs(pharos_road_information
  "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/RoadNetworks.msg"
  "${MSG_I_FLAGS}"
  "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Road.msg;/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Lane.msg;/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Waypoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pharos_road_information
)

### Generating Services

### Generating Module File
_generate_module_nodejs(pharos_road_information
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pharos_road_information
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(pharos_road_information_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(pharos_road_information_generate_messages pharos_road_information_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Workzone_xy.msg" NAME_WE)
add_dependencies(pharos_road_information_generate_messages_nodejs _pharos_road_information_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Workzone_list.msg" NAME_WE)
add_dependencies(pharos_road_information_generate_messages_nodejs _pharos_road_information_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Road.msg" NAME_WE)
add_dependencies(pharos_road_information_generate_messages_nodejs _pharos_road_information_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Waypoint.msg" NAME_WE)
add_dependencies(pharos_road_information_generate_messages_nodejs _pharos_road_information_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Lane.msg" NAME_WE)
add_dependencies(pharos_road_information_generate_messages_nodejs _pharos_road_information_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Lanes.msg" NAME_WE)
add_dependencies(pharos_road_information_generate_messages_nodejs _pharos_road_information_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/RoadNetworks.msg" NAME_WE)
add_dependencies(pharos_road_information_generate_messages_nodejs _pharos_road_information_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(pharos_road_information_gennodejs)
add_dependencies(pharos_road_information_gennodejs pharos_road_information_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS pharos_road_information_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(pharos_road_information
  "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Workzone_xy.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pharos_road_information
)
_generate_msg_py(pharos_road_information
  "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Workzone_list.msg"
  "${MSG_I_FLAGS}"
  "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Workzone_xy.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pharos_road_information
)
_generate_msg_py(pharos_road_information
  "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Road.msg"
  "${MSG_I_FLAGS}"
  "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Lane.msg;/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Waypoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pharos_road_information
)
_generate_msg_py(pharos_road_information
  "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Waypoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pharos_road_information
)
_generate_msg_py(pharos_road_information
  "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Lane.msg"
  "${MSG_I_FLAGS}"
  "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Waypoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pharos_road_information
)
_generate_msg_py(pharos_road_information
  "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Lanes.msg"
  "${MSG_I_FLAGS}"
  "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Lane.msg;/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Waypoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pharos_road_information
)
_generate_msg_py(pharos_road_information
  "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/RoadNetworks.msg"
  "${MSG_I_FLAGS}"
  "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Road.msg;/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Lane.msg;/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Waypoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pharos_road_information
)

### Generating Services

### Generating Module File
_generate_module_py(pharos_road_information
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pharos_road_information
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(pharos_road_information_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(pharos_road_information_generate_messages pharos_road_information_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Workzone_xy.msg" NAME_WE)
add_dependencies(pharos_road_information_generate_messages_py _pharos_road_information_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Workzone_list.msg" NAME_WE)
add_dependencies(pharos_road_information_generate_messages_py _pharos_road_information_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Road.msg" NAME_WE)
add_dependencies(pharos_road_information_generate_messages_py _pharos_road_information_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Waypoint.msg" NAME_WE)
add_dependencies(pharos_road_information_generate_messages_py _pharos_road_information_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Lane.msg" NAME_WE)
add_dependencies(pharos_road_information_generate_messages_py _pharos_road_information_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/Lanes.msg" NAME_WE)
add_dependencies(pharos_road_information_generate_messages_py _pharos_road_information_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pharos-main/i30_ws/src/pharos/pharos_road_information/msg/RoadNetworks.msg" NAME_WE)
add_dependencies(pharos_road_information_generate_messages_py _pharos_road_information_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(pharos_road_information_genpy)
add_dependencies(pharos_road_information_genpy pharos_road_information_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS pharos_road_information_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pharos_road_information)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pharos_road_information
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(pharos_road_information_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET nav_msgs_generate_messages_cpp)
  add_dependencies(pharos_road_information_generate_messages_cpp nav_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pharos_road_information)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pharos_road_information
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(pharos_road_information_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET nav_msgs_generate_messages_eus)
  add_dependencies(pharos_road_information_generate_messages_eus nav_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pharos_road_information)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pharos_road_information
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(pharos_road_information_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET nav_msgs_generate_messages_lisp)
  add_dependencies(pharos_road_information_generate_messages_lisp nav_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pharos_road_information)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pharos_road_information
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(pharos_road_information_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET nav_msgs_generate_messages_nodejs)
  add_dependencies(pharos_road_information_generate_messages_nodejs nav_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pharos_road_information)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pharos_road_information\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pharos_road_information
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(pharos_road_information_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET nav_msgs_generate_messages_py)
  add_dependencies(pharos_road_information_generate_messages_py nav_msgs_generate_messages_py)
endif()
