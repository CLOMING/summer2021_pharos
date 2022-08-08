# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "wave_pkg: 39 messages, 0 services")

set(MSG_I_FLAGS "-Iwave_pkg:/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(wave_pkg_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeListLL_t.msg" NAME_WE)
add_custom_target(_wave_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "wave_pkg" "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeListLL_t.msg" ""
)

get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeListXY_t.msg" NAME_WE)
add_custom_target(_wave_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "wave_pkg" "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeListXY_t.msg" "wave_pkg/NodeAttributeSetXY:wave_pkg/NodeOffsetPointXY_t"
)

get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/Position3D.msg" NAME_WE)
add_custom_target(_wave_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "wave_pkg" "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/Position3D.msg" ""
)

get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/MovementList_t.msg" NAME_WE)
add_custom_target(_wave_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "wave_pkg" "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/MovementList_t.msg" "wave_pkg/TimeChangeDetails:wave_pkg/MovementEventList_t"
)

get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/SupplementalVehicleExtensions_t.msg" NAME_WE)
add_custom_target(_wave_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "wave_pkg" "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/SupplementalVehicleExtensions_t.msg" ""
)

get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/BSM.msg" NAME_WE)
add_custom_target(_wave_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "wave_pkg" "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/BSM.msg" "wave_pkg/PathHistory_t:wave_pkg/VehicleSize_t:wave_pkg/VehicleSafetyExtensions_t:wave_pkg/BrakeSystemStatus_t:wave_pkg/BSMcoreData:wave_pkg/AccelerationSet4Way_t:wave_pkg/PositionalAccuracy_t:wave_pkg/SpecialVehicleExtensions_t:wave_pkg/BSMpartII:wave_pkg/SupplementalVehicleExtensions_t"
)

get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/VehicleSize_t.msg" NAME_WE)
add_custom_target(_wave_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "wave_pkg" "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/VehicleSize_t.msg" ""
)

get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/RTCM.msg" NAME_WE)
add_custom_target(_wave_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "wave_pkg" "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/RTCM.msg" "wave_pkg/RTCMmessageList_t:wave_pkg/FullPositionVector:wave_pkg/AntennaOffsetSet_t:wave_pkg/RTCMheader"
)

get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/SPAT.msg" NAME_WE)
add_custom_target(_wave_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "wave_pkg" "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/SPAT.msg" "wave_pkg/MovementList_t:wave_pkg/TimeChangeDetails:wave_pkg/IntersectionReferenceID_t:wave_pkg/MovementEventList_t:wave_pkg/IntersectionStateList_t"
)

get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/PositionalAccuracy_t.msg" NAME_WE)
add_custom_target(_wave_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "wave_pkg" "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/PositionalAccuracy_t.msg" ""
)

get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/MAP.msg" NAME_WE)
add_custom_target(_wave_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "wave_pkg" "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/MAP.msg" "wave_pkg/ConnectsToList:wave_pkg/NodeOffsetPointXY_t:wave_pkg/LaneList_t:wave_pkg/IntersectionReferenceID:wave_pkg/NodeAttributeSetXY:wave_pkg/LaneAttributes_t:wave_pkg/IntersectionGeometryList:wave_pkg/Position3D_t:wave_pkg/NodeListXY_t"
)

get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/BSMpartII.msg" NAME_WE)
add_custom_target(_wave_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "wave_pkg" "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/BSMpartII.msg" "wave_pkg/PathHistory_t:wave_pkg/VehicleSafetyExtensions_t:wave_pkg/SpecialVehicleExtensions_t:wave_pkg/SupplementalVehicleExtensions_t"
)

get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/BSMcoreData.msg" NAME_WE)
add_custom_target(_wave_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "wave_pkg" "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/BSMcoreData.msg" "wave_pkg/VehicleSize_t:wave_pkg/BrakeSystemStatus_t:wave_pkg/PositionalAccuracy_t:wave_pkg/AccelerationSet4Way_t"
)

get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionStateList_t.msg" NAME_WE)
add_custom_target(_wave_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "wave_pkg" "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionStateList_t.msg" "wave_pkg/TimeChangeDetails:wave_pkg/IntersectionReferenceID_t:wave_pkg/MovementEventList_t:wave_pkg/MovementList_t"
)

get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TimeChangeDetails.msg" NAME_WE)
add_custom_target(_wave_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "wave_pkg" "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TimeChangeDetails.msg" ""
)

get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/FullPositionVector.msg" NAME_WE)
add_custom_target(_wave_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "wave_pkg" "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/FullPositionVector.msg" ""
)

get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/GeographicalPath.msg" NAME_WE)
add_custom_target(_wave_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "wave_pkg" "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/GeographicalPath.msg" "wave_pkg/NodeListLL_t:wave_pkg/OffsetSystem_t:wave_pkg/Position3D"
)

get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/LaneList_t.msg" NAME_WE)
add_custom_target(_wave_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "wave_pkg" "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/LaneList_t.msg" "wave_pkg/NodeAttributeSetXY:wave_pkg/ConnectsToList:wave_pkg/NodeOffsetPointXY_t:wave_pkg/LaneAttributes_t:wave_pkg/NodeListXY_t"
)

get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/LaneAttributes_t.msg" NAME_WE)
add_custom_target(_wave_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "wave_pkg" "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/LaneAttributes_t.msg" ""
)

get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionReferenceID_t.msg" NAME_WE)
add_custom_target(_wave_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "wave_pkg" "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionReferenceID_t.msg" ""
)

get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeOffsetPointXY_t.msg" NAME_WE)
add_custom_target(_wave_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "wave_pkg" "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeOffsetPointXY_t.msg" ""
)

get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/Position3D_t.msg" NAME_WE)
add_custom_target(_wave_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "wave_pkg" "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/Position3D_t.msg" ""
)

get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/MovementEventList_t.msg" NAME_WE)
add_custom_target(_wave_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "wave_pkg" "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/MovementEventList_t.msg" "wave_pkg/TimeChangeDetails"
)

get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/ConnectsToList.msg" NAME_WE)
add_custom_target(_wave_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "wave_pkg" "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/ConnectsToList.msg" ""
)

get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TravelerDataFrameList_t.msg" NAME_WE)
add_custom_target(_wave_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "wave_pkg" "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TravelerDataFrameList_t.msg" "wave_pkg/NodeListLL_t:wave_pkg/TravelerDataFrame__content_PR:wave_pkg/OffsetSystem_t:wave_pkg/Position3D:wave_pkg/GeographicalPath"
)

get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/RTCMheader.msg" NAME_WE)
add_custom_target(_wave_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "wave_pkg" "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/RTCMheader.msg" "wave_pkg/AntennaOffsetSet_t"
)

get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TravelerDataFrame__content_PR.msg" NAME_WE)
add_custom_target(_wave_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "wave_pkg" "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TravelerDataFrame__content_PR.msg" ""
)

get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/VehicleSafetyExtensions_t.msg" NAME_WE)
add_custom_target(_wave_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "wave_pkg" "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/VehicleSafetyExtensions_t.msg" "wave_pkg/PathHistory_t"
)

get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/AccelerationSet4Way_t.msg" NAME_WE)
add_custom_target(_wave_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "wave_pkg" "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/AccelerationSet4Way_t.msg" ""
)

get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/PathHistory_t.msg" NAME_WE)
add_custom_target(_wave_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "wave_pkg" "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/PathHistory_t.msg" ""
)

get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionGeometryList.msg" NAME_WE)
add_custom_target(_wave_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "wave_pkg" "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionGeometryList.msg" "wave_pkg/ConnectsToList:wave_pkg/LaneList_t:wave_pkg/NodeOffsetPointXY_t:wave_pkg/IntersectionReferenceID:wave_pkg/LaneAttributes_t:wave_pkg/NodeAttributeSetXY:wave_pkg/Position3D_t:wave_pkg/NodeListXY_t"
)

get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeAttributeSetXY.msg" NAME_WE)
add_custom_target(_wave_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "wave_pkg" "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeAttributeSetXY.msg" ""
)

get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/RTCMmessageList_t.msg" NAME_WE)
add_custom_target(_wave_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "wave_pkg" "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/RTCMmessageList_t.msg" ""
)

get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionReferenceID.msg" NAME_WE)
add_custom_target(_wave_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "wave_pkg" "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionReferenceID.msg" ""
)

get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/OffsetSystem_t.msg" NAME_WE)
add_custom_target(_wave_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "wave_pkg" "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/OffsetSystem_t.msg" "wave_pkg/NodeListLL_t"
)

get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/BrakeSystemStatus_t.msg" NAME_WE)
add_custom_target(_wave_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "wave_pkg" "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/BrakeSystemStatus_t.msg" ""
)

get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/SpecialVehicleExtensions_t.msg" NAME_WE)
add_custom_target(_wave_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "wave_pkg" "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/SpecialVehicleExtensions_t.msg" ""
)

get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/AntennaOffsetSet_t.msg" NAME_WE)
add_custom_target(_wave_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "wave_pkg" "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/AntennaOffsetSet_t.msg" ""
)

get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TIM.msg" NAME_WE)
add_custom_target(_wave_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "wave_pkg" "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TIM.msg" "wave_pkg/TravelerDataFrame__content_PR:wave_pkg/OffsetSystem_t:wave_pkg/NodeListLL_t:wave_pkg/GeographicalPath:wave_pkg/Position3D:wave_pkg/TravelerDataFrameList_t"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeListLL_t.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/wave_pkg
)
_generate_msg_cpp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeListXY_t.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeAttributeSetXY.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeOffsetPointXY_t.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/wave_pkg
)
_generate_msg_cpp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/BSMcoreData.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/VehicleSize_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/BrakeSystemStatus_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/PositionalAccuracy_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/AccelerationSet4Way_t.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/wave_pkg
)
_generate_msg_cpp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/MovementList_t.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TimeChangeDetails.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/MovementEventList_t.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/wave_pkg
)
_generate_msg_cpp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/RTCMheader.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/AntennaOffsetSet_t.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/wave_pkg
)
_generate_msg_cpp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/VehicleSize_t.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/wave_pkg
)
_generate_msg_cpp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/RTCM.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/RTCMmessageList_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/FullPositionVector.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/AntennaOffsetSet_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/RTCMheader.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/wave_pkg
)
_generate_msg_cpp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/SPAT.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/MovementList_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TimeChangeDetails.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionReferenceID_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/MovementEventList_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionStateList_t.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/wave_pkg
)
_generate_msg_cpp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/PositionalAccuracy_t.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/wave_pkg
)
_generate_msg_cpp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/AccelerationSet4Way_t.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/wave_pkg
)
_generate_msg_cpp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/BSMpartII.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/PathHistory_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/VehicleSafetyExtensions_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/SpecialVehicleExtensions_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/SupplementalVehicleExtensions_t.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/wave_pkg
)
_generate_msg_cpp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/Position3D.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/wave_pkg
)
_generate_msg_cpp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionStateList_t.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TimeChangeDetails.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionReferenceID_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/MovementEventList_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/MovementList_t.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/wave_pkg
)
_generate_msg_cpp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeOffsetPointXY_t.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/wave_pkg
)
_generate_msg_cpp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/Position3D_t.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/wave_pkg
)
_generate_msg_cpp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TimeChangeDetails.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/wave_pkg
)
_generate_msg_cpp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/FullPositionVector.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/wave_pkg
)
_generate_msg_cpp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/LaneList_t.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeAttributeSetXY.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/ConnectsToList.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeOffsetPointXY_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/LaneAttributes_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeListXY_t.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/wave_pkg
)
_generate_msg_cpp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionReferenceID.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/wave_pkg
)
_generate_msg_cpp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionReferenceID_t.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/wave_pkg
)
_generate_msg_cpp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/LaneAttributes_t.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/wave_pkg
)
_generate_msg_cpp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/MovementEventList_t.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TimeChangeDetails.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/wave_pkg
)
_generate_msg_cpp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionGeometryList.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/ConnectsToList.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/LaneList_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeOffsetPointXY_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionReferenceID.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/LaneAttributes_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeAttributeSetXY.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/Position3D_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeListXY_t.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/wave_pkg
)
_generate_msg_cpp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/ConnectsToList.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/wave_pkg
)
_generate_msg_cpp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TravelerDataFrameList_t.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeListLL_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TravelerDataFrame__content_PR.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/OffsetSystem_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/Position3D.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/GeographicalPath.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/wave_pkg
)
_generate_msg_cpp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/BSM.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/PathHistory_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/VehicleSize_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/VehicleSafetyExtensions_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/BrakeSystemStatus_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/BSMcoreData.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/AccelerationSet4Way_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/PositionalAccuracy_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/SpecialVehicleExtensions_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/BSMpartII.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/SupplementalVehicleExtensions_t.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/wave_pkg
)
_generate_msg_cpp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TravelerDataFrame__content_PR.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/wave_pkg
)
_generate_msg_cpp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/VehicleSafetyExtensions_t.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/PathHistory_t.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/wave_pkg
)
_generate_msg_cpp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/MAP.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/ConnectsToList.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeOffsetPointXY_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/LaneList_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionReferenceID.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeAttributeSetXY.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/LaneAttributes_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionGeometryList.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/Position3D_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeListXY_t.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/wave_pkg
)
_generate_msg_cpp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/RTCMmessageList_t.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/wave_pkg
)
_generate_msg_cpp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeAttributeSetXY.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/wave_pkg
)
_generate_msg_cpp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/PathHistory_t.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/wave_pkg
)
_generate_msg_cpp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/SupplementalVehicleExtensions_t.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/wave_pkg
)
_generate_msg_cpp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/OffsetSystem_t.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeListLL_t.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/wave_pkg
)
_generate_msg_cpp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TIM.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TravelerDataFrame__content_PR.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/OffsetSystem_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeListLL_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/GeographicalPath.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/Position3D.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TravelerDataFrameList_t.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/wave_pkg
)
_generate_msg_cpp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/BrakeSystemStatus_t.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/wave_pkg
)
_generate_msg_cpp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/SpecialVehicleExtensions_t.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/wave_pkg
)
_generate_msg_cpp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/AntennaOffsetSet_t.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/wave_pkg
)
_generate_msg_cpp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/GeographicalPath.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeListLL_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/OffsetSystem_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/Position3D.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/wave_pkg
)

### Generating Services

### Generating Module File
_generate_module_cpp(wave_pkg
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/wave_pkg
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(wave_pkg_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(wave_pkg_generate_messages wave_pkg_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeListLL_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_cpp _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeListXY_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_cpp _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/Position3D.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_cpp _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/MovementList_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_cpp _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/SupplementalVehicleExtensions_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_cpp _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/BSM.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_cpp _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/VehicleSize_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_cpp _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/RTCM.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_cpp _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/SPAT.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_cpp _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/PositionalAccuracy_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_cpp _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/MAP.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_cpp _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/BSMpartII.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_cpp _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/BSMcoreData.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_cpp _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionStateList_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_cpp _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TimeChangeDetails.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_cpp _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/FullPositionVector.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_cpp _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/GeographicalPath.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_cpp _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/LaneList_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_cpp _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/LaneAttributes_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_cpp _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionReferenceID_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_cpp _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeOffsetPointXY_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_cpp _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/Position3D_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_cpp _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/MovementEventList_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_cpp _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/ConnectsToList.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_cpp _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TravelerDataFrameList_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_cpp _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/RTCMheader.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_cpp _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TravelerDataFrame__content_PR.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_cpp _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/VehicleSafetyExtensions_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_cpp _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/AccelerationSet4Way_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_cpp _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/PathHistory_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_cpp _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionGeometryList.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_cpp _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeAttributeSetXY.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_cpp _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/RTCMmessageList_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_cpp _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionReferenceID.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_cpp _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/OffsetSystem_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_cpp _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/BrakeSystemStatus_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_cpp _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/SpecialVehicleExtensions_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_cpp _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/AntennaOffsetSet_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_cpp _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TIM.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_cpp _wave_pkg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(wave_pkg_gencpp)
add_dependencies(wave_pkg_gencpp wave_pkg_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS wave_pkg_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeListLL_t.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/wave_pkg
)
_generate_msg_eus(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeListXY_t.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeAttributeSetXY.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeOffsetPointXY_t.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/wave_pkg
)
_generate_msg_eus(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/BSMcoreData.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/VehicleSize_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/BrakeSystemStatus_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/PositionalAccuracy_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/AccelerationSet4Way_t.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/wave_pkg
)
_generate_msg_eus(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/MovementList_t.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TimeChangeDetails.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/MovementEventList_t.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/wave_pkg
)
_generate_msg_eus(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/RTCMheader.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/AntennaOffsetSet_t.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/wave_pkg
)
_generate_msg_eus(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/VehicleSize_t.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/wave_pkg
)
_generate_msg_eus(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/RTCM.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/RTCMmessageList_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/FullPositionVector.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/AntennaOffsetSet_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/RTCMheader.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/wave_pkg
)
_generate_msg_eus(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/SPAT.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/MovementList_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TimeChangeDetails.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionReferenceID_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/MovementEventList_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionStateList_t.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/wave_pkg
)
_generate_msg_eus(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/PositionalAccuracy_t.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/wave_pkg
)
_generate_msg_eus(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/AccelerationSet4Way_t.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/wave_pkg
)
_generate_msg_eus(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/BSMpartII.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/PathHistory_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/VehicleSafetyExtensions_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/SpecialVehicleExtensions_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/SupplementalVehicleExtensions_t.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/wave_pkg
)
_generate_msg_eus(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/Position3D.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/wave_pkg
)
_generate_msg_eus(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionStateList_t.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TimeChangeDetails.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionReferenceID_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/MovementEventList_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/MovementList_t.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/wave_pkg
)
_generate_msg_eus(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeOffsetPointXY_t.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/wave_pkg
)
_generate_msg_eus(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/Position3D_t.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/wave_pkg
)
_generate_msg_eus(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TimeChangeDetails.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/wave_pkg
)
_generate_msg_eus(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/FullPositionVector.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/wave_pkg
)
_generate_msg_eus(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/LaneList_t.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeAttributeSetXY.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/ConnectsToList.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeOffsetPointXY_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/LaneAttributes_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeListXY_t.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/wave_pkg
)
_generate_msg_eus(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionReferenceID.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/wave_pkg
)
_generate_msg_eus(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionReferenceID_t.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/wave_pkg
)
_generate_msg_eus(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/LaneAttributes_t.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/wave_pkg
)
_generate_msg_eus(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/MovementEventList_t.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TimeChangeDetails.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/wave_pkg
)
_generate_msg_eus(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionGeometryList.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/ConnectsToList.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/LaneList_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeOffsetPointXY_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionReferenceID.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/LaneAttributes_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeAttributeSetXY.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/Position3D_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeListXY_t.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/wave_pkg
)
_generate_msg_eus(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/ConnectsToList.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/wave_pkg
)
_generate_msg_eus(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TravelerDataFrameList_t.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeListLL_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TravelerDataFrame__content_PR.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/OffsetSystem_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/Position3D.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/GeographicalPath.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/wave_pkg
)
_generate_msg_eus(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/BSM.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/PathHistory_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/VehicleSize_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/VehicleSafetyExtensions_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/BrakeSystemStatus_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/BSMcoreData.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/AccelerationSet4Way_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/PositionalAccuracy_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/SpecialVehicleExtensions_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/BSMpartII.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/SupplementalVehicleExtensions_t.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/wave_pkg
)
_generate_msg_eus(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TravelerDataFrame__content_PR.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/wave_pkg
)
_generate_msg_eus(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/VehicleSafetyExtensions_t.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/PathHistory_t.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/wave_pkg
)
_generate_msg_eus(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/MAP.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/ConnectsToList.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeOffsetPointXY_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/LaneList_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionReferenceID.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeAttributeSetXY.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/LaneAttributes_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionGeometryList.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/Position3D_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeListXY_t.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/wave_pkg
)
_generate_msg_eus(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/RTCMmessageList_t.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/wave_pkg
)
_generate_msg_eus(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeAttributeSetXY.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/wave_pkg
)
_generate_msg_eus(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/PathHistory_t.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/wave_pkg
)
_generate_msg_eus(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/SupplementalVehicleExtensions_t.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/wave_pkg
)
_generate_msg_eus(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/OffsetSystem_t.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeListLL_t.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/wave_pkg
)
_generate_msg_eus(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TIM.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TravelerDataFrame__content_PR.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/OffsetSystem_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeListLL_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/GeographicalPath.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/Position3D.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TravelerDataFrameList_t.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/wave_pkg
)
_generate_msg_eus(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/BrakeSystemStatus_t.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/wave_pkg
)
_generate_msg_eus(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/SpecialVehicleExtensions_t.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/wave_pkg
)
_generate_msg_eus(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/AntennaOffsetSet_t.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/wave_pkg
)
_generate_msg_eus(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/GeographicalPath.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeListLL_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/OffsetSystem_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/Position3D.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/wave_pkg
)

### Generating Services

### Generating Module File
_generate_module_eus(wave_pkg
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/wave_pkg
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(wave_pkg_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(wave_pkg_generate_messages wave_pkg_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeListLL_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_eus _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeListXY_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_eus _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/Position3D.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_eus _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/MovementList_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_eus _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/SupplementalVehicleExtensions_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_eus _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/BSM.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_eus _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/VehicleSize_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_eus _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/RTCM.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_eus _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/SPAT.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_eus _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/PositionalAccuracy_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_eus _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/MAP.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_eus _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/BSMpartII.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_eus _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/BSMcoreData.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_eus _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionStateList_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_eus _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TimeChangeDetails.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_eus _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/FullPositionVector.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_eus _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/GeographicalPath.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_eus _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/LaneList_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_eus _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/LaneAttributes_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_eus _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionReferenceID_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_eus _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeOffsetPointXY_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_eus _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/Position3D_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_eus _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/MovementEventList_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_eus _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/ConnectsToList.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_eus _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TravelerDataFrameList_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_eus _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/RTCMheader.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_eus _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TravelerDataFrame__content_PR.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_eus _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/VehicleSafetyExtensions_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_eus _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/AccelerationSet4Way_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_eus _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/PathHistory_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_eus _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionGeometryList.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_eus _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeAttributeSetXY.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_eus _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/RTCMmessageList_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_eus _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionReferenceID.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_eus _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/OffsetSystem_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_eus _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/BrakeSystemStatus_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_eus _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/SpecialVehicleExtensions_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_eus _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/AntennaOffsetSet_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_eus _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TIM.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_eus _wave_pkg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(wave_pkg_geneus)
add_dependencies(wave_pkg_geneus wave_pkg_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS wave_pkg_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeListLL_t.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/wave_pkg
)
_generate_msg_lisp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeListXY_t.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeAttributeSetXY.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeOffsetPointXY_t.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/wave_pkg
)
_generate_msg_lisp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/BSMcoreData.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/VehicleSize_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/BrakeSystemStatus_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/PositionalAccuracy_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/AccelerationSet4Way_t.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/wave_pkg
)
_generate_msg_lisp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/MovementList_t.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TimeChangeDetails.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/MovementEventList_t.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/wave_pkg
)
_generate_msg_lisp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/RTCMheader.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/AntennaOffsetSet_t.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/wave_pkg
)
_generate_msg_lisp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/VehicleSize_t.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/wave_pkg
)
_generate_msg_lisp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/RTCM.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/RTCMmessageList_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/FullPositionVector.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/AntennaOffsetSet_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/RTCMheader.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/wave_pkg
)
_generate_msg_lisp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/SPAT.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/MovementList_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TimeChangeDetails.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionReferenceID_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/MovementEventList_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionStateList_t.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/wave_pkg
)
_generate_msg_lisp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/PositionalAccuracy_t.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/wave_pkg
)
_generate_msg_lisp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/AccelerationSet4Way_t.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/wave_pkg
)
_generate_msg_lisp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/BSMpartII.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/PathHistory_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/VehicleSafetyExtensions_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/SpecialVehicleExtensions_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/SupplementalVehicleExtensions_t.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/wave_pkg
)
_generate_msg_lisp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/Position3D.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/wave_pkg
)
_generate_msg_lisp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionStateList_t.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TimeChangeDetails.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionReferenceID_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/MovementEventList_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/MovementList_t.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/wave_pkg
)
_generate_msg_lisp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeOffsetPointXY_t.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/wave_pkg
)
_generate_msg_lisp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/Position3D_t.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/wave_pkg
)
_generate_msg_lisp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TimeChangeDetails.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/wave_pkg
)
_generate_msg_lisp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/FullPositionVector.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/wave_pkg
)
_generate_msg_lisp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/LaneList_t.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeAttributeSetXY.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/ConnectsToList.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeOffsetPointXY_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/LaneAttributes_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeListXY_t.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/wave_pkg
)
_generate_msg_lisp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionReferenceID.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/wave_pkg
)
_generate_msg_lisp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionReferenceID_t.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/wave_pkg
)
_generate_msg_lisp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/LaneAttributes_t.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/wave_pkg
)
_generate_msg_lisp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/MovementEventList_t.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TimeChangeDetails.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/wave_pkg
)
_generate_msg_lisp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionGeometryList.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/ConnectsToList.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/LaneList_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeOffsetPointXY_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionReferenceID.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/LaneAttributes_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeAttributeSetXY.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/Position3D_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeListXY_t.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/wave_pkg
)
_generate_msg_lisp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/ConnectsToList.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/wave_pkg
)
_generate_msg_lisp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TravelerDataFrameList_t.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeListLL_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TravelerDataFrame__content_PR.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/OffsetSystem_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/Position3D.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/GeographicalPath.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/wave_pkg
)
_generate_msg_lisp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/BSM.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/PathHistory_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/VehicleSize_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/VehicleSafetyExtensions_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/BrakeSystemStatus_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/BSMcoreData.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/AccelerationSet4Way_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/PositionalAccuracy_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/SpecialVehicleExtensions_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/BSMpartII.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/SupplementalVehicleExtensions_t.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/wave_pkg
)
_generate_msg_lisp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TravelerDataFrame__content_PR.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/wave_pkg
)
_generate_msg_lisp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/VehicleSafetyExtensions_t.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/PathHistory_t.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/wave_pkg
)
_generate_msg_lisp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/MAP.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/ConnectsToList.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeOffsetPointXY_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/LaneList_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionReferenceID.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeAttributeSetXY.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/LaneAttributes_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionGeometryList.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/Position3D_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeListXY_t.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/wave_pkg
)
_generate_msg_lisp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/RTCMmessageList_t.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/wave_pkg
)
_generate_msg_lisp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeAttributeSetXY.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/wave_pkg
)
_generate_msg_lisp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/PathHistory_t.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/wave_pkg
)
_generate_msg_lisp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/SupplementalVehicleExtensions_t.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/wave_pkg
)
_generate_msg_lisp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/OffsetSystem_t.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeListLL_t.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/wave_pkg
)
_generate_msg_lisp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TIM.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TravelerDataFrame__content_PR.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/OffsetSystem_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeListLL_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/GeographicalPath.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/Position3D.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TravelerDataFrameList_t.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/wave_pkg
)
_generate_msg_lisp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/BrakeSystemStatus_t.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/wave_pkg
)
_generate_msg_lisp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/SpecialVehicleExtensions_t.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/wave_pkg
)
_generate_msg_lisp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/AntennaOffsetSet_t.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/wave_pkg
)
_generate_msg_lisp(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/GeographicalPath.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeListLL_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/OffsetSystem_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/Position3D.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/wave_pkg
)

### Generating Services

### Generating Module File
_generate_module_lisp(wave_pkg
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/wave_pkg
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(wave_pkg_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(wave_pkg_generate_messages wave_pkg_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeListLL_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_lisp _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeListXY_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_lisp _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/Position3D.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_lisp _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/MovementList_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_lisp _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/SupplementalVehicleExtensions_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_lisp _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/BSM.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_lisp _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/VehicleSize_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_lisp _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/RTCM.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_lisp _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/SPAT.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_lisp _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/PositionalAccuracy_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_lisp _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/MAP.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_lisp _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/BSMpartII.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_lisp _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/BSMcoreData.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_lisp _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionStateList_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_lisp _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TimeChangeDetails.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_lisp _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/FullPositionVector.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_lisp _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/GeographicalPath.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_lisp _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/LaneList_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_lisp _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/LaneAttributes_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_lisp _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionReferenceID_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_lisp _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeOffsetPointXY_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_lisp _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/Position3D_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_lisp _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/MovementEventList_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_lisp _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/ConnectsToList.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_lisp _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TravelerDataFrameList_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_lisp _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/RTCMheader.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_lisp _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TravelerDataFrame__content_PR.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_lisp _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/VehicleSafetyExtensions_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_lisp _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/AccelerationSet4Way_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_lisp _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/PathHistory_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_lisp _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionGeometryList.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_lisp _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeAttributeSetXY.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_lisp _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/RTCMmessageList_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_lisp _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionReferenceID.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_lisp _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/OffsetSystem_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_lisp _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/BrakeSystemStatus_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_lisp _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/SpecialVehicleExtensions_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_lisp _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/AntennaOffsetSet_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_lisp _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TIM.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_lisp _wave_pkg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(wave_pkg_genlisp)
add_dependencies(wave_pkg_genlisp wave_pkg_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS wave_pkg_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeListLL_t.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/wave_pkg
)
_generate_msg_nodejs(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeListXY_t.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeAttributeSetXY.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeOffsetPointXY_t.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/wave_pkg
)
_generate_msg_nodejs(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/BSMcoreData.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/VehicleSize_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/BrakeSystemStatus_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/PositionalAccuracy_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/AccelerationSet4Way_t.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/wave_pkg
)
_generate_msg_nodejs(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/MovementList_t.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TimeChangeDetails.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/MovementEventList_t.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/wave_pkg
)
_generate_msg_nodejs(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/RTCMheader.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/AntennaOffsetSet_t.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/wave_pkg
)
_generate_msg_nodejs(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/VehicleSize_t.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/wave_pkg
)
_generate_msg_nodejs(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/RTCM.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/RTCMmessageList_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/FullPositionVector.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/AntennaOffsetSet_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/RTCMheader.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/wave_pkg
)
_generate_msg_nodejs(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/SPAT.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/MovementList_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TimeChangeDetails.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionReferenceID_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/MovementEventList_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionStateList_t.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/wave_pkg
)
_generate_msg_nodejs(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/PositionalAccuracy_t.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/wave_pkg
)
_generate_msg_nodejs(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/AccelerationSet4Way_t.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/wave_pkg
)
_generate_msg_nodejs(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/BSMpartII.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/PathHistory_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/VehicleSafetyExtensions_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/SpecialVehicleExtensions_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/SupplementalVehicleExtensions_t.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/wave_pkg
)
_generate_msg_nodejs(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/Position3D.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/wave_pkg
)
_generate_msg_nodejs(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionStateList_t.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TimeChangeDetails.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionReferenceID_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/MovementEventList_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/MovementList_t.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/wave_pkg
)
_generate_msg_nodejs(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeOffsetPointXY_t.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/wave_pkg
)
_generate_msg_nodejs(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/Position3D_t.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/wave_pkg
)
_generate_msg_nodejs(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TimeChangeDetails.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/wave_pkg
)
_generate_msg_nodejs(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/FullPositionVector.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/wave_pkg
)
_generate_msg_nodejs(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/LaneList_t.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeAttributeSetXY.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/ConnectsToList.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeOffsetPointXY_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/LaneAttributes_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeListXY_t.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/wave_pkg
)
_generate_msg_nodejs(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionReferenceID.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/wave_pkg
)
_generate_msg_nodejs(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionReferenceID_t.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/wave_pkg
)
_generate_msg_nodejs(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/LaneAttributes_t.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/wave_pkg
)
_generate_msg_nodejs(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/MovementEventList_t.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TimeChangeDetails.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/wave_pkg
)
_generate_msg_nodejs(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionGeometryList.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/ConnectsToList.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/LaneList_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeOffsetPointXY_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionReferenceID.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/LaneAttributes_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeAttributeSetXY.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/Position3D_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeListXY_t.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/wave_pkg
)
_generate_msg_nodejs(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/ConnectsToList.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/wave_pkg
)
_generate_msg_nodejs(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TravelerDataFrameList_t.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeListLL_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TravelerDataFrame__content_PR.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/OffsetSystem_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/Position3D.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/GeographicalPath.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/wave_pkg
)
_generate_msg_nodejs(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/BSM.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/PathHistory_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/VehicleSize_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/VehicleSafetyExtensions_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/BrakeSystemStatus_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/BSMcoreData.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/AccelerationSet4Way_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/PositionalAccuracy_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/SpecialVehicleExtensions_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/BSMpartII.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/SupplementalVehicleExtensions_t.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/wave_pkg
)
_generate_msg_nodejs(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TravelerDataFrame__content_PR.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/wave_pkg
)
_generate_msg_nodejs(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/VehicleSafetyExtensions_t.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/PathHistory_t.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/wave_pkg
)
_generate_msg_nodejs(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/MAP.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/ConnectsToList.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeOffsetPointXY_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/LaneList_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionReferenceID.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeAttributeSetXY.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/LaneAttributes_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionGeometryList.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/Position3D_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeListXY_t.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/wave_pkg
)
_generate_msg_nodejs(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/RTCMmessageList_t.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/wave_pkg
)
_generate_msg_nodejs(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeAttributeSetXY.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/wave_pkg
)
_generate_msg_nodejs(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/PathHistory_t.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/wave_pkg
)
_generate_msg_nodejs(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/SupplementalVehicleExtensions_t.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/wave_pkg
)
_generate_msg_nodejs(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/OffsetSystem_t.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeListLL_t.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/wave_pkg
)
_generate_msg_nodejs(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TIM.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TravelerDataFrame__content_PR.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/OffsetSystem_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeListLL_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/GeographicalPath.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/Position3D.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TravelerDataFrameList_t.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/wave_pkg
)
_generate_msg_nodejs(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/BrakeSystemStatus_t.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/wave_pkg
)
_generate_msg_nodejs(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/SpecialVehicleExtensions_t.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/wave_pkg
)
_generate_msg_nodejs(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/AntennaOffsetSet_t.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/wave_pkg
)
_generate_msg_nodejs(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/GeographicalPath.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeListLL_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/OffsetSystem_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/Position3D.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/wave_pkg
)

### Generating Services

### Generating Module File
_generate_module_nodejs(wave_pkg
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/wave_pkg
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(wave_pkg_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(wave_pkg_generate_messages wave_pkg_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeListLL_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_nodejs _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeListXY_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_nodejs _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/Position3D.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_nodejs _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/MovementList_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_nodejs _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/SupplementalVehicleExtensions_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_nodejs _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/BSM.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_nodejs _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/VehicleSize_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_nodejs _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/RTCM.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_nodejs _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/SPAT.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_nodejs _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/PositionalAccuracy_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_nodejs _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/MAP.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_nodejs _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/BSMpartII.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_nodejs _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/BSMcoreData.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_nodejs _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionStateList_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_nodejs _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TimeChangeDetails.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_nodejs _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/FullPositionVector.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_nodejs _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/GeographicalPath.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_nodejs _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/LaneList_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_nodejs _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/LaneAttributes_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_nodejs _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionReferenceID_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_nodejs _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeOffsetPointXY_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_nodejs _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/Position3D_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_nodejs _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/MovementEventList_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_nodejs _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/ConnectsToList.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_nodejs _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TravelerDataFrameList_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_nodejs _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/RTCMheader.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_nodejs _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TravelerDataFrame__content_PR.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_nodejs _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/VehicleSafetyExtensions_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_nodejs _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/AccelerationSet4Way_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_nodejs _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/PathHistory_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_nodejs _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionGeometryList.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_nodejs _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeAttributeSetXY.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_nodejs _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/RTCMmessageList_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_nodejs _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionReferenceID.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_nodejs _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/OffsetSystem_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_nodejs _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/BrakeSystemStatus_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_nodejs _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/SpecialVehicleExtensions_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_nodejs _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/AntennaOffsetSet_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_nodejs _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TIM.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_nodejs _wave_pkg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(wave_pkg_gennodejs)
add_dependencies(wave_pkg_gennodejs wave_pkg_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS wave_pkg_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeListLL_t.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/wave_pkg
)
_generate_msg_py(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeListXY_t.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeAttributeSetXY.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeOffsetPointXY_t.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/wave_pkg
)
_generate_msg_py(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/BSMcoreData.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/VehicleSize_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/BrakeSystemStatus_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/PositionalAccuracy_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/AccelerationSet4Way_t.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/wave_pkg
)
_generate_msg_py(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/MovementList_t.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TimeChangeDetails.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/MovementEventList_t.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/wave_pkg
)
_generate_msg_py(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/RTCMheader.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/AntennaOffsetSet_t.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/wave_pkg
)
_generate_msg_py(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/VehicleSize_t.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/wave_pkg
)
_generate_msg_py(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/RTCM.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/RTCMmessageList_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/FullPositionVector.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/AntennaOffsetSet_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/RTCMheader.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/wave_pkg
)
_generate_msg_py(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/SPAT.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/MovementList_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TimeChangeDetails.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionReferenceID_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/MovementEventList_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionStateList_t.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/wave_pkg
)
_generate_msg_py(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/PositionalAccuracy_t.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/wave_pkg
)
_generate_msg_py(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/AccelerationSet4Way_t.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/wave_pkg
)
_generate_msg_py(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/BSMpartII.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/PathHistory_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/VehicleSafetyExtensions_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/SpecialVehicleExtensions_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/SupplementalVehicleExtensions_t.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/wave_pkg
)
_generate_msg_py(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/Position3D.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/wave_pkg
)
_generate_msg_py(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionStateList_t.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TimeChangeDetails.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionReferenceID_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/MovementEventList_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/MovementList_t.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/wave_pkg
)
_generate_msg_py(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeOffsetPointXY_t.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/wave_pkg
)
_generate_msg_py(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/Position3D_t.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/wave_pkg
)
_generate_msg_py(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TimeChangeDetails.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/wave_pkg
)
_generate_msg_py(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/FullPositionVector.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/wave_pkg
)
_generate_msg_py(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/LaneList_t.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeAttributeSetXY.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/ConnectsToList.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeOffsetPointXY_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/LaneAttributes_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeListXY_t.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/wave_pkg
)
_generate_msg_py(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionReferenceID.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/wave_pkg
)
_generate_msg_py(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionReferenceID_t.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/wave_pkg
)
_generate_msg_py(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/LaneAttributes_t.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/wave_pkg
)
_generate_msg_py(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/MovementEventList_t.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TimeChangeDetails.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/wave_pkg
)
_generate_msg_py(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionGeometryList.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/ConnectsToList.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/LaneList_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeOffsetPointXY_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionReferenceID.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/LaneAttributes_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeAttributeSetXY.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/Position3D_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeListXY_t.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/wave_pkg
)
_generate_msg_py(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/ConnectsToList.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/wave_pkg
)
_generate_msg_py(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TravelerDataFrameList_t.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeListLL_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TravelerDataFrame__content_PR.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/OffsetSystem_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/Position3D.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/GeographicalPath.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/wave_pkg
)
_generate_msg_py(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/BSM.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/PathHistory_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/VehicleSize_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/VehicleSafetyExtensions_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/BrakeSystemStatus_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/BSMcoreData.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/AccelerationSet4Way_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/PositionalAccuracy_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/SpecialVehicleExtensions_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/BSMpartII.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/SupplementalVehicleExtensions_t.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/wave_pkg
)
_generate_msg_py(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TravelerDataFrame__content_PR.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/wave_pkg
)
_generate_msg_py(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/VehicleSafetyExtensions_t.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/PathHistory_t.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/wave_pkg
)
_generate_msg_py(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/MAP.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/ConnectsToList.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeOffsetPointXY_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/LaneList_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionReferenceID.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeAttributeSetXY.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/LaneAttributes_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionGeometryList.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/Position3D_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeListXY_t.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/wave_pkg
)
_generate_msg_py(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/RTCMmessageList_t.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/wave_pkg
)
_generate_msg_py(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeAttributeSetXY.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/wave_pkg
)
_generate_msg_py(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/PathHistory_t.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/wave_pkg
)
_generate_msg_py(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/SupplementalVehicleExtensions_t.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/wave_pkg
)
_generate_msg_py(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/OffsetSystem_t.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeListLL_t.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/wave_pkg
)
_generate_msg_py(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TIM.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TravelerDataFrame__content_PR.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/OffsetSystem_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeListLL_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/GeographicalPath.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/Position3D.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TravelerDataFrameList_t.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/wave_pkg
)
_generate_msg_py(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/BrakeSystemStatus_t.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/wave_pkg
)
_generate_msg_py(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/SpecialVehicleExtensions_t.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/wave_pkg
)
_generate_msg_py(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/AntennaOffsetSet_t.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/wave_pkg
)
_generate_msg_py(wave_pkg
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/GeographicalPath.msg"
  "${MSG_I_FLAGS}"
  "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeListLL_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/OffsetSystem_t.msg;/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/Position3D.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/wave_pkg
)

### Generating Services

### Generating Module File
_generate_module_py(wave_pkg
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/wave_pkg
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(wave_pkg_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(wave_pkg_generate_messages wave_pkg_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeListLL_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_py _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeListXY_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_py _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/Position3D.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_py _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/MovementList_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_py _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/SupplementalVehicleExtensions_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_py _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/BSM.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_py _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/VehicleSize_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_py _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/RTCM.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_py _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/SPAT.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_py _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/PositionalAccuracy_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_py _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/MAP.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_py _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/BSMpartII.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_py _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/BSMcoreData.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_py _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionStateList_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_py _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TimeChangeDetails.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_py _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/FullPositionVector.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_py _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/GeographicalPath.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_py _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/LaneList_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_py _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/LaneAttributes_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_py _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionReferenceID_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_py _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeOffsetPointXY_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_py _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/Position3D_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_py _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/MovementEventList_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_py _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/ConnectsToList.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_py _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TravelerDataFrameList_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_py _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/RTCMheader.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_py _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TravelerDataFrame__content_PR.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_py _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/VehicleSafetyExtensions_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_py _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/AccelerationSet4Way_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_py _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/PathHistory_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_py _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionGeometryList.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_py _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/NodeAttributeSetXY.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_py _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/RTCMmessageList_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_py _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/IntersectionReferenceID.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_py _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/OffsetSystem_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_py _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/BrakeSystemStatus_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_py _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/SpecialVehicleExtensions_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_py _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/AntennaOffsetSet_t.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_py _wave_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/choi/i30_ws/src/pharos_drivers/wave_pkg/msg/TIM.msg" NAME_WE)
add_dependencies(wave_pkg_generate_messages_py _wave_pkg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(wave_pkg_genpy)
add_dependencies(wave_pkg_genpy wave_pkg_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS wave_pkg_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/wave_pkg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/wave_pkg
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/wave_pkg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/wave_pkg
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/wave_pkg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/wave_pkg
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/wave_pkg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/wave_pkg
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/wave_pkg)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/wave_pkg\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/wave_pkg
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
