cmake_minimum_required(VERSION 2.8.3)
project(ebimu_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  std_msgs
  pharos_msgs #이걸 추가 하니까 pharos_msgs를 사용할 수 있구나
#  message_generation
)

generate_messages(
        DEPENDENCIES
        std_msgs

#        pharos_msgs
)

catkin_package(
  CATKIN_DEPENDS
    serial
    std_msgs

    pharos_msgs
#    geometry_msgs
)


#add_message_files(
#
#
#)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(serial_example_node src/serial_example_node.cpp)
add_executable(imuStringToFloat64 src/imuStringToFloat64.cpp)
add_executable(ebimuSetting src/ebimuSetting.cpp)
add_executable(inputConsoleEbimuSetting src/inputConsoleEbimuSetting.cpp)
add_executable(ebimuSettingClassVer src/ebimuSettingClassVer.cpp)
add_executable(ebimuCalibration src/ebimuCalibration.cpp)
add_executable(imuStringToFloat64_bagfile src/imuStringToFloat64bagfileReason.cpp)
add_executable(setting_for_imuReset src/setting_for_imuReset.cpp)

target_link_libraries(serial_example_node
  ${catkin_LIBRARIES}
)
target_link_libraries(imuStringToFloat64
        ${catkin_LIBRARIES}
        )
target_link_libraries(ebimuSetting
        ${catkin_LIBRARIES}
        )
target_link_libraries(inputConsoleEbimuSetting
        ${catkin_LIBRARIES}
        )
target_link_libraries(ebimuSettingClassVer
        ${catkin_LIBRARIES}
        )
target_link_libraries(ebimuCalibration
        ${catkin_LIBRARIES}
        )
target_link_libraries(imuStringToFloat64_bagfile
        ${catkin_LIBRARIES}
        )
target_link_libraries(setting_for_imuReset
        ${catkin_LIBRARIES}
        )

add_dependencies(serial_example_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(imuStringToFloat64 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
add_dependencies(ebimuSetting ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(inputConsoleEbimuSetting ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(ebimuSettingClassVer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(ebimuCalibration ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(imuStringToFloat64_bagfile ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(setting_for_imuReset ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})