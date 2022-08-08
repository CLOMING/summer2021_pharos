# 2022 산업자원통상부 자율주행자동차 경진대회
___
#### Changelog
###### v1.0.0
- Initial Release  
###### v1.1.0
- [pharos_map] Add DaeguPG Drivable map (DRV,OBST map - v1.2.0)*
- [pharos_bringup] Add bringup_sensor.launch
- [pharos_bringup] Add simulation-purpose grouped-launch files*
- [pharos_bringup] Grouped Rviz modified
- [pharos_driver] Fix LiDAR Driver
- [pharos_perception] Optimizing Package Configuration
- [pharos_perception] Cluster Integarted to Python-Open3D
- [external_source] Add time delayed roslaunch (Multi-Sensor Init)
###### v1.3.0
- [pharos_perception] Redifined Code Structure
- [pharos_driver] Multi-Band Ublox ZED-F9P RTK, NTRIP Server Added

___
### Run Sequence
- ROS system
```bash
roscore  
rosparam set /use_sim_time true
rosbag play --pause ${rosbag directory} 
```
- TF / Rviz 
```bash
roslaunch pharos_utm_odometry tf2_utm_odometry.launch frame:=gps
roslaunch pharos_map_server pharos_map_server.launch 
```
- Vehicle Sensor
```bash
roslaunch velodyne_pointcloud pharos_velodyne.launch  
roslaunch ouster_ros pharos_ouster.launch replay:=true
```
- Vehicle Perception
```bash
roslaunch pharos_perception perception_Mission.launch (Mission1 - highway mission)
roslaunch pharos_perception perception_Mission_uphill.launch (Mission2 - uphill mission)

roslaunch pharos_perception LiDAR_Fusing.launch
roslaunch pharos_perception LiDAR_Map_Filter.launch
roslaunch pharos_perception LiDAR_Obstacle.launch
```
- Vehicle Localization
```bash
roslaunch pharos_ekf ublox_ekf.launch  
roslaunch pharos_bringup pg_main.launch 
<!--roslaunch pharos_mcl_pretreat pretreat_tilt.launch-->
<!--roslaunch pharos_mcl pharos_MCL.launch-->
<!--roslaunch pharos_localization predict_ekf.launch bag:=true-->
```
- Vehicle Planning Mission1
```bash
roslaunch pharos_path_planner global_path_planning.launch  
roslaunch pharos_speed_planner speed_planner.launch  
roslaunch pharos_path_planner 3d_path_planning.launch 
roslaunch pharos_trajectory_observer trajectory_observer.launch
<!--roslaunch pharos_behavior_planner behavior_planner.launch-->
```
- Vehicle Planning Mission2
```bash
roslaunch pharos_path_planner global_path_planning_mission2.launch  
roslaunch pharos_path_planner 3d_path_planning.launch  
roslaunch pharos_trajectory_observer trajectory_observer.launch
```

***
### ouster 패킷 사용법  
#### 메타데이터 저장(PC마다 한번만)  
roscd  
cd ..  
cp src/pharos_drivers/ouster_driver/ouster_ros/config/os1-128.meta ~/.ros/os1-128.meta  

### 백파일 실행(use_sim_time 안켜면 현재시간으로 출력됨)
rosparam set /use_sim_time true  
roslaunch ouster_ros ouster.launch replay:=true  

### Localization
ouster 단일 mcl 수정 중...
현재 /odom/mcl frame_id "map" 으로 되어있음.
0325/area_all.bag 175초 부근 발산 확인 필요.

### Map 사용시 주의사항
현재 .pgm 맵은 /odom 좌표계가 아닌 /map 좌표계에 맞춰져있음
tf listener 등을 이용하여 위치 데이터를 map 좌표계로 변환하여 사용(출력 할 때 다시 odom 좌표계로 변환하기)

### Ouster 메타데이터 위치 본인 PC 로컬 경로 적어서 푸시 하지 마세요
<arg name="metadata" default="os1-128.meta" doc="path to read or write metadata file when replaying or receiving sensor data, respectively"/> <<이거 바꾸지 마세요
default 경로 ~/.ros 폴더입니다.
~/.ros/os1-128.meta 로 만들면 됩니다.

### TF Sequence

```bash
roscore  
rosbag play ${bag파일 위치 ex) ~/Documents/Bagfile/kcity_01_full_mission.bag}  
roslaunch pharos_localization tf2_utm_odometry.launch  
roslaunch pharos_bringup pg_rviz.launch  
roslaunch pharos_localization ublox_ekf.launch bag:=true  
```
