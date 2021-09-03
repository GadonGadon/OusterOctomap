
# OusterOctomap
## 설치환경
 - ubuntu 18.04
 - ROS Melodic
## 사용장비
 - Ouster OS0-32-G
 - Intel NUC

## 기본 세팅
### 다운로드, catkin_make
    cd ~/catkin_ws/src
    git clone https://github.com/GadonGadon/OusterOctomap
    cd ~/catkin_ws
    catkin_make -DCMAKE_BUILD_TYPE=_Release_
 ### Ouster Lidar 실행
 - ifconfig 으로 현재행 연결된 네트워크 인터페이스 이름(eno0, eth0, ...), ip 확인
  - `sudo ip add flush dev eno1`
  -  `ip addr show dev eno1` ip 제대로 등록 되었는지확인(DOWN일경우 센서연결 안됨, UP일경우 연결 됨)
 -  `sudo ip link set eno1 up`
  -  `sudo ip addr add 192.168.0.23/24 dev eno1` 192.168.0.23ip 등록
 - `sudo dnsmasq -C /dev/null -kd -F 192.168.0.0,192.168.0.100 -i eno1 --bind-dynamic` 할당받을 IP범위 지정(기다리면 허가된 ip주소 나옴)
  -  `gedit ~/catkin_ws/src/ouster_example/ouster_ros`
  -  sensor_hostname에 허가된 ip주소 입력
  - udp_dest에 192.168.0.23 입력 
  -  `roslaunch ouster_ros ouster.launch metadata:=metadata.json` ouster 런치파일 실행

 ### OctoMap
  -  `rqt` rqt창 열기
  -  Plugins -> Topics -> Topic Moniter 열기
  - ![스크린샷, 2021-09-03 08-41-10](https://user-images.githubusercontent.com/59405201/131929616-d74a28f0-c273-437e-b223-1f8676df3514.png)
- Ouster 라이다에서 나오는 PointCloud2메세지의 Topic명 확인(/os_cloud_node/points)
- Plugins -> Visualizaion -> TF Tree 열기
- ![스크린샷, 2021-09-03 08-47-32](https://user-images.githubusercontent.com/59405201/131929858-f834ff70-880e-4ee7-b602-b7882d2e3927.png)-os_sensor와 연결된 최상위 프레임 확인(testWorld/odom)
- `gedit ~/catkin_ws/src/octomap_mapping/octomap_server/launch/octomap_mapping.launch`
- frame_id의 value에 확인한 최상위 프레임 넣기
- cloud_in에 확인한 PointCloud2 Topic넣기
- ` roslaunch octomap_server octomap_mapping.launch` OctoMap 실행

### rviz
- `rviz`
- Fixed Frame에 최상위 프레임 지정(testWorld/odom)
- ![스크린샷, 2021-09-03 08-54-06](https://user-images.githubusercontent.com/59405201/131930356-432db18f-2dca-478e-87df-071bafa28569.png)- Add -> By Topic을 눌러 원하는 데이터 선택
	- /occupied_cells_vis_array MarkerArray -> OctoMap
	- /os_cloud_node imu -> Ouster라이다의 imu
	- /os_cloud_node PointCloud2 -> Ouster라이다의 PointCloud
	- /projected_map Map -> OctoMap에서 생성된 Map

## 참고자료
1. https://github.com/ouster-lidar/ouster_example
2. https://github.com/OctoMap/octomap_mapping


