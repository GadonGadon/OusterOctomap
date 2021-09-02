
# OusterOctomap
## 설치환경
 - ubuntu 18.04
 - ROS Melodic
## 사용장비
 - Ouster OS0-32-G

## 기본 세팅
### 다운로드, catkin_make
    cd ~/catkin_ws/src
    git clone https://github.com/GadonGadon/OusterOctomap
    cd ~/catkin_ws
    catkin_make -DCMAKE_BUILD_TYPE=_Release_
 ### Ouster Lidar 실행
 1. ifconfig 으로 현재행 연결된 네트워크 인터페이스 이름(eno0, eth0, ...), ip 확인
 2. `sudo ip add flush dev eno1`
 3. `ip addr show dev eno1` ip 제대로 등록 되었는지확인(DOWN일경우 센서연결 안됨, UP일경우 연결 됨)
 4. `sudo ip link set eno1 up`
 5. `sudo ip addr add 192.168.0.23/24 dev eno1` 192.168.0.23ip 등록
 6. `sudo dnsmasq -C /dev/null -kd -F 192.168.0.0,192.168.0.100 -i eno1 --bind-dynamic` 할당받을 IP범위 지정(기다리면 허가된 ip주소 나옴)
 7. `gedit ~/catkin_ws/src/ouster_example/ouster_ros`
 8. sensor_hostname에 허가된 ip주소 입력
 9. udp_dest에 192.168.0.23 입력 
 10.`roslaunch ouster_ros ouster.launch metadata:=metadata.json` ouster 런치파일 실행
 



