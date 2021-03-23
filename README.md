# 파일 구성
control_node, comm_node, cam_node로 구성 <br>
노드 실행은 MAVROS_node, cam_node, control_node, comm_node 순으로 켜기 <br>
각 노드에서 하는 일 

* cam_node <br>
캠 동작<br>
pub : 노드 상태 <br>
srv : 캠 On/Off 

* control_node <br>
기체 제어 <br>
pub : 노드 상태, 제어 상태, 제어 명령 <br>
sub : 기체 상태 <br>
call: 제어 명령 등

* comm_node <br>
통신 처리 및 명령 전달<br>
sub : 기체 상태, 노드 상태 <br> 
call: 제어 명령, 캠 On/Off <br>

# 노드 검증 방법 
* cam_node 끄고 켜기 <br>
  [Terminal 1] @ **python cam_node.py** <br>
  [Terminal 1] _Ready to take video_ <br>
  [Terminal 2] @ **rosservice call /cam_node/cam_power 1** <br>
  [Terminal 1] _resp : true_ <br>
  [Terminal 2] CamState : True <br>
  [Terminal 2] @ **rosservice call /cam_node/cam_power 0** <br>
  [Terminal 1] _resp : true_ <br>
  [Terminal 2] CamState : False <br>

* comm_node 통신 장비 안쓰고 메시지 보내기 <br>
  [Terminal 1] @ **python cam_node.py** <br>
  [Terminal 2] @ **python comm_node.py fake** <br>
  cam_node와 comm_node는 1Hz로 캠을 끄고 켜며 캠 상태를 주고 받는다. <br>
  [Terminal 3] @ **rosservice call /fake_serial/topics "test message"**<br>
  [Terminal 1] remaining 100, received test messages<br>

# How to force upstarting to the machine?
Refer from robot-upstart and https://roboticsbackend.com/make-ros-launch-start-on-boot-with-robot_upstart/ <br>
**sudo apt-get install ros-melodic-robot-upstart** <br>
No need to run roscore <br>
**rosrun robot_upstart install <PKG_NAME> <LAUNCHFILE_NAME> --job <JOB_NAME> --symlink**
After this, just run following:
**sudo systemctl daemon-reload**
That launch file is installed, but is not running yet.

## How to disabling 
sudo systemctl disable <JOB_NAME>.service

## Want to run at the start
sudo systemctl start <JOB_NAME>.service
## Stop
sudo systemctl stop <JOB_NAME>.service

## Permission
cd /etc/udev/rules.d/<br>
sudo touch local.rules<br>
ACTION=="add", KERNEL=="dialout", MODE="0666"<br>

# 업로드 어떻게 함? - How to upload my files
##처음하는거면##
echo "# mavros_repo" >> README.md   
git init   
git add README.md   
git commit -m "first commit"   
git branch -M main   
git remote add origin https://github.com/Kimbyung-wook/mavros_repo.git   
git push -u origin main         

##있는거에다가 하는거면##
git add . # 수정한거 다 올릴려구
git commit -m "Commentary" # 커밋한거 할 말 있으면?
git push -u origin main # 가랏!

# 어디에 이 저장소를 복사하면 되죠? - Where do i clone this repo to?
아래 주소를 따라서 소스를 빌드한 후에
https://github.com/mavlink/mavros/tree/master/mavros#source-installation
catkin_ws/src 에 git clone 하면 됩니다.




# 본 문서는 다음을 참고하여 만들었습니다.
**Jaeyoung-Lim님의 moducdulab_ros**   
https://github.com/Jaeyoung-Lim/modudculab_ros   
**MAVROS Offboard control example**   
https://dev.px4.io/master/en/ros/mavros_offboard.html   
**mavlink/mavros의 test_mavros**   
https://github.com/mavlink/mavros/tree/master/test_mavros   
**PX4-Autopilot의 MAVROS integration Test files**
https://github.com/PX4/PX4-Autopilot/tree/master/integrationtests/python_src/px4_it/mavros
