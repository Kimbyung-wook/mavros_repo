# 업로드 어떻게 함?
# How to upload my files
echo "# mavros_repo" >> README.md   
git init   
git add README.md   
git commit -m "first commit"   
git branch -M main   
git remote add origin https://github.com/Kimbyung-wook/mavros_repo.git   
git push -u origin main         

# 어디에 이 저장소를 복사하면 되죠?
# Where do i clone this repo to?
아래 주소를 따라서 소스를 빌드한 후에
https://github.com/mavlink/mavros/tree/master/mavros#source-installation
catkin_ws/src 에 git clone 하면 됩니다.


# 본 문서는 다음을 참고하여 만들었습니다.
# Refer from
**Jaeyoung-Lim님의 moducdulab_ros**   
https://github.com/Jaeyoung-Lim/modudculab_ros   
**MAVROS Offboard control example**   
https://dev.px4.io/master/en/ros/mavros_offboard.html   
**mavlink/mavros의 test_mavros**   
https://github.com/mavlink/mavros/tree/master/test_mavros   
