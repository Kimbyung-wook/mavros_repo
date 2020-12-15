echo "# mavros_repo" >> README.md
git init
git add README.md
git commit -m "first commit"
git branch -M main
git remote add origin https://github.com/Kimbyung-wook/mavros_repo.git
git push -u origin main

본 문서는 다음을 참고하여 만들었습니다.
**임재용님의 moducdulab_ros**
https://github.com/Jaeyoung-Lim/modudculab_ros
**MAVROS Offboard control example**
https://dev.px4.io/master/en/ros/mavros_offboard.html
**mavlink/mavros의 test_mavros**
https://github.com/mavlink/mavros/tree/master/test_mavros
