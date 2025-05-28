#!/bin/bash
#cd /home/ubuntu/colcon_ws/src/new_update_repo/
#git pull
#sleep 3
screen -dmS ROBO bash
#screen -S ROBO -X screen -t UPDATE_REPO bash -ic "bash /home/ubuntu/colcon_ws/src/new_update_repo/update_repo.sh"
sleep 1
screen -S ROBO -X screen -t RASPICAM bash -ic "ros2 run camera_ros camera_node --ros-args -p format:='RGB888' -p width:=320 -p height:=240 -p camera_info_url:='file:///home/ubuntu/.ros/camera_info/ost.yaml'"
sleep 5
#screen -S ROBO -X screen -t REALSENSE2 bash -ic "ros2 run realsense2_camera realsense2_camera_node"
#sleep 5
screen -S ROBO -X screen -t LIDAR bash -ic "ros2 launch turtlebot3_bringup robot.launch.py"
sleep 2
screen -S ROBO -X screen -t SERVOARM bash -ic "ros2 run insperbot servo_arm"
sleep 1
screen -S ROBO -X screen -t TELINHA bash -ic "ros2 run insperbot menu"
sleep 1
screen -S ROBO -X screen -t APRILTAG bash -ic "ros2 run apriltag_ros apriltag_node --ros-args   -r image_rect:=/camera/image_raw   -r camera_info:=/camera/camera_info --params-file /home/ubuntu/turtlebot3_ws/apriltag.yaml"
sleep 1
screen -S ROBO -X screen -t PUBLICAAPRILTAG bash -ic "ros2 run insperbot april_tag"
sleep 1
screen -S ROBO -X screen -t YOLO bash -ic "ros2 run insperbot yolo"
sleep 1

echo 0
