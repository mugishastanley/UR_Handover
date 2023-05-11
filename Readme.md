Installation Instructions
Running Instructions
0. Build the App
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
1. Launch the driver command:
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=yyy.yyy.yyy.yyy use_fake_hardware:=true launch_rviz:=false initial_joint_controller:=joint_trajectory_controller

2. Launch the visuals
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5 launch_rviz:=true use_fake_hardware:=true

3. Plan and execute sample trajectory to move robot
ros2 launch hello_moveit_ur hello_moveit_ur_launch.py
