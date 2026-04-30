# Notes:
# 
# If previous robot has already launched ur_robot_driver, make sure this robot is disconnected from the network and reconnect after successful launch
# 
# If any of the camera drivers does not launch successfully, make sure it is connected to the computer and confirm with sudo dmesg -w
# This is observed to occur commonly on SIRAR where realsense will not be recognized unless dmesg is open when launching
gnome-terminal -- ros2 launch ur_robot_driver ur10e.launch.py robot_ip:=192.168.131.60 initial_joint_controller:=forward_velocity_controller
gnome-terminal -- ros2 launch azure_kinect_ros_driver sirar.launch.py
gnome-terminal -- ros2 launch realsense2_camera rs_launch.py camera_name:=realsense camera_namespace:=sirar   align_depth.enable:=true enable_sync:=true
gnome-terminal -- ros2 launch net_ft_driver net_ft_broadcaster.launch.py sensor_type:=ati ip_address:=192.168.131.30

gnome-terminal -- ros2 run tf_broadcasters tf_human2kinect --ros-args -p robot:=sirar
gnome-terminal -- ros2 run tf_broadcasters tf_kinect2ridge --ros-args -p robot:=sirar
gnome-terminal -- ros2 run tf_broadcasters tf_ridge2armbase --ros-args -p robot:=sirar 
# gnome-terminal -- ros2 run tf_broadcasters tf_armee2realsense --ros-args -p robot:=sirar    
# gnome-terminal -- ros2 run tf_broadcasters tf_realsense2cable --ros-args -p robot:=sirar
gnome-terminal -- ros2 run tf_broadcasters tf_armee2ftsensor --ros-args -p robot:=sirar 

gnome-terminal -- ros2 run redundancy_resolution redundancy_resolver --ros-args -r __ns:=/sirar
gnome-terminal -- ros2 run control_nodes vel_open_loop_node --ros-args -r __ns:=/sirar
gnome-terminal -- ros2 run control_nodes pos_control_node --ros-args -r __ns:=/sirar
# gnome-terminal -- ros2 run control_nodes ee_pos_publisher_node --ros-args -r __ns:=/sirar
gnome-terminal -- ros2 run control_nodes state_machine_node --ros-args -r __ns:=/sirar
gnome-terminal -- ros2 run redundancy_resolution open_loop_arm_cmd_republisher --ros-args -r __ns:=/sirar
gnome-terminal -- ros2 run robot_localization rel_pos_node --ros-args -r __ns:=/sirar
gnome-terminal -- ros2 run robot_localization rel_dist_node -ros-args -r __ns:=/sirar

gnome-terminal -- ros2 run control_nodes ridge_control_node
gnome-terminal -- ros2 run spacenav spacenav_node

gnome-terminal -- python3 ~/UR5e-Robot-Autonomously-Picks-Aruco-Markers-ROS-Noetic/ros2_aruco/detect_vis.py --ros-args \
  -p marker_length_m:=0.16 \
  -p target_id:=27 \
  -p publish_coordinate_convention:=optical \
  -p swap_yz_translation:=false \
  -p flip_z_translation:=false \
  -p publish_tf_frame:=true \
  -p marker_frame_name:=aruco_tag

# Hand-eye calibration (easy_handeye2)
gnome-terminal -- bash -c "source ~/easy_handeye2_ws/install/setup.bash && \
  ros2 launch easy_handeye2 calibrate.launch.py \
    name:=ur10e_eye_in_hand \
    calibration_type:=eye_in_hand \
    robot_base_frame:=siraRbase \
    robot_effector_frame:=siraRtool0_controller \
    tracking_base_frame:=realsense_color_optical_frame \
    tracking_marker_frame:=aruco_tag"