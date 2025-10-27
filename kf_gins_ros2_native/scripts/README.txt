KF-GINS Dataset Player (ROS 2 Foxy)
===================================

Publishes KF-GINS text-format IMU/GNSS files to ROS 2 topics so `kf_gins_ros2_native` can ingest them.

Topics
------
- `/imu/data` (sensor_msgs/Imu): fields carry **increments** dtheta (rad) and dvel (m/s). Set `imu_is_delta:=True` when launching the native node.
- `/gps/fix_cov` (sensor_msgs/NavSatFix): covariance set as diagonal [E^2, N^2, U^2] at indices [0,4,8].

Usage
-----
    python3 play_kfgins_dataset.py --imu /home/yang/KF-GINS/dataset/IMU.txt       --gnss /home/yang/KF-GINS/dataset/GNSS_Pos.txt       --time-scale 1.0

Then launch the native node (another terminal):

    source ~/kf_gins_ws/install/setup.bash
    ros2 launch kf_gins_ros2_native native.launch.py       imu_topic:=/imu/data gnss_topic:=/gps/fix_cov imu_is_delta:=True       pose_decimation:=10 max_path_points:=20000

If your file names differ, open `~/KF-GINS/dataset/kf-gins.yaml` to see the exact paths, then pass them to the player.
