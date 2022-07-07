
%I_T_C = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/cam_imu_results/repErrResidual/I_T_C_2022_01_28_15_06_51.txt');
%I_T_C = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/cam_imu_results/repErrResidual/I_T_C_2022_01_28_15_11_20.txt');
%I_T_C = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/cam_imu_results/repErrResidual/I_T_C_2022_01_28_15_15_06.txt');
%I_T_C = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/cam_imu_results/repErrResidual/I_T_C_2022_01_28_15_20_21.txt');
I_T_C = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/cam_imu_results/repErrResidual/I_T_C_2022_01_28_15_30_47.txt');

%I_T_L = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/lidar_imu_results/I_T_L_2022_01_28_15_06_51.txt');
%I_T_L = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/lidar_imu_results/I_T_L_2022_01_28_15_11_20.txt');
%I_T_L = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/lidar_imu_results/I_T_L_2022_01_28_15_15_06.txt');
%I_T_L = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/lidar_imu_results/I_T_L_2022_01_28_15_20_21.txt');
I_T_L = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/lidar_imu_results/I_T_L_2022_01_28_15_30_47.txt');

C_T_L = inv(I_T_C)*I_T_L;
dlmwrite('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/C_T_L_concatanated.txt', ...
    C_T_L, 'delimiter',' ','newline','pc');

