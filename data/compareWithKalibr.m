I_T_C_kalibr = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/kalibr_results/I_T_C_kalibr_2022_01_28_15_06_51.txt');
%I_T_C_kalibr = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/kalibr_results/I_T_C_kalibr_2022_01_28_15_11_20.txt');
%I_T_C_kalibr = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/kalibr_results/I_T_C_kalibr_2022_01_28_15_15_06.txt');
%I_T_C_kalibr = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/kalibr_results/I_T_C_kalibr_2022_01_28_15_20_21.txt');
%I_T_C_kalibr = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/kalibr_results/I_T_C_kalibr_2022_01_28_15_30_47.txt');

I_T_C = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/cam_imu_results/I_T_C_2022_01_28_15_06_51.txt');
%I_T_C = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/cam_imu_results/I_T_C_2022_01_28_15_11_20.txt');
%I_T_C = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/cam_imu_results/I_T_C_2022_01_28_15_15_06.txt');
%I_T_C = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/cam_imu_results/I_T_C_2022_01_28_15_20_21.txt');
%I_T_C = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/cam_imu_results/I_T_C_2022_01_28_15_30_47.txt');

T_err = I_T_C*inv(I_T_C_kalibr);
R_err = T_err(1:3, 1:3);
t_err = T_err(1:3, 4)'
eul_err = rotm2eul(R_err)*180/pi
% euler_IC = rotm2eul(I_T_C(1:3, 1:3))*180/pi;
% euler_IC_kalibr = rotm2eul(I_T_C_kalibr(1:3, 1:3))*180/pi;

% dlmwrite('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/I_T_C_kalibr.txt', ...
%     I_T_C_kalibr, 'delimiter',' ','newline','pc');