clear all;
close all;

% I_T_C_kalibr = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/kalibr_results/I_T_C_kalibr_2022_01_28_15_06_51.txt');
% I_T_C_ic = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/cam_imu_results/repErrResidual/I_T_C_2022_01_28_15_06_51.txt');
% I_T_C_jc = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/camera_imu_lidar_results/I_T_C_final_2022_01_28_15_06_51.txt');

% I_T_C_kalibr = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/kalibr_results/I_T_C_kalibr_2022_01_28_15_11_20.txt');
% I_T_C_ic = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/cam_imu_results/repErrResidual/I_T_C_2022_01_28_15_11_20.txt');
% I_T_C_jc = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/camera_imu_lidar_results/I_T_C_final_2022_01_28_15_11_20.txt');

% I_T_C_kalibr = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/kalibr_results/I_T_C_kalibr_2022_01_28_15_15_06.txt');
% I_T_C_ic = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/cam_imu_results/repErrResidual/I_T_C_2022_01_28_15_15_06.txt');
% I_T_C_jc = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/camera_imu_lidar_results/I_T_C_final_2022_01_28_15_15_06.txt');

I_T_C_kalibr = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/kalibr_results/I_T_C_kalibr_2022_01_28_15_20_21.txt');
I_T_C_ic = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/cam_imu_results/repErrResidual/I_T_C_2022_01_28_15_20_21.txt');
I_T_C_jc = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/camera_imu_lidar_results/I_T_C_final_2022_01_28_15_20_21.txt');

% I_T_C_kalibr = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/kalibr_results/I_T_C_kalibr_2022_01_28_15_30_47.txt');
% I_T_C_ic = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/cam_imu_results/repErrResidual/I_T_C_2022_01_28_15_30_47.txt');
% I_T_C_jc = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/camera_imu_lidar_results/I_T_C_final_2022_01_28_15_30_47.txt');

I_R_C_kalibr = I_T_C_kalibr(1:3, 1:3);
euler_IRC_kalibr = rotm2eul(I_R_C_kalibr)*180/pi;
I_t_C_kalibr = I_T_C_kalibr(1:3, 4)';

I_R_C_ic = I_T_C_ic(1:3, 1:3);
euler_IRC_ic = rotm2eul(I_R_C_ic)*180/pi;
I_t_C_ic = I_T_C_ic(1:3, 4)';

I_R_C_jc = I_T_C_jc(1:3, 1:3);
euler_IRC_jc = rotm2eul(I_R_C_jc)*180/pi;
I_t_C_jc = I_T_C_jc(1:3, 4)';

I_T_C_diff = I_T_C_kalibr*inv(I_T_C_jc);
I_R_C_diff = I_T_C_diff(1:3, 1:3);
euler_IRC_diff = rotm2eul(I_R_C_diff)*180/pi;
I_t_C_diff = I_T_C_diff(1:3, 4)';

euler_t_kalibr = [euler_IRC_kalibr, I_t_C_kalibr*100]
%euler_t_ic = [euler_IRC_ic, I_t_C_ic]
euler_t_jc = [euler_IRC_jc, I_t_C_jc*100]
euler_t_diff = [euler_IRC_diff, I_t_C_diff*100]