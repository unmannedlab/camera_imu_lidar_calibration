clear all;
close all;

I_T_C_kalibr = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/kalibr_results/I_T_C_kalibr_2022_01_28_15_06_51.txt');
%I_T_C_repErr = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/cam_imu_results/repErrResidual/I_T_C_2022_01_28_15_06_51.txt');
%I_T_C_mocal = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/cam_imu_results/mocalResidual/I_T_C_2022_01_28_15_06_51.txt');
 
%I_T_C_kalibr = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/kalibr_results/I_T_C_kalibr_2022_01_28_15_11_20.txt');
%I_T_C_repErr = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/cam_imu_results/repErrResidual/I_T_C_2022_01_28_15_11_20.txt');
%I_T_C_mocal = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/cam_imu_results/mocalResidual/I_T_C_2022_01_28_15_11_20.txt');

%I_T_C_kalibr = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/kalibr_results/I_T_C_kalibr_2022_01_28_15_15_06.txt');
%I_T_C_repErr = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/cam_imu_results/repErrResidual/I_T_C_2022_01_28_15_15_06.txt');
%I_T_C_mocal = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/cam_imu_results/mocalResidual/I_T_C_2022_01_28_15_15_06.txt');

%I_T_C_kalibr = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/kalibr_results/I_T_C_kalibr_2022_01_28_15_20_21.txt');
%I_T_C_repErr = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/cam_imu_results/repErrResidual/I_T_C_2022_01_28_15_20_21.txt');
%I_T_C_mocal = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/cam_imu_results/mocalResidual/I_T_C_2022_01_28_15_20_21.txt');
 
%I_T_C_kalibr = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/kalibr_results/I_T_C_kalibr_2022_01_28_15_30_47.txt');
%I_T_C_repErr = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/cam_imu_results/repErrResidual/I_T_C_2022_01_28_15_30_47.txt');
%I_T_C_mocal = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/cam_imu_results/mocalResidual/I_T_C_2022_01_28_15_30_47.txt');

I_T_C_err_mocal = I_T_C_kalibr*inv(I_T_C_mocal);
I_T_C_err_repErr = I_T_C_kalibr*inv(I_T_C_repErr);

I_R_C_kalibr = I_T_C_kalibr(1:3, 1:3);
euler_IRC_kalibr = rotm2eul(I_R_C_kalibr)*180/pi;
I_t_C_kalibr = I_T_C_kalibr(1:3, 4)';
euler_t_kalibr = [euler_IRC_kalibr, I_t_C_kalibr*100]

I_R_C_mocal = I_T_C_mocal(1:3, 1:3);
euler_IRC_mocal = rotm2eul(I_R_C_mocal)*180/pi;
I_t_C_mocal = I_T_C_mocal(1:3, 4)';
euler_t_mocal = [euler_IRC_mocal, I_t_C_mocal*100]

I_R_C_repErr = I_T_C_repErr(1:3, 1:3);
euler_IRC_repErr = rotm2eul(I_R_C_repErr)*180/pi;
I_t_C_repErr = I_T_C_repErr(1:3, 4)';
euler_t_repErr = [euler_IRC_repErr, I_t_C_repErr*100]

I_R_C_err_mocal = I_T_C_err_mocal(1:3, 1:3);
euler_IRC_err_mocal = rotm2eul(I_R_C_err_mocal)*180/pi;
I_t_C_err_mocal = I_T_C_err_mocal(1:3, 4)';
euler_t_err_mocal = [euler_IRC_err_mocal, I_t_C_err_mocal*100]

I_R_C_err_repErr = I_T_C_err_repErr(1:3, 1:3);
euler_IRC_err_repErr = rotm2eul(I_R_C_err_repErr)*180/pi;
I_t_C_err_repErr = I_T_C_err_repErr(1:3, 4)';
euler_t_err_repErr = [euler_IRC_err_repErr, I_t_C_err_repErr*100]