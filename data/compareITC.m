clear all;
close all;

%I_T_C_mocal = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/cam_imu_results/mocalResidual/I_T_C_2022_01_28_15_06_51.txt');
%I_T_C_mocal = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/cam_imu_results/mocalResidual/I_T_C_2022_01_28_15_11_20.txt');
%I_T_C_mocal = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/cam_imu_results/mocalResidual/I_T_C_2022_01_28_15_15_06.txt');
%I_T_C_mocal = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/cam_imu_results/mocalResidual/I_T_C_2022_01_28_15_20_21.txt');
I_T_C_mocal = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/cam_imu_results/mocalResidual/I_T_C_2022_01_28_15_30_47.txt');

%I_T_C_repErr = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/cam_imu_results/repErrResidual/I_T_C_2022_01_28_15_06_51.txt');
%I_T_C_repErr = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/cam_imu_results/repErrResidual/I_T_C_2022_01_28_15_11_20.txt');
%I_T_C_repErr = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/cam_imu_results/repErrResidual/I_T_C_2022_01_28_15_15_06.txt');
%I_T_C_repErr = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/cam_imu_results/repErrResidual/I_T_C_2022_01_28_15_20_21.txt');
I_T_C_repErr = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/cam_imu_results/repErrResidual/I_T_C_2022_01_28_15_30_47.txt');

%I_T_C_kalibr = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/kalibr_results/I_T_C_kalibr_2022_01_28_15_06_51.txt');
%I_T_C_kalibr = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/kalibr_results/I_T_C_kalibr_2022_01_28_15_11_20.txt');
%I_T_C_kalibr = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/kalibr_results/I_T_C_kalibr_2022_01_28_15_15_06.txt');
%I_T_C_kalibr = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/kalibr_results/I_T_C_kalibr_2022_01_28_15_20_21.txt');
I_T_C_kalibr = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/kalibr_results/I_T_C_kalibr_2022_01_28_15_30_47.txt');

T_err_mocal = I_T_C_kalibr*inv(I_T_C_mocal);
R_err_mocal = T_err_mocal(1:3, 1:3);
t_err_mocal = T_err_mocal(1:3, 4)';
t_err_mocal_norm = norm(t_err_mocal)*100;
eul_err_mocal = rotm2eul(R_err_mocal)*180/pi;
eul_err_mocal_norm = norm(eul_err_mocal);
t_eul_mocal_norm = [t_err_mocal_norm, eul_err_mocal_norm]

T_err_repErr = I_T_C_kalibr*inv(I_T_C_repErr);
R_err_repErr = T_err_repErr(1:3, 1:3);
t_err_repErr = T_err_repErr(1:3, 4)';
t_err_repErr_norm = norm(t_err_repErr)*100;
eul_err_repErr = rotm2eul(R_err_repErr)*180/pi;
eul_err_repErr_norm = norm(eul_err_repErr);
t_eul_repErr_norm = [t_err_repErr_norm, eul_err_repErr_norm]

T_err_mocal_repErr = I_T_C_mocal*inv(I_T_C_repErr);
R_err_mocal_repErr = T_err_mocal_repErr(1:3, 1:3);
t_err_mocal_repErr = T_err_mocal_repErr(1:3, 4)';
t_err_mocal_repErr_norm = norm(t_err_mocal_repErr)*100;
eul_err_mocal_repErr = rotm2eul(R_err_mocal_repErr)*180/pi;
eul_err_mocal_repErr_norm = norm(eul_err_mocal_repErr);
t_eul_repErr_norm = [t_err_repErr_t_err_mocal_repErr_normnorm, eul_err_repErr_norm]

