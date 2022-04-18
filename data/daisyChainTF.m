% I_T_C = importdata('/home/usl/catkin_ws/src/camera_imu_calibration/data/I_T_C_final.txt');
I_T_C = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/kalibr_results/I_T_C_kalibr_2022_01_28_15_06_51.txt');
%I_T_C = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/kalibr_results/I_T_C_kalibr_2022_01_28_15_11_20.txt');
%I_T_C = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/kalibr_results/I_T_C_kalibr_2022_01_28_15_15_06.txt');
%I_T_C = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/kalibr_results/I_T_C_kalibr_2022_01_28_15_20_21.txt');
%I_T_C = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/kalibr_results/I_T_C_kalibr_2022_01_28_15_30_47.txt');

I_T_L = importdata('/home/usl/catkin_ws/src/imu_lidar_calibration/linkalibr/data/I_T_L_final.txt');
C_T_L = inv(I_T_C)*I_T_L;
% dlmwrite('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/C_T_L_final_3_kalibr.txt', ...
%     C_T_L, 'delimiter',' ','newline','pc');

