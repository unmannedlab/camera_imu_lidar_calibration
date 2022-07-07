clear all;

%% 1
I_T_C_joint = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/camera_imu_lidar_results/I_T_C_final_2022_01_28_15_06_51.txt');
I_T_L_joint = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/camera_imu_lidar_results/I_T_L_final_2022_01_28_15_06_51.txt');
I_T_C_individual = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/cam_imu_results/repErrResidual/I_T_C_2022_01_28_15_06_51.txt');
I_T_L_individual = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/lidar_imu_results/I_T_L_2022_01_28_15_06_51.txt');

I_R_L_individual = I_T_L_individual(1:3, 1:3);
euler_IRL_individual = rotm2eul(I_R_L_individual)*180/pi;
I_t_L_individual = I_T_L_individual(1:3, 4)';
euler_t_ITL_individual = [euler_IRL_individual, I_t_L_individual]

I_R_L_joint = I_T_L_joint(1:3, 1:3);
euler_IRL_joint = rotm2eul(I_R_L_joint)*180/pi;
I_t_L_joint = I_T_L_joint(1:3, 4)';
euler_t_ITL_joint = [euler_IRL_joint, I_t_L_joint]

I_R_C_individual = I_T_C_individual(1:3, 1:3);
euler_IRC_individual = rotm2eul(I_R_C_individual)*180/pi;
I_t_C_individual = I_T_C_individual(1:3, 4)';
euler_t_ITC_individual = [euler_IRC_individual, I_t_C_individual]

I_R_L_joint = I_T_L_joint(1:3, 1:3);
euler_IRL_joint = rotm2eul(I_R_L_joint)*180/pi;
I_t_L_joint = I_T_L_joint(1:3, 4)';
euler_t_ITL_joint = [euler_IRL_joint, I_t_L_joint]

I_T_C_diff = I_T_C_individual*inv(I_T_C_joint);
I_R_C_diff = I_T_C_diff(1:3, 1:3);
I_t_C_diff = I_T_C_diff(1:3, 4)';
eul_IRC_diff = rotm2eul(I_R_C_diff)*180/pi;
eul_t_diff_IC = [eul_IRC_diff, I_t_C_diff]
IC_percentage_change = 100*eul_t_diff_IC./euler_t_ITC_individual

I_T_L_diff = I_T_L_individual*inv(I_T_L_joint);
I_R_L_diff = I_T_L_diff(1:3, 1:3);
I_t_L_diff = I_T_L_diff(1:3, 4)';
eul_IRL_diff = rotm2eul(I_R_L_diff)*180/pi;
eul_t_diff_IL = [eul_IRL_diff, I_t_L_diff]
IL_percentage_change = 100*eul_t_diff_IL./euler_t_ITL_individual

% %% 2
I_T_C_joint = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/camera_imu_lidar_results/I_T_C_final_2022_01_28_15_11_20.txt');
I_T_L_joint = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/camera_imu_lidar_results/I_T_L_final_2022_01_28_15_11_20.txt');
I_T_C_individual = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/cam_imu_results/repErrResidual/I_T_C_2022_01_28_15_11_20.txt');
I_T_L_individual = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/lidar_imu_results/I_T_L_2022_01_28_15_11_20.txt');

I_R_L_individual = I_T_L_individual(1:3, 1:3);
euler_IRL_individual = rotm2eul(I_R_L_individual)*180/pi;
I_t_L_individual = I_T_L_individual(1:3, 4)';
euler_t_ITL_individual = [euler_IRL_individual, I_t_L_individual]

I_R_L_joint = I_T_L_joint(1:3, 1:3);
euler_IRL_joint = rotm2eul(I_R_L_joint)*180/pi;
I_t_L_joint = I_T_L_joint(1:3, 4)';
euler_t_ITL_joint = [euler_IRL_joint, I_t_L_joint]

I_R_C_individual = I_T_C_individual(1:3, 1:3);
euler_IRC_individual = rotm2eul(I_R_C_individual)*180/pi;
I_t_C_individual = I_T_C_individual(1:3, 4)';
euler_t_ITC_individual = [euler_IRC_individual, I_t_C_individual]

I_R_L_joint = I_T_L_joint(1:3, 1:3);
euler_IRL_joint = rotm2eul(I_R_L_joint)*180/pi;
I_t_L_joint = I_T_L_joint(1:3, 4)';
euler_t_ITL_joint = [euler_IRL_joint, I_t_L_joint]

I_T_C_diff = I_T_C_individual*inv(I_T_C_joint);
I_R_C_diff = I_T_C_diff(1:3, 1:3);
I_t_C_diff = I_T_C_diff(1:3, 4)';
eul_IRC_diff = rotm2eul(I_R_C_diff)*180/pi;
eul_t_diff_IC = [eul_IRC_diff, I_t_C_diff]
IC_percentage_change = 100*eul_t_diff_IC./euler_t_ITC_individual

I_T_L_diff = I_T_L_individual*inv(I_T_L_joint);
I_R_L_diff = I_T_L_diff(1:3, 1:3);
I_t_L_diff = I_T_L_diff(1:3, 4)';
eul_IRL_diff = rotm2eul(I_R_L_diff)*180/pi;
eul_t_diff_IL = [eul_IRL_diff, I_t_L_diff]
IL_percentage_change = 100*eul_t_diff_IL./euler_t_ITL_individual
% % % %% 3
I_T_C_joint = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/camera_imu_lidar_results/I_T_C_final_2022_01_28_15_15_06.txt');
I_T_L_joint = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/camera_imu_lidar_results/I_T_L_final_2022_01_28_15_15_06.txt');
I_T_C_individual = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/cam_imu_results/repErrResidual/I_T_C_2022_01_28_15_15_06.txt');
I_T_L_individual = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/lidar_imu_results/I_T_L_2022_01_28_15_15_06.txt');

I_R_L_individual = I_T_L_individual(1:3, 1:3);
euler_IRL_individual = rotm2eul(I_R_L_individual)*180/pi;
I_t_L_individual = I_T_L_individual(1:3, 4)';
euler_t_ITL_individual = [euler_IRL_individual, I_t_L_individual]

I_R_L_joint = I_T_L_joint(1:3, 1:3);
euler_IRL_joint = rotm2eul(I_R_L_joint)*180/pi;
I_t_L_joint = I_T_L_joint(1:3, 4)';
euler_t_ITL_joint = [euler_IRL_joint, I_t_L_joint]

I_R_C_individual = I_T_C_individual(1:3, 1:3);
euler_IRC_individual = rotm2eul(I_R_C_individual)*180/pi;
I_t_C_individual = I_T_C_individual(1:3, 4)';
euler_t_ITC_individual = [euler_IRC_individual, I_t_C_individual]

I_R_L_joint = I_T_L_joint(1:3, 1:3);
euler_IRL_joint = rotm2eul(I_R_L_joint)*180/pi;
I_t_L_joint = I_T_L_joint(1:3, 4)';
euler_t_ITL_joint = [euler_IRL_joint, I_t_L_joint]

I_T_C_diff = I_T_C_individual*inv(I_T_C_joint);
I_R_C_diff = I_T_C_diff(1:3, 1:3);
I_t_C_diff = I_T_C_diff(1:3, 4)';
eul_IRC_diff = rotm2eul(I_R_C_diff)*180/pi;
eul_t_diff_IC = [eul_IRC_diff, I_t_C_diff]
IC_percentage_change = 100*eul_t_diff_IC./euler_t_ITC_individual

I_T_L_diff = I_T_L_individual*inv(I_T_L_joint);
I_R_L_diff = I_T_L_diff(1:3, 1:3);
I_t_L_diff = I_T_L_diff(1:3, 4)';
eul_IRL_diff = rotm2eul(I_R_L_diff)*180/pi;
eul_t_diff_IL = [eul_IRL_diff, I_t_L_diff]
IL_percentage_change = 100*eul_t_diff_IL./euler_t_ITL_individual
% % % % 
% % % % %% 4
I_T_C_joint = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/camera_imu_lidar_results/I_T_C_final_2022_01_28_15_20_21.txt');
I_T_L_joint = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/camera_imu_lidar_results/I_T_L_final_2022_01_28_15_20_21.txt');
I_T_C_individual = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/cam_imu_results/repErrResidual/I_T_C_2022_01_28_15_20_21.txt');
I_T_L_individual = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/lidar_imu_results/I_T_L_2022_01_28_15_20_21.txt');

I_R_L_individual = I_T_L_individual(1:3, 1:3);
euler_IRL_individual = rotm2eul(I_R_L_individual)*180/pi;
I_t_L_individual = I_T_L_individual(1:3, 4)';
euler_t_ITL_individual = [euler_IRL_individual, I_t_L_individual]

I_R_L_joint = I_T_L_joint(1:3, 1:3);
euler_IRL_joint = rotm2eul(I_R_L_joint)*180/pi;
I_t_L_joint = I_T_L_joint(1:3, 4)';
euler_t_ITL_joint = [euler_IRL_joint, I_t_L_joint]

I_R_C_individual = I_T_C_individual(1:3, 1:3);
euler_IRC_individual = rotm2eul(I_R_C_individual)*180/pi;
I_t_C_individual = I_T_C_individual(1:3, 4)';
euler_t_ITC_individual = [euler_IRC_individual, I_t_C_individual]

I_R_L_joint = I_T_L_joint(1:3, 1:3);
euler_IRL_joint = rotm2eul(I_R_L_joint)*180/pi;
I_t_L_joint = I_T_L_joint(1:3, 4)';
euler_t_ITL_joint = [euler_IRL_joint, I_t_L_joint]

I_T_C_diff = I_T_C_individual*inv(I_T_C_joint);
I_R_C_diff = I_T_C_diff(1:3, 1:3);
I_t_C_diff = I_T_C_diff(1:3, 4)';
eul_IRC_diff = rotm2eul(I_R_C_diff)*180/pi;
eul_t_diff_IC = [eul_IRC_diff, I_t_C_diff]
IC_percentage_change = 100*eul_t_diff_IC./euler_t_ITC_individual

I_T_L_diff = I_T_L_individual*inv(I_T_L_joint);
I_R_L_diff = I_T_L_diff(1:3, 1:3);
I_t_L_diff = I_T_L_diff(1:3, 4)';
eul_IRL_diff = rotm2eul(I_R_L_diff)*180/pi;
eul_t_diff_IL = [eul_IRL_diff, I_t_L_diff]
IL_percentage_change = 100*eul_t_diff_IL./euler_t_ITL_individual
% % % 
% % % %% 5
I_T_C_joint = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/camera_imu_lidar_results/I_T_C_final_2022_01_28_15_30_47.txt');
I_T_L_joint = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/camera_imu_lidar_results/I_T_L_final_2022_01_28_15_30_47.txt');
I_T_C_individual = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/cam_imu_results/repErrResidual/I_T_C_2022_01_28_15_30_47.txt');
I_T_L_individual = importdata('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/lidar_imu_results/I_T_L_2022_01_28_15_30_47.txt');

I_R_L_individual = I_T_L_individual(1:3, 1:3);
euler_IRL_individual = rotm2eul(I_R_L_individual)*180/pi;
I_t_L_individual = I_T_L_individual(1:3, 4)';
euler_t_ITL_individual = [euler_IRL_individual, I_t_L_individual]

I_R_L_joint = I_T_L_joint(1:3, 1:3);
euler_IRL_joint = rotm2eul(I_R_L_joint)*180/pi;
I_t_L_joint = I_T_L_joint(1:3, 4)';
euler_t_ITL_joint = [euler_IRL_joint, I_t_L_joint]

I_R_C_individual = I_T_C_individual(1:3, 1:3);
euler_IRC_individual = rotm2eul(I_R_C_individual)*180/pi;
I_t_C_individual = I_T_C_individual(1:3, 4)';
euler_t_ITC_individual = [euler_IRC_individual, I_t_C_individual]

I_R_L_joint = I_T_L_joint(1:3, 1:3);
euler_IRL_joint = rotm2eul(I_R_L_joint)*180/pi;
I_t_L_joint = I_T_L_joint(1:3, 4)';
euler_t_ITL_joint = [euler_IRL_joint, I_t_L_joint]

I_T_C_diff = I_T_C_individual*inv(I_T_C_joint);
I_R_C_diff = I_T_C_diff(1:3, 1:3);
I_t_C_diff = I_T_C_diff(1:3, 4)';
eul_IRC_diff = rotm2eul(I_R_C_diff)*180/pi;
eul_t_diff_IC = [eul_IRC_diff, I_t_C_diff]
IC_percentage_change = 100*eul_t_diff_IC./euler_t_ITC_individual

I_T_L_diff = I_T_L_individual*inv(I_T_L_joint);
I_R_L_diff = I_T_L_diff(1:3, 1:3);
I_t_L_diff = I_T_L_diff(1:3, 4)';
eul_IRL_diff = rotm2eul(I_R_L_diff)*180/pi;
eul_t_diff_IL = [eul_IRL_diff, I_t_L_diff]
IL_percentage_change = 100*eul_t_diff_IL./euler_t_ITL_individual

