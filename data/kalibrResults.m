I_T_C_kalibr_2022_01_28_15_20_21 = [0.01649452, 0.01459704,  0.9997574, 0.07272748;
                                    0.99968208, 0.01883008, -0.0167682, 0.12541803;
                                   -0.01907028, 0.99971614, -0.01428181, -0.07559903;
                                    0,          0,          0,          1];

I_T_C_kalibr_2022_01_28_15_11_20 = [ 0.01770378,  0.01469552,  0.99973527,  0.07331534;
                                     0.99966975,  0.01836761, -0.01797262,  0.12664824;
                                    -0.01862686,  0.9997233,  -0.01436549, -0.07536455;
                                              0,          0,          0,          1];
I_T_C_kalibr_2022_01_28_15_15_06 = [ 0.01708571,  0.0142516,   0.99975245,  0.07334434;
                                     0.99967944,  0.01844132, -0.01734735,  0.12591802;
                                    -0.01868398,  0.99972837, -0.01393195, -0.07538584;
                                              0,          0,          0,          1   ];

I_T_C_kalibr_2022_01_28_15_06_51 = [ 0.01798445,  0.01477116,  0.99972915,  0.07328248;
                                     0.99965978,  0.01862689, -0.01825842,  0.12584637;
                                    -0.01889154,  0.99971739, -0.01443114, -0.07470323;
                                              0,           0,           0,          1];

I_T_C_kalibr_2022_01_28_15_30_47 = [ 0.01648643,  0.01482037,  0.99975425,  0.073027;
                                     0.9996874,   0.01855249, -0.01676035,  0.12555912;
                                   -0.01879633,  0.99971804, -0.01450987, -0.0751377;
                                             0,          0,          0,          1];

dlmwrite('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/kalibr_results/I_T_C_kalibr_2022_01_28_15_20_21.txt', ...
    I_T_C_kalibr_2022_01_28_15_20_21, 'delimiter',' ','newline','pc');

dlmwrite('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/kalibr_results/I_T_C_kalibr_2022_01_28_15_11_20.txt', ...
    I_T_C_kalibr_2022_01_28_15_11_20, 'delimiter',' ','newline','pc');

dlmwrite('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/kalibr_results/I_T_C_kalibr_2022_01_28_15_15_06.txt', ...
    I_T_C_kalibr_2022_01_28_15_15_06, 'delimiter',' ','newline','pc');

dlmwrite('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/kalibr_results/I_T_C_kalibr_2022_01_28_15_06_51.txt', ...
    I_T_C_kalibr_2022_01_28_15_06_51, 'delimiter',' ','newline','pc');

dlmwrite('/home/usl/catkin_ws/src/camera_imu_lidar_calibration/data/kalibr_results/I_T_C_kalibr_2022_01_28_15_30_47.txt', ...
    I_T_C_kalibr_2022_01_28_15_30_47, 'delimiter',' ','newline','pc');