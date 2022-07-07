//
// Created by usl on 12/9/20.
//
#ifndef CALIB_PARSE_ROS_H
#define CALIB_PARSE_ROS_H

#include <ros/ros.h>
#include <core/calibManagerOptions.h>

namespace calib_estimator {
    /// This function will load parameters from the ros node handler / parameter server
    calibManagerOptions parse_ros_nodehandler(ros::NodeHandle &nh) {
        /// Our lincalib manager options with defaults
        calibManagerOptions params;

        /// Estimator
        // Main EKF parameters
        ROS_INFO_STREAM("Reading General Estimator Parameters");
        nh.param<bool>("use_fej", params.state_options.do_fej, params.state_options.do_fej);
        nh.param<bool>("use_imuavg", params.state_options.imu_avg, params.state_options.imu_avg);
        nh.param<bool>("use_rk4int", params.state_options.use_rk4_integration,
                       params.state_options.use_rk4_integration);
        nh.param<bool>("do_calib_lidar_imu_timeoffset", params.state_options.do_calib_lidar_imu_timeoffset,
                       params.state_options.do_calib_lidar_imu_timeoffset);
        nh.param<bool>("do_calib_lidar_imu_extrinsic", params.state_options.do_calib_lidar_imu_extrinsic,
                       params.state_options.do_calib_lidar_imu_extrinsic);
        nh.param<bool>("do_calib_camera_imu_timeoffset", params.state_options.do_calib_camera_imu_timeoffset,
                       params.state_options.do_calib_camera_imu_timeoffset);
        nh.param<bool>("do_calib_camera_imu_extrinsic", params.state_options.do_calib_camera_imu_extrinsic,
                       params.state_options.do_calib_camera_imu_extrinsic);
        nh.param<int>("max_clones", params.state_options.max_clone_size, params.state_options.max_clone_size);

        nh.param<double>("state_init_trans_x_noise_lidarimu", params.state_options.trans_x_noise_lidarimu,
                         params.state_options.trans_x_noise_lidarimu);
        nh.param<double>("state_init_trans_y_noise_lidarimu", params.state_options.trans_y_noise_lidarimu,
                         params.state_options.trans_y_noise_lidarimu);
        nh.param<double>("state_init_trans_z_noise_lidarimu", params.state_options.trans_z_noise_lidarimu,
                         params.state_options.trans_z_noise_lidarimu);
        nh.param<double>("state_init_rot_x_noise_lidarimu", params.state_options.rot_x_noise_lidarimu,
                         params.state_options.rot_x_noise_lidarimu);
        nh.param<double>("state_init_rot_y_noise_lidarimu", params.state_options.rot_y_noise_lidarimu,
                         params.state_options.rot_y_noise_lidarimu);
        nh.param<double>("state_init_rot_z_noise_lidarimu", params.state_options.rot_z_noise_lidarimu,
                         params.state_options.rot_z_noise_lidarimu);
        nh.param<double>("state_init_time_offset_noise_lidarimu", params.state_options.time_offset_noise_lidarimu,
                         params.state_options.time_offset_noise_lidarimu);

        nh.param<double>("state_init_trans_x_noise_cameraimu", params.state_options.trans_x_noise_cameraimu,
                         params.state_options.trans_x_noise_cameraimu);
        nh.param<double>("state_init_trans_y_noise_cameraimu", params.state_options.trans_y_noise_cameraimu,
                         params.state_options.trans_y_noise_cameraimu);
        nh.param<double>("state_init_trans_z_noise_cameraimu", params.state_options.trans_z_noise_cameraimu,
                         params.state_options.trans_z_noise_cameraimu);
        nh.param<double>("state_init_rot_x_noise_cameraimu", params.state_options.rot_x_noise_cameraimu,
                         params.state_options.rot_x_noise_cameraimu);
        nh.param<double>("state_init_rot_y_noise_cameraimu", params.state_options.rot_y_noise_cameraimu,
                         params.state_options.rot_y_noise_cameraimu);
        nh.param<double>("state_init_rot_z_noise_cameraimu", params.state_options.rot_z_noise_cameraimu,
                         params.state_options.rot_z_noise_cameraimu);
        nh.param<double>("state_init_time_offset_noise_cameraimu", params.state_options.time_offset_noise_cameraimu,
                         params.state_options.time_offset_noise_cameraimu);

        /// Filter initialization
        ROS_INFO_STREAM("Reading Filter Initialization Parameters");
        nh.param<double>("init_window_time", params.init_window_time,
                         params.init_window_time);
        nh.param<double>("init_imu_thresh", params.init_imu_thresh,
                         params.init_imu_thresh);

        /// Noise
        // Our noise values for inertial sensor
        ROS_INFO_STREAM("Reading IMU Noise Parameters");
        nh.param<double>("gyroscope_noise_density", params.imu_noises.sigma_w,
                         params.imu_noises.sigma_w);
        nh.param<double>("accelerometer_noise_density", params.imu_noises.sigma_a,
                         params.imu_noises.sigma_a);
        nh.param<double>("gyroscope_random_walk", params.imu_noises.sigma_wb,
                         params.imu_noises.sigma_wb);
        nh.param<double>("accelerometer_random_walk", params.imu_noises.sigma_ab,
                         params.imu_noises.sigma_ab);

        // Read update parameters
        ROS_INFO_STREAM("Reading Updater Chi2 Multiplier");
        nh.param<int>("updater_chi2_multiplier", params.updaterOptions.chi2_multiplier, params.updaterOptions.chi2_multiplier);
        nh.param<bool>("updater_do_chi2_check", params.updaterOptions.do_chi2_check, params.updaterOptions.do_chi2_check);

        ROS_INFO_STREAM("Reading Rotation and Noise Update");
        nh.param<double>("updater_rotation_noise", params.updaterOptions.noise_rotation, params.updaterOptions.noise_rotation);
        nh.param<double>("updater_translation_noise", params.updaterOptions.noise_translation, params.updaterOptions.noise_translation);
        nh.param<double>("updater_pixel_noise", params.updaterOptions.noise_pixel, params.updaterOptions.noise_pixel);

        /// Global gravity
        ROS_INFO_STREAM("Reading Gravity");
        std::vector<double> gravity = {params.gravity(0), params.gravity(1), params.gravity(2)};
        nh.param<std::vector<double>>("gravity", gravity, gravity);
        assert(gravity.size() == 3);
        params.gravity << gravity.at(0), gravity.at(1), gravity.at(2);

        /// State
        // Timeoffset from lidar to IMU
        ROS_INFO_STREAM("Reading initial Lidar IMU Timeoffset");
        nh.param<double>("calib_lidar_imu_dt", params.calib_lidar_imu_dt, params.calib_lidar_imu_dt);
        ROS_INFO_STREAM("Reading initial Camera IMU Timeoffset");
        nh.param<double>("calib_camera_imu_dt", params.calib_camera_imu_dt, params.calib_camera_imu_dt);

        /// Our lidar extrinsics transform
        Eigen::Matrix4d I_T_L;
        std::vector<double> matrix_I_T_L;
        std::vector<double> matrix_I_T_L_default = {1,0,0,0,
                                                    0,1,0,0,
                                                    0,0,1,0,
                                                    0,0,0,1};

        /// Our camera extrinsics transform
        Eigen::Matrix4d I_T_C;
        std::vector<double> matrix_I_T_C;
        std::vector<double> matrix_I_T_C_default = {1,0,0,0,
                                                    0,1,0,0,
                                                    0,0,1,0,
                                                    0,0,0,1};

        /// Camera tracking parameters
        ROS_INFO_STREAM("Reading Camera Tracking Parameters");
        nh.param("checkerboard_dx", params.checkerboard_dx, params.checkerboard_dx);
        nh.param("checkerboard_dy", params.checkerboard_dy, params.checkerboard_dy);
        nh.param("checkerboard_rows", params.checkerboard_rows, params.checkerboard_rows);
        nh.param("checkerboard_cols", params.checkerboard_cols, params.checkerboard_cols);
        nh.param("camera_calibration_file_path", params.camera_calibration_file_path, params.camera_calibration_file_path);

        /// NDT Resolution
        ROS_INFO_STREAM("Reading NDT Resolution");
        nh.param("ndt_resolution", params.ndtResolution, params.ndtResolution);

        /// Undistortion Flag
        ROS_INFO_STREAM("Reading Undistortion Flag");
        nh.param("do_undistortion", params.do_undistortion, params.do_undistortion);

        ROS_INFO_STREAM("Reading lin output file names");
        nh.param("inertial_trajectory_filename", params.inertial_trajectory_filename, params.inertial_trajectory_filename);

        nh.param("inertial_bias_filename", params.inertial_bias_filename, params.inertial_bias_filename);
        nh.param("inertial_velocity_filename", params.inertial_velocity_filename, params.inertial_velocity_filename);
        nh.param("lidar_inertial_calib_extrinsic_filename", params.lidar_inertial_calib_extrinsic_filename,
                 params.lidar_inertial_calib_extrinsic_filename);
        nh.param("lidar_inertial_calib_dt_filename", params.lidar_inertial_calib_dt_filename,
                 params.lidar_inertial_calib_dt_filename);
        nh.param("visual_inertial_calib_extrinsic_filename", params.visual_inertial_calib_extrinsic_filename,
                 params.visual_inertial_calib_extrinsic_filename);
        nh.param("visual_inertial_calib_dt_filename", params.visual_inertial_calib_dt_filename,
                 params.visual_inertial_calib_dt_filename);

        ROS_INFO_STREAM("Reading lo output trajectory file name");
        nh.param("lidar_odometry_trajectory_filename", params.lidar_odometry_trajectory_filename, params.lidar_odometry_trajectory_filename);

//        if(params.do_undistortion) {
//            params.inertial_trajectory_filename = params.inertial_trajectory_filename + "_undistortedcloud.csv";
//            params.inertial_bias_filename = params.inertial_bias_filename + "_undistortedcloud.csv";
//            params.inertial_velocity_filename = params.inertial_velocity_filename + "_undistortedcloud.csv";
//            params.lidar_inertial_calib_extrinsic_filename = params.lidar_inertial_calib_extrinsic_filename + "_undistortedcloud.csv";
//            params.lidar_inertial_calib_dt_filename = params.lidar_inertial_calib_dt_filename + "_undistortedcloud.csv";
//            params.lidar_odometry_trajectory_filename = params.lidar_odometry_trajectory_filename + "_undistortedcloud.csv";
//        } else {
//            params.inertial_trajectory_filename = params.inertial_trajectory_filename + "_rawcloud.csv";
//            params.inertial_bias_filename = params.inertial_bias_filename + "_rawcloud.csv";
//            params.inertial_velocity_filename = params.inertial_velocity_filename + "_rawcloud.csv";
//            params.lidar_inertial_calib_extrinsic_filename = params.lidar_inertial_calib_extrinsic_filename + "_rawcloud.csv";
//            params.lidar_inertial_calib_dt_filename = params.lidar_inertial_calib_dt_filename + "_rawcloud.csv";
//            params.lidar_odometry_trajectory_filename = params.lidar_odometry_trajectory_filename + "_rawcloud.csv";
//        }

        /// File to read the initial lidar imu calibration result from
        ROS_INFO_STREAM("Reading the filename to read the initial lidar imu calibration result to");
        nh.param("init_lidar_inertial_calibration_result_filename", params.init_lidar_inertial_calibration_result_filename,
                 params.init_lidar_inertial_calibration_result_filename);

        std::ifstream initial_calib_lidar_inertial(params.init_lidar_inertial_calibration_result_filename);
        std::string word_lidar;
        int i = 0; int j = 0;
        I_T_L = Eigen::Matrix4d::Identity();
        // Read in from ROS, and save into our eigen mat
        ROS_INFO_STREAM("Reading initial I_T_L");
        while (initial_calib_lidar_inertial >> word_lidar){
            I_T_L(i, j) = atof(word_lidar.c_str());
            j++;
            if(j>3) {
                j = 0;
                i++;
            }
        }
        /// Load these into our state
        Eigen::Matrix<double,7,1> lidar_imu_extrinsics_ITL;
        lidar_imu_extrinsics_ITL.block(0,0,4,1) = rot_2_quat(I_T_L.block(0,0,3,3));
        lidar_imu_extrinsics_ITL.block(4,0,3,1) = I_T_L.block(0,3,3,1);
        params.lidar_imu_extrinsics = lidar_imu_extrinsics_ITL;

        /// File to write the final lidar inertial calibration result to
        ROS_INFO_STREAM("Reading the filename to write the final lidar inertial calibration result to");
        nh.param("lidar_inertial_calibration_result_filename", params.lidar_inertial_calibration_result_filename,
                 params.lidar_inertial_calibration_result_filename);

        /// File to read the initial camera imu calibration result from
        ROS_INFO_STREAM("Reading the filename to read the initial camera imu calibration result to");
        nh.param("init_camera_inertial_calibration_result_filename", params.init_camera_inertial_calibration_result_filename,
                 params.init_camera_inertial_calibration_result_filename);

        std::ifstream initial_calib_camera_inertial(params.init_camera_inertial_calibration_result_filename);
        std::string word_camera;
        i = 0; j = 0;
        I_T_C = Eigen::Matrix4d::Identity();
        // Read in from ROS, and save into our eigen mat
        ROS_INFO_STREAM("Reading initial I_T_C");
        while (initial_calib_camera_inertial >> word_camera){
            I_T_C(i, j) = atof(word_camera.c_str());
            j++;
            if(j>3) {
                j = 0;
                i++;
            }
        }
        /// Load these into our state
        Eigen::Matrix<double,7,1> camera_imu_extrinsics_ITC;
        camera_imu_extrinsics_ITC.block(0,0,4,1) = rot_2_quat(I_T_C.block(0,0,3,3));
        camera_imu_extrinsics_ITC.block(4,0,3,1) = I_T_C.block(0,3,3,1);
        params.camera_imu_extrinsics = camera_imu_extrinsics_ITC;

        /// File to write the final camera inertial calibration result to
        ROS_INFO_STREAM("Reading the filename to write the final camera inertial calibration result to");
        nh.param("camera_inertial_calibration_result_filename", params.camera_inertial_calibration_result_filename,
                 params.camera_inertial_calibration_result_filename);

        /// File to write the final camera lidar calibration result to
        ROS_INFO_STREAM("Reading the filename to write the final camera lidar calibration result to");
        nh.param("camera_lidar_calibration_result_filename", params.camera_lidar_calibration_result_filename,
                 params.camera_lidar_calibration_result_filename);

        /// Scan output folder names
        ROS_INFO_STREAM("Reading the folder name to write the raw and deskewed scan");
        nh.param("raw_scan_folder_name", params.raw_scan_folder_name, params.raw_scan_folder_name);
        nh.param("deskewed_scan_folder_name", params.deskewed_scan_folder_name, params.deskewed_scan_folder_name);

        /// Limit map size params
        nh.param("limit_map_size", params.limit_map_size, params.limit_map_size);
        nh.param("no_of_scans_for_map", params.no_of_scans_for_map, params.no_of_scans_for_map );

        nh.param("downsample_for_mapping", params.downSampleForMapping, params.downSampleForMapping);
        nh.param("gen_map_data", params.gen_map_data, params.gen_map_data);

        /// Residual file name
        nh.param("residual_filename", params.residual_filename, params.residual_filename);

        return params;
    }
}
#endif //CALIB_PARSE_ROS_H

