//
// Created by usl on 11/6/20.
//

#ifndef CALIB_STATEOPTIONS_H
#define CALIB_STATEOPTIONS_H

#include <types/Type.h>

using namespace calib_type;

namespace calib_estimator {
    /// Struct which stores all filter options
    struct StateOptions {
        /// Bool to determine whether or not to do First Estimate Jacobians
        bool do_fej = true;

        /// Bool to determine whether or not to use imu message averaging
        bool imu_avg = false;

        /// Bool to determine whether we want to use Runge-Kutta 4 IMU integration
        bool use_rk4_integration = true;

        /// Bool to determine whether or not to calibrate Lidar to IMU time offset
        bool do_calib_lidar_imu_timeoffset = false;

        /// Bool to determine whether or not to calibrate LIDAR to IMU (I_R_L (I_q_L), I_t_L) extrinsics
        bool do_calib_lidar_imu_extrinsic = false;

        /// Bool to determine whether or not to calibrate Lidar to IMU time offset
        bool do_calib_camera_imu_timeoffset = false;

        /// Bool to determine whether or not to calibrate LIDAR to IMU (I_R_L (I_q_L), I_t_L) extrinsics
        bool do_calib_camera_imu_extrinsic = false;

        /// Max clone size of sliding window
        int max_clone_size = 11;

        /// Initial Lidar-IMU Uncertainty
        double rot_x_noise_lidarimu = 0.2;
        double rot_y_noise_lidarimu = 0.2;
        double rot_z_noise_lidarimu = 0.2;
        double trans_x_noise_lidarimu = 0.3;
        double trans_y_noise_lidarimu = 0.3;
        double trans_z_noise_lidarimu = 0.3;
        double time_offset_noise_lidarimu = 0.01;

        /// Initial Camera-IMU Uncertainty
        double rot_x_noise_cameraimu = 0.2;
        double rot_y_noise_cameraimu = 0.2;
        double rot_z_noise_cameraimu = 0.2;
        double trans_x_noise_cameraimu = 0.3;
        double trans_y_noise_cameraimu = 0.3;
        double trans_z_noise_cameraimu = 0.3;
        double time_offset_noise_cameraimu = 0.01;

        /// Print function of what parameters we have loaded
        void print() {
            printf("\t- use_fej: %d\n", do_fej);
            printf("\t- use_imuavg: %d\n", imu_avg);
            printf("\t- use_rk4int: %d\n", use_rk4_integration);
            printf("\t- do_calib_lidar_imu_timeoffset?: %d\n", do_calib_lidar_imu_timeoffset);
            printf("\t- do_calib_lidar_imu_extrinsic?: %d\n", do_calib_lidar_imu_extrinsic);
            printf("\t- do_calib_camera_imu_timeoffset?: %d\n", do_calib_camera_imu_timeoffset);
            printf("\t- do_calib_camera_imu_extrinsic?: %d\n", do_calib_camera_imu_extrinsic);
            printf("\t- max_clone_size: %d\n", max_clone_size);

            printf("\t- Lidar-IMU \n");
            printf("\t- rot_x_noise_lidarimu: %f\n", rot_x_noise_lidarimu);
            printf("\t- rot_y_noise_lidarimu: %f\n", rot_y_noise_lidarimu);
            printf("\t- rot_z_noise_lidarimu: %f\n", rot_z_noise_lidarimu);
            printf("\t- trans_x_noise_lidarimu: %f\n", trans_x_noise_lidarimu);
            printf("\t- trans_y_noise_lidarimu: %f\n", trans_y_noise_lidarimu);
            printf("\t- trans_z_noise_lidarimu: %f\n", trans_z_noise_lidarimu);
            printf("\t- time_offset_noise_lidarimu: %f\n", time_offset_noise_lidarimu);

            printf("\t- Camera-IMU \n");
            printf("\t- rot_x_noise_cameraimu: %f\n", rot_x_noise_cameraimu);
            printf("\t- rot_y_noise_cameraimu: %f\n", rot_y_noise_cameraimu);
            printf("\t- rot_z_noise_cameraimu: %f\n", rot_z_noise_cameraimu);
            printf("\t- trans_x_noise_cameraimu: %f\n", trans_x_noise_cameraimu);
            printf("\t- trans_y_noise_cameraimu: %f\n", trans_y_noise_cameraimu);
            printf("\t- trans_z_noise_cameraimu: %f\n", trans_z_noise_cameraimu);
            printf("\t- time_offset_noise_cameraimu: %f\n", time_offset_noise_cameraimu);
        }
    };
};
#endif //CALIB_STATEOPTIONS_H
