//
// Created by usl on 3/24/22.
//

#ifndef CAMERA_IMU_LIDAR_CALIBRATION_CAMERAPOSETRACKING_H
#define CAMERA_IMU_LIDAR_CALIBRATION_CAMERAPOSETRACKING_H

#include <Eigen/Core>
#include <Eigen/Dense>

#include "utils/quat_ops.h"
#include "utils/math_utils.h"
#include "utils/eigen_utils.h"

#include "types/Pose.h"
#include "relpose/relativePose.h"

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include <fstream>
#include <memory>

namespace calib_core {
    class cameraPoseTracking {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        typedef std::shared_ptr<cameraPoseTracking> Ptr;
        struct Odom {
            double timestamp;
            Eigen::Matrix4d pose;
        };
        explicit  cameraPoseTracking(double dx_ = 0.05, double dy_ = 0.05,
                                     int checkerboard_rows_ = 6,
                                     int checkerboard_cols_ = 7,
                                     std::string cam_config_file_path_ = "");
        void readCameraParams(std::string cam_config_file_path,
                              int &image_height, int &image_width,
                              cv::Mat &D, cv::Mat &K);
        bool feedImage(double timestamp, cv::Mat input_image,
                       std::vector<cv::Point3f> &objectpoints_C0, std::vector<cv::Point2f> &imagepoints);
        Eigen::Vector4d getPlaneEquation();
        void solvePnPProblem();
        double visualizeImageProjections(cv::Mat rvec, cv::Mat tvec);
        void estimateCameraPose();
        Odom getCameraPose();
        relativePose getRelativePose();
        double checkReprojections(Eigen::Matrix4d I_T_C, Eigen::Matrix4d I0_T_Ik);

    private:
        cv::Mat image_in_for_poseestimation;
        cv::Mat image_in_for_visualization;

        cv::Mat projection_matrix;
        cv::Mat distCoeff;
        int image_height, image_width;

        std::vector<cv::Point2f> image_points;
        std::vector<cv::Point2f> image_points_undistorted;
        std::vector<cv::Point3f> object_points;
        std::vector<cv::Point3f> object_points_C0;

        double dx, dy;
        int checkerboard_rows, checkerboard_cols;

        cv::Mat tvec, rvec;
        cv::Mat C_R_B_cv;
        Eigen::Vector3d n_c;
        double d_c;
        Eigen::Vector4d nd_c; // [n_c, d_c]

        Eigen::Matrix3d C_R_B_eig;
        Eigen::Vector3d C_p_B_eig;

        Eigen::Matrix4d C_T_B_eig;
        Eigen::Matrix4d B_T_C_eig;
        Eigen::Matrix4d B_T_C_eig_first;

        std::string cam_config_file_path;

        bool first_frame;
        double current_timestamp;
        double previous_timestamp;

        Odom currentpose;

        Eigen::Matrix4d C0_T_Ck;
        Eigen::Matrix4d C0_T_Ck_1;

        relativePose latestRP; // latest relative pose

        double cumulative_rep_err = 0;
    };
}

#endif //CAMERA_IMU_LIDAR_CALIBRATION_CAMERAPOSETRACKING_H
