//
// Created by usl on 3/24/22.
//

#include "cameraPoseTracking.h"

namespace calib_core {
    cameraPoseTracking::cameraPoseTracking(double dx_, double dy_,
                                           int checkerboard_rows_,
                                           int checkerboard_cols_,
                                           std::string cam_config_file_path_) {
        cam_config_file_path = cam_config_file_path_;
        dx = dx_;
        dy = dy_;
        checkerboard_rows = checkerboard_rows_;
        checkerboard_cols = checkerboard_cols_;

        projection_matrix = cv::Mat::zeros(3, 3, CV_64F);
        distCoeff = cv::Mat::zeros(5, 1, CV_64F);

        readCameraParams(cam_config_file_path, image_height, image_width, distCoeff, projection_matrix);

        for(int i = 0; i < checkerboard_rows; i++)
            for (int j = 0; j < checkerboard_cols; j++)
                object_points.emplace_back(cv::Point3f(i*dx, j*dy, 0.0));

        first_frame = true;

        C0_T_Ck_1 = Eigen::Matrix4d::Identity();
    }

    void cameraPoseTracking::readCameraParams(std::string cam_config_file_path,
                                              int &image_height,
                                              int &image_width,
                                              cv::Mat &D,
                                              cv::Mat &K) {
        cv::FileStorage fs_cam_config(cam_config_file_path, cv::FileStorage::READ);
        if(!fs_cam_config.isOpened())
            std::cerr << "Error: Wrong path: " << cam_config_file_path << std::endl;
        fs_cam_config["image_height"] >> image_height;
        fs_cam_config["image_width"] >> image_width;
        std::cout << "image_height: " << image_height << std::endl;
        std::cout << "image_width: " << image_width << std::endl;
        fs_cam_config["k1"] >> D.at<double>(0);
        fs_cam_config["k2"] >> D.at<double>(1);
        fs_cam_config["p1"] >> D.at<double>(2);
        fs_cam_config["p2"] >> D.at<double>(3);
        fs_cam_config["k3"] >> D.at<double>(4);
        fs_cam_config["fx"] >> K.at<double>(0, 0);
        fs_cam_config["fy"] >> K.at<double>(1, 1);
        fs_cam_config["cx"] >> K.at<double>(0, 2);
        fs_cam_config["cy"] >> K.at<double>(1, 2);
        C_T_B_eig = Eigen::Matrix4d::Identity();
        B_T_C_eig = Eigen::Matrix4d::Identity();
    }

    bool cameraPoseTracking::feedImage(double timestamp, cv::Mat input_image,
                                       std::vector<cv::Point3f> &objectpoints_C0,
                                       std::vector<cv::Point2f> &imagepoints) {
        current_timestamp = timestamp;
        image_in_for_poseestimation = input_image.clone();
        image_in_for_visualization = input_image;
        cv::cvtColor(image_in_for_poseestimation, image_in_for_poseestimation, cv::COLOR_BGR2GRAY);
        image_points.clear();
        imagepoints.clear();
        bool boardDetectedInCam = cv::findChessboardCorners(image_in_for_poseestimation,
                                                            cv::Size(checkerboard_cols, checkerboard_rows),
                                                            image_points);
        image_points_undistorted.clear();
        if(boardDetectedInCam){
            cv::undistortPoints(image_points, image_points_undistorted, projection_matrix, distCoeff);
            imagepoints = image_points_undistorted;
            cv::drawChessboardCorners(image_in_for_visualization, cv::Size(checkerboard_cols, checkerboard_rows),
                                      image_points, boardDetectedInCam);
            if(boardDetectedInCam) {
                assert(image_points.size() == object_points.size());
                estimateCameraPose();
                objectpoints_C0.clear();
                objectpoints_C0 = object_points_C0;
            }
        }
        return boardDetectedInCam;
    }

    void cameraPoseTracking::solvePnPProblem() {
        cv::solvePnP(object_points, image_points, projection_matrix, distCoeff, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
    }

    double cameraPoseTracking::visualizeImageProjections(cv::Mat rvec, cv::Mat tvec) {
        std::vector<cv::Point2f> projected_points;
        cv::projectPoints(object_points_C0, rvec, tvec, projection_matrix, distCoeff, projected_points, cv::noArray());

        double error = 0;
        for(int i = 0; i < projected_points.size(); i++){
//            cv::circle(image_in_for_visualization, projected_points[i], 3, cv::Scalar(185, 185, 45), -1, cv::LINE_AA, 0);
            error += (projected_points[i].x - image_points[i].x)*(projected_points[i].x - image_points[i].x);
            error += (projected_points[i].y - image_points[i].y)*(projected_points[i].y - image_points[i].y);
            cv::arrowedLine(image_in_for_visualization, image_points[i], projected_points[i], cv::Scalar(185, 185, 45),
                            2, cv::LINE_4);
        }

        error = sqrt(error/projected_points.size());
        return error;
    }

    Eigen::Vector4d cameraPoseTracking::getPlaneEquation() {
        return nd_c;
    }

    void cameraPoseTracking::estimateCameraPose() {
        solvePnPProblem();

        cv::Rodrigues(rvec, C_R_B_cv);
        cv::cv2eigen(C_R_B_cv, C_R_B_eig);
        C_p_B_eig = Eigen::Vector3d(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
        n_c = C_R_B_eig.block<3,1>(0, 2);
        d_c = -n_c.dot(C_p_B_eig);
        nd_c = Eigen::Vector4d(n_c.x(), n_c.y(), n_c.z(), d_c);
        C_T_B_eig.block(0, 0, 3, 3) = C_R_B_eig;
        C_T_B_eig.block(0, 3, 3, 1) = C_p_B_eig;

        B_T_C_eig = C_T_B_eig.inverse();

        if (first_frame) {
            B_T_C_eig_first = B_T_C_eig;
            Eigen::Matrix3d W_R_C0 = B_T_C_eig_first.block(0, 0, 3, 3);
            Eigen::Vector3d W_t_C0 = B_T_C_eig_first.block(0, 3, 3, 1);
            Eigen::Matrix3d C0_R_W = W_R_C0.transpose();
            Eigen::Vector3d C0_t_W = -C0_R_W*W_t_C0;
            object_points_C0.clear();
            for (int i = 0; i < object_points.size(); ++i) {
                Eigen::Vector3d X_B = Eigen::Vector3d(object_points[i].x, object_points[i].y, object_points[i].z);
                Eigen::Vector3d X_C0 = C0_R_W*X_B + C0_t_W;
                cv::Point3d X_C0_cv = cv::Point3d(X_C0.x(), X_C0.y(), X_C0.z());
                object_points_C0.push_back(X_C0_cv);
            }
        }

        C0_T_Ck = B_T_C_eig_first.inverse() * B_T_C_eig;
        Eigen::Matrix4d deltaPose = C0_T_Ck_1.inverse()*C0_T_Ck;

        if(!first_frame) {
            latestRP.timestamp_i = previous_timestamp;
            latestRP.timestamp_j = current_timestamp;
            latestRP.odometry_ij = deltaPose;
        }

        currentpose.timestamp = current_timestamp;
        currentpose.pose = C0_T_Ck;

        C0_T_Ck_1 = C0_T_Ck;
        previous_timestamp = current_timestamp;
        first_frame = false;
    }

    double cameraPoseTracking::checkReprojections(Eigen::Matrix4d I_T_C, Eigen::Matrix4d I0_T_Ik) {
        Eigen::Matrix4d b_T_c = I_T_C.inverse() * I0_T_Ik * I_T_C;
        Eigen::Matrix4d c_T_b = b_T_c.inverse();
        Eigen::Matrix3d c_R_b = c_T_b.block(0, 0, 3, 3);
        cv::Mat c_R_b_cv, rvec;
        Eigen::Vector3d c_t_b = c_T_b.block(0, 3, 3, 1);
        cv::Mat c_t_b_cv;
        cv::eigen2cv(c_R_b, c_R_b_cv);
        cv::eigen2cv(c_t_b, c_t_b_cv);
        cv::Rodrigues(c_R_b_cv, rvec);

        double reperr = visualizeImageProjections(rvec, c_t_b_cv);
        cumulative_rep_err += reperr;
        cv::putText(image_in_for_visualization, //target image
                    "Current Rep Error: " + std::to_string(reperr), //text
                    cv::Point(image_in_for_visualization.cols / 2-400, 80),
                    cv::FONT_HERSHEY_DUPLEX,
                    2.0,
                    CV_RGB(118, 185, 0), //font color
                    4);
        cv::putText(image_in_for_visualization, //target image
                    "Cumulative Rep Error: " + std::to_string(cumulative_rep_err), //text
                    cv::Point(image_in_for_visualization.cols / 2-400, 160),
                    cv::FONT_HERSHEY_DUPLEX,
                    2.0,
                    CV_RGB(118, 185, 0), //font color
                    4);
        return reperr;
    }

    relativePose cameraPoseTracking::getRelativePose() {
        return latestRP;
    }

    cameraPoseTracking::Odom cameraPoseTracking::getCameraPose() {
        return currentpose;
    }
}