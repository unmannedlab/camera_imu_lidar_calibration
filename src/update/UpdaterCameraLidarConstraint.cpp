//
// Created by usl on 4/5/22.
//

#include "UpdaterCameraLidarConstraint.h"

using namespace calib_core;
void calib_estimator::UpdaterCameraLidarConstraint::updatePlaneToPlaneConstraint(State *current_state,
                                                                                 Eigen::Vector4d nd_c, double timestamp_camera,
                                                                                 Eigen::Vector4d nd_l, double timestamp_lidar,
                                                                                 bool &did_update) {

    if(nd_c(3) < 0)
        nd_c = -nd_c;

    if(nd_l(3) < 0)
        nd_l = -nd_l;

    /// IMU pose at lidar measurement timestamp
    Pose *imuPose_i = current_state->_clones_IMU.at(timestamp_lidar);
    Eigen::Matrix<double, 3, 3> Ii_R_G = imuPose_i->Rot();
    Eigen::Matrix<double, 3, 1> G_p_Ii = imuPose_i->pos();

    /// IMU pose at camera measurement timestamp
    Pose *imuPose_j = current_state->_clones_IMU.at(timestamp_camera);
    Eigen::Matrix<double, 3, 3> Ij_R_G = imuPose_j->Rot();
    Eigen::Matrix<double, 3, 1> G_p_Ij = imuPose_j->pos();

    /// Lidar to IMU
    Pose *calibration_LidarToIMU = current_state->_calib_LIDARtoIMU;
    Eigen::Matrix<double, 3, 3> I_R_L = calibration_LidarToIMU->Rot();
    Eigen::Matrix<double, 3, 1> I_p_L = calibration_LidarToIMU->pos();

    /// Camera to IMU
    Pose *calibration_CameraToIMU = current_state->_calib_CAMERAtoIMU;
    Eigen::Matrix<double, 3, 3> I_R_C = calibration_CameraToIMU->Rot();
    Eigen::Matrix<double, 3, 1> I_p_C = calibration_CameraToIMU->pos();

    ///
    Eigen::Matrix<double, 3, 3> cj_R_li_hat = I_R_C.transpose() * Ij_R_G * Ii_R_G.transpose() * I_R_L;
    Eigen::Matrix<double, 3, 1> cj_p_li_hat = I_R_C.transpose() * Ij_R_G * (Ii_R_G.transpose() * I_p_L + G_p_Ii) - I_R_C.transpose() * (I_p_C + Ij_R_G * G_p_Ij);

    Eigen::Matrix<double, 3, 1> n_c = Eigen::Vector3d(nd_c(0), nd_c(1), nd_c(2));
    Eigen::Matrix<double, 3, 1> n_l = Eigen::Vector3d(nd_l(0), nd_l(1), nd_l(2));

    /// Residual from alignment of Normals
    Eigen::Matrix<double, 3, 1> rR = n_c - cj_R_li_hat * n_l;

    /// Residual from alignment of distance
    double rP = n_c.transpose() * cj_p_li_hat + nd_c(3) - nd_l(3);

    Eigen::Matrix<double, 4, 1> res;
    res(0) = rR(0);
    res(1) = rR(1);
    res(2) = rR(2);
    res(3) = rP;

    std::vector<Type*> x_order;
    int total_hx = 0;

    /// Add the clone i: IMU pose corresponding prev update time
    x_order.push_back(imuPose_i);
    total_hx += imuPose_i->size();

    /// Add the clone j: IMU pose corresponding curr update time
    x_order.push_back(imuPose_j);
    total_hx += imuPose_j->size();

    if(current_state->_options.do_calib_lidar_imu_extrinsic) {
        /// Add the extrinsic calibration param
        x_order.push_back(calibration_LidarToIMU);
        total_hx += calibration_LidarToIMU->size();
    }

    if(current_state->_options.do_calib_camera_imu_extrinsic) {
        /// Add the extrinsic calibration param
        x_order.push_back(calibration_CameraToIMU);
        total_hx += calibration_CameraToIMU->size();
    }

    Eigen::MatrixXd H_rR_IiRG = I_R_C.transpose()*Ij_R_G*Ii_R_G.transpose()* skew_x(I_R_L*n_l);
    Eigen::MatrixXd H_rR_IjRG = -I_R_C.transpose()* skew_x(Ij_R_G*Ii_R_G.transpose()*I_R_L*n_l);
    Eigen::MatrixXd H_rR_IRC = I_R_C.transpose()* skew_x(Ij_R_G*Ii_R_G.transpose()*I_R_L*n_l);
    Eigen::MatrixXd H_rR_IRL = -I_R_C.transpose()*Ij_R_G*Ii_R_G.transpose()* skew_x(I_R_L*n_l);

    Eigen::MatrixXd H_CjpLi_IiRG = -I_R_C.transpose()*Ij_R_G*Ii_R_G.transpose()*skew_x(I_p_L); Eigen::MatrixXd H_CjpLi_GpIi = Eigen::Matrix3d::Identity();
    Eigen::MatrixXd H_CjpLi_IjRG = I_R_C.transpose() * skew_x(Ij_R_G * (Ii_R_G.transpose() * I_p_L + G_p_Ii - G_p_Ij)); Eigen::MatrixXd H_CjpLi_GpIj = Eigen::Matrix3d::Identity();

    Eigen::MatrixXd H_CjpLi_IRC = -I_R_C.transpose() * skew_x(Ij_R_G * (Ii_R_G.transpose() * I_p_L + G_p_Ii - G_p_Ij)); Eigen::MatrixXd H_CjpLi_IpC = -I_R_C.transpose();
    Eigen::MatrixXd H_CjpLi_IRL = I_R_C.transpose()*Ij_R_G*Ii_R_G.transpose()*skew_x(I_p_L); Eigen::MatrixXd H_CjpLi_IpL = I_R_C.transpose()*Ij_R_G*Ii_R_G.transpose();

    Eigen::MatrixXd H_rP_IiRG = n_c.transpose()*H_CjpLi_IiRG; Eigen::MatrixXd H_rP_GpIi = n_c.transpose()*H_CjpLi_GpIi;
    Eigen::MatrixXd H_rP_IjRG = n_c.transpose() * H_CjpLi_IjRG; Eigen::MatrixXd H_rP_GpIj = n_c.transpose() * H_CjpLi_GpIj;

    Eigen::MatrixXd H_rP_IRC = n_c.transpose() * H_CjpLi_IRC; Eigen::MatrixXd H_rP_IpC = n_c.transpose()*H_CjpLi_IpC;
    Eigen::MatrixXd H_rP_IRL = n_c.transpose()*H_CjpLi_IRL; Eigen::MatrixXd H_rP_IpL = n_c.transpose()*H_CjpLi_IpL;

    Eigen::MatrixXd H_x = Eigen::MatrixXd::Zero(4, total_hx);
    H_x.block(0, 0, 3, 3) = -H_rR_IiRG; H_x.block(0, 3, 3, 3) = Eigen::Matrix3d::Zero();
    H_x.block(0, 6, 3, 3) = -H_rR_IjRG; H_x.block(0, 9, 3, 3) = Eigen::Matrix3d::Zero();
    H_x.block(0, 12, 3, 3) = -H_rR_IRL; H_x.block(0, 15, 3, 3) = Eigen::Matrix3d::Zero();
    H_x.block(0, 18, 3, 3) = -H_rR_IRC; H_x.block(0, 21, 3, 3) = Eigen::Matrix3d::Zero();

    H_x.block(3, 0, 1, 3) = -H_rP_IiRG; H_x.block(3, 3, 1, 3) = -H_rP_GpIi;
    H_x.block(3, 6, 1, 3) = -H_rP_IjRG; H_x.block(3, 9, 1, 3) = -H_rP_GpIj;
    H_x.block(3, 12, 1, 3) = -H_rP_IRL; H_x.block(3, 15, 1, 3) = -H_rP_IpL;
    H_x.block(3, 18, 1, 3) = -H_rP_IRC; H_x.block(3, 21, 1, 3) = -H_rP_IpC;


//    H_x.block(0, 0, 3, 3) = -H_rR_IRL; H_x.block(0, 3, 3, 3) = Eigen::Matrix3d::Zero();
//    H_x.block(0, 6, 3, 3) = -H_rR_IRC; H_x.block(0, 9, 3, 3) = Eigen::Matrix3d::Zero();
//    H_x.block(3, 0, 1, 3) = -H_rP_IRL; H_x.block(3, 3, 1, 3) = -H_rP_IpL;
//    H_x.block(3, 6, 1, 3) = -H_rP_IRC; H_x.block(3, 9, 1, 3) = -H_rP_IpC;

    Eigen::Matrix3d R_rot = std::pow(0.05, 2)*Eigen::Matrix3d::Identity();
    double R_trans = std::pow(0.05, 2);

    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(4, 4);
    R.block(0, 0, 3, 3) = R_rot;
    R(3, 3) = R_trans;

    /// Chi2 Check
    Eigen::MatrixXd P = StateHelper::get_marginal_covariance(current_state, x_order);
    Eigen::MatrixXd S = H_x * P * H_x.transpose() + R;
    double chi2 = res.dot(S.llt().solve(res));

    boost::math::chi_squared chi_squared_dist(4); // I know res.rows() is 4
    double chi2_check = boost::math::quantile(chi_squared_dist, 0.95);
    if(_options.do_chi2_check) {
        if (chi2 > chi2_check/10) {
            printf(BOLDRED "[Update : LiDAR-Camera] unsuccessful updatePlaneToPlaneConstraint \n" RESET);
            did_update = false;
        } else {
            /// Update
            StateHelper::EKFUpdate(current_state, x_order, H_x, res, R);
            printf(BOLDGREEN "[Update : LiDAR-Camera] successful updatePlaneToPlaneConstraint \n" RESET);
            did_update = true;
        }
    }
}

void calib_estimator::UpdaterCameraLidarConstraint::updatePlaneToPlaneConstraint(State *current_state,
                                                                                 Eigen::Matrix4d G_T_I0,
                                                                                 std::vector<cv::Point3f> object_points_c0,
                                                                                 Eigen::Vector4d nd_L, double timestamp,
                                                                                 bool &did_update) {
    if(nd_L(3) < 0)
        nd_L = -nd_L;

    /// Initial IMU pose t0
    Eigen::Matrix<double, 3, 3> G_R_I0 = G_T_I0.block(0, 0, 3, 3);
    Eigen::Matrix<double, 3, 3> I0_R_G = G_R_I0.transpose();
    Eigen::Matrix<double, 3, 1> G_p_I0 = G_T_I0.block(0, 3, 3, 1);

    /// IMU pose at time tk
    Pose *imuPose_k = current_state->_clones_IMU.at(timestamp);
    Eigen::Matrix<double, 3, 3> Ik_R_G_hat = imuPose_k->Rot(); // Ik_R_G_hat
    Eigen::Matrix<double, 3, 1> G_p_Ik_hat = imuPose_k->pos(); // p_Ik in G

    /// IMU to Camera extrinsic calibration I_T_C
    Pose *I_calib_C = current_state->_calib_CAMERAtoIMU;
    Eigen::Matrix<double, 3, 3> I_R_C_hat = I_calib_C->Rot();
    Eigen::Matrix<double, 3, 1> I_p_C_hat = I_calib_C->pos();

    /// IMU to Lidar extrinsic calibration I_T_L
    Pose *I_calib_L = current_state->_calib_LIDARtoIMU;
    Eigen::Matrix<double, 3, 3> I_R_L_hat = I_calib_L->Rot();
    Eigen::Matrix<double, 3, 1> I_p_L_hat = I_calib_L->pos();

    std::vector<Type*> x_order;
    int total_hx = 0;

    x_order.push_back(imuPose_k);
    total_hx += imuPose_k->size();

    if(current_state->_options.do_calib_camera_imu_extrinsic) {
        /// Add the extrinsic calibration param
        x_order.push_back(I_calib_C);
        total_hx += I_calib_C->size();
    }

    if(current_state->_options.do_calib_lidar_imu_extrinsic) {
        /// Add the extrinsic calibration param
        x_order.push_back(I_calib_L);
        total_hx += I_calib_L->size();
    }

    ///
    Eigen::Matrix<double, 3, 3> Lk_R_C0 = I_R_L_hat.transpose()*Ik_R_G_hat*I0_R_G.transpose()*I_R_C_hat;
    Eigen::Matrix<double, 3, 1> Lk_p_C0 = I_R_L_hat.transpose()*Ik_R_G_hat*(I0_R_G.transpose()*I_p_C_hat + G_p_I0)
                                        - I_R_L_hat.transpose()*(I_p_L_hat + Ik_R_G_hat*G_p_Ik_hat);

    ///
    Eigen::Matrix<double, 3, 3> H_LkpC0_IRL = -I_R_L_hat.transpose()* skew_x(Ik_R_G_hat*(I0_R_G.transpose()*I_p_C_hat + G_p_I0 - G_p_Ik_hat));
    Eigen::Matrix<double, 3, 3> H_LkpC0_IpL = -I_R_L_hat.transpose();

    Eigen::Matrix<double, 3, 3> H_LkpC0_IRC = I_R_L_hat.transpose()*Ik_R_G_hat*I0_R_G.transpose()* skew_x(I_p_C_hat);
    Eigen::Matrix<double, 3, 3> H_LkpC0_IpC = I_R_L_hat.transpose()*Ik_R_G_hat*I0_R_G.transpose();

    Eigen::Matrix<double, 3, 3> H_LkpC0_IkRG = I_R_L_hat.transpose()* skew_x(Ik_R_G_hat*(I0_R_G.transpose()*I_p_C_hat + G_p_I0 - G_p_Ik_hat));
    Eigen::Matrix<double, 3, 3> H_LkpC0_GpIk = -I_R_L_hat.transpose()*Ik_R_G_hat;

    ///
    Eigen::Vector3d n_L = Eigen::Vector3d(nd_L(0), nd_L(1), nd_L(2));
    double d_L = nd_L(3);
    Eigen::Matrix<double, 42, 1> res;
    Eigen::MatrixXd H_x = Eigen::MatrixXd::Zero(42, total_hx);
    for (int i = 0; i < object_points_c0.size(); ++i) {
        Eigen::Vector3d P_C0 = Eigen::Vector3d(object_points_c0[i].x, object_points_c0[i].y, object_points_c0[i].z);
        Eigen::Vector3d P_Lk = Lk_R_C0*P_C0 + Lk_p_C0;
        double residual = n_L.transpose()*P_Lk + d_L;
        res(i, 0) = residual;
        Eigen::Matrix<double, 1, 3> H_r_IkRG = n_L.transpose()*(I_R_L_hat.transpose()* skew_x(Ik_R_G_hat*I0_R_G.transpose()*I_R_C_hat*P_C0) + H_LkpC0_IkRG);
        Eigen::Matrix<double, 1, 3> H_r_GpIk = n_L.transpose()*H_LkpC0_GpIk;

        Eigen::Matrix<double, 1, 3> H_r_IRC = n_L.transpose()*(I_R_L_hat.transpose()*Ik_R_G_hat*I0_R_G.transpose()*skew_x(I_R_C_hat*P_C0) + H_LkpC0_IRC);
        Eigen::Matrix<double, 1, 3> H_r_IpC = n_L.transpose()*H_LkpC0_IpC;

        Eigen::Matrix<double, 1, 3> H_r_IRL = n_L.transpose()*(-I_R_L_hat.transpose()* skew_x(Ik_R_G_hat*I0_R_G.transpose()*I_R_C_hat*P_C0) + H_LkpC0_IRL);
        Eigen::Matrix<double, 1, 3> H_r_IpL = n_L.transpose()*H_LkpC0_IpL;

        H_x.block(i, 0, 1, 3) = -H_r_IkRG; H_x.block(i, 3, 1, 3) = -H_r_GpIk;
        H_x.block(i, 6, 1, 3) = -H_r_IRC; H_x.block(i, 9, 1, 3) = -H_r_IpC;
        H_x.block(i, 12, 1, 3) = -H_r_IRL; H_x.block(i, 15, 1, 3) = -H_r_IpL;
    }


    Eigen::MatrixXd R = std::pow(0.05, 2)*Eigen::MatrixXd::Identity(42, 42);

    /// Chi2 Check
    Eigen::MatrixXd P = StateHelper::get_marginal_covariance(current_state, x_order);
    Eigen::MatrixXd S = H_x * P * H_x.transpose() + R;
    double chi2 = res.dot(S.llt().solve(res));

    boost::math::chi_squared chi_squared_dist(42); // I know res.rows() is 42
    double chi2_check = boost::math::quantile(chi_squared_dist, 0.95);
    if(_options.do_chi2_check) {
        if (chi2 > chi2_check/10) {
            printf(BOLDRED "[Update : LiDAR-Camera] unsuccessful updatePlaneToPlaneConstraint \n" RESET);
            did_update = false;
        } else {
            /// Update
            StateHelper::EKFUpdate(current_state, x_order, H_x, res, R);
            printf(BOLDGREEN "[Update : LiDAR-Camera] successful updatePlaneToPlaneConstraint \n" RESET);
            did_update = true;
        }
    }
}

void calib_estimator::UpdaterCameraLidarConstraint::updatePlaneToPlaneConstraint(State *current_state, Eigen::Matrix4d G_T_I0,
                                                                                 std::vector<cv::Point3f> object_points_c0,
                                                                                 std::map<double, Eigen::Vector4d> nd_l_vector,
                                                                                 bool &did_update) {
    /// Initial IMU pose t0
    Eigen::Matrix<double, 3, 3> G_R_I0 = G_T_I0.block(0, 0, 3, 3);
    Eigen::Matrix<double, 3, 3> I0_R_G = G_R_I0.transpose();
    Eigen::Matrix<double, 3, 1> G_p_I0 = G_T_I0.block(0, 3, 3, 1);

    /// IMU to Camera extrinsic calibration I_T_C
    Pose *I_calib_C = current_state->_calib_CAMERAtoIMU;
    Eigen::Matrix<double, 3, 3> I_R_C_hat = I_calib_C->Rot();
    Eigen::Matrix<double, 3, 1> I_p_C_hat = I_calib_C->pos();

    /// IMU to Lidar extrinsic calibration I_T_L
    Pose *I_calib_L = current_state->_calib_LIDARtoIMU;
    Eigen::Matrix<double, 3, 3> I_R_L_hat = I_calib_L->Rot();
    Eigen::Matrix<double, 3, 1> I_p_L_hat = I_calib_L->pos();

    std::vector<Type*> x_order;
    int total_hx = 0;

    if(current_state->_options.do_calib_camera_imu_extrinsic) {
        /// Add the extrinsic calibration param
        x_order.push_back(I_calib_C);
        total_hx += I_calib_C->size();
    }

    if(current_state->_options.do_calib_lidar_imu_extrinsic) {
        /// Add the extrinsic calibration param
        x_order.push_back(I_calib_L);
        total_hx += I_calib_L->size();
    }

    std::vector<double> clonetimes;
    for(const auto x:nd_l_vector) {
        if(current_state->_clones_IMU.count(x.first)) {
            clonetimes.push_back(x.first);
            x_order.push_back(current_state->_clones_IMU.at(x.first));
            total_hx += current_state->_clones_IMU.at(x.first)->size();
        }
    }

    int no_of_good_measurements = clonetimes.size();
    Eigen::VectorXd res = Eigen::VectorXd::Zero(42*no_of_good_measurements);
    Eigen::MatrixXd H_x = Eigen::MatrixXd::Zero(42*no_of_good_measurements, total_hx);
    int row_counter = 0;
    int clone_counter = 0;
    for (const auto clonetime:clonetimes) {
        /// IMU pose at time tk
        Pose *imuPose_k = current_state->_clones_IMU.at(clonetime);
        Eigen::Matrix<double, 3, 3> Ik_R_G_hat = imuPose_k->Rot(); /// Ik_R_G_hat
        Eigen::Matrix<double, 3, 1> G_p_Ik_hat = imuPose_k->pos(); /// p_Ik in G

        ///
        Eigen::Vector4d nd_Lk = nd_l_vector.at(clonetime);
        if(nd_Lk(3) < 0) {
            nd_Lk = -nd_Lk;
        }
        Eigen::Vector3d n_Lk = Eigen::Vector3d(nd_Lk(0), nd_Lk(1), nd_Lk(2));
        double d_Lk = nd_Lk(3);
        ///
        Eigen::Matrix<double, 3, 3> Lk_R_C0 = I_R_L_hat.transpose()*Ik_R_G_hat*I0_R_G.transpose()*I_R_C_hat;
        Eigen::Matrix<double, 3, 1> Lk_p_C0 = I_R_L_hat.transpose()*Ik_R_G_hat*(I0_R_G.transpose()*I_p_C_hat + G_p_I0)
                                              - I_R_L_hat.transpose()*(I_p_L_hat + Ik_R_G_hat*G_p_Ik_hat);

        ///
        Eigen::Matrix<double, 3, 3> H_LkpC0_IRL = -I_R_L_hat.transpose()* skew_x(Ik_R_G_hat*(I0_R_G.transpose()*I_p_C_hat + G_p_I0 - G_p_Ik_hat));
        Eigen::Matrix<double, 3, 3> H_LkpC0_IpL = -I_R_L_hat.transpose();

        Eigen::Matrix<double, 3, 3> H_LkpC0_IRC = I_R_L_hat.transpose()*Ik_R_G_hat*I0_R_G.transpose()* skew_x(I_p_C_hat);
        Eigen::Matrix<double, 3, 3> H_LkpC0_IpC = I_R_L_hat.transpose()*Ik_R_G_hat*I0_R_G.transpose();

        Eigen::Matrix<double, 3, 3> H_LkpC0_IkRG = I_R_L_hat.transpose()* skew_x(Ik_R_G_hat*(I0_R_G.transpose()*I_p_C_hat + G_p_I0 - G_p_Ik_hat));
        Eigen::Matrix<double, 3, 3> H_LkpC0_GpIk = -I_R_L_hat.transpose()*Ik_R_G_hat;

        for (const auto object_point_c0:object_points_c0) {
            Eigen::Vector3d P_C0 = Eigen::Vector3d(object_point_c0.x, object_point_c0.y, object_point_c0.z);
            Eigen::Vector3d P_Lk = Lk_R_C0*P_C0 + Lk_p_C0;
            double residual = n_Lk.transpose()*P_Lk + d_Lk;
            res.block(row_counter, 0, 1, 1) << residual;
            Eigen::Matrix<double, 1, 3> H_r_IkRG = n_Lk.transpose()*(I_R_L_hat.transpose()* skew_x(Ik_R_G_hat*I0_R_G.transpose()*I_R_C_hat*P_C0) + H_LkpC0_IkRG);
            Eigen::Matrix<double, 1, 3> H_r_GpIk = n_Lk.transpose()*H_LkpC0_GpIk;

            Eigen::Matrix<double, 1, 3> H_r_IRC = n_Lk.transpose()*(I_R_L_hat.transpose()*Ik_R_G_hat*I0_R_G.transpose()*skew_x(I_R_C_hat*P_C0) + H_LkpC0_IRC);
            Eigen::Matrix<double, 1, 3> H_r_IpC = n_Lk.transpose()*H_LkpC0_IpC;

            Eigen::Matrix<double, 1, 3> H_r_IRL = n_Lk.transpose()*(-I_R_L_hat.transpose()* skew_x(Ik_R_G_hat*I0_R_G.transpose()*I_R_C_hat*P_C0) + H_LkpC0_IRL);
            Eigen::Matrix<double, 1, 3> H_r_IpL = n_Lk.transpose()*H_LkpC0_IpL;

            H_x.block(row_counter, 0, 1, 3) = -H_r_IRC;
            H_x.block(row_counter, 3, 1, 3) = -H_r_IpC;
            H_x.block(row_counter, 6, 1, 3) = -H_r_IRL;
            H_x.block(row_counter, 9, 1, 3) = -H_r_IpL;

            H_x.block(row_counter, 9 + 3*(clone_counter+1), 1, 3) = -H_r_IkRG;
            H_x.block(row_counter, 9 + 3*(clone_counter+2), 1, 3) = -H_r_GpIk;

            row_counter++;
        }
        clone_counter++;
    }

    Eigen::MatrixXd R = std::pow(0.05, 2)*Eigen::MatrixXd::Identity(42*no_of_good_measurements, 42*no_of_good_measurements);

    /// Chi2 Check
    Eigen::MatrixXd P = StateHelper::get_marginal_covariance(current_state, x_order);
    Eigen::MatrixXd S = H_x * P * H_x.transpose() + R;
    double chi2 = res.dot(S.llt().solve(res));

    boost::math::chi_squared chi_squared_dist(42*no_of_good_measurements); // I know res.rows() is 42
    double chi2_check = boost::math::quantile(chi_squared_dist, 0.95);
    if(_options.do_chi2_check) {
        if (chi2 > chi2_check/10) {
            printf(BOLDRED "[Update : LiDAR-Camera] unsuccessful updatePlaneToPlaneConstraint \n" RESET);
            did_update = false;
        } else {
            /// Update
            StateHelper::EKFUpdate(current_state, x_order, H_x, res, R);
            printf(BOLDGREEN "[Update : LiDAR-Camera] successful updatePlaneToPlaneConstraint \n" RESET);
            did_update = true;
        }
    }
}

