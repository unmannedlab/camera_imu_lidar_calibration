//
// Created by usl on 3/24/22.
//

#include "UpdaterCameraTracking.h"

using namespace calib_core;
using namespace calib_estimator;

double UpdaterCameraTracking::updateImage2Image(State *current_state, relativePose lodom, bool &did_update) {
    /// Li_T_Lj
    Eigen::Matrix4d Li_T_Lj = lodom.odometry_ij;

    /// Odometry time stamps
    /// This has to correspond to the previous state
    /// I assume it does, have to remove this assumption later
    double odom_ts_i = lodom.timestamp_i;
    /// This has to correspond to the current state
    /// I assume it does, have to remove this assumption later
    double odom_ts_j = lodom.timestamp_j;

    /// IMU pose at time stamp i
    Pose *imuPose_i = current_state->_clones_IMU.at(odom_ts_i);
    Eigen::Matrix<double, 3, 3> Ii_R_G = imuPose_i->Rot(); // Ii_R_G
    Eigen::Matrix<double, 3, 1> G_p_Ii = imuPose_i->pos(); // p_Ii in G

    /// IMU pose at time stamp j
    Pose *imuPose_j = current_state->_clones_IMU.at(odom_ts_j);
    Eigen::Matrix<double, 3, 3> Ij_R_G = imuPose_j->Rot(); // Ij_R_G
    Eigen::Matrix<double, 3, 1> G_p_Ij = imuPose_j->pos(); // p_Ij in G
    Eigen::Matrix<double, 3, 3> G_R_Ij = Ij_R_G.transpose(); // G_R_Ij

    /// IMU to Camera extrinsic calibration
    Pose *calibration = current_state->_calib_CAMERAtoIMU;
    Eigen::Matrix<double, 3, 3> I_R_C = calibration->Rot();
    Eigen::Matrix<double, 3, 1> I_p_C = calibration->pos();
    /// Predicted measurements using best estimates of states
    Eigen::Matrix<double, 3, 3> Li_R_Lj_hat = I_R_C.transpose() * Ii_R_G * G_R_Ij * I_R_C;
    Eigen::Matrix<double, 3, 1> Li_p_Lj_hat = I_R_C.transpose() * (Ii_R_G * G_R_Ij * I_p_C + Ii_R_G * (G_p_Ij - G_p_Ii) - I_p_C );

    /// True measurements Li_R_Lj,  Li_p_Lj
    Eigen::Matrix<double, 3, 3> Li_R_Lj = Li_T_Lj.block(0, 0, 3, 3);
    Eigen::Matrix<double, 3, 1> Li_p_Lj = Li_T_Lj.block(0, 3, 3, 1);

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
        x_order.push_back(calibration);
        total_hx += calibration->size();
    }

    /// Assign Jacobian blocks
    /// H1 block, corresponding to res1
    Eigen::MatrixXd H1_xi_rot = Eigen::MatrixXd::Zero(3, 3);
    Eigen::MatrixXd H1_xi_trans = Eigen::MatrixXd::Zero(3, 3);
    Eigen::MatrixXd H1_xj_rot = Eigen::MatrixXd::Zero(3, 3);
    Eigen::MatrixXd H1_xj_trans = Eigen::MatrixXd::Zero(3, 3);
    H1_xi_rot = I_R_C.transpose();
    H1_xj_rot = -I_R_C.transpose() * Ii_R_G * Ij_R_G.transpose();

    Eigen::MatrixXd H_x = Eigen::MatrixXd::Zero(3, total_hx);
    H_x.block(0, 0, 3, 3) = H1_xi_rot;
    H_x.block(0, 3, 3, 3) = H1_xi_trans;
    H_x.block(0, 6, 3, 3) = H1_xj_rot;
    H_x.block(0, 9, 3, 3) = H1_xj_trans;

    Eigen::MatrixXd H1_xc_rot = Eigen::MatrixXd::Zero(3, 3);
    Eigen::MatrixXd H1_xc_trans = Eigen::MatrixXd::Zero(3, 3);
    if(current_state->_options.do_calib_lidar_imu_extrinsic) {
        H1_xc_rot = I_R_C.transpose() * (Ii_R_G * Ij_R_G.transpose() - Eigen::MatrixXd::Identity(3, 3));
        H_x.block(0, 12, 3, 3) = H1_xc_rot;
        H_x.block(0, 15, 3, 3) = H1_xc_trans;
    }

    /// Assign measurement residual
    Eigen::Matrix<double, 3, 1> res_rot =  Log_so3(Li_R_Lj*Li_R_Lj_hat.transpose());
    Eigen::MatrixXd R = std::pow(_options.noise_rotation, 2)*Eigen::MatrixXd::Identity(3, 3);

    /// Chi2 Check
    Eigen::MatrixXd P = StateHelper::get_marginal_covariance(current_state, x_order);
    Eigen::MatrixXd S = H_x * P * H_x.transpose() + R;
    double chi2 = res_rot.dot(S.llt().solve(res_rot));

    /// TODO: I actually need to check if the dimension should be 3 or not, what does the dimension really mean physically?
    boost::math::chi_squared chi_squared_dist(3); // I know res_trans.rows() is 3
    double chi2_check = boost::math::quantile(chi_squared_dist, 0.95);

    if(_options.do_chi2_check) {
        if (chi2 > chi2_check/10) {
            printf(BOLDRED "[Update : Camera] unsuccessful updateImage2Image \n" RESET);
            did_update = false;
            return -1;
        }
    }
    /// Update
    StateHelper::EKFUpdate(current_state, x_order, H_x, res_rot,
                           std::pow(_options.noise_rotation, 2) * Eigen::MatrixXd::Identity(3, 3));
    printf(BOLDGREEN "[Update : Camera] successful updateImage2Image \n" RESET);
    did_update = true;
    return sqrt(chi2);
}

double UpdaterCameraTracking::updateImage2FirstImage(State *current_state, Eigen::Matrix4d L1_T_Lk, Eigen::Matrix4d G_T_I1,
                                                     double timestamp, bool& did_update) {
    /// Initial IMU pose
    Eigen::Matrix<double, 3, 3> G_R_I1 = G_T_I1.block(0, 0, 3, 3);
    Eigen::Matrix<double, 3, 1> G_p_I1 = G_T_I1.block(0, 3, 3, 1);

    /// IMU pose at time stamp k
    Pose *imuPose_k = current_state->_clones_IMU.at(timestamp);
    Eigen::Matrix<double, 3, 3> Ik_R_G = imuPose_k->Rot(); // Ik_R_G
    Eigen::Matrix<double, 3, 1> G_p_Ik = imuPose_k->pos(); // p_Ik in G

    /// L0_R_Lk, L0_t_Lk
    Eigen::Matrix3d L1_R_Lk = L1_T_Lk.block(0, 0, 3, 3);
    Eigen::Vector3d L1_p_Lk = L1_T_Lk.block(0, 3, 3, 1);

    /// IMU to LIDAR extrinsic calibration
    Pose *calibration = current_state->_calib_CAMERAtoIMU;
    Eigen::Matrix<double, 3, 3> I_R_C = calibration->Rot();
    Eigen::Matrix<double, 3, 1> I_p_C = calibration->pos();

    std::vector<Type*> x_order;
    int total_hx = 0;

    /// Add the clone i: IMU pose corresponding prev update time
    x_order.push_back(imuPose_k);
    total_hx += imuPose_k->size();

    if(current_state->_options.do_calib_lidar_imu_extrinsic) {
        /// Add the extrinsic calibration param
        x_order.push_back(calibration);
        total_hx += calibration->size();
    }

    /// Assign Jacobian blocks

    /// H2 block, corresponding to res2 (translation residual)
    Eigen::MatrixXd H2_xk_rot = Eigen::MatrixXd::Zero(3, 3);
    Eigen::MatrixXd H2_xk_trans = Eigen::MatrixXd::Zero(3, 3);
    H2_xk_rot = -I_R_C.transpose() * G_R_I1.transpose() * Ik_R_G.transpose() * skew_x(I_p_C);
    H2_xk_trans = I_R_C.transpose() * G_R_I1.transpose();

    Eigen::MatrixXd H2_xc_rot = Eigen::MatrixXd::Zero(3, 3);
    Eigen::MatrixXd H2_xc_trans = Eigen::MatrixXd::Zero(3, 3);
    H2_xc_rot = -I_R_C.transpose() * skew_x(G_R_I1.transpose() * (G_p_Ik - G_p_I1) +
                                            (G_R_I1.transpose()*Ik_R_G.transpose() - Eigen::Matrix3d::Identity()) * I_p_C) +
                I_R_C.transpose() * (G_R_I1.transpose() * Ik_R_G.transpose() - Eigen::Matrix3d::Identity()) * skew_x(I_p_C);
    H2_xc_trans = I_R_C.transpose() * (G_R_I1.transpose() * Ik_R_G.transpose() - Eigen::Matrix3d::Identity());

    Eigen::MatrixXd H_x = Eigen::MatrixXd::Zero(3, total_hx);

    H_x.block(0, 0, 3, 3) = H2_xk_rot;
    H_x.block(0, 3, 3, 3) = H2_xk_trans;

    if(current_state->_options.do_calib_lidar_imu_extrinsic) {
        H_x.block(0, 6, 3, 3) = H2_xc_rot;
        H_x.block(0, 9, 3, 3) = H2_xc_trans;
    }

    /// Predicted measurement
    Eigen::Matrix<double, 3, 1> L1_p_Lk_hat = I_R_C.transpose() * (G_R_I1.transpose() * Ik_R_G.transpose() * I_p_C +
                                                                   G_R_I1.transpose() * (G_p_Ik - G_p_I1) - I_p_C); // Predicted translation

    /// Residual calculation
    Eigen::Matrix<double, 3, 1> res_trans = L1_p_Lk - L1_p_Lk_hat;

    Eigen::MatrixXd R = std::pow(_options.noise_translation, 2)*Eigen::MatrixXd::Identity(3, 3);

    /// Chi2 Check
    Eigen::MatrixXd P = StateHelper::get_marginal_covariance(current_state, x_order);
    Eigen::MatrixXd S = H_x * P * H_x.transpose() + R;
    double chi2 = res_trans.dot(S.llt().solve(res_trans));

    boost::math::chi_squared chi_squared_dist(3); // I know res_trans.rows() is 3
    double chi2_check = boost::math::quantile(chi_squared_dist, 0.95);

    /// TODO: I actually need to check if the dimension should be 3 or not, what does the dimension really mean physically?
    if(_options.do_chi2_check) {
        if (chi2 > chi2_check/10) {
            printf(BOLDRED "[Update : Camera] unsuccessful updateImage2FirstImage\n" RESET);
            did_update = false;
            return -1;
        }
    }

    /// Update
    StateHelper::EKFUpdate(current_state, x_order, H_x, res_trans, R);
    printf(BOLDGREEN "[Update : Camera] successful updateImage2FirstImage\n" RESET);
    did_update = true;
    return sqrt(chi2);
}

double UpdaterCameraTracking::updatePixelBased(State *current_state, Eigen::Matrix4d G_T_I0,
                                               std::vector<cv::Point2f> pixel_measurements,
                                               std::vector<cv::Point3f> object_points_c0,
                                               double timestamp, bool &did_update) {
    /// Initial IMU pose
    Eigen::Matrix<double, 3, 3> G_R_I0_hat = G_T_I0.block(0, 0, 3, 3);
    Eigen::Matrix<double, 3, 3> I0_R_G_hat = G_R_I0_hat.transpose();
    Eigen::Matrix<double, 3, 1> G_p_I0_hat = G_T_I0.block(0, 3, 3, 1);

    /// IMU pose at time stamp k
    Pose *imuPose_k = current_state->_clones_IMU.at(timestamp);
    Eigen::Matrix<double, 3, 3> Ik_R_G_hat = imuPose_k->Rot(); // Ik_R_G_hat
    Eigen::Matrix<double, 3, 1> G_p_Ik_hat = imuPose_k->pos(); // p_Ik in G

    /// IMU to Camera extrinsic calibration
    Pose *calibration = current_state->_calib_CAMERAtoIMU;
    Eigen::Matrix<double, 3, 3> R_c_hat = calibration->Rot();
    Eigen::Matrix<double, 3, 1> p_c_hat = calibration->pos();

    std::vector<Type*> x_order;
    int total_hx = 0;

    /// Add the clone i: IMU pose corresponding prev update time
    x_order.push_back(imuPose_k);
    total_hx += imuPose_k->size();

    if(current_state->_options.do_calib_camera_imu_extrinsic) {
        /// Add the extrinsic calibration param
        x_order.push_back(calibration);
        total_hx += calibration->size();
    }

    Eigen::Matrix<double, 3, 3> C_R_B_hat = R_c_hat.transpose() * Ik_R_G_hat * I0_R_G_hat.transpose() * R_c_hat;
    Eigen::Matrix<double, 3, 1> C_p_B_hat = R_c_hat.transpose() * ((Ik_R_G_hat * I0_R_G_hat.transpose() - Eigen::Matrix3d::Identity()) * p_c_hat
                                                                   + Ik_R_G_hat * (G_p_I0_hat - G_p_Ik_hat));

    /// Jacobian with respect to Extrinsic Calibration
    Eigen::Matrix<double, 3, 3> cRb_H_Rc = R_c_hat.transpose() * (Ik_R_G_hat * I0_R_G_hat.transpose() - Eigen::Matrix3d::Identity());
    Eigen::Matrix<double, 3, 3> cRb_H_pc = Eigen::Matrix3d::Zero();
    Eigen::Matrix<double, 3, 3> cpb_H_Rc = -R_c_hat.transpose() * skew_x(Ik_R_G_hat*I0_R_G_hat.transpose()*p_c_hat + Ik_R_G_hat*(G_p_I0_hat - G_p_Ik_hat) - p_c_hat)
                                           + R_c_hat.transpose()*(Ik_R_G_hat*I0_R_G_hat.transpose() - Eigen::Matrix3d::Identity())*skew_x(p_c_hat);
    Eigen::Matrix<double, 3, 3> cpb_H_pc = R_c_hat.transpose()*(Ik_R_G_hat*I0_R_G_hat.transpose() - Eigen::Matrix3d::Identity());

    /// Jacobian with respect to Ik_R_G, G_p_Ik
    Eigen::Matrix<double, 3, 3> cRb_H_IkRG = R_c_hat.transpose();
    Eigen::Matrix<double, 3, 3> cRb_H_GpIk = Eigen::Matrix3d::Zero();
    Eigen::Matrix<double, 3, 3> cpb_H_IkRG = R_c_hat.transpose()* skew_x(Ik_R_G_hat*(I0_R_G_hat.transpose()*p_c_hat + (G_p_I0_hat - G_p_Ik_hat)));
    Eigen::Matrix<double, 3, 3> cpb_H_GpIk = -R_c_hat.transpose()*Ik_R_G_hat;

    ///
    Eigen::Matrix<double, 84, 1> res;
    Eigen::MatrixXd H_x = Eigen::MatrixXd::Zero(84, total_hx);
    for (int i = 0; i < pixel_measurements.size(); ++i) {
        Eigen::Vector3d P_B = Eigen::Vector3d(object_points_c0[i].x, object_points_c0[i].y, object_points_c0[i].z);
        Eigen::Vector3d P_C = C_R_B_hat*P_B + C_p_B_hat;
        double X_C = P_C.x(); double Y_C = P_C.y(); double Z_C = P_C.z();
        Eigen::Matrix<double, 2, 1> measurement;
        measurement(0, 0) = pixel_measurements[i].x;
        measurement(1, 0) = pixel_measurements[i].y;
        Eigen::Matrix<double, 2, 1> predicted_measurement;
        predicted_measurement(0, 0) = X_C/Z_C;
        predicted_measurement(1, 0) = Y_C/Z_C;

        Eigen::Matrix<double, 2, 1> res_pixel = measurement - predicted_measurement;
        res(2*i, 0) = res_pixel(0, 0);
        res(2*i+1, 0) = res_pixel(1, 0);

        Eigen::Matrix<double, 2, 3> pi_H_Pc;
        pi_H_Pc << 1/Z_C,     0, -X_C/(Z_C*Z_C),
                0, 1/Z_C, -Y_C/(Z_C*Z_C);

        Eigen::Matrix<double, 3, 3> Pc_H_Rc = skew_x(C_R_B_hat*P_B)*cRb_H_Rc + cpb_H_Rc;
        Eigen::Matrix<double, 3, 3> Pc_H_pc = cpb_H_pc;

        Eigen::Matrix<double, 3, 3> Pc_H_IkRG = skew_x(C_R_B_hat*P_B)*cRb_H_IkRG + cpb_H_IkRG;
        Eigen::Matrix<double, 3, 3> Pc_H_GpIk = cpb_H_GpIk;

        Eigen::Matrix<double, 2, 3> pi_H_Rc = pi_H_Pc*Pc_H_Rc;
        Eigen::Matrix<double, 2, 3> pi_H_pc = pi_H_Pc*Pc_H_pc;
        Eigen::Matrix<double, 2, 3> pi_H_IkRG = pi_H_Pc*Pc_H_IkRG;
        Eigen::Matrix<double, 2, 3> pi_H_GpIk = pi_H_Pc*Pc_H_GpIk;

        H_x.block(2*i, 0, 2, 3) = pi_H_IkRG;
        H_x.block(2*i, 3, 2, 3) = pi_H_GpIk;
        H_x.block(2*i, 6, 2, 3) = pi_H_Rc;
        H_x.block(2*i, 9, 2, 3) = pi_H_pc;
    }

    Eigen::MatrixXd R = std::pow(_options.noise_pixel, 2)*Eigen::MatrixXd::Identity(84, 84);

    /// Chi2 Check
    Eigen::MatrixXd P = StateHelper::get_marginal_covariance(current_state, x_order);
    Eigen::MatrixXd S = H_x * P * H_x.transpose() + R;
    double chi2 = res.dot(S.llt().solve(res));

    boost::math::chi_squared chi_squared_dist(84); // 84 = no of rows
    double chi2_check = boost::math::quantile(chi_squared_dist, 0.95);

    /// TODO: I actually need to check what does the dimension really mean physically?
    if(_options.do_chi2_check) {
        if (chi2 > chi2_check/10) {
            printf(BOLDRED "[Update : Camera] unsuccessful updatePixelBased\n" RESET);
            did_update = false;
            return -1;
        }
    }

    /// Update
    StateHelper::EKFUpdate(current_state, x_order, H_x, res, R);
    printf(BOLDGREEN "[Update : Camera] successful updatePixelBased\n" RESET);
    did_update = true;
    return sqrt(chi2);
}