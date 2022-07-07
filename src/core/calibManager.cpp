//
// Created by usl on 12/9/20.
//

#include "calibManager.h"

using namespace calib_core;
using namespace calib_type;
using namespace calib_estimator;

calib_estimator::calibManager::calibManager(calibManagerOptions& params_) {
    /// Startup  message
    // Nice startup message
    printf("=======================================\n");
    printf("Lidar Inertial Calibration ON-MANIFOLD EKF IS STARTING\n");
    printf("=======================================\n");

    /// Read camera params
    K = cv::Mat::zeros(3, 3, CV_64F);
    D = cv::Mat::zeros(5, 1, CV_64F);
    cv::FileStorage fs_cam_config(params.camera_calibration_file_path, cv::FileStorage::READ);
    if(!fs_cam_config.isOpened())
        std::cerr << "Error: Wrong path: " << params.camera_calibration_file_path << std::endl;
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
    std::cout << "K: " << std::endl;
    std::cout << K << std::endl;
    std::cout << "D: " << std::endl;
    std::cout << D << std::endl;

    /// Nice debug
    this->params = params_;
    params.print_estimator();
    params.print_noise();
    params.print_state();
    params.print_trackers();

    /// File to store lin trajectory
    trajfile_csv.open(params.inertial_trajectory_filename);
    bias_csv.open(params.inertial_bias_filename);
    velocity_csv.open(params.inertial_velocity_filename);
    calib_lidar_imu_extrinsic_csv.open(params.lidar_inertial_calib_extrinsic_filename);
    calib_lidar_imu_dt_csv.open(params.lidar_inertial_calib_dt_filename);
    calib_camera_imu_extrinsic_csv.open(params.visual_inertial_calib_extrinsic_filename);
    calib_camera_imu_dt_csv.open(params.visual_inertial_calib_dt_filename);
    residual_filename_csv_writer.open(params.residual_filename);

    /// Create the state
    state = new State(params.state_options);

    /// Time-offset from Lidar-IMU
    Eigen::VectorXd temp_lidarimu_dt;
    temp_lidarimu_dt.resize(1);
    temp_lidarimu_dt(0) = params.calib_lidar_imu_dt;
    state->_calib_dt_LIDARtoIMU->set_value(temp_lidarimu_dt);
    state->_calib_dt_LIDARtoIMU->set_fe(temp_lidarimu_dt);

    /// Extrinsic Calibration Lidar-IMU
    state->_calib_LIDARtoIMU->set_value(params.lidar_imu_extrinsics);
    state->_calib_LIDARtoIMU->set_fe(params.lidar_imu_extrinsics);

    /// Time-offset from Camera-IMU
    Eigen::VectorXd temp_cameraimu_dt;
    temp_cameraimu_dt.resize(1);
    temp_cameraimu_dt(0) = params.calib_camera_imu_dt;
    state->_calib_dt_CAMERAtoIMU->set_value(temp_cameraimu_dt);
    state->_calib_dt_CAMERAtoIMU->set_fe(temp_cameraimu_dt);

    /// Extrinsic Calibration Camera-IMU
    state->_calib_CAMERAtoIMU->set_value(params.camera_imu_extrinsics);
    state->_calib_CAMERAtoIMU->set_fe(params.camera_imu_extrinsics);

    /// Propagator
    propagator = new Propagator(params.imu_noises, params.gravity);

    /// Initializer
    initializer = new InertialInitializer(params.gravity, params.init_window_time, params.init_imu_thresh);

    /// Make the Updator
    updaterLidarOdometry = new UpdaterLidarOdometry(params.updaterOptions);
    updaterCameraTracking = new UpdaterCameraTracking(params.updaterOptions);
    updaterCameraLidarConstraint = new UpdaterCameraLidarConstraint(params.updaterOptions);

    /// Initialize Lidar Odometry object
    LOdom = std::make_shared<calib_core::LidarOdometry>(params.ndtResolution,
                                                        params.lidar_odometry_trajectory_filename, params.downSampleForMapping);

    /// Initialize the Camera Pose Object
    cameraTracking = std::make_shared<calib_core::cameraPoseTracking>(params.checkerboard_dx, params.checkerboard_dy,
                                                                      params.checkerboard_rows, params.checkerboard_cols,
                                                                      params.camera_calibration_file_path);

    /// Make Cardboard Detector
    lidar_plane_detector = new lidarPlaneDetector();
}

void calib_estimator::calibManager::projectLidarPointsOnImage() {
    if (calibration_board_points != nullptr && !image_measurement.empty()) {
        std::vector<cv::Point3f> object_points_L;
        std::vector<double> object_points_intensities;
        for (int i = 0; i < calibration_board_points->points.size(); ++i) {
            cv::Point3f object_point_L = cv::Point3f (calibration_board_points->points[i].x,
                                                      calibration_board_points->points[i].y, calibration_board_points->points[i].z);
            object_points_intensities.push_back(calibration_board_points->points[i].intensity);
            object_points_L.push_back(object_point_L);
        }
        Pose* I_calib_L = state->_calib_LIDARtoIMU;
        Pose* I_calib_C = state->_calib_CAMERAtoIMU;
        Eigen::Matrix3d I_R_L = I_calib_L->Rot();
        Eigen::Vector3d I_p_L = I_calib_L->pos();
        Eigen::Matrix4d I_T_L = Eigen::Matrix4d::Identity();
        I_T_L.block(0, 0, 3, 3) = I_R_L;
        I_T_L.block(0, 3, 3, 1) = I_p_L;
        Eigen::Matrix3d I_R_C = I_calib_C->Rot();
        Eigen::Vector3d I_p_C = I_calib_C->pos();
        Eigen::Matrix4d I_T_C = Eigen::Matrix4d::Identity();
        I_T_C.block(0, 0, 3, 3) = I_R_C;
        I_T_C.block(0, 3, 3, 1) = I_p_C;

        /// IMU pose at lidar measurement timestamp
        Pose *imuPose_i = state->_clones_IMU.at(timestamp_lidar);
        Eigen::Matrix<double, 3, 3> Ii_R_G = imuPose_i->Rot();
        Eigen::Matrix<double, 3, 1> G_p_Ii = imuPose_i->pos();
        Eigen::Matrix4d G_T_Ii = Eigen::Matrix4d::Identity();
        G_T_Ii.block(0, 0, 3, 3) = Ii_R_G.transpose();
        G_T_Ii.block(0, 3, 3, 1) = G_p_Ii;

        /// IMU pose at camera measurement timestamp
        Pose *imuPose_j = state->_clones_IMU.at(timestamp_camera);
        Eigen::Matrix<double, 3, 3> Ij_R_G = imuPose_j->Rot();
        Eigen::Matrix<double, 3, 1> G_p_Ij = imuPose_j->pos();
        Eigen::Matrix4d G_T_Ij = Eigen::Matrix4d::Identity();
        G_T_Ij.block(0, 0, 3, 3) = Ij_R_G.transpose();
        G_T_Ij.block(0, 3, 3, 1) = G_p_Ij;

        Eigen::Matrix4d C_T_L = I_T_C.inverse()*G_T_Ij.inverse()*G_T_Ii*I_T_L;
        Eigen::Matrix3d C_R_L = C_T_L.block(0, 0, 3, 3);
        Eigen::Vector3d C_t_L = C_T_L.block(0, 3, 3, 1);
        cv::Mat rvec, tvec;
        cv::eigen2cv(C_R_L, rvec);
        cv::eigen2cv(C_t_L, tvec);
        std::vector<cv::Point2f> projected_points;
        cv::projectPoints(object_points_L, rvec, tvec, K, D, projected_points, cv::noArray());
        for(int i = 0; i < projected_points.size(); i++) {
            cv::circle(image_measurement, projected_points[i], 3, cv::Scalar(object_points_intensities[i],
                                                                             object_points_intensities[i], object_points_intensities[i]),
                       cv::FILLED, cv::LINE_4);
        }

        Eigen::Vector4d nd_l = nd_l_vector.at(timestamp_lidar);
        Eigen::Vector4d nd_c = nd_c_vector.at(timestamp_camera);

        if (nd_l(3) < 0) {
            nd_l = -nd_l;
        }

        if (nd_c(3) < 0) {
            nd_c = -nd_c;
        }

        std::cout << "nd_c: " << nd_c.transpose() << std::endl;
        std::cout << "nd_l: " << nd_l.transpose() << std::endl;

        cv::Mat image_out;
        cv::resize(image_measurement, image_out, cv::Size(), 0.5, 0.5);
        cv::imshow("Image", image_out);
        cv::waitKey(10);
    }
}

void calib_estimator::calibManager::do_undistortion(double timestamp,
                                                    TPointCloud& scan_raw,
                                                    TPointCloud::Ptr& scan_out,
                                                    Eigen::Matrix4d& T_ndt_predict) {
    scan_out->header = scan_raw.header;
    scan_out->is_dense = scan_raw.is_dense;

    /// IMU to LIDAR extrinsic calibration
    Pose *calibration = state->_calib_LIDARtoIMU;
    Eigen::Matrix<double, 3, 3> I_R_L = calibration->Rot();
    Eigen::Matrix<double, 3, 1> I_t_L = calibration->pos();

    Eigen::Matrix<double, 13, 1> imu_state_start;

    double pointStartTimeStamp;
    std::vector<uint32_t > point_timestamps;
    std::map<uint32_t , Eigen::Matrix<double, 13, 1> > stamped_poses;

    ros::Time time_start = ros::Time::now();
    scan_out->header = scan_raw.header;
    scan_out->height = scan_raw.height;
    scan_out->width = scan_raw.width;
    scan_out->is_dense = scan_raw.is_dense;
    scan_out->resize(scan_raw.width*scan_raw.height);
    for(int h = 0; h < scan_raw.height; h++) {
        for(int w = 0; w < scan_raw.width; w++) {
            TPoint scan_point = scan_raw.at(w, h);
            uint32_t point_timestamp = scan_raw.at(w, h).t;
            Eigen::Vector3d skewedPoint = Eigen::Vector3d(scan_point.x, scan_point.y, scan_point.z);
            /// Ignore NaNs
            if(isnan(scan_point.x) || isnan(scan_point.y) || isnan(scan_point.z)) {
                continue;
            }

            if (!point_timestamps.empty()) {
                auto it = find(point_timestamps.begin(), point_timestamps.end(), point_timestamp);
                if (it == point_timestamps.end()) {
                    /// New timestamp
                    point_timestamps.push_back(point_timestamp);
                    double pointCurrTimeStamp = timestamp + point_timestamp/1e9;
                    Eigen::Matrix<double, 13, 1> imu_state_plus;
                    propagator->fast_state_propagate(state, pointCurrTimeStamp, imu_state_plus);
                    stamped_poses.insert(std::make_pair(point_timestamp, imu_state_plus));
                }
            } else {
                /// This is the first point
//                assert(i == 0);
                point_timestamps.push_back(point_timestamp);
                pointStartTimeStamp = timestamp + point_timestamp/1e9;
                propagator->fast_state_propagate(state, pointStartTimeStamp, imu_state_start);
                stamped_poses.insert(std::make_pair(point_timestamp, imu_state_start));
            }
            Eigen::Matrix<double, 13, 1> imu_state_plus = stamped_poses.find(point_timestamp)->second;
            Eigen::Vector3d deskewedPoint = deskewPoint(imu_state_start, imu_state_plus, skewedPoint, I_R_L, I_t_L);
            TPoint deskewed_scan_point;
            deskewed_scan_point.x = deskewedPoint.x();
            deskewed_scan_point.y = deskewedPoint.y();
            deskewed_scan_point.z = deskewedPoint.z();
            deskewed_scan_point.intensity = scan_point.intensity;
            deskewed_scan_point.ring = scan_point.ring;
            deskewed_scan_point.range = scan_point.range;
            scan_out->at(w, h) = deskewed_scan_point;
        }
    }
    auto max_it = max_element(std::begin(point_timestamps), std::end(point_timestamps)); // c++11
    auto min_it = min_element(std::begin(point_timestamps), std::end(point_timestamps)); // c++11
//    ROS_INFO_STREAM("No of time stamps: " << point_timestamps.size());
    Eigen::Matrix<double, 13, 1> imu_state_plus_start;
    propagator->fast_state_propagate(state, timestamp + (double)*min_it/1e9, imu_state_plus_start);
    Eigen::Matrix<double, 4, 1> quat_start = imu_state_plus_start.block(0, 0, 4, 1);
    Eigen::Matrix<double, 3, 1> pos_start = imu_state_plus_start.block(4, 0, 3, 1);
    Eigen::Matrix<double, 3, 3> start_R_G = calib_core::quat_2_Rot(quat_start);

    Eigen::Matrix<double, 13, 1> imu_state_plus_end;
    propagator->fast_state_propagate(state, timestamp + (double)*max_it/1e9, imu_state_plus_end);
    Eigen::Matrix<double, 4, 1> quat_end = imu_state_plus_end.block(0, 0, 4, 1);
    Eigen::Matrix<double, 3, 1> pos_end = imu_state_plus_start.block(4, 0, 3, 1);
    Eigen::Matrix<double, 3, 3> end_R_G = calib_core::quat_2_Rot(quat_end);

    Eigen::Matrix<double, 3, 3> start_R_end = start_R_G*end_R_G.transpose();
    Eigen::Matrix<double, 3, 1> start_p_end = start_R_G * (pos_end - pos_start);

    Eigen::Matrix3d Lstart_R_Lend = I_R_L.transpose()*start_R_end*I_R_L;
    Eigen::Vector3d Lstart_t_Lend = I_R_L.transpose()*start_R_end*I_t_L + I_R_L.transpose()*(start_p_end - I_t_L);

    T_ndt_predict = Eigen::Matrix4d::Identity();
    T_ndt_predict.block(0, 0, 3, 3) = Lstart_R_Lend;
    T_ndt_predict.block(0, 3, 3, 1) = Lstart_t_Lend;

    ros::Time time_end = ros::Time::now();
    std::cout << YELLOW << "Time taken for deskewing: " << time_end.toSec() - time_start.toSec() << " [s]"<< std::endl;
}

TPointCloud::Ptr calib_estimator::calibManager::getCalibrationBoardPoints() {
    return calibration_board_points;
}

Eigen::Vector3d calib_estimator::calibManager::deskewPoint(Eigen::Matrix<double, 13, 1> start_point_state,
                                                           Eigen::Matrix<double, 13, 1> current_point_state,
                                                           Eigen::Vector3d skewedPoint,
                                                           Eigen::Matrix3d I_R_L,
                                                           Eigen::Vector3d I_t_L) {

    Eigen::Matrix<double, 4, 1> Istart_q_G = start_point_state.block(0, 0, 4, 1);
    Eigen::Matrix<double, 3, 3> Istart_R_G = calib_core::quat_2_Rot(Istart_q_G);
    Eigen::Vector3d p_Istart_in_G = start_point_state.block(4, 0, 3, 1);

    Eigen::Matrix<double, 4, 1> Icurr_q_G = current_point_state.block(0, 0, 4, 1);
    Eigen::Matrix<double, 3, 3> Icurr_R_G = calib_core::quat_2_Rot(Icurr_q_G);
    Eigen::Vector3d p_Icurr_in_G = current_point_state.block(4, 0, 3, 1);

    Eigen::Matrix<double, 3, 3> Istart_R_Icurr = Istart_R_G*Icurr_R_G.transpose();
    Eigen::Vector3d Istart_t_Icurr = Istart_R_G * (p_Icurr_in_G - p_Istart_in_G);

    Eigen::Matrix3d Lstart_R_Lcurr = I_R_L.transpose()*Istart_R_Icurr*I_R_L;
    Eigen::Vector3d Lstart_t_Lcurr = I_R_L.transpose()*Istart_R_Icurr*I_t_L + I_R_L.transpose()*(Istart_t_Icurr - I_t_L);

    Eigen::Vector3d deskewedPoint = Lstart_R_Lcurr * skewedPoint + Lstart_t_Lcurr;

    return deskewedPoint;
}

bool calib_estimator::calibManager::try_to_initialize() {

    /// Returns from our initializer
    double time0;
    Eigen::Matrix<double, 4, 1> q_GtoI0;
    Eigen::Matrix<double, 3, 1> b_w0, v_I0inG, b_a0, p_I0inG;

    /// Try to initialize the system
    /// We will wait for a jerk
    bool wait_for_jerk = true;
    bool success = initializer->initialize_with_imu(time0, q_GtoI0, b_w0,v_I0inG,
                                                    b_a0, p_I0inG, wait_for_jerk);

    /// Return if it failed
    if(!success) {
        return false;
    }

    /// Make big vector (q, p, v, bg, ba)
    Eigen::Matrix<double,16,1> imu_val;
    imu_val.block(0,0,4,1) = q_GtoI0;
    imu_val.block(4,0,3,1) << 0,0,0;
    imu_val.block(7,0,3,1) = v_I0inG;
    imu_val.block(10,0,3,1) = b_w0;
    imu_val.block(13,0,3,1) = b_a0;
    state->_imu->set_value(imu_val);
    state->_imu->set_fe(imu_val);
    state->_timestamp = time0;
    startup_time = time0;
    Eigen::Matrix3d I0_R_G = calib_core::quat_2_Rot(q_GtoI0);
    Eigen::Matrix3d G_R_I0 = I0_R_G.transpose();
    Eigen::Vector3d G_t_I0 = state->_imu->pos();
    G_T_I0.block(0, 0, 3, 3) = G_R_I0;
    G_T_I0.block(0, 3, 3, 1) = G_t_I0;
    logData();
    return true;
}

void calib_estimator::calibManager::feed_measurement_imu(double timestamp_imu, Eigen::Vector3d wm, Eigen::Vector3d am) {
    /// Push into the propagator
    propagator->feed_imu(timestamp_imu, wm, am);

    /// Push into our initializer
    if(!is_initialized_calibrator) {
        initializer->feed_imu(timestamp_imu, wm, am);
    }
}

void calib_estimator::calibManager::feed_measurement_camera(double timestamp, cv::Mat image_in) {
    if(!is_initialized_calibrator) {
        is_initialized_calibrator = try_to_initialize();
        if(!is_initialized_calibrator)
            return;
    }
    bool boarddetected = cameraTracking->feedImage(timestamp, image_in, objectpoints_c0, imagepoints);

    bool did_propagate_update = do_propagate_update(timestamp, boarddetected, image_in);

    if(state->_clones_IMU.size() == 1) {
        /// G_T_I1
        Eigen::Matrix<double, 4, 1> q_GtoI1 = state->_imu->quat();
        Eigen::Matrix3d I1_R_G = calib_core::quat_2_Rot(q_GtoI1);
        Eigen::Matrix3d G_R_I1 = I1_R_G.transpose();
        Eigen::Vector3d G_t_I1 = state->_imu->pos();
        G_T_I1.block(0, 0, 3, 3) = G_R_I1;
        G_T_I1.block(0, 3, 3, 1) = G_t_I1;
    }

    Eigen::Matrix4d G_T_Ik = Eigen::Matrix4d::Identity();
    Eigen::Matrix3d Ik_R_G = calib_core::quat_2_Rot(state->_imu->quat());
    Eigen::Matrix3d G_R_Ik = Ik_R_G.transpose();
    Eigen::Vector3d G_t_Ik = state->_imu->pos();
    G_T_Ik.block(0, 0, 3, 3) = G_R_Ik;
    G_T_Ik.block(0, 3, 3, 1) = G_t_Ik;
    Eigen::Matrix4d I0_T_Ik = G_T_I0.inverse()*G_T_Ik;
    Pose *calibration = state->_calib_CAMERAtoIMU;
    Eigen::Matrix3d I_R_C = calibration->Rot();
    Eigen::Vector3d I_t_C = calibration->pos();
    Eigen::Matrix4d I_T_C = Eigen::Matrix4d::Identity();
    I_T_C.block(0, 0, 3, 3) = I_R_C;
    I_T_C.block(0, 3, 3, 1) = I_t_C;
    reprojection_error = cameraTracking->checkReprojections(I_T_C, I0_T_Ik);
    ///
    if (nd_c_vector.size() > 25) {
        nd_c_vector.erase(nd_c_vector.begin());
    }
    nd_c_vector.insert(std::pair<double, Eigen::Vector4d>(timestamp, cameraTracking->getPlaneEquation()));
    ///
//    do_lidar_camera_update();
    /// Printing for debug
    logData();
}

void calib_estimator::calibManager::feed_measurement_lidar(double timestamp, TPointCloud ::Ptr cloud_raw) {
    if(!is_initialized_calibrator) {
        is_initialized_calibrator = try_to_initialize();
        if(!is_initialized_calibrator)
            return;
    }

    if(params.do_undistortion) {
        TPointCloud::Ptr cloud_undistorted(new TPointCloud);
        Eigen::Matrix4d T_ndt_predict = Eigen::Matrix4d::Identity();
        raw_cloud = *cloud_raw;
        do_undistortion(timestamp, raw_cloud, cloud_undistorted, T_ndt_predict);
        calibration_board_points = lidar_plane_detector->detectCalibrationTarget(cloud_undistorted);
        if(calibration_board_points != nullptr) {
            if (nd_l_vector.size() > 25) {
                nd_l_vector.erase(nd_l_vector.begin());
            }
            nd_l_vector.insert(std::pair<double, Eigen::Vector4d>(timestamp, lidar_plane_detector->getPlaneEquation()));
        }
        VPointCloud::Ptr cloud_XYZI_undistorted(new VPointCloud);
        TPointCloud2VPointCloud(cloud_undistorted, cloud_XYZI_undistorted);
        undistorted_cloud = *cloud_XYZI_undistorted;

        /// Lidar Odometry
        LOdom->feedScan(timestamp, cloud_XYZI_undistorted, T_ndt_predict);
    } else {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_XYZI(new pcl::PointCloud<pcl::PointXYZI>);
        TPointCloud2VPointCloud(cloud_raw, cloud_XYZI);

        /// Lidar Odometry
        LOdom->feedScan(timestamp, cloud_XYZI);
    }

    /// Propagate and Update
    do_propagate_update(timestamp);

    if(state->_clones_IMU.size() == 1) {
        /// G_T_I1
        Eigen::Matrix<double, 4, 1> q_GtoI1 = state->_imu->quat();
        Eigen::Matrix3d I1_R_G = calib_core::quat_2_Rot(q_GtoI1);
        Eigen::Matrix3d G_R_I1 = I1_R_G.transpose();
        Eigen::Vector3d G_t_I1 = state->_imu->pos();
        G_T_I1.block(0, 0, 3, 3) = G_R_I1;
        G_T_I1.block(0, 3, 3, 1) = G_t_I1;
    }
    ///
    do_lidar_camera_update();
    if(calibration_board_points != nullptr) {
        projectLidarPointsOnImage();
    }
    /// Printing for debug
    logData();
}

void calib_estimator::calibManager::do_propagate_update(double timestamp) {
    timestamp_lidar = timestamp;
    if(state->_timestamp >= timestamp) {
        printf(YELLOW "do_propagate_update: LIDAR-IMU: Stepping back in time!!! (prop dt = %3f)\n" RESET, (timestamp-state->_timestamp));
        return;
    }
    /// Propagate the state forward to the current update time
    /// Also augment it with a clone!
    propagator->propagate_and_clone(state, timestamp);
    /// Return if we are unable to propagate
    if (state->_timestamp != timestamp) {
        printf(RED "[PROP]: Propagator unable to propagate the state forward in time!\n" RESET);
        printf(RED "[PROP]: It has been %.3f since last time we propagated\n" RESET,timestamp-state->_timestamp);
        return;
    }

    /// Basically getting the timestamp of the first scan in the NDT map
    /// The surfelAssociation code needs this /// TODO: Remove this
    if(first_propagation) {
        first_propagation = false;
        map_time = timestamp;
    }
    if(state->_clones_IMU.size() < 2) {
        printf(YELLOW "[calib_estimator::calibManager::do_propagate_update-LIDAR] state->_clones_IMU.size() must be > 2\n");
        return;
    }
    /// Marginalize the oldest clone if needed
    if(did_update_lidarimu_1 && did_update_lidarimu_2) {
        StateHelper::marginalize_old_clone(state);
    }
    relativePose rP = LOdom->get_latest_relativePose();
    Eigen::Matrix4d L1_T_Lk = LOdom->get_current_odometry().pose;
    updaterLidarOdometry->updateScan2Scan(state, rP, did_update_lidarimu_1);
//    updaterLidarOdometry->updateScan2GlobalMapRotation(state, L1_T_Lk, G_T_I1, timestamp, did_update_lidarimu_1);
    updaterLidarOdometry->updateScan2GlobalMapTranslation(state, L1_T_Lk, G_T_I1, timestamp, did_update_lidarimu_2);
    if(did_update_lidarimu_1 && did_update_lidarimu_2) {
        /// Update our distance traveled
        if(timelastupdate != -1 && state->_clones_IMU.find(timelastupdate) != state->_clones_IMU.end()) {
            Eigen::Matrix<double,3,1> dx = state->_imu->pos() - state->_clones_IMU.at(timelastupdate)->pos();
            distance += dx.norm();
        }
        timelastupdate = timestamp;
        if(params.limit_map_size) {
            if(LOdom->get_odom_data().size() < params.no_of_scans_for_map)
                LOdom->append_and_update(true);
            else
                LOdom->append_and_update(false);
        } else {
            LOdom->append_and_update(true);
        }
    }
}

bool calib_estimator::calibManager::do_propagate_update(double timestamp, bool boarddetected, cv::Mat image) {
    timestamp_camera = timestamp;
    image_measurement = image;
    if(state->_timestamp >= timestamp) {
        printf(YELLOW "do_propagate_update: CAMERA-IMU: Stepping back in time!!! (prop dt = %3f)\n" RESET, (timestamp-state->_timestamp));
        return false;
    }
    /// Propagate the state forward to the current update time
    /// Also augment it with a clone!
    propagator->propagate_and_clone(state, timestamp);
    /// Return if we are unable to propagate
    if (state->_timestamp != timestamp) {
        printf(RED "[PROP]: Propagator unable to propagate the state forward in time!\n" RESET);
        printf(RED "[PROP]: It has been %.3f since last time we propagated\n" RESET,timestamp-state->_timestamp);
        return false;
    }
    if (boarddetected) {
        /// Marginalize the oldest clone if needed
        if(did_update_cameraimu) {
            StateHelper::marginalize_old_clone(state);
        }
        double residual3 = updaterCameraTracking->updatePixelBased(state, G_T_I0, imagepoints,
                                                                   objectpoints_c0, timestamp, did_update_cameraimu);
        if(did_update_cameraimu) {
            return true;
        }
    }
    return false;
}

void calib_estimator::calibManager::do_lidar_camera_update() {
    if(state->_clones_IMU.size() >= params.state_options.max_clone_size) {
        std::cout << BOLDBLUE << "Do Lidar Camera Update" << std::endl;
//        std::cout << BOLDCYAN << "Diff b/w Lidar Camera timestamp: " << fabs(timestamp_camera - timestamp_lidar) << std::endl;
//        std::cout << BOLDCYAN << "nd_c: " << nd_c_vector.at(timestamp_camera).transpose() << std::endl;
//        std::cout << BOLDCYAN << "nd_c size: " << nd_c_vector.size() << std::endl;
//        std::cout << BOLDCYAN << "nd_l: " << nd_l_vector.at(timestamp_lidar).transpose() << std::endl;
//        std::cout << BOLDCYAN << "nd_l size: " << nd_l_vector.size() << std::endl;
        bool do_update_lidar_camera1 = false;
        if(state->_clones_IMU.count(timestamp_camera) != 0 && state->_clones_IMU.count(timestamp_lidar) != 0) {
            residual_value = updaterCameraLidarConstraint->updatePlaneToPlaneConstraint(state,
                                                                       nd_c_vector.at(timestamp_camera).transpose(), timestamp_camera,
                                                                       nd_l_vector.at(timestamp_lidar).transpose(), timestamp_lidar,
                                                                       do_update_lidar_camera1);
        }
//        for(auto x : nd_l_vector) {
//            if(state->_clones_IMU.count(x.first) != 0) {
//                bool do_update_lidar_camera2 = false;
//                updaterCameraLidarConstraint->updatePlaneToPlaneConstraint(state, G_T_I0, objectpoints_c0,
//                                                                           x.second, x.first,
//                                                                           do_update_lidar_camera2);
//            }
//        }
//        bool do_update_lidar_camera3 = false;
//        updaterCameraLidarConstraint->updatePlaneToPlaneConstraint(state, G_T_I0, objectpoints_c0, nd_l_vector, do_update_lidar_camera3);
    }
}

void calib_estimator::calibManager::logData() {
//    std::cout << YELLOW << "Started Printing" << std::endl;
    Pose* calib_lidar2imu = state->_calib_LIDARtoIMU;
    Pose* calib_camera2imu = state->_calib_CAMERAtoIMU;

    Eigen::Matrix3d I_R_G = calib_core::quat_2_Rot(state->_imu->quat());
    Eigen::Vector3d G_euler_I = (I_R_G.transpose()).eulerAngles(0, 1, 2);
    double roll = atan2(sin(G_euler_I.x()), cos(G_euler_I.x()))*180/M_PI;
    double pitch = atan2(sin(G_euler_I.y()), cos(G_euler_I.y()))*180/M_PI;
    double yaw = atan2(sin(G_euler_I.z()), cos(G_euler_I.z()))*180/M_PI;

    /// 1
    std::vector<Type*> statevars_pose;
    statevars_pose.push_back(state->_imu->pose());
    Eigen::Matrix<double, 6, 6> covariance_imu_pose = StateHelper::get_marginal_covariance(state, statevars_pose);
    trajfile_csv << ros::Time(state->_timestamp).toNSec() << ", "
                 << state->_imu->quat().x() << ", " << state->_imu->quat().y() << ", "
                 << state->_imu->quat().z() << ", " << state->_imu->quat().w() << ", "
                 << state->_imu->pos().x() << ", " << state->_imu->pos().y() << ", "
                 << state->_imu->pos().z() << ", " << state->_imu->vel().x() << ", "
                 << state->_imu->vel().y() << ", " << state->_imu->vel().z() << std::endl;

    /// 2
    std::vector<Type*> statevars_bias_a;
    statevars_bias_a.push_back(state->_imu->ba());
    Eigen::Matrix<double, 3, 3> covariance_imu_ba = StateHelper::get_marginal_covariance(state, statevars_bias_a);
    std::vector<Type*> statevars_bias_g;
    statevars_bias_g.push_back(state->_imu->bg());
    Eigen::Matrix<double, 3, 3> covariance_imu_bg = StateHelper::get_marginal_covariance(state, statevars_bias_g);
    bias_csv << state->_imu->bias_a().x() << ", " << state->_imu->bias_a().y() << ", " << state->_imu->bias_a().z() << ", "
             << state->_imu->bias_g().x() << ", " << state->_imu->bias_g().y() << ", " << state->_imu->bias_g().z() << ", "
             << sqrt(covariance_imu_ba(0, 0)) << ", " << sqrt(covariance_imu_ba(1, 1)) << ", " << sqrt(covariance_imu_ba(2, 2)) << ", "
             << sqrt(covariance_imu_bg(0, 0)) << ", " << sqrt(covariance_imu_bg(1, 1)) << ", " << sqrt(covariance_imu_bg(2, 2)) << std::endl;

    /// 3
    std::vector<Type*> statevars_velocity;
    statevars_velocity.push_back(state->_imu->v());
    Eigen::Matrix<double, 3, 3> covariance_imu_velocity = StateHelper::get_marginal_covariance(state, statevars_velocity);
    velocity_csv << state->_imu->vel().x() << ", " << state->_imu->vel().y() << ", " << state->_imu->vel().z() << ", "
                 << sqrt(covariance_imu_velocity(0, 0)) << ", "<< sqrt(covariance_imu_velocity(1, 1)) << ", "<< sqrt(covariance_imu_velocity(2, 2)) << std::endl;

    /// 4
    std::vector<Type*> statevars_calib_lidar2imu_extrinsic;
    statevars_calib_lidar2imu_extrinsic.push_back(state->_calib_LIDARtoIMU);
    Eigen::Matrix<double, 6, 6> covariance_calib_lidar2imu_extrinsic = StateHelper::get_marginal_covariance(state, statevars_calib_lidar2imu_extrinsic);
    calib_lidar_imu_extrinsic_csv << calib_lidar2imu->quat()(0) << "," << calib_lidar2imu->quat()(1) << ", " << calib_lidar2imu->quat()(2) << ", " << calib_lidar2imu->quat()(3) << ", "
                                  << calib_lidar2imu->pos()(0) << "," << calib_lidar2imu->pos()(1) << "," << calib_lidar2imu->pos()(2) << ", "
                                  << sqrt(covariance_calib_lidar2imu_extrinsic(0, 0)) << ", " << sqrt(covariance_calib_lidar2imu_extrinsic(1, 1)) << ", " << sqrt(covariance_calib_lidar2imu_extrinsic(2, 2)) << ", "
                                  << sqrt(covariance_calib_lidar2imu_extrinsic(3, 3)) << ", " << sqrt(covariance_calib_lidar2imu_extrinsic(4, 4)) << ", " << sqrt(covariance_calib_lidar2imu_extrinsic(5, 5)) << std::endl;

    /// 5
    std::vector<Type*> statevars_calib_lidar2imu_dt;
    statevars_calib_lidar2imu_dt.push_back(state->_calib_dt_LIDARtoIMU);
    Eigen::Matrix<double, 1, 1> covariance_calib_lidar2imu_dt = StateHelper::get_marginal_covariance(state, statevars_calib_lidar2imu_dt);
    calib_lidar_imu_dt_csv << state->_calib_dt_LIDARtoIMU->value()(0) << ", " << sqrt(covariance_calib_lidar2imu_dt(0, 0)) << std::endl;

    /// 4
    std::vector<Type*> statevars_calib_camera2imu_extrinsic;
    statevars_calib_camera2imu_extrinsic.push_back(state->_calib_CAMERAtoIMU);
    Eigen::Matrix<double, 6, 6> covariance_calib_camera2imu_extrinsic = StateHelper::get_marginal_covariance(state, statevars_calib_camera2imu_extrinsic);
    calib_camera_imu_extrinsic_csv << calib_camera2imu->quat()(0) << "," << calib_camera2imu->quat()(1) << ", " << calib_camera2imu->quat()(2) << ", " << calib_camera2imu->quat()(3) << ", "
                                  << calib_camera2imu->pos()(0) << "," << calib_camera2imu->pos()(1) << "," << calib_camera2imu->pos()(2) << ", "
                                  << sqrt(covariance_calib_camera2imu_extrinsic(0, 0)) << ", " << sqrt(covariance_calib_camera2imu_extrinsic(1, 1)) << ", " << sqrt(covariance_calib_camera2imu_extrinsic(2, 2)) << ", "
                                  << sqrt(covariance_calib_camera2imu_extrinsic(3, 3)) << ", " << sqrt(covariance_calib_camera2imu_extrinsic(4, 4)) << ", " << sqrt(covariance_calib_camera2imu_extrinsic(5, 5)) << std::endl;

    /// 5
    std::vector<Type*> statevars_calib_camera2imu_dt;
    statevars_calib_camera2imu_dt.push_back(state->_calib_dt_CAMERAtoIMU);
    Eigen::Matrix<double, 1, 1> covariance_calib_camera2imu_dt = StateHelper::get_marginal_covariance(state, statevars_calib_camera2imu_dt);
    calib_camera_imu_dt_csv << state->_calib_dt_CAMERAtoIMU->value()(0) << ", " << sqrt(covariance_calib_camera2imu_dt(0, 0)) << std::endl;

    ///
    std::cout << "residual_value: " << residual_value << std::endl;
    residual_filename_csv_writer << residual_value << std::endl;
}

