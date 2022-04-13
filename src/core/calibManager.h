//
// Created by usl on 12/8/20.
//

#ifndef CALIB_ESTIMATOR_CALIBMANAGER_H
#define CALIB_ESTIMATOR_CALIBMANAGER_H

#include <string>
#include <algorithm>
#include <fstream>
#include <Eigen/StdVector>
#include <boost/filesystem.hpp>

#include "init/InertialInitializer.h"

#include "state/State.h"
#include "state/StateHelper.h"
#include "state/Propagator.h"
#include "update/UpdaterLidarOdometry.h"
#include "update/UpdaterCameraTracking.h"
#include "update/UpdaterCameraLidarConstraint.h"

#include "track/lidarOdometry.h"
#include "track/cameraPoseTracking.h"

#include "track/lidarPlaneDetector.h"

#include "calibManagerOptions.h"

#include "utils/pcl_utils.h"

namespace calib_estimator {
    /// Core class that manages the entire system
    class calibManager {
    public:
        /// Constructor that will load all configuration variables
        calibManager(calibManagerOptions& param_);

        /// Feed function for inertial data
        void feed_measurement_imu(double timestamp_imu, Eigen::Vector3d wm, Eigen::Vector3d am);

        /// Feed function for lidar data
        void feed_measurement_lidar(double timestamp_lidar, pcl::PointCloud<calib_core::PointXYZIR8Y>::Ptr cloud_in);

        /// Feed function for camera data
        void feed_measurement_camera(double timestamp_camera, cv::Mat image_in);

        /// If we initialized or not
        bool initialized() {
            return is_initialized_calibrator;
        }

        /// Timestamp the system was initialized at
        double initialized_time() {
            return startup_time;
        }

        /// Accessor to get the current state
        State* get_state() {
            return state;
        }

        /// Accessor to get the current propagator
        Propagator* get_propagator() {
            return propagator;
        }

        /// Accessor to Lidar Odometry object
        LidarOdometry::Ptr get_track_lidar() {
            return LOdom;
        }

        /// Accessor to Camera Odometry object
        cameraPoseTracking::Ptr get_track_camera() {
            return cameraTracking;
        }

        /// Returns the last timestamp we have marginalized (true if we have a state)
        bool hist_last_marg_state(double &timestamp, Eigen::Matrix<double,7,1> &stateinG) {
            if(hist_last_marginalized_time != -1) {
                timestamp = hist_last_marginalized_time;
                stateinG = hist_stateinG.at(hist_last_marginalized_time);
                return true;
            } else {
                timestamp = -1;
                stateinG.setZero();
                return false;
            }
        }

        /// This will deskew a single point
        Eigen::Vector3d deskewPoint(Eigen::Matrix<double, 13, 1> start_point_state,
                                     Eigen::Matrix<double, 13, 1> current_point_state,
                                     Eigen::Vector3d skewedPoint,
                                     Eigen::Matrix3d I_R_L,
                                     Eigen::Vector3d I_t_L);

        /// This will deskew the entire scan/pointcloud
        void do_undistortion(double timestamp,
                             pcl::PointCloud<calib_core::PointXYZIR8Y>& scan_raw,
                             pcl::PointCloud<calib_core::PointXYZIR8Y>::Ptr& scan_out,
                             Eigen::Matrix4d& T_ndt_predict);

        /// This function will try to initialize the state
        /// This function could also be repurposed to re-initialize the system after failure
        bool try_to_initialize();

        /// Boolean if we are initialized or not
        bool is_initialized_calibrator = false;

        /// G_T_I0;
        Eigen::Matrix4d G_T_I0 = Eigen::Matrix4d::Identity();

        /// G_T_I1;
        Eigen::Matrix4d G_T_I1 = Eigen::Matrix4d::Identity();

        /// Get undistorted cloud
        VPointCloud get_undistorted_cloud() {
            return undistorted_cloud;
        }

        /// Get the time stamp of the first scan (used for building map)
        double get_map_time() {
            return map_time;
        }

        /// Print State for debugging
        void logData();

        /// Get the points on the detected calibration board
        TPointCloud::Ptr getCalibrationBoardPoints();

        /// Project points
        void projectLidarPointsOnImage();

    protected:

        /// This will do propagation and updates
        // Lidar
        void do_propagate_update(double timestamp);
        // Camera
        bool do_propagate_update(double timestamp, bool boarddetected, cv::Mat image);
        // Lidar-Camera
        void do_lidar_camera_update();

        ///The following will update our historical tracking information
        void update_keyframe_historical_information();


        /// Manager of parameters
        calibManagerOptions params;

        /// Our master state object
        State* state;

        /// Propagator of our state
        Propagator* propagator;

        /// State initializer
        InertialInitializer* initializer;

        /// Lidar Updater
        UpdaterLidarOdometry* updaterLidarOdometry;

        /// Camera Updater
        UpdaterCameraTracking* updaterCameraTracking;

        /// Lidar-Camera Updater
        UpdaterCameraLidarConstraint* updaterCameraLidarConstraint;

        /// Lidar Odometry object (Tracker)
        LidarOdometry::Ptr LOdom;

        /// Camera Odometry object (Tracker)
        cameraPoseTracking::Ptr cameraTracking;

        /// Track the distance travelled
        double timelastupdate = -1;
        double distance = 0;

        /// Start-up time of the filter
        double startup_time = -1;

        /// Historical information of the filter
        double hist_last_marginalized_time = -1;
        std::map<double, Eigen::Matrix<double, 7, 1> > hist_stateinG;

        std::ofstream trajfile_csv;
        std::ofstream bias_csv;
        std::ofstream velocity_csv;
        std::ofstream calib_lidar_imu_extrinsic_csv;
        std::ofstream calib_lidar_imu_dt_csv;
        std::ofstream calib_camera_imu_extrinsic_csv;
        std::ofstream calib_camera_imu_dt_csv;

        /// Raw Pointcloud
        TPointCloud raw_cloud;

        /// Undistorted Pointcloud
        VPointCloud undistorted_cloud;

        /// For Surfel Association
        double map_time;
        bool first_propagation = true;

        /// Update flags
        bool did_update_lidarimu_1 = false, did_update_lidarimu_2 = false;
        bool did_update_cameraimu = false;

        /// Reprojection Error for verification
        double reprojection_error;

        /// For pixel based update
        std::vector<cv::Point2f> imagepoints;
        std::vector<cv::Point3f> objectpoints_c0;

        /// Timestamps
        double timestamp_lidar;
        double timestamp_camera;

        /// Planar measurements
        std::map<double, Eigen::Vector4d> nd_c_vector;
        std::map<double, Eigen::Vector4d> nd_l_vector;

        /// Calibration board detector Lidar
        lidarPlaneDetector* lidar_plane_detector;
        TPointCloud::Ptr calibration_board_points;

        /// Camera image
        cv::Mat image_measurement;
        /// Camera params
        int image_height;
        int image_width;
        cv::Mat K, D;
    };
}
#endif //CALIB_ESTIMATOR_CALIBMANAGER_H
