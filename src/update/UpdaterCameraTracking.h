//
// Created by usl on 3/24/22.
//

#ifndef CAMERA_IMU_LIDAR_CALIBRATION_UPDATERCAMERATRACKING_H
#define CAMERA_IMU_LIDAR_CALIBRATION_UPDATERCAMERATRACKING_H

#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include "state/State.h"
#include "state/StateHelper.h"
#include "utils/quat_ops.h"
#include "utils/color.h"

#include "UpdaterOptions.h"

#include "relpose/relativePose.h"

#include <boost/math/distributions/chi_squared.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

namespace calib_estimator {
    /// Will compute the system for the lidar odometry measurement and update the filter
    class UpdaterCameraTracking {
    public:
        UpdaterCameraTracking(UpdaterOptions &options) : _options(options) {
            for(int i = 1; i < 500; i++) {
                boost::math::chi_squared chi_squared_dist(i);
                chi_squared_table[i] = boost::math::quantile(chi_squared_dist, 0.95);
            }
        }

        /// Given camera odometry, this will use them to update the state
        /// state: The current state of the system
        /// lodom: relative lidar odometry result that can be used for update
        double updateImage2Image(State *current_state, relativePose lodom, bool &did_update);

        /// Given camera odometry, this will use them to update the state
        /// state: The current state of the system
        /// L0_T_Lk: global camera odometry result that can be used for update
        double updateImage2FirstImage(State *current_state, Eigen::Matrix4d L0_T_Lk, Eigen::Matrix4d G_T_I1, double timestamp, bool &did_update);

        /// Given camera pixel detections, this will use them to update the state
        /// state: The current state of the system
        double updatePixelBased(State *current_state, Eigen::Matrix4d G_T_I0,
                                std::vector<cv::Point2f> pixel_measurements,
                                std::vector<cv::Point3f> object_points_c0,
                                double timestamp, bool &did_update);

    protected:

        /// Options used during update
        UpdaterOptions _options;

        /// Chi squared 95th percentile table (lookup would be size of residual)
        std::map<int, double> chi_squared_table;

    };
};

#endif //CAMERA_IMU_LIDAR_CALIBRATION_UPDATERCAMERATRACKING_H
