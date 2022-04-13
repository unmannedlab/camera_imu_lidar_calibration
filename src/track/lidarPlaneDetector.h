//
// Created by usl on 4/3/22.
//

#ifndef CAMERA_IMU_LIDAR_CALIBRATION_LIDARPLANEDETECTOR_H
#define CAMERA_IMU_LIDAR_CALIBRATION_LIDARPLANEDETECTOR_H

#define PCL_NO_PRECOMPILE // !! BEFORE ANY PCL INCLUDE!!

#include "utils/math_utils.h"
#include "utils/eigen_utils.h"
#include "utils/quat_ops.h"
#include "utils/pcl_utils.h"

#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <pcl/ModelCoefficients.h>

namespace calib_core {
    class lidarPlaneDetector {
    public:
        lidarPlaneDetector() {
            std::cout <<"-- Plane Target Detector Initialized --" << std::endl;
        }
        TPointCloud::Ptr detectCalibrationTarget(TPointCloud::Ptr scan_raw);
        Eigen::Vector4d getPlaneEquation();

    private:
        Eigen::Vector4d nd_l;
    };
}
#endif //CAMERA_IMU_LIDAR_CALIBRATION_LIDARPLANEDETECTOR_H
