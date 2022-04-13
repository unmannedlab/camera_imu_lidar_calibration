//
// Created by usl on 4/3/22.
//

#include "lidarPlaneDetector.h"

namespace calib_core {
    TPointCloud::Ptr lidarPlaneDetector::detectCalibrationTarget(TPointCloud::Ptr scan_raw) {
        TPointCloud::Ptr cloud_filtered_x(new TPointCloud);
        TPointCloud::Ptr cloud_filtered_xy(new TPointCloud);
        TPointCloud::Ptr cloud_filtered_xyz(new TPointCloud);
        TPointCloud::Ptr plane(new TPointCloud);
        TPointCloud::Ptr plane_filtered(new TPointCloud);

        /// Pass through filters
        pcl::PassThrough<TPoint> pass_x;
        pass_x.setInputCloud(scan_raw);
        pass_x.setFilterFieldName("x");
        pass_x.setFilterLimits(0.5, 2);
        pass_x.filter(*cloud_filtered_x);

        pcl::PassThrough<TPoint> pass_y;
        pass_y.setInputCloud(cloud_filtered_x);
        pass_y.setFilterFieldName("y");
        pass_y.setFilterLimits(-1, 1);
        pass_y.filter(*cloud_filtered_xy);

        pcl::PassThrough<TPoint> pass_z;
        pass_z.setInputCloud(cloud_filtered_xy);
        pass_z.setFilterFieldName("z");
        pass_z.setFilterLimits(-0.5, 0.5);
        pass_z.filter(*cloud_filtered_xyz);

        /// Plane Segmentation
        pcl::SampleConsensusModelPlane<TPoint>::Ptr model_p(new pcl::SampleConsensusModelPlane<TPoint>(cloud_filtered_xyz));
        pcl::RandomSampleConsensus<TPoint> ransac(model_p);
        ransac.setDistanceThreshold(0.01);
        ransac.computeModel();
        std::vector<int> inliers_indicies;
        ransac.getInliers(inliers_indicies);
        pcl::copyPointCloud<TPoint>(*cloud_filtered_xyz, inliers_indicies, *plane);
        double a = ransac.model_coefficients_(0);
        double b = ransac.model_coefficients_(1);
        double c = ransac.model_coefficients_(2);
        double sqrt_abc = sqrt(a*a + b*b + c*c);
        double d = ransac.model_coefficients_(3);

        nd_l = Eigen::Vector4d(a/sqrt_abc, b/sqrt_abc,c/sqrt_abc, d/sqrt_abc);

        /// Statistical Outlier Removal
        pcl::StatisticalOutlierRemoval<TPoint> sor;
        sor.setInputCloud(plane);
        sor.setMeanK (50);
        sor.setStddevMulThresh (1);
        sor.filter (*plane_filtered);

        return plane_filtered;
    }

    Eigen::Vector4d lidarPlaneDetector::getPlaneEquation() {
        return nd_l;
    }
}