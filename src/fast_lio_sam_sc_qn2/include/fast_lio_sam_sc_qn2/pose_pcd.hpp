#ifndef FAST_LIO_SAM_SC_QN2_POSE_PCD_HPP
#define FAST_LIO_SAM_SC_QN2_POSE_PCD_HPP

#include <fast_lio_sam_sc_qn2/utilities.hpp>

struct PosePcd
{
    pcl::PointCloud<PointType> pcd_;
    Eigen::Matrix4d pose_eig_ = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d pose_corrected_eig_ = Eigen::Matrix4d::Identity();
    double timestamp_ = 0.0;
    int idx_ = 0;
    bool processed_ = false;

    PosePcd() = default;
    PosePcd(const nav_msgs::msg::Odometry &odom_in,
            const sensor_msgs::msg::PointCloud2 &pcd_in,
            const int &idx_in,
            const bool cloud_is_in_world_frame = true);
};

inline PosePcd::PosePcd(const nav_msgs::msg::Odometry &odom_in,
                        const sensor_msgs::msg::PointCloud2 &pcd_in,
                        const int &idx_in,
                        const bool cloud_is_in_world_frame)
{
    pose_eig_ = odomMsgToPoseEig(odom_in);
    pose_corrected_eig_ = pose_eig_;

    pcl::PointCloud<PointType> tmp_pcd;
    pcl::fromROSMsg(pcd_in, tmp_pcd);
    pcd_ = cloud_is_in_world_frame ? transformPcd(tmp_pcd, pose_eig_.inverse()) : tmp_pcd;

    timestamp_ = static_cast<double>(odom_in.header.stamp.sec) +
                 static_cast<double>(odom_in.header.stamp.nanosec) * 1e-9;
    idx_ = idx_in;
}

#endif
