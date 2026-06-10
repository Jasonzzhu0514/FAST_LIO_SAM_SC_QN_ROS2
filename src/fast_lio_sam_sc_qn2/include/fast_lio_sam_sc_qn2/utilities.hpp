#ifndef FAST_LIO_SAM_SC_QN2_UTILITIES_HPP
#define FAST_LIO_SAM_SC_QN2_UTILITIES_HPP

#include <string>

#include <Eigen/Eigen>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

using PointType = pcl::PointXYZI;

inline pcl::PointCloud<PointType>::Ptr voxelizePcd(const pcl::PointCloud<PointType> &pcd_in,
                                                   const float voxel_res)
{
    pcl::VoxelGrid<PointType> voxelgrid;
    voxelgrid.setLeafSize(voxel_res, voxel_res, voxel_res);
    pcl::PointCloud<PointType>::Ptr pcd_in_ptr(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr pcd_out(new pcl::PointCloud<PointType>);
    pcd_in_ptr->reserve(pcd_in.size());
    pcd_out->reserve(pcd_in.size());
    *pcd_in_ptr = pcd_in;
    voxelgrid.setInputCloud(pcd_in_ptr);
    voxelgrid.filter(*pcd_out);
    return pcd_out;
}

inline pcl::PointCloud<PointType>::Ptr voxelizePcd(const pcl::PointCloud<PointType>::Ptr &pcd_in,
                                                   const float voxel_res)
{
    pcl::VoxelGrid<PointType> voxelgrid;
    voxelgrid.setLeafSize(voxel_res, voxel_res, voxel_res);
    pcl::PointCloud<PointType>::Ptr pcd_out(new pcl::PointCloud<PointType>);
    pcd_out->reserve(pcd_in->size());
    voxelgrid.setInputCloud(pcd_in);
    voxelgrid.filter(*pcd_out);
    return pcd_out;
}

inline tf2::Quaternion rotationMatrixToTfQuat(const Eigen::Matrix3d &rot)
{
    Eigen::Quaterniond quat(rot);
    quat.normalize();
    return tf2::Quaternion(quat.x(), quat.y(), quat.z(), quat.w());
}

inline gtsam::Pose3 poseEigToGtsamPose(const Eigen::Matrix4d &pose_eig_in)
{
    const auto quat = rotationMatrixToTfQuat(pose_eig_in.block<3, 3>(0, 0));
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    return gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, yaw),
                        gtsam::Point3(pose_eig_in(0, 3),
                                      pose_eig_in(1, 3),
                                      pose_eig_in(2, 3)));
}

inline Eigen::Matrix4d gtsamPoseToPoseEig(const gtsam::Pose3 &gtsam_pose_in)
{
    Eigen::Matrix4d pose_eig_out = Eigen::Matrix4d::Identity();
    pose_eig_out.block<3, 3>(0, 0) = gtsam_pose_in.rotation().matrix();
    pose_eig_out(0, 3) = gtsam_pose_in.translation().x();
    pose_eig_out(1, 3) = gtsam_pose_in.translation().y();
    pose_eig_out(2, 3) = gtsam_pose_in.translation().z();
    return pose_eig_out;
}

inline geometry_msgs::msg::PoseStamped poseEigToPoseStamped(const Eigen::Matrix4d &pose_eig_in,
                                                            const std::string &frame_id)
{
    const auto quat = rotationMatrixToTfQuat(pose_eig_in.block<3, 3>(0, 0));
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = frame_id;
    pose.pose.position.x = pose_eig_in(0, 3);
    pose.pose.position.y = pose_eig_in(1, 3);
    pose.pose.position.z = pose_eig_in(2, 3);
    pose.pose.orientation.w = quat.w();
    pose.pose.orientation.x = quat.x();
    pose.pose.orientation.y = quat.y();
    pose.pose.orientation.z = quat.z();
    return pose;
}

inline geometry_msgs::msg::TransformStamped poseEigToTransformStamped(const Eigen::Matrix4d &pose,
                                                                      const std::string &parent_frame,
                                                                      const std::string &child_frame)
{
    const auto quat = rotationMatrixToTfQuat(pose.block<3, 3>(0, 0));
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = parent_frame;
    transform.child_frame_id = child_frame;
    transform.transform.translation.x = pose(0, 3);
    transform.transform.translation.y = pose(1, 3);
    transform.transform.translation.z = pose(2, 3);
    transform.transform.rotation.x = quat.x();
    transform.transform.rotation.y = quat.y();
    transform.transform.rotation.z = quat.z();
    transform.transform.rotation.w = quat.w();
    return transform;
}

inline geometry_msgs::msg::PoseStamped gtsamPoseToPoseStamped(const gtsam::Pose3 &gtsam_pose_in,
                                                              const std::string &frame_id)
{
    return poseEigToPoseStamped(gtsamPoseToPoseEig(gtsam_pose_in), frame_id);
}

inline Eigen::Matrix4d odomMsgToPoseEig(const nav_msgs::msg::Odometry &odom_in)
{
    const auto &q_msg = odom_in.pose.pose.orientation;
    tf2::Quaternion quat(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
    quat.normalize();
    tf2::Matrix3x3 tf_rot(quat);

    Eigen::Matrix3d rot_mat_eig;
    for (int row = 0; row < 3; ++row)
    {
        for (int col = 0; col < 3; ++col)
        {
            rot_mat_eig(row, col) = tf_rot[row][col];
        }
    }

    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    pose.block<3, 3>(0, 0) = rot_mat_eig;
    pose(0, 3) = odom_in.pose.pose.position.x;
    pose(1, 3) = odom_in.pose.pose.position.y;
    pose(2, 3) = odom_in.pose.pose.position.z;
    return pose;
}

template<typename T>
inline sensor_msgs::msg::PointCloud2 pclToPclRos(const pcl::PointCloud<T> &cloud,
                                                 const std::string &frame_id)
{
    sensor_msgs::msg::PointCloud2 cloud_ros;
    pcl::toROSMsg(cloud, cloud_ros);
    cloud_ros.header.frame_id = frame_id;
    return cloud_ros;
}

template<typename T>
inline pcl::PointCloud<T> transformPcd(const pcl::PointCloud<T> &cloud_in,
                                       const Eigen::Matrix4d &pose_tf)
{
    if (cloud_in.empty())
    {
        return cloud_in;
    }
    pcl::PointCloud<T> pcl_out;
    pcl::transformPointCloud(cloud_in, pcl_out, pose_tf);
    return pcl_out;
}

#endif
