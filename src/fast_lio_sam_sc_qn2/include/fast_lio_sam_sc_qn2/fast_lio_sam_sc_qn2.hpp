#ifndef FAST_LIO_SAM_SC_QN2_MAIN_HPP
#define FAST_LIO_SAM_SC_QN2_MAIN_HPP

#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl/io/pcd_io.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>

#include <fast_lio_sam_sc_qn2/loop_closure.hpp>
#include <fast_lio_sam_sc_qn2/pose_pcd.hpp>
#include <fast_lio_sam_sc_qn2/utilities.hpp>

namespace fs = std::filesystem;
using OdomPcdSyncPolicy =
    message_filters::sync_policies::ApproximateTime<nav_msgs::msg::Odometry,
                                                    sensor_msgs::msg::PointCloud2>;

class FastLioSamScQn2 : public rclcpp::Node
{
public:
    explicit FastLioSamScQn2(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~FastLioSamScQn2() override;

private:
    void declareParameters();
    void loadParameters();
    void setupRosInterfaces();

    void updateOdomsAndPaths(const PosePcd &pose_pcd_in);
    bool checkIfKeyframe(const PosePcd &pose_pcd_in, const PosePcd &latest_pose_pcd);
    visualization_msgs::msg::Marker getLoopMarkers(const gtsam::Values &corrected_esti_in);
    void saveResults(const std::string &save_dir, bool from_destructor);

    void odomPcdCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg,
                         const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pcd_msg);
    void saveFlagCallback(const std_msgs::msg::String::SharedPtr msg);
    void loopTimerFunc();
    void visTimerFunc();

    std::string map_frame_;
    std::string robot_frame_;
    std::string odom_topic_;
    std::string cloud_topic_;
    std::string corrected_odom_topic_;
    std::string corrected_path_topic_;
    std::string original_odom_topic_;
    std::string original_path_topic_;
    std::string corrected_map_topic_;
    std::string corrected_current_pcd_topic_;
    std::string loop_detection_topic_;
    std::string realtime_pose_topic_;
    std::string debug_src_topic_;
    std::string debug_dst_topic_;
    std::string debug_coarse_aligned_topic_;
    std::string debug_fine_aligned_topic_;
    std::string save_trigger_topic_;
    std::string save_directory_;
    std::string seq_name_;
    std::string maps_directory_name_;
    std::string session_name_;
    std::string resolved_session_name_;
    std::string session_timestamp_format_;
    std::string scans_directory_name_;
    std::string poses_matrix_filename_;
    std::string poses_kitti_filename_;
    std::string poses_tum_filename_;
    std::string scan_file_extension_;
    std::string optimized_map_filename_;
    std::string raw_map_filename_;
    std::string bag_directory_name_;
    bool input_cloud_is_world_frame_ = true;
    bool publish_corrected_map_continuously_ = true;
    int input_sync_queue_size_ = 20;
    int transient_qos_depth_ = 10;
    int realtime_qos_depth_ = 10;
    int save_trigger_qos_depth_ = 1;
    int scan_file_index_width_ = 6;

    std::mutex realtime_pose_mutex_, keyframes_mutex_;
    std::mutex graph_mutex_, vis_mutex_;
    std::mutex save_mutex_;
    Eigen::Matrix4d last_corrected_pose_ = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d odom_delta_ = Eigen::Matrix4d::Identity();
    PosePcd current_frame_;
    std::vector<PosePcd> keyframes_;
    int current_keyframe_idx_ = 0;

    bool is_initialized_ = false;
    bool loop_added_flag_ = false;
    bool loop_added_flag_vis_ = false;
    std::shared_ptr<gtsam::ISAM2> isam_handler_ = nullptr;
    gtsam::NonlinearFactorGraph gtsam_graph_;
    gtsam::Values init_esti_;
    gtsam::Values corrected_esti_;
    double keyframe_thr_ = 1.5;
    double voxel_res_ = 0.3;
    std::vector<std::pair<size_t, size_t>> loop_idx_pairs_;

    pcl::PointCloud<pcl::PointXYZ> odoms_, corrected_odoms_;
    nav_msgs::msg::Path odom_path_, corrected_path_;
    bool global_map_vis_switch_ = true;

    bool save_map_pcd_ = true;
    bool save_map_bag_ = false;
    bool save_in_kitti_format_ = true;
    bool results_saved_ = false;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr corrected_odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr corrected_path_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr corrected_current_pcd_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr corrected_pcd_map_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr loop_detection_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr realtime_pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_src_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_dst_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_coarse_aligned_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_fine_aligned_pub_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_save_flag_;
    std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> sub_odom_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> sub_pcd_;
    std::shared_ptr<message_filters::Synchronizer<OdomPcdSyncPolicy>> sub_odom_pcd_sync_;
    rclcpp::TimerBase::SharedPtr loop_timer_, vis_timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

    std::shared_ptr<LoopClosure> loop_closure_;
    LoopClosureConfig lc_config_;
};

#endif
