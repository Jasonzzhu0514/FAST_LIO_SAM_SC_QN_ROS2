#include <fast_lio_sam_sc_qn2/fast_lio_sam_sc_qn2.hpp>

#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include <rosbag2_cpp/writer.hpp>

using namespace std::chrono_literals;

namespace
{
std::string requireStringParameter(const rclcpp::Node &node, const std::string &name, const bool allow_empty = false)
{
    const auto value = node.get_parameter(name).as_string();
    if (!allow_empty && value.empty())
    {
        throw std::runtime_error("Required string parameter is empty: " + name);
    }
    return value;
}

std::string makeTimestamp(const std::string &format)
{
    const auto now = std::chrono::system_clock::now();
    const auto time = std::chrono::system_clock::to_time_t(now);
    std::tm local_time{};
    localtime_r(&time, &local_time);

    std::ostringstream stream;
    stream << std::put_time(&local_time, format.c_str());
    return stream.str();
}
}  // namespace

FastLioSamScQn2::FastLioSamScQn2(const rclcpp::NodeOptions &options):
    Node("fast_lio_sam_sc_qn2_node", options)
{
    declareParameters();
    loadParameters();

    loop_closure_ = std::make_shared<LoopClosure>(lc_config_);

    gtsam::ISAM2Params isam_params;
    isam_params.relinearizeThreshold = 0.01;
    isam_params.relinearizeSkip = 1;
    isam_handler_ = std::make_shared<gtsam::ISAM2>(isam_params);

    odom_path_.header.frame_id = map_frame_;
    corrected_path_.header.frame_id = map_frame_;

    setupRosInterfaces();
    RCLCPP_INFO(get_logger(), "fast_lio_sam_sc_qn2 started. Input: odom=%s, cloud=%s",
                odom_topic_.c_str(), cloud_topic_.c_str());
}

void FastLioSamScQn2::declareParameters()
{
    declare_parameter<std::string>("basic.map_frame", "");
    declare_parameter<std::string>("basic.robot_frame", "");
    declare_parameter<double>("basic.loop_update_hz", 2.0);
    declare_parameter<double>("basic.vis_hz", 1.0);
    declare_parameter<std::string>("input.odom_topic", "");
    declare_parameter<std::string>("input.cloud_topic", "");
    declare_parameter<bool>("input.cloud_is_in_world_frame", true);
    declare_parameter<int>("input.sync_queue_size", 20);

    declare_parameter<std::string>("output.corrected_odom_topic", "");
    declare_parameter<std::string>("output.corrected_path_topic", "");
    declare_parameter<std::string>("output.original_odom_topic", "");
    declare_parameter<std::string>("output.original_path_topic", "");
    declare_parameter<std::string>("output.corrected_map_topic", "");
    declare_parameter<std::string>("output.corrected_current_pcd_topic", "");
    declare_parameter<std::string>("output.loop_detection_topic", "");
    declare_parameter<std::string>("output.realtime_pose_topic", "");
    declare_parameter<std::string>("output.debug_src_topic", "");
    declare_parameter<std::string>("output.debug_dst_topic", "");
    declare_parameter<std::string>("output.debug_coarse_aligned_topic", "");
    declare_parameter<std::string>("output.debug_fine_aligned_topic", "");
    declare_parameter<std::string>("output.save_trigger_topic", "");
    declare_parameter<int>("output.transient_qos_depth", 10);
    declare_parameter<int>("output.realtime_qos_depth", 10);
    declare_parameter<int>("output.save_trigger_qos_depth", 1);

    declare_parameter<double>("keyframe.keyframe_threshold", 1.5);
    declare_parameter<int>("keyframe.num_submap_keyframes", 10);
    declare_parameter<bool>("keyframe.enable_submap_matching", false);

    declare_parameter<double>("quatro_nano_gicp_voxel_resolution", 0.3);
    declare_parameter<double>("save_voxel_resolution", 0.3);
    declare_parameter<double>("scancontext_max_correspondence_distance", 35.0);

    declare_parameter<int>("nano_gicp.thread_number", 0);
    declare_parameter<double>("nano_gicp.icp_score_threshold", 1.5);
    declare_parameter<int>("nano_gicp.correspondences_number", 15);
    declare_parameter<double>("nano_gicp.max_correspondence_distance", 35.0);
    declare_parameter<int>("nano_gicp.max_iter", 32);
    declare_parameter<double>("nano_gicp.transformation_epsilon", 0.01);
    declare_parameter<double>("nano_gicp.euclidean_fitness_epsilon", 0.01);
    declare_parameter<int>("nano_gicp.ransac.max_iter", 5);
    declare_parameter<double>("nano_gicp.ransac.outlier_rejection_threshold", 1.0);

    declare_parameter<bool>("quatro.enable", true);
    declare_parameter<bool>("quatro.optimize_matching", true);
    declare_parameter<double>("quatro.distance_threshold", 35.0);
    declare_parameter<int>("quatro.max_correspondences", 500);
    declare_parameter<double>("quatro.fpfh_normal_radius", 0.9);
    declare_parameter<double>("quatro.fpfh_radius", 1.5);
    declare_parameter<bool>("quatro.estimating_scale", false);
    declare_parameter<double>("quatro.noise_bound", 0.3);
    declare_parameter<int>("quatro.rotation.num_max_iter", 50);
    declare_parameter<double>("quatro.rotation.gnc_factor", 1.4);
    declare_parameter<double>("quatro.rotation.rot_cost_diff_threshold", 0.0001);

    declare_parameter<bool>("result.save_map_pcd", true);
    declare_parameter<bool>("result.save_map_bag", false);
    declare_parameter<bool>("result.save_in_kitti_format", true);
    declare_parameter<std::string>("result.seq_name", "");
    declare_parameter<std::string>("result.save_directory", "");
    declare_parameter<std::string>("result.maps_directory_name", "");
    declare_parameter<std::string>("result.session_name", "");
    declare_parameter<std::string>("result.session_timestamp_format", "");
    declare_parameter<std::string>("result.scans_directory_name", "");
    declare_parameter<std::string>("result.poses_matrix_filename", "");
    declare_parameter<std::string>("result.poses_kitti_filename", "");
    declare_parameter<std::string>("result.poses_tum_filename", "");
    declare_parameter<int>("result.scan_file_index_width", 6);
    declare_parameter<std::string>("result.scan_file_extension", "");
    declare_parameter<std::string>("result.optimized_map_filename", "");
    declare_parameter<std::string>("result.raw_map_filename", "");
    declare_parameter<std::string>("result.bag_directory_name", "");
}

void FastLioSamScQn2::loadParameters()
{
    map_frame_ = requireStringParameter(*this, "basic.map_frame");
    robot_frame_ = requireStringParameter(*this, "basic.robot_frame");
    odom_topic_ = requireStringParameter(*this, "input.odom_topic");
    cloud_topic_ = requireStringParameter(*this, "input.cloud_topic");
    input_cloud_is_world_frame_ = get_parameter("input.cloud_is_in_world_frame").as_bool();
    input_sync_queue_size_ = get_parameter("input.sync_queue_size").as_int();

    corrected_odom_topic_ = requireStringParameter(*this, "output.corrected_odom_topic");
    corrected_path_topic_ = requireStringParameter(*this, "output.corrected_path_topic");
    original_odom_topic_ = requireStringParameter(*this, "output.original_odom_topic");
    original_path_topic_ = requireStringParameter(*this, "output.original_path_topic");
    corrected_map_topic_ = requireStringParameter(*this, "output.corrected_map_topic");
    corrected_current_pcd_topic_ = requireStringParameter(*this, "output.corrected_current_pcd_topic");
    loop_detection_topic_ = requireStringParameter(*this, "output.loop_detection_topic");
    realtime_pose_topic_ = requireStringParameter(*this, "output.realtime_pose_topic");
    debug_src_topic_ = requireStringParameter(*this, "output.debug_src_topic");
    debug_dst_topic_ = requireStringParameter(*this, "output.debug_dst_topic");
    debug_coarse_aligned_topic_ = requireStringParameter(*this, "output.debug_coarse_aligned_topic");
    debug_fine_aligned_topic_ = requireStringParameter(*this, "output.debug_fine_aligned_topic");
    save_trigger_topic_ = requireStringParameter(*this, "output.save_trigger_topic");
    transient_qos_depth_ = get_parameter("output.transient_qos_depth").as_int();
    realtime_qos_depth_ = get_parameter("output.realtime_qos_depth").as_int();
    save_trigger_qos_depth_ = get_parameter("output.save_trigger_qos_depth").as_int();

    keyframe_thr_ = get_parameter("keyframe.keyframe_threshold").as_double();
    lc_config_.num_submap_keyframes_ = get_parameter("keyframe.num_submap_keyframes").as_int();
    lc_config_.enable_submap_matching_ = get_parameter("keyframe.enable_submap_matching").as_bool();

    voxel_res_ = get_parameter("save_voxel_resolution").as_double();
    lc_config_.voxel_res_ = get_parameter("quatro_nano_gicp_voxel_resolution").as_double();
    lc_config_.scancontext_max_correspondence_distance_ =
        get_parameter("scancontext_max_correspondence_distance").as_double();

    auto &gc = lc_config_.gicp_config_;
    gc.nano_thread_number_ = get_parameter("nano_gicp.thread_number").as_int();
    gc.icp_score_thr_ = get_parameter("nano_gicp.icp_score_threshold").as_double();
    gc.nano_correspondences_number_ = get_parameter("nano_gicp.correspondences_number").as_int();
    gc.max_corr_dist_ = get_parameter("nano_gicp.max_correspondence_distance").as_double();
    gc.nano_max_iter_ = get_parameter("nano_gicp.max_iter").as_int();
    gc.transformation_epsilon_ = get_parameter("nano_gicp.transformation_epsilon").as_double();
    gc.euclidean_fitness_epsilon_ = get_parameter("nano_gicp.euclidean_fitness_epsilon").as_double();
    gc.nano_ransac_max_iter_ = get_parameter("nano_gicp.ransac.max_iter").as_int();
    gc.ransac_outlier_rejection_threshold_ =
        get_parameter("nano_gicp.ransac.outlier_rejection_threshold").as_double();

    auto &qc = lc_config_.quatro_config_;
    lc_config_.enable_quatro_ = get_parameter("quatro.enable").as_bool();
    qc.use_optimized_matching_ = get_parameter("quatro.optimize_matching").as_bool();
    qc.quatro_distance_threshold_ = get_parameter("quatro.distance_threshold").as_double();
    qc.quatro_max_num_corres_ = get_parameter("quatro.max_correspondences").as_int();
    qc.fpfh_normal_radius_ = get_parameter("quatro.fpfh_normal_radius").as_double();
    qc.fpfh_radius_ = get_parameter("quatro.fpfh_radius").as_double();
    qc.estimate_scale_ = get_parameter("quatro.estimating_scale").as_bool();
    qc.noise_bound_ = get_parameter("quatro.noise_bound").as_double();
    qc.quatro_max_iter_ = get_parameter("quatro.rotation.num_max_iter").as_int();
    qc.rot_gnc_factor_ = get_parameter("quatro.rotation.gnc_factor").as_double();
    qc.rot_cost_diff_thr_ = get_parameter("quatro.rotation.rot_cost_diff_threshold").as_double();

    save_map_pcd_ = get_parameter("result.save_map_pcd").as_bool();
    save_map_bag_ = get_parameter("result.save_map_bag").as_bool();
    save_in_kitti_format_ = get_parameter("result.save_in_kitti_format").as_bool();
    seq_name_ = requireStringParameter(*this, "result.seq_name");
    save_directory_ = requireStringParameter(*this, "result.save_directory", true);
    maps_directory_name_ = requireStringParameter(*this, "result.maps_directory_name");
    session_name_ = requireStringParameter(*this, "result.session_name", true);
    session_timestamp_format_ = requireStringParameter(*this, "result.session_timestamp_format");
    scans_directory_name_ = requireStringParameter(*this, "result.scans_directory_name");
    poses_matrix_filename_ = requireStringParameter(*this, "result.poses_matrix_filename");
    poses_kitti_filename_ = requireStringParameter(*this, "result.poses_kitti_filename");
    poses_tum_filename_ = requireStringParameter(*this, "result.poses_tum_filename");
    scan_file_index_width_ = get_parameter("result.scan_file_index_width").as_int();
    scan_file_extension_ = requireStringParameter(*this, "result.scan_file_extension");
    optimized_map_filename_ = requireStringParameter(*this, "result.optimized_map_filename");
    raw_map_filename_ = requireStringParameter(*this, "result.raw_map_filename");
    bag_directory_name_ = requireStringParameter(*this, "result.bag_directory_name");
    if (save_directory_.empty())
    {
        save_directory_ = fs::current_path().string();
    }
    resolved_session_name_ =
        session_name_.empty() ? seq_name_ + "_" + makeTimestamp(session_timestamp_format_) : session_name_;
}

void FastLioSamScQn2::setupRosInterfaces()
{
    const auto transient_qos = rclcpp::QoS(transient_qos_depth_).transient_local();
    const auto realtime_qos = rclcpp::QoS(realtime_qos_depth_);
    corrected_odom_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(corrected_odom_topic_, transient_qos);
    corrected_path_pub_ = create_publisher<nav_msgs::msg::Path>(corrected_path_topic_, transient_qos);
    odom_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(original_odom_topic_, transient_qos);
    path_pub_ = create_publisher<nav_msgs::msg::Path>(original_path_topic_, transient_qos);
    corrected_pcd_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(corrected_map_topic_, transient_qos);
    corrected_current_pcd_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(corrected_current_pcd_topic_, realtime_qos);
    loop_detection_pub_ = create_publisher<visualization_msgs::msg::Marker>(loop_detection_topic_, transient_qos);
    realtime_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(realtime_pose_topic_, realtime_qos);
    debug_src_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(debug_src_topic_, transient_qos);
    debug_dst_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(debug_dst_topic_, transient_qos);
    debug_coarse_aligned_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(debug_coarse_aligned_topic_, transient_qos);
    debug_fine_aligned_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(debug_fine_aligned_topic_, transient_qos);

    sub_odom_ = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>(this, odom_topic_);
    sub_pcd_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(this, cloud_topic_);
    sub_odom_pcd_sync_ = std::make_shared<message_filters::Synchronizer<OdomPcdSyncPolicy>>(
        OdomPcdSyncPolicy(input_sync_queue_size_), *sub_odom_, *sub_pcd_);
    sub_odom_pcd_sync_->registerCallback(
        std::bind(&FastLioSamScQn2::odomPcdCallback, this, std::placeholders::_1, std::placeholders::_2));

    sub_save_flag_ = create_subscription<std_msgs::msg::String>(
        save_trigger_topic_, save_trigger_qos_depth_,
        std::bind(&FastLioSamScQn2::saveFlagCallback, this, std::placeholders::_1));

    const double loop_update_hz = get_parameter("basic.loop_update_hz").as_double();
    const double vis_hz = get_parameter("basic.vis_hz").as_double();
    loop_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / loop_update_hz),
                                    std::bind(&FastLioSamScQn2::loopTimerFunc, this));
    vis_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / vis_hz),
                                   std::bind(&FastLioSamScQn2::visTimerFunc, this));
    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

void FastLioSamScQn2::odomPcdCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg,
                                      const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pcd_msg)
{
    Eigen::Matrix4d last_odom_tf = current_frame_.pose_eig_;
    current_frame_ = PosePcd(*odom_msg, *pcd_msg, current_keyframe_idx_, input_cloud_is_world_frame_);

    {
        std::lock_guard<std::mutex> lock(realtime_pose_mutex_);
        odom_delta_ = odom_delta_ * last_odom_tf.inverse() * current_frame_.pose_eig_;
        current_frame_.pose_corrected_eig_ = last_corrected_pose_ * odom_delta_;

        auto realtime_pose = poseEigToPoseStamped(current_frame_.pose_corrected_eig_, map_frame_);
        realtime_pose.header.stamp = now();
        realtime_pose_pub_->publish(realtime_pose);

        auto transform = poseEigToTransformStamped(current_frame_.pose_corrected_eig_, map_frame_, robot_frame_);
        transform.header.stamp = now();
        broadcaster_->sendTransform(transform);
    }

    auto corrected_current = pclToPclRos(transformPcd(current_frame_.pcd_, current_frame_.pose_corrected_eig_), map_frame_);
    corrected_current.header.stamp = now();
    corrected_current_pcd_pub_->publish(corrected_current);

    if (!is_initialized_)
    {
        keyframes_.push_back(current_frame_);
        updateOdomsAndPaths(current_frame_);

        const auto variance_vector =
            (gtsam::Vector(6) << 1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2).finished();
        auto prior_noise = gtsam::noiseModel::Diagonal::Variances(variance_vector);
        gtsam_graph_.add(gtsam::PriorFactor<gtsam::Pose3>(0,
                                                          poseEigToGtsamPose(current_frame_.pose_eig_),
                                                          prior_noise));
        init_esti_.insert(current_keyframe_idx_, poseEigToGtsamPose(current_frame_.pose_eig_));
        current_keyframe_idx_++;
        loop_closure_->updateScancontext(current_frame_.pcd_);
        last_corrected_pose_ = current_frame_.pose_corrected_eig_;
        is_initialized_ = true;
        return;
    }

    PosePcd latest_keyframe;
    {
        std::lock_guard<std::mutex> lock(keyframes_mutex_);
        latest_keyframe = keyframes_.back();
    }
    if (!checkIfKeyframe(current_frame_, latest_keyframe))
    {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(keyframes_mutex_);
        keyframes_.push_back(current_frame_);
    }

    const auto variance_vector =
        (gtsam::Vector(6) << 1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2).finished();
    auto odom_noise = gtsam::noiseModel::Diagonal::Variances(variance_vector);
    Eigen::Matrix4d previous_corrected_pose = Eigen::Matrix4d::Identity();
    {
        std::lock_guard<std::mutex> lock(keyframes_mutex_);
        previous_corrected_pose = keyframes_[current_keyframe_idx_ - 1].pose_corrected_eig_;
    }
    gtsam::Pose3 pose_from = poseEigToGtsamPose(previous_corrected_pose);
    gtsam::Pose3 pose_to = poseEigToGtsamPose(current_frame_.pose_corrected_eig_);
    {
        std::lock_guard<std::mutex> lock(graph_mutex_);
        gtsam_graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(current_keyframe_idx_ - 1,
                                                            current_keyframe_idx_,
                                                            pose_from.between(pose_to),
                                                            odom_noise));
        init_esti_.insert(current_keyframe_idx_, pose_to);
    }
    current_keyframe_idx_++;
    loop_closure_->updateScancontext(current_frame_.pcd_);

    {
        std::lock_guard<std::mutex> lock(vis_mutex_);
        updateOdomsAndPaths(current_frame_);
    }

    {
        std::lock_guard<std::mutex> lock(graph_mutex_);
        isam_handler_->update(gtsam_graph_, init_esti_);
        isam_handler_->update();
        if (loop_added_flag_)
        {
            isam_handler_->update();
            isam_handler_->update();
            isam_handler_->update();
        }
        gtsam_graph_.resize(0);
        init_esti_.clear();
    }

    {
        std::lock_guard<std::mutex> lock(realtime_pose_mutex_);
        corrected_esti_ = isam_handler_->calculateEstimate();
        last_corrected_pose_ = gtsamPoseToPoseEig(corrected_esti_.at<gtsam::Pose3>(corrected_esti_.size() - 1));
        odom_delta_ = Eigen::Matrix4d::Identity();
    }

    if (loop_added_flag_)
    {
        std::lock_guard<std::mutex> lock(keyframes_mutex_);
        for (size_t i = 0; i < corrected_esti_.size(); ++i)
        {
            keyframes_[i].pose_corrected_eig_ = gtsamPoseToPoseEig(corrected_esti_.at<gtsam::Pose3>(i));
        }
        loop_added_flag_ = false;
    }
}

void FastLioSamScQn2::loopTimerFunc()
{
    if (!is_initialized_ || keyframes_.empty())
    {
        return;
    }

    PosePcd latest_keyframe;
    std::vector<PosePcd> keyframes_snapshot;
    {
        std::lock_guard<std::mutex> lock(keyframes_mutex_);
        if (keyframes_.back().processed_)
        {
            return;
        }
        keyframes_.back().processed_ = true;
        latest_keyframe = keyframes_.back();
        keyframes_snapshot = keyframes_;
    }

    const int closest_keyframe_idx = loop_closure_->fetchCandidateKeyframeIdx(latest_keyframe, keyframes_snapshot);
    if (closest_keyframe_idx < 0)
    {
        return;
    }

    const auto reg_output = loop_closure_->performLoopClosure(latest_keyframe, keyframes_snapshot, closest_keyframe_idx);
    if (reg_output.is_valid_)
    {
        RCLCPP_INFO(get_logger(), "Loop closure accepted. Score: %.3f", reg_output.score_);
        const auto score = reg_output.score_;
        gtsam::Pose3 pose_from =
            poseEigToGtsamPose(reg_output.pose_between_eig_ * latest_keyframe.pose_corrected_eig_);
        gtsam::Pose3 pose_to = poseEigToGtsamPose(keyframes_snapshot[closest_keyframe_idx].pose_corrected_eig_);
        auto variance_vector = (gtsam::Vector(6) << score, score, score, score, score, score).finished();
        auto loop_noise = gtsam::noiseModel::Diagonal::Variances(variance_vector);
        {
            std::lock_guard<std::mutex> lock(graph_mutex_);
            gtsam_graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(latest_keyframe.idx_,
                                                                closest_keyframe_idx,
                                                                pose_from.between(pose_to),
                                                                loop_noise));
        }
        loop_idx_pairs_.push_back({latest_keyframe.idx_, static_cast<size_t>(closest_keyframe_idx)});
        loop_added_flag_vis_ = true;
        loop_added_flag_ = true;
    }
    else
    {
        RCLCPP_WARN(get_logger(), "Loop closure rejected. Score: %.3f", reg_output.score_);
    }

    auto stamp = now();
    auto src = pclToPclRos(loop_closure_->getSourceCloud(), map_frame_);
    src.header.stamp = stamp;
    debug_src_pub_->publish(src);
    auto dst = pclToPclRos(loop_closure_->getTargetCloud(), map_frame_);
    dst.header.stamp = stamp;
    debug_dst_pub_->publish(dst);
    auto final_aligned = pclToPclRos(loop_closure_->getFinalAlignedCloud(), map_frame_);
    final_aligned.header.stamp = stamp;
    debug_fine_aligned_pub_->publish(final_aligned);
    auto coarse_aligned = pclToPclRos(loop_closure_->getCoarseAlignedCloud(), map_frame_);
    coarse_aligned.header.stamp = stamp;
    debug_coarse_aligned_pub_->publish(coarse_aligned);
}

void FastLioSamScQn2::visTimerFunc()
{
    if (!is_initialized_)
    {
        return;
    }

    const auto stamp = now();
    if (loop_added_flag_vis_)
    {
        gtsam::Values corrected_esti_copied;
        pcl::PointCloud<pcl::PointXYZ> corrected_odoms;
        nav_msgs::msg::Path corrected_path;
        corrected_path.header.frame_id = map_frame_;
        corrected_path.header.stamp = stamp;
        {
            std::lock_guard<std::mutex> lock(realtime_pose_mutex_);
            corrected_esti_copied = corrected_esti_;
        }

        for (size_t i = 0; i < corrected_esti_copied.size(); ++i)
        {
            gtsam::Pose3 pose = corrected_esti_copied.at<gtsam::Pose3>(i);
            corrected_odoms.points.emplace_back(pose.translation().x(),
                                                pose.translation().y(),
                                                pose.translation().z());
            auto stamped_pose = gtsamPoseToPoseStamped(pose, map_frame_);
            stamped_pose.header.stamp = stamp;
            corrected_path.poses.push_back(stamped_pose);
        }

        if (!loop_idx_pairs_.empty())
        {
            auto markers = getLoopMarkers(corrected_esti_copied);
            markers.header.stamp = stamp;
            loop_detection_pub_->publish(markers);
        }

        {
            std::lock_guard<std::mutex> lock(vis_mutex_);
            corrected_odoms_ = corrected_odoms;
            corrected_path_.poses = corrected_path.poses;
        }
        loop_added_flag_vis_ = false;
    }

    {
        std::lock_guard<std::mutex> lock(vis_mutex_);
        odom_path_.header.stamp = stamp;
        corrected_path_.header.stamp = stamp;
        auto ori_odom = pclToPclRos(odoms_, map_frame_);
        ori_odom.header.stamp = stamp;
        odom_pub_->publish(ori_odom);
        path_pub_->publish(odom_path_);
        auto corrected_odom = pclToPclRos(corrected_odoms_, map_frame_);
        corrected_odom.header.stamp = stamp;
        corrected_odom_pub_->publish(corrected_odom);
        corrected_path_pub_->publish(corrected_path_);
    }

    if (global_map_vis_switch_ && corrected_pcd_map_pub_->get_subscription_count() > 0)
    {
        pcl::PointCloud<PointType>::Ptr corrected_map(new pcl::PointCloud<PointType>());
        {
            std::lock_guard<std::mutex> lock(keyframes_mutex_);
            if (!keyframes_.empty())
            {
                corrected_map->reserve(keyframes_[0].pcd_.size() * keyframes_.size());
            }
            for (const auto &keyframe : keyframes_)
            {
                *corrected_map += transformPcd(keyframe.pcd_, keyframe.pose_corrected_eig_);
            }
        }
        const auto voxelized_map = voxelizePcd(corrected_map, voxel_res_);
        auto map_msg = pclToPclRos(*voxelized_map, map_frame_);
        map_msg.header.stamp = stamp;
        corrected_pcd_map_pub_->publish(map_msg);
        global_map_vis_switch_ = false;
    }
    if (!global_map_vis_switch_ && corrected_pcd_map_pub_->get_subscription_count() == 0)
    {
        global_map_vis_switch_ = true;
    }
}

void FastLioSamScQn2::saveFlagCallback(const std_msgs::msg::String::SharedPtr msg)
{
    const std::string save_dir = msg->data.empty() ? save_directory_ : msg->data;
    try
    {
        saveResults(save_dir, false);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(get_logger(), "Failed to save mapping results: %s", e.what());
    }
}

void FastLioSamScQn2::saveResults(const std::string &save_dir, const bool from_destructor)
{
    std::lock_guard<std::mutex> save_lock(save_mutex_);

    std::vector<PosePcd> keyframes_snapshot;
    {
        std::lock_guard<std::mutex> lock(keyframes_mutex_);
        keyframes_snapshot = keyframes_;
    }

    if (keyframes_snapshot.empty())
    {
        return;
    }

    const fs::path session_directory = fs::path(save_dir) / maps_directory_name_ / resolved_session_name_;
    const fs::path scans_directory = session_directory / scans_directory_name_;
    if (fs::exists(session_directory))
    {
        fs::remove_all(session_directory);
    }
    fs::create_directories(session_directory);

    if (save_in_kitti_format_)
    {
        fs::create_directories(scans_directory);

        std::ofstream matrix_pose_file(session_directory / poses_matrix_filename_);
        std::ofstream kitti_pose_file(session_directory / poses_kitti_filename_);
        std::ofstream tum_pose_file(session_directory / poses_tum_filename_);
        tum_pose_file << "#timestamp x y z qx qy qz qw\n";

        for (size_t i = 0; i < keyframes_snapshot.size(); ++i)
        {
            std::stringstream scan_filename;
            scan_filename << std::setw(scan_file_index_width_) << std::setfill('0') << i
                          << scan_file_extension_;
            const fs::path scan_path = scans_directory / scan_filename.str();
            pcl::io::savePCDFileASCII<PointType>(scan_path.string(), keyframes_snapshot[i].pcd_);

            const auto &pose = keyframes_snapshot[i].pose_corrected_eig_;
            matrix_pose_file << pose(0, 0) << " " << pose(0, 1) << " " << pose(0, 2) << " "
                             << pose(0, 3) << " " << pose(1, 0) << " " << pose(1, 1) << " "
                             << pose(1, 2) << " " << pose(1, 3) << " " << pose(2, 0) << " "
                             << pose(2, 1) << " " << pose(2, 2) << " " << pose(2, 3) << "\n";
            kitti_pose_file << pose(0, 0) << " " << pose(0, 1) << " " << pose(0, 2) << " "
                            << pose(0, 3) << " " << pose(1, 0) << " " << pose(1, 1) << " "
                            << pose(1, 2) << " " << pose(1, 3) << " " << pose(2, 0) << " "
                            << pose(2, 1) << " " << pose(2, 2) << " " << pose(2, 3) << "\n";

            const auto lidar_optim_pose = poseEigToPoseStamped(keyframes_snapshot[i].pose_corrected_eig_, map_frame_);
            tum_pose_file << std::fixed << std::setprecision(8) << keyframes_snapshot[i].timestamp_
                          << " " << lidar_optim_pose.pose.position.x
                          << " " << lidar_optim_pose.pose.position.y
                          << " " << lidar_optim_pose.pose.position.z
                          << " " << lidar_optim_pose.pose.orientation.x
                          << " " << lidar_optim_pose.pose.orientation.y
                          << " " << lidar_optim_pose.pose.orientation.z
                          << " " << lidar_optim_pose.pose.orientation.w << "\n";
        }
    }

    if (save_map_bag_)
    {
        const fs::path bag_directory = session_directory / bag_directory_name_;
        rosbag2_cpp::Writer writer;
        writer.open(bag_directory.string());
        for (const auto &keyframe : keyframes_snapshot)
        {
            auto pcd_msg = pclToPclRos(keyframe.pcd_, map_frame_);
            auto pose_msg = poseEigToPoseStamped(keyframe.pose_corrected_eig_, map_frame_);
            const auto timestamp_ns = static_cast<int64_t>(keyframe.timestamp_ * 1e9);
            const auto timestamp = rclcpp::Time(timestamp_ns);
            builtin_interfaces::msg::Time stamp_msg;
            stamp_msg.sec = static_cast<int32_t>(timestamp_ns / 1000000000LL);
            stamp_msg.nanosec = static_cast<uint32_t>(timestamp_ns % 1000000000LL);
            pcd_msg.header.stamp = stamp_msg;
            pose_msg.header.stamp = stamp_msg;
            writer.write(pcd_msg, "/keyframe_pcd", timestamp);
            writer.write(pose_msg, "/keyframe_pose", timestamp);
        }
        writer.close();
    }

    if (save_map_pcd_)
    {
        fs::create_directories(session_directory);
        pcl::PointCloud<PointType>::Ptr corrected_map(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr raw_map(new pcl::PointCloud<PointType>());
        corrected_map->reserve(keyframes_snapshot[0].pcd_.size() * keyframes_snapshot.size());
        raw_map->reserve(keyframes_snapshot[0].pcd_.size() * keyframes_snapshot.size());
        for (const auto &keyframe : keyframes_snapshot)
        {
            *corrected_map += transformPcd(keyframe.pcd_, keyframe.pose_corrected_eig_);
            *raw_map += transformPcd(keyframe.pcd_, keyframe.pose_eig_);
        }
        const auto voxelized_corrected_map = voxelizePcd(corrected_map, voxel_res_);
        const auto voxelized_raw_map = voxelizePcd(raw_map, voxel_res_);
        pcl::io::savePCDFileASCII<PointType>((session_directory / optimized_map_filename_).string(),
                                             *voxelized_corrected_map);
        pcl::io::savePCDFileASCII<PointType>((session_directory / raw_map_filename_).string(),
                                             *voxelized_raw_map);
    }

    RCLCPP_INFO(get_logger(), "Saved mapping results to %s%s", session_directory.string().c_str(),
                from_destructor ? " during shutdown" : "");
}

FastLioSamScQn2::~FastLioSamScQn2()
{
    try
    {
        saveResults(save_directory_, true);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(get_logger(), "Failed to save mapping results during shutdown: %s", e.what());
    }
}

void FastLioSamScQn2::updateOdomsAndPaths(const PosePcd &pose_pcd_in)
{
    odoms_.points.emplace_back(pose_pcd_in.pose_eig_(0, 3),
                               pose_pcd_in.pose_eig_(1, 3),
                               pose_pcd_in.pose_eig_(2, 3));
    corrected_odoms_.points.emplace_back(pose_pcd_in.pose_corrected_eig_(0, 3),
                                         pose_pcd_in.pose_corrected_eig_(1, 3),
                                         pose_pcd_in.pose_corrected_eig_(2, 3));
    auto odom_pose = poseEigToPoseStamped(pose_pcd_in.pose_eig_, map_frame_);
    auto corrected_pose = poseEigToPoseStamped(pose_pcd_in.pose_corrected_eig_, map_frame_);
    odom_pose.header.stamp = now();
    corrected_pose.header.stamp = now();
    odom_path_.poses.emplace_back(odom_pose);
    corrected_path_.poses.emplace_back(corrected_pose);
}

visualization_msgs::msg::Marker FastLioSamScQn2::getLoopMarkers(const gtsam::Values &corrected_esti_in)
{
    visualization_msgs::msg::Marker edges;
    edges.type = visualization_msgs::msg::Marker::LINE_LIST;
    edges.scale.x = 0.12f;
    edges.header.frame_id = map_frame_;
    edges.pose.orientation.w = 1.0f;
    edges.color.r = 1.0f;
    edges.color.g = 1.0f;
    edges.color.b = 1.0f;
    edges.color.a = 1.0f;

    for (const auto &loop_idx_pair : loop_idx_pairs_)
    {
        if (loop_idx_pair.first >= corrected_esti_in.size() ||
            loop_idx_pair.second >= corrected_esti_in.size())
        {
            continue;
        }
        const auto pose = corrected_esti_in.at<gtsam::Pose3>(loop_idx_pair.first);
        const auto pose2 = corrected_esti_in.at<gtsam::Pose3>(loop_idx_pair.second);
        geometry_msgs::msg::Point p, p2;
        p.x = pose.translation().x();
        p.y = pose.translation().y();
        p.z = pose.translation().z();
        p2.x = pose2.translation().x();
        p2.y = pose2.translation().y();
        p2.z = pose2.translation().z();
        edges.points.push_back(p);
        edges.points.push_back(p2);
    }
    return edges;
}

bool FastLioSamScQn2::checkIfKeyframe(const PosePcd &pose_pcd_in, const PosePcd &latest_pose_pcd)
{
    return keyframe_thr_ <
           (latest_pose_pcd.pose_corrected_eig_.block<3, 1>(0, 3) -
            pose_pcd_in.pose_corrected_eig_.block<3, 1>(0, 3))
               .norm();
}
