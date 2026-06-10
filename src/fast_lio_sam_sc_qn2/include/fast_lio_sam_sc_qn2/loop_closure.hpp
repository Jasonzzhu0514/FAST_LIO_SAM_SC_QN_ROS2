#ifndef FAST_LIO_SAM_SC_QN2_LOOP_CLOSURE_HPP
#define FAST_LIO_SAM_SC_QN2_LOOP_CLOSURE_HPP

#include <limits>
#include <memory>
#include <mutex>
#include <tuple>
#include <utility>
#include <vector>

#include <Eigen/Eigen>
#include <fast_lio_sam_sc_qn2/pose_pcd.hpp>
#include <fast_lio_sam_sc_qn2/utilities.hpp>
#include <nano_gicp/nano_gicp.hpp>
#include <nano_gicp/point_type_nano_gicp.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <quatro/quatro_module.h>
#include <scancontext_tro/Scancontext.h>

using PcdPair = std::tuple<pcl::PointCloud<PointType>, pcl::PointCloud<PointType>>;

struct NanoGICPConfig
{
    int nano_thread_number_ = 0;
    int nano_correspondences_number_ = 15;
    int nano_max_iter_ = 32;
    int nano_ransac_max_iter_ = 5;
    double max_corr_dist_ = 35.0;
    double icp_score_thr_ = 1.5;
    double transformation_epsilon_ = 0.01;
    double euclidean_fitness_epsilon_ = 0.01;
    double ransac_outlier_rejection_threshold_ = 1.0;
};

struct QuatroConfig
{
    bool use_optimized_matching_ = true;
    bool estimate_scale_ = false;
    int quatro_max_num_corres_ = 500;
    int quatro_max_iter_ = 50;
    double quatro_distance_threshold_ = 35.0;
    double fpfh_normal_radius_ = 0.90;
    double fpfh_radius_ = 1.50;
    double noise_bound_ = 0.30;
    double rot_gnc_factor_ = 1.40;
    double rot_cost_diff_thr_ = 0.0001;
};

struct LoopClosureConfig
{
    bool enable_quatro_ = true;
    bool enable_submap_matching_ = false;
    int num_submap_keyframes_ = 10;
    double voxel_res_ = 0.3;
    double scancontext_max_correspondence_distance_ = 35.0;
    NanoGICPConfig gicp_config_;
    QuatroConfig quatro_config_;
};

struct RegistrationOutput
{
    bool is_valid_ = false;
    bool is_converged_ = false;
    double score_ = std::numeric_limits<double>::max();
    Eigen::Matrix4d pose_between_eig_ = Eigen::Matrix4d::Identity();
};

class LoopClosure
{
public:
    explicit LoopClosure(const LoopClosureConfig &config);
    ~LoopClosure();

    void updateScancontext(const pcl::PointCloud<PointType> &cloud);
    int fetchCandidateKeyframeIdx(const PosePcd &query_keyframe,
                                  const std::vector<PosePcd> &keyframes);
    PcdPair setSrcAndDstCloud(const std::vector<PosePcd> &keyframes,
                              int src_idx,
                              int dst_idx,
                              int submap_range,
                              double voxel_res,
                              bool enable_quatro,
                              bool enable_submap_matching);
    RegistrationOutput icpAlignment(const pcl::PointCloud<PointType> &src,
                                    const pcl::PointCloud<PointType> &dst);
    RegistrationOutput coarseToFineAlignment(const pcl::PointCloud<PointType> &src,
                                             const pcl::PointCloud<PointType> &dst);
    RegistrationOutput performLoopClosure(const PosePcd &query_keyframe,
                                          const std::vector<PosePcd> &keyframes,
                                          int closest_keyframe_idx);
    pcl::PointCloud<PointType> getSourceCloud();
    pcl::PointCloud<PointType> getTargetCloud();
    pcl::PointCloud<PointType> getCoarseAlignedCloud();
    pcl::PointCloud<PointType> getFinalAlignedCloud();
    int getClosestKeyframeidx();

private:
    SCManager sc_manager_;
    std::mutex sc_mutex_;
    std::mutex registration_mutex_;
    nano_gicp::NanoGICP<PointType, PointType> nano_gicp_;
    std::shared_ptr<quatro<PointType>> quatro_handler_ = nullptr;
    int closest_keyframe_idx_ = -1;
    pcl::PointCloud<PointType>::Ptr src_cloud_;
    pcl::PointCloud<PointType>::Ptr dst_cloud_;
    pcl::PointCloud<PointType> coarse_aligned_;
    pcl::PointCloud<PointType> aligned_;
    LoopClosureConfig config_;
};

#endif
