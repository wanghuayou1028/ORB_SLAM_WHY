// This file is part of CFD-SLAM - Combined Feature and Direct Method

#ifdef CFD_USE_ROS
#include <vikit/params_helper.h>
#endif
#include <Config.h>

namespace ORB_SLAM2 {

Config::Config() :
#ifdef CFD_USE_ROS
    trace_name(vk::getParam<string>("cfd/trace_name", "cfd")),
    trace_dir(vk::getParam<string>("cfd/trace_dir", "/tmp")),
    n_pyr_levels(vk::getParam<int>("cfd/n_pyr_levels", 3)),
    use_imu(vk::getParam<bool>("cfd/use_imu", false)),
    core_n_kfs(vk::getParam<int>("cfd/core_n_kfs", 3)),
    map_scale(vk::getParam<double>("cfd/map_scale", 1.0)),
    grid_size(vk::getParam<int>("cfd/grid_size", 30)),
    init_min_disparity(vk::getParam<double>("cfd/init_min_disparity", 50.0)),
    init_min_tracked(vk::getParam<int>("cfd/init_min_tracked", 50)),
    init_min_inliers(vk::getParam<int>("cfd/init_min_inliers", 40)),
    klt_max_level(vk::getParam<int>("cfd/klt_max_level", 4)),
    klt_min_level(vk::getParam<int>("cfd/klt_min_level", 2)),
    reproj_thresh(vk::getParam<double>("cfd/reproj_thresh", 2.0)),
    poseoptim_thresh(vk::getParam<double>("cfd/poseoptim_thresh", 2.0)),
    poseoptim_num_iter(vk::getParam<int>("cfd/poseoptim_num_iter", 10)),
    structureoptim_max_pts(vk::getParam<int>("cfd/structureoptim_max_pts", 20)),
    structureoptim_num_iter(vk::getParam<int>("cfd/structureoptim_num_iter", 5)),
    loba_thresh(vk::getParam<double>("cfd/loba_thresh", 2.0)),
    loba_robust_huber_width(vk::getParam<double>("cfd/loba_robust_huber_width", 1.0)),
    loba_num_iter(vk::getParam<int>("cfd/loba_num_iter", 0)),
    kfselect_mindist(vk::getParam<double>("cfd/kfselect_mindist", 0.12)),
    triang_min_corner_score(vk::getParam<double>("cfd/triang_min_corner_score", 20.0)),
    triang_half_patch_size(vk::getParam<int>("cfd/triang_half_patch_size", 4)),
    subpix_n_iter(vk::getParam<int>("cfd/subpix_n_iter", 10)),
    max_n_kfs(vk::getParam<int>("cfd/max_n_kfs", 10)),
    img_imu_delay(vk::getParam<double>("cfd/img_imu_delay", 0.0)),
    max_fts(vk::getParam<int>("cfd/max_fts", 120)),
    quality_min_fts(vk::getParam<int>("cfd/quality_min_fts", 50)),
    quality_max_drop_fts(vk::getParam<int>("cfd/quality_max_drop_fts", 40))
#else
    trace_name("cfd"),
    trace_dir("/tmp"),
    n_pyr_levels(3),
    use_imu(false),
    core_n_kfs(3),
    map_scale(1.0),
    grid_size(25),
    init_min_disparity(50.0),
    init_min_tracked(50),
    init_min_inliers(40),
    klt_max_level(4),
    klt_min_level(2),
    reproj_thresh(2.0),
    poseoptim_thresh(2.0),
    poseoptim_num_iter(10),
    structureoptim_max_pts(20),
    structureoptim_num_iter(5),
    loba_thresh(2.0),
    loba_robust_huber_width(1.0),
    loba_num_iter(0),
    kfselect_mindist(0.12),
    triang_min_corner_score(20.0),
    triang_half_patch_size(4),
    subpix_n_iter(10),
    max_n_kfs(0),
    img_imu_delay(0.0),
    max_fts(120),
    quality_min_fts(50),
    quality_max_drop_fts(40)
#endif
{}

Config& Config::getInstance()
{
  static Config instance; // Instantiated on first use and guaranteed to be destroyed
  return instance;
}

} // namespace ORB_SLAM2

