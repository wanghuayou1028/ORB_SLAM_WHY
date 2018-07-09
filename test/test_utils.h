// This file is part of CFD-SLAM - Combined Feature and Direct Method

#ifndef TEST_UTILS_H_
#define TEST_UTILS_H_

#include <string.h>
#include <cstdlib> // for getenv
#ifdef SVO_USE_ROS
# include <ros/package.h>
# include <vikit/params_helper.h>
#endif

namespace ORB_SLAM2 {
namespace test_utils {

std::string getDatasetDir()
{
  const char* env_dir = std::getenv("SVO_DATASET_DIR");
#ifdef SVO_USE_ROS
  std::string dataset_dir(ros::package::getPath("svo")+"/test/data");
  if(env_dir != NULL)
    dataset_dir = std::string(env_dir);
  return dataset_dir;
#else
  return std::string(env_dir);
#endif
}

std::string getTraceDir()
{
#ifdef SVO_USE_ROS
  std::string default_dir(ros::package::getPath("svo")+"/test/results");
  return vk::getParam<std::string>("svo/trace_dir", default_dir);
#else
  return "/tmp";
#endif
}

} // namespace test_utils
} // namespace ORB_SLAM2

#endif // TEST_UTILS_H_
