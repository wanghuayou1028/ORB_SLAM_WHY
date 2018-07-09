// This file is part of CFD-SLAM - Combined Feature and Direct Method


#ifndef CFD_GLOBAL_H_
#define CFD_GLOBAL_H_

#include <list>
#include <vector>
#include <string>
#include <stdint.h>
#include <stdio.h>
#include <math.h>

#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <sophus/se3.h>
#include <vikit/performance_monitor.h>
#include <boost/shared_ptr.hpp>
#include<Eigen/StdVector>
#ifndef RPG_CFD_VIKIT_IS_VECTOR_SPECIALIZED //Guard for rpg_vikit
#define RPG_CFD_VIKIT_IS_VECTOR_SPECIALIZED
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector3d)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector2d)
#endif

#ifdef CFD_USE_ROS
  #include <ros/console.h>
  #define CFD_DEBUG_STREAM(x) ROS_DEBUG_STREAM(x)
  #define CFD_INFO_STREAM(x) ROS_INFO_STREAM(x)
  #define CFD_WARN_STREAM(x) ROS_WARN_STREAM(x)
  #define CFD_WARN_STREAM_THROTTLE(rate, x) ROS_WARN_STREAM_THROTTLE(rate, x)
  #define CFD_ERROR_STREAM(x) ROS_ERROR_STREAM(x)
#else
  #define CFD_INFO_STREAM(x) std::cerr<<"\033[0;0m[INFO] "<<x<<"\033[0;0m"<<std::endl;
  #ifdef CFD_DEBUG_OUTPUT
    #define CFD_DEBUG_STREAM(x) CFD_INFO_STREAM(x)
  #else
    #define CFD_DEBUG_STREAM(x)
  #endif
  #define CFD_WARN_STREAM(x) std::cerr<<"\033[0;33m[WARN] "<<x<<"\033[0;0m"<<std::endl;
  #define CFD_ERROR_STREAM(x) std::cerr<<"\033[1;31m[ERROR] "<<x<<"\033[0;0m"<<std::endl;
  #include <chrono> // Adapted from rosconsole. Copyright (c) 2008, Willow Garage, Inc.
  #define CFD_WARN_STREAM_THROTTLE(rate, x) \
    do { \
      static double __log_stream_throttle__last_hit__ = 0.0; \
      std::chrono::time_point<std::chrono::system_clock> __log_stream_throttle__now__ = \
      std::chrono::system_clock::now(); \
      if (__log_stream_throttle__last_hit__ + rate <= \
          std::chrono::duration_cast<std::chrono::seconds>( \
          __log_stream_throttle__now__.time_since_epoch()).count()) { \
        __log_stream_throttle__last_hit__ = \
        std::chrono::duration_cast<std::chrono::seconds>( \
        __log_stream_throttle__now__.time_since_epoch()).count(); \
        CFD_WARN_STREAM(x); \
      } \
    } while(0)
#endif

namespace ORB_SLAM2 {

  using namespace Eigen;
  using namespace Sophus;

  const double EPS = 0.0000000001;
  const double PI = 3.14159265;

#ifdef CFD_TRACE
  extern vk::PerformanceMonitor* g_permon;
  #define CFD_LOG(value) g_permon->log(std::string((#value)),(value))
  #define CFD_LOG2(value1, value2) CFD_LOG(value1); CFD_LOG(value2)
  #define CFD_LOG3(value1, value2, value3) CFD_LOG2(value1, value2); CFD_LOG(value3)
  #define CFD_LOG4(value1, value2, value3, value4) CFD_LOG2(value1, value2); CFD_LOG2(value3, value4)
  #define CFD_START_TIMER(name) g_permon->startTimer((name))
  #define CFD_STOP_TIMER(name) g_permon->stopTimer((name))
#else
  #define CFD_LOG(v)
  #define CFD_LOG2(v1, v2)
  #define CFD_LOG3(v1, v2, v3)
  #define CFD_LOG4(v1, v2, v3, v4)
  #define CFD_START_TIMER(name)
  #define CFD_STOP_TIMER(name)
#endif

  class Frame;
  typedef boost::shared_ptr<Frame> FramePtr;
} // namespace ORB_SLAM2

#endif // CFD_GLOBAL_H_
