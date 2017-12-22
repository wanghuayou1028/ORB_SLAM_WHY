/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef POINTCLOUDMAPPING_H
#define POINTCLOUDMAPPING_H
 
#include "System.h"
#include "Tracking.h"
#include "KeyFrame.h"
#include <vector>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <condition_variable>
// #include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/console/parse.h>

#include <mutex>
#include <thread>

namespace ORB_SLAM2
{
class Tracking;
class KeyFrame;


class PointCloudMapping
{
public:
    // typedef pcl::PointXYZRGB PointXYZRGB;
    // typedef pcl::PointCloud<PointXYZRGB> PointCloudXYZRGB;

    // typedef pcl::PointXYZRGBA PointT;
    // typedef pcl::PointCloud<PointT> PointCloud;

    // constructor
    PointCloudMapping(const string &strSettingPath);

    void Run();

    void InsertKeyFrame( KeyFrame* pKF, cv::Mat& ImColor, cv::Mat& ImDepth );
    // void InsertKeyFrame( cv::Mat& ImColor, cv::Mat& ImDepth);
    // void InsertKeyFrame(KeyFrame* pKF); 
    
    cv::Mat get_mCVCurrentColorImg()
    {
        return mCVCurrentColorImg;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr get_globalPointCloudMap()
    {
        return globalPointCloudMap;
    }

    // Thread Synch
    void RequestStop();
    bool Stop();
    bool isStopped();
    bool stopRequested();
    bool AcceptKeyFrames();
    void SetAcceptKeyFrames(bool flag);
    bool SetNotStop(bool flag);

    void RequestFinish();
    bool isFinished();

    int KeyframesInQueue(){
        unique_lock<std::mutex> lock(mMutexNewKFs);
        return mlNewKeyFrames.size();
    }

protected:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr generatePointCloud(KeyFrame* pKF, cv::Mat& ImColor, cv::Mat& ImDepth);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr globalPointCloudMap;

    bool CheckNewKeyFrames();

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    // These variables will be assigned in System.cc
    
    std::list<KeyFrame*> mlNewKeyFrames;
    std::list<cv::Mat>   mlColorImgs;
    std::list<cv::Mat>   mlDepthImgs;

    KeyFrame* mpCurrentKeyFrame;
    cv::Mat   mCVCurrentColorImg;
    cv::Mat   mCVCurrentDepthImg;

    std::mutex mMutexNewKFs;

    bool mbStopped;
    bool mbStopRequested;
    bool mbNotStop;
    std::mutex mMutexStop;

    bool mbAcceptKeyFrames;
    std::mutex mMutexAccept;

    double resolution = 0.04;
    pcl::VoxelGrid<pcl::PointXYZRGB>  voxel;
};

}//namespace ORB_SLAM2
 
#endif // POINTCLOUDMAPPING_H
 