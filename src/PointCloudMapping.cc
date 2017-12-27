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


#include "PointCloudMapping.h"
#include <KeyFrame.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
//  #include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/io/pcd_io.h>
#include "Converter.h"
 
// #include <boost/make_shared.hpp>

#include <mutex>
#include<thread>

using namespace std;
using namespace cv;

namespace ORB_SLAM2
{

PointCloudMapping::PointCloudMapping(const string &strSettingPath): mbFinishRequested(false), mbFinished(true),
    mbStopped(false), mbStopRequested(false), mbNotStop(false), mbAcceptKeyFrames(true)
{
    globalPointCloudMap = boost::make_shared< pcl::PointCloud<pcl::PointXYZRGB> >();

    // viewerThread = make_shared<thread>( bind(&PointCloudMapping::viewer, this ) );
    
    // for point cloud resolution
    cv::FileStorage fpSettings(strSettingPath, cv::FileStorage::READ);
    resolution = fpSettings["PointCloudMapping.Resolution"];

    voxel.setLeafSize( resolution, resolution, resolution);
}

void PointCloudMapping::Run()
{
    mbFinished = false;

    // cv::namedWindow("ColorImg", CV_WINDOW_AUTOSIZE);

    while(1)
    {
        // Tracking will see that Point Cloud Mapping is busy
        SetAcceptKeyFrames(false);
        // if( !CheckNewKeyFrames() )
        //     cout << "Point Cloud Mapping has no new KeyFrame" << endl;

        // Check if there are keyframes in the queue
        if( CheckNewKeyFrames() )
        {
            // cout << "Point Cloud Mapping has new keyFrame" << endl;
            {
                unique_lock<mutex> lock(mMutexNewKFs);
                mpCurrentKeyFrame = mlNewKeyFrames.front();
                mCVCurrentColorImg = mlColorImgs.front();
                mCVCurrentDepthImg = mlDepthImgs.front();

                // cout << "mpCurrentKeyFrame.size()" << mlNewKeyFrames.size() << endl;

                mlNewKeyFrames.pop_front();
                mlColorImgs.pop_front();
                mlDepthImgs.pop_front();

                // test the image 
                // mCVCurrentColorImg( cv::Rect ( 0,0,100,100 ) ).setTo ( 0 ); // 将左上角100*100的块置零
                
                // imshow("ORB-SLAM: PointCloud", mCVCurrentColorImg);
                // cv::waitKey(1.0/30.0);

                pcl::PointCloud<pcl::PointXYZRGB>::Ptr p = generatePointCloud( mpCurrentKeyFrame, mCVCurrentColorImg, mCVCurrentDepthImg );
                *globalPointCloudMap += *p;

                // pcl::PointCloud< pcl::PointXYZRGB >::Ptr tmp( new pcl::PointCloud< pcl::PointXYZRGB >() );                                
                // voxel.setInputCloud( globalPointCloudMap );
                // voxel.filter( *tmp );
                // globalPointCloudMap->swap( *tmp );

                cout<<"generated point cloud size= " << globalPointCloudMap->points.size() << endl;
            }

            // cout << mCVCurrentColorImg.cols << endl;
            // cout << mCVCurrentColorImg.rows << endl;

            if( globalPointCloudMap->points.size() != 0 )
            {
                pcl::io::savePCDFileBinary("PointCloud.pcd", *globalPointCloudMap );
            }
            
            // TODO: dense mapping
        }
        else if( Stop() )
        {
            // Safe area to stop
            while( isStopped() && !CheckFinish() )
            {
                usleep(3000);
            }
            if( CheckFinish() )
                break;
        }

        SetAcceptKeyFrames(true);

        if(CheckFinish())
            break;
    }

    pcl::io::savePCDFileBinary("PointCloud.pcd", *globalPointCloudMap );
    
    SetFinish();
}

void PointCloudMapping::InsertKeyFrame(KeyFrame* pKF, cv::Mat& ImColor, cv::Mat& ImDepth)
{
    // cout<<"receive a keyframe, id = "<<pKF->mnId<<endl;
    unique_lock<mutex> lock(mMutexNewKFs);
    mlNewKeyFrames.push_back( pKF );
    mlColorImgs.push_back( ImColor.clone() );
    mlDepthImgs.push_back( ImDepth.clone() );
}

// void PointCloudMapping::InsertKeyFrame(KeyFrame* pKF)
// {
//     // cout<<"receive a keyframe, id = "<<pKF->mnId<<endl;
//     unique_lock<mutex> lock(mMutexNewKFs);
//     mlNewKeyFrames.push_back( pKF );
//     // mlColorImgs.push_back( ImColor.clone() );
//     // mlDepthImgs.push_back( ImDepth.clone() );
// }

bool PointCloudMapping::CheckNewKeyFrames()
{
    unique_lock<mutex> lock(mMutexNewKFs);
    return(!mlNewKeyFrames.empty());
}

pcl::PointCloud< pcl::PointXYZRGB >::Ptr PointCloudMapping::generatePointCloud(KeyFrame* pKF, cv::Mat& ImColor, cv::Mat& ImDepth)
{
    pcl::PointCloud< pcl::PointXYZRGB >::Ptr tmp( new pcl::PointCloud< pcl::PointXYZRGB >() );
    // point cloud is null ptr
    // cout << "ImDepth.rows: " << ImDepth.rows << "ImDepth.cols: " << ImDepth.cols << endl;
    // cout << "ImColor.rows: " << ImColor.rows << "ImDepth.cols: " << ImDepth.cols << endl;
    for ( int m=0; m<ImDepth.rows; m+=3 )
    {
        for ( int n=0; n<ImDepth.cols; n+=3 )
        {
            float d = ImDepth.ptr<float>(m)[n];
            // if the data is from Kinect, mm style
            if(d > 10)
                d /= 1000;
            // cout << "The depth of image pixel " << m << "*" << n << " is " << d << endl;
            if ( d < 0.01 || d > 8 )
                continue;
            pcl::PointXYZRGB p;
            p.z = d;
            p.x = ( n - pKF->cx) * p.z / pKF->fx;
            p.y = ( m - pKF->cy) * p.z / pKF->fy;
 
            p.b = ImColor.ptr<uchar>(m)[n*3];
            p.g = ImColor.ptr<uchar>(m)[n*3+1];
            p.r = ImColor.ptr<uchar>(m)[n*3+2];
 
            tmp->points.push_back(p);
        }
    }
 
    Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat( pKF->GetPose() );
    pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud(new pcl::PointCloud< pcl::PointXYZRGB >());
    pcl::transformPointCloud( *tmp, *cloud, T.inverse().matrix());
    cloud->is_dense = false;
 
    cout<<"generate point cloud for pKF "<<pKF->mnId<<", size="<<cloud->points.size()<<endl;
    return cloud;
}
 
 
// void PointCloudMapping::viewer()
// {
//     //  pcl::visualization::CloudViewer viewer("viewer");
//     boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
//     viewer->setBackgroundColor (0, 0, 0);
    
//     while(1)
//     {
//         {
//             unique_lock<mutex> lck_shutdown( shutDownMutex );
//             if (shutDownFlag)
//             {
//                 break;
//             }
//         }
//         {
//             unique_lock<mutex> lck_keyframeUpdated( keyFrameUpdateMutex );
//             keyFrameUpdated.wait( lck_keyframeUpdated );
//         }
 
//         // keyframe is updated
//         size_t N=0;
//         {
//             unique_lock<mutex> lck( keyframeMutex );
//             N = keyframes.size();
//         }
 
//         for ( size_t i=lastKeyframeSize; i<N ; i++ )
//         {
//             PointCloud::Ptr p = generatePointCloud( keyframes[i], colorImgs[i], depthImgs[i] );
//             *globalMap += *p;
//         }
//         PointCloud::Ptr tmp(new PointCloud());
//         voxel.setInputCloud( globalMap );
//         voxel.filter( *tmp );
//         globalMap->swap( *tmp );

//         // PointCloudXYZRGB::Ptr cloud_xyzrgb(new PointCloudXYZRGB);
//         // pcl::copyPointCloud(*globalMap, *cloud_xyzrgb);
//         // //  viewer.showCloud( globalMap );
//         // //  viewer = rgbVis(pointCloud); 
        
        
//         // // --------------------------------------------
//         // // -----Open 3D viewer and add point cloud-----
//         // // --------------------------------------------
//         // pcl::visualization::PointCloudColorHandlerRGBField<PointXYZRGB> rgb(cloud_xyzrgb);
//         // viewer->addPointCloud<PointXYZRGB> (cloud_xyzrgb, rgb, "sample cloud");
//         // viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
//         // viewer->addCoordinateSystem (1.0);
//         // viewer->initCameraParameters ();
        
//         // viewer->spinOnce (1);
//         // //  while (!viewer->wasStopped ())
//         // //  {
//         // //      viewer->spinOnce (100);
//         // //     //  boost::this_thread::sleep (boost::posix_time::microseconds (100000));
//         // //  }

//         // cout << "show global map, size=" << globalMap->points.size() << endl;
//         // lastKeyframeSize = N;
//     }
// }

void PointCloudMapping::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopRequested = true;
}

bool PointCloudMapping::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    if( mbStopRequested && !mbNotStop)
    {
        mbStopped = true;
        cout << "Point Cloud Mapping STOP" << endl;
        return true;
    }

    return false;
}

bool PointCloudMapping::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool PointCloudMapping::stopRequested()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopRequested;
}

bool PointCloudMapping::AcceptKeyFrames()
{
    unique_lock<mutex> lock(mMutexAccept);
    return mbAcceptKeyFrames;
}

void PointCloudMapping::SetAcceptKeyFrames(bool flag)
{
    unique_lock<mutex> lock(mMutexAccept);
    mbAcceptKeyFrames = flag;
}

bool PointCloudMapping::SetNotStop(bool flag)
{
    unique_lock<mutex> lock(mMutexStop);

    if(flag && mbStopped)
        return false;
    
    mbNotStop = flag;

    return true;
}

void PointCloudMapping::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool PointCloudMapping::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void PointCloudMapping::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;    
    unique_lock<mutex> lock2(mMutexStop);
    mbStopped = true;
}

bool PointCloudMapping::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

}//namespace ORB_SLAM2
 