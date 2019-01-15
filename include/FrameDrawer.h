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

#ifndef FRAMEDRAWER_H
#define FRAMEDRAWER_H

#include "Tracking.h"
#include "MapPoint.h"
#include "Map.h"

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include<mutex>


namespace ORB_SLAM2
{

class Tracking;
class Viewer;

class FrameDrawer
{
public:
    FrameDrawer(Map* pMap);

    // Update info from the last processed frame.
    // 将tracking线程的数据拷贝到绘图线程（图像、特征点、地图、跟踪状态）
    void Update(Tracking *pTracker);

    // Draw last processed frame.
    // 函数中只会在初始化或者正常跟踪的情况下才会显示特征点相关的信息
    // 所以，在跟丢的时候，图像就不会显示特征点信息
    cv::Mat DrawFrame();

protected:

    void DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText);

    // Info of the frame to be drawn
    // 这一帧中将要被画出来的信息
    cv::Mat mIm;
    int N;
    vector<cv::KeyPoint> mvCurrentKeys; // 关键点// KeyPoints in current frame
    vector<bool> mvbMap, mvbVO; // Tracked MapPoints in current frame
    bool mbOnlyTracking;
    int mnTracked, mnTrackedVO;  // 统计追踪的地图点数量、VO点数量（仅被当前帧观测到，没有被关键帧观测到的点）
    vector<cv::KeyPoint> mvIniKeys; // Initialization: KeyPoints in reference frame
    vector<int> mvIniMatches; // Initialization: correspondeces with reference keypoints 存放的是初始帧中每一个特征点在当前帧中的对应编号
    int mState; // 系统状态

    Map* mpMap;

    std::mutex mMutex;
};

} //namespace ORB_SLAM

#endif // FRAMEDRAWER_H
