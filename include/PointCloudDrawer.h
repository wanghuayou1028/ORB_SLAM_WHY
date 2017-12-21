// /**
// * This file is part of ORB-SLAM2.
// *
// * Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
// * For more information see <https://github.com/raulmur/ORB_SLAM2>
// *
// * ORB-SLAM2 is free software: you can redistribute it and/or modify
// * it under the terms of the GNU General Public License as published by
// * the Free Software Foundation, either version 3 of the License, or
// * (at your option) any later version.
// *
// * ORB-SLAM2 is distributed in the hope that it will be useful,
// * but WITHOUT ANY WARRANTY; without even the implied warranty of
// * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// * GNU General Public License for more details.
// *
// * You should have received a copy of the GNU General Public License
// * along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
// */

// #ifndef POINTCLOUDDRAWER_H
// #define POINTCLOUDDRAWER_H

// #include "PointCloudMapping.h"
// #include "Viewer.h"

// #include <opencv2/core/core.hpp>
// #include <opencv2/highgui/highgui.hpp>

// #include <mutex>

// namespace ORB_SLAM2
// {

// class PointCloudMapping;
// class Viewer;

// class PointCloudDrawer 
// {
// public:
//     PointCloudDrawer();

//     cv::Mat Update(PointCloudMapping* pPointCloudMapper);

// protected:
//     cv::Mat mIm;
    
//     std::mutex mMutex;

// };

// }

// #endif // POINTCLOUDDRAWER_H