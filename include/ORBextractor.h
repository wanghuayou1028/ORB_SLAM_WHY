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
/*
* 作者采用ORB特征子，在orb-slam中，作者对opencv中的orb源码进行了修改，将特征进行均匀化、
* 具体调用和opencv一致，也是重载了函数调用操作符operator()。
*/

#ifndef ORBEXTRACTOR_H
#define ORBEXTRACTOR_H

#include <vector>
#include <list>
#include <opencv/cv.h>


namespace ORB_SLAM2
{
// 分配四叉树时用到的节点类型
class ExtractorNode
{
public:
    ExtractorNode():bNoMore(false){} // 初始构造时设置bNoMore为false

    void DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4);

    std::vector<cv::KeyPoint> vKeys;
    cv::Point2i UL, UR, BL, BR;
    std::list<ExtractorNode>::iterator lit;
    bool bNoMore;
};

class ORBextractor
{
public:
    
    enum {HARRIS_SCORE=0, FAST_SCORE=1 };

    ORBextractor(int nfeatures, float scaleFactor, int nlevels,
                 int iniThFAST, int minThFAST);

    ~ORBextractor(){}

    // Compute the ORB features and descriptors on an image.
    // ORB are dispersed on the image using an octree.
    // Mask is ignored in the current implementation.
    void operator()( cv::InputArray image, cv::InputArray mask,
      std::vector<cv::KeyPoint>& keypoints,
      cv::OutputArray descriptors);

    // 括号运算符输入图像，并且传入引用参数, keypoints, descriptors用于存储计算得到的特征点及其描述子
    int inline GetLevels(){
        return nlevels;}

    float inline GetScaleFactor(){
        return scaleFactor;}

    std::vector<float> inline GetScaleFactors(){
        return mvScaleFactor;
    }

    std::vector<float> inline GetInverseScaleFactors(){
        return mvInvScaleFactor;
    }

    std::vector<float> inline GetScaleSigmaSquares(){
        return mvLevelSigma2;
    }

    std::vector<float> inline GetInverseScaleSigmaSquares(){
        return mvInvLevelSigma2;
    }

    std::vector<cv::Mat> mvImagePyramid; // 图像金字塔

protected:

    void ComputePyramid(cv::Mat image); //　函数构造金字塔
    // 对金字塔图像进行角点检测，通过四叉树的方式进行计算
    void ComputeKeyPointsOctTree(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);  
    // 四叉树方式分配特征点  
    std::vector<cv::KeyPoint> DistributeOctTree(const std::vector<cv::KeyPoint>& vToDistributeKeys, const int &minX,
                                           const int &maxX, const int &minY, const int &maxY, const int &nFeatures, const int &level);

    void ComputeKeyPointsOld(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);
    std::vector<cv::Point> pattern; // point is equal to point2i　//!<用于存放训练的模板 存储关键点附近patch的点对

    int nfeatures; //1000 Number of features per image
    double scaleFactor; //1.2 Scale factor between levels in the scale pyramid	
    int nlevels; //8  Number of levels in the scale pyramid	
    int iniThFAST; //　检测FAST角点的阈值
    int minThFAST; // 在 iniThFAST 没有检测到角点的前提下，降低的阈值

    std::vector<int> mnFeaturesPerLevel; // 每层特征的个数

    std::vector<int> umax;  // 用于存储计算特征方向时，图像每个v坐标对应最大的u坐标

    std::vector<float> mvScaleFactor;  //!<每层的相对于原始图像的缩放比例，{1，1.2，...}
    std::vector<float> mvInvScaleFactor; //每层的相对于原始图像的缩放比例的倒数     
    std::vector<float> mvLevelSigma2; // mvScaleFactor * mvScaleFactor
    std::vector<float> mvInvLevelSigma2;
};

} //namespace ORB_SLAM

#endif

