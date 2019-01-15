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

#ifndef KEYFRAME_H
#define KEYFRAME_H

#include "MapPoint.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "ORBextractor.h"
#include "Frame.h"
#include "KeyFrameDatabase.h"

#include <mutex>


namespace ORB_SLAM2
{

class Map;
class MapPoint;
class Frame;
class KeyFrameDatabase;

class KeyFrame
{
public:
    KeyFrame(Frame &F, Map* pMap, KeyFrameDatabase* pKFDB);

    // Pose functions
    void SetPose(const cv::Mat &Tcw);
    cv::Mat GetPose();
    cv::Mat GetPoseInverse();
    cv::Mat GetCameraCenter();
    cv::Mat GetStereoCenter();
    cv::Mat GetRotation();
    cv::Mat GetTranslation();

    // Bag of Words Representation
    void ComputeBoW();

    // Covisibility graph functions
    void AddConnection(KeyFrame* pKF, const int &weight);
    /**
     * 当前关键帧帧与传入的pKF（输入）有共视的时候，增加连接关系。weight（输入）。
     * 表示的是当前关键帧和pKF共同看到了多少个地图点。 这个函数改变的是公共变量：
     * std::map<KeyFrame*,int> mConnectedKeyFrameWeights;
     */
    void EraseConnection(KeyFrame* pKF);
    /**
     * 删除当前帧与关键帧pKF的连接关系，也是操作这个公共变量std::map<KeyFrame*,int>
     * mConnectedKeyFrameWeights;完了之后记得跟新下，UpdayeBestCovisible.
     */
    void UpdateConnections();
    /*
     * 主要是跟新，该帧与其他关键帧的连接关系。第一步，找出这个关键帧里面的每一个地图点，
     * 并且找出其他同样也能看到这个地图点的关键帧们，跟新一下当前帧与其他关键帧的共视权重。
     * 直到找完当前帧的所有地图点。我们将结果放到一个容器里面   map<KeyFrame*,int> KFcounter;
     * 第一个表示当前帧与哪个关键帧有共视关系，第二个参数表示共视点的个数（权重）。
     * 第二步：挑选出那些共视权重大于等于15的那些，
     * 放入容器vector<pair<int,KeyFrame*> > vPairs; 然后把vPair从大小进行排序。
     * 第三步更新最小生成树（将权重最大的那帧定义为当前帧的父节点）
     * 最后得到的结果：   得到公有变量mConnectedKeyFrameWeights，所有与当前帧有公视关系的关键帧及其权重。
     * 得到公共变量：mvpOrderedConnectedKeyFrames ：权重大于15的关键帧 按权重排列的vector向量。
     * 得到公共变量：mvOrderedWeights ：权重（大于15）从大到小排列的vector向量。
     * 以及最小生成树的连接关系。（与当前帧有最大权重的最为该帧的父亲，该帧作为孩子）。
     */
    void UpdateBestCovisibles();
    /*
     * 这个改变的是mvpOrderedConnectedKeyFrames,和mvOrderedWeights容器里面的值顺序，就是保证这里的值是从大到小排列的。
     */
    std::set<KeyFrame *> GetConnectedKeyFrames(); //得到与该关键帧有共视的关键帧set集合
    std::vector<KeyFrame* > GetVectorCovisibleKeyFrames(); //得到与该关键帧有共视（权重大于15）的关键帧的有序vector集合
    std::vector<KeyFrame*> GetBestCovisibilityKeyFrames(const int &N); //返回最好的共视向量集合 有<=N个关键帧
    std::vector<KeyFrame*> GetCovisiblesByWeight(const int &w); //返回的是大于该权重w的 所有共视关键帧
    int GetWeight(KeyFrame* pKF); //得到pKF与当前帧的权重


    // Spanning tree functions
    void AddChild(KeyFrame* pKF);
    void EraseChild(KeyFrame* pKF);
    void ChangeParent(KeyFrame* pKF);
    std::set<KeyFrame*> GetChilds(); //得到最小生成树，该帧的所有孩子。
    KeyFrame* GetParent();
    bool hasChild(KeyFrame* pKF);

    // Loop Edges
    void AddLoopEdge(KeyFrame* pKF); //可能检查到很多回环关键帧
    std::set<KeyFrame*> GetLoopEdges(); //返回这些可能是回环的关键帧,改变的是公共变量spLoopEdges;值。

    // MapPoint observation functions
    void AddMapPoint(MapPoint* pMP, const size_t &idx);
    void EraseMapPointMatch(const size_t &idx);
    void EraseMapPointMatch(MapPoint* pMP);
    void ReplaceMapPointMatch(const size_t &idx, MapPoint* pMP);
    std::set<MapPoint*> GetMapPoints(); // 得到map point不是bad 的mappoint 集合
    std::vector<MapPoint*> GetMapPointMatches();
    int TrackedMapPoints(const int &minObs); // 返回该关键帧里面，被跟踪的关键点的个数
    MapPoint* GetMapPoint(const size_t &idx);

    // KeyPoint functions
    std::vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r) const;
    cv::Mat UnprojectStereo(int i);

    // Image
    bool IsInImage(const float &x, const float &y) const;

    // Enable/Disable bad flag changes
    void SetNotErase();
    void SetErase();

    // Set/check bad flag
    void SetBadFlag();
    /*
     * 该函数将该关键帧帧变成bad,mbToBeErased变成true，就是代表”删除了“。
     * 删除了该帧，第一要改变，该帧其连接帧的连接关系，删除该帧对地图点的观测。
     * 第二改变最小生成树，改变最小生成树的具体实现：。具体实现看代码。
     * 主要思路：因为删除了当前帧，所有当前帧下面的孩子们（n个孩子）的父节点都要改变。
     * 我们就循环n次，重新为这n个孩子找父节点，父节点出现在与这些孩子节点有最多的共视权重的帧，
     * 并且在父节点候选容器（ParentCandidates）中里面的帧。
     * 选好了父亲点之后，把这个孩子节点加入剩下的孩子节点的父亲候选节点（sParentCandidates.insert(pC);），
     * 删除这个孩子节点（mspChildrens.erase(pC);）。
     */
    bool isBad();

    // Compute Scene Depth (q=2 median). Used in monocular.
    float ComputeSceneMedianDepth(const int q);

    static bool weightComp( int a, int b){
        return a>b;
    }

    static bool lId(KeyFrame* pKF1, KeyFrame* pKF2){
        return pKF1->mnId<pKF2->mnId;
    }


    // The following variables are accesed from only 1 thread or never change (no mutex needed).
public:

    static long unsigned int nNextId;
    long unsigned int mnId;
    const long unsigned int mnFrameId;

    const double mTimeStamp;

    // Grid (to speed up feature matching)
    const int mnGridCols;
    const int mnGridRows;
    const float mfGridElementWidthInv;
    const float mfGridElementHeightInv;

    // Variables used by the tracking
    long unsigned int mnTrackReferenceForFrame; // mnTrackReferenceForFrame防止重复添加局部关键帧
    long unsigned int mnFuseTargetForKF; // 为了防止融合地图点重复关键帧的标志位

    // Variables used by the local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnBAFixedForKF;

    // Variables used by the keyframe database
    long unsigned int mnLoopQuery; //为了不对同一个关键帧进行重复检测，这是一个标签
    int mnLoopWords; //如果当前帧与该帧每共有一个单词，则加一，该关键帧与当前闭环探测关键帧的共有的单词数
    float mLoopScore; //与当前帧的闭环分数
    long unsigned int mnRelocQuery;
    int mnRelocWords; //该关键帧与当前帧所共有的单词数
    float mRelocScore; //与当前帧的重定位分数

    // Variables used by loop closing
    cv::Mat mTcwGBA;
    cv::Mat mTcwBefGBA;
    long unsigned int mnBAGlobalForKF;

    // Calibration parameters
    const float fx, fy, cx, cy, invfx, invfy, mbf, mb, mThDepth;

    // Number of KeyPoints
    const int N;

    // KeyPoints, stereo coordinate and descriptors (all associated by an index)
    const std::vector<cv::KeyPoint> mvKeys;
    const std::vector<cv::KeyPoint> mvKeysUn; // 去畸变后的特征点
    const std::vector<float> mvuRight; // negative value for monocular points
    const std::vector<float> mvDepth; // negative value for monocular points
    const cv::Mat mDescriptors;

    //BoW
    DBoW2::BowVector mBowVec;
    DBoW2::FeatureVector mFeatVec;

    // Pose relative to parent (this is computed when bad flag is activated)
    cv::Mat mTcp;

    // Scale
    const int mnScaleLevels;
    const float mfScaleFactor;
    const float mfLogScaleFactor;
    const std::vector<float> mvScaleFactors;
    const std::vector<float> mvLevelSigma2;
    const std::vector<float> mvInvLevelSigma2;

    // Image bounds and calibration
    const int mnMinX;
    const int mnMinY;
    const int mnMaxX;
    const int mnMaxY;
    const cv::Mat mK;


    // The following variables need to be accessed trough a mutex to be thread safe.
protected:

    // SE3 Pose and camera center
    cv::Mat Tcw;
    cv::Mat Twc;
    cv::Mat Ow;

    cv::Mat Cw; // Stereo middel point. Only for visualization

    // MapPoints associated to keypoints
    // 与关键帧相关联的地图点
    std::vector<MapPoint*> mvpMapPoints;

    // BoW
    KeyFrameDatabase* mpKeyFrameDB;
    ORBVocabulary* mpORBvocabulary;

    // Grid over the image to speed up feature matching
    std::vector< std::vector <std::vector<size_t> > > mGrid;

    std::map<KeyFrame*,int> mConnectedKeyFrameWeights;  // 存放的是与当前帧有共视的关键帧和共视地图点数
    std::vector<KeyFrame*> mvpOrderedConnectedKeyFrames; // 将排序后的所连接的关键帧排序后存在这个变量中
    std::vector<int> mvOrderedWeights; // 将排序后的权重存在这个变量中

    // Spanning Tree and Loop Edges
    bool mbFirstConnection;
    KeyFrame* mpParent;
    std::set<KeyFrame*> mspChildrens; // 该关键帧的子关键帧
    std::set<KeyFrame*> mspLoopEdges; // 与该关键帧形成闭环边的关键帧

    // Bad flags
    bool mbNotErase;
    bool mbToBeErased;
    bool mbBad;    

    float mHalfBaseline; // Only for visualization

    Map* mpMap;

    std::mutex mMutexPose;
    std::mutex mMutexConnections;
    std::mutex mMutexFeatures;
};

} //namespace ORB_SLAM

#endif // KEYFRAME_H
