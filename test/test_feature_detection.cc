// This file is part of CFD-SLAM - Combined Feature and Direct Method

#include <string>
#include <global.h>
#include <Config.h>
#include <Frame.h>
#include <feature_detection.h>
// #include <depth_filter.h>
#include <feature.h>
#include <vikit/timer.h>
#include <vikit/vision.h>
#include <vikit/abstract_camera.h>
#include <vikit/atan_camera.h>
#include "test_utils.h"
#include <boost/shared_ptr.hpp>

namespace ORB_SLAM2{

using namespace Eigen;
using namespace std;

void testCornerDetector(cv::Mat& img)
{
    cv::FileStorage fSettings("/home/why/SLAM/project/orb_slam_ws/src/ORB_SLAM_WHY/Examples/Monocular/TUM1.yaml", cv::FileStorage::READ);
    
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    float bf = 40.0;
    float ThDepth = 40.0;

    cv::imshow("img", img);
    cv::waitKey(0);

    cout << K << endl;
    cout << DistCoef << endl; 
    cout << bf << endl;
    cout << ThDepth << endl;
    
    // vk::AbstractCamera* cam = new vk::ATANCamera(752, 480, 0.511496, 0.802603, 0.530199, 0.496011, 0.934092);
    FramePtr frame(new Frame(img, 0.0, K, DistCoef, bf, ThDepth));

    cv::imshow("img", img);
    cv::waitKey(0);

    // Corner detection
    vk::Timer t;
    Features fts;
    feature_detection::FastDetector fast_detector(
        img.cols, img.rows, Config::gridSize(), Config::nPyrLevels());
    for(int i=0; i<100; ++i)
    {
        fast_detector.detect(frame.get(), frame->img_pyr_, Config::triangMinCornerScore(), fts);
    }
    printf("Fast corner detection took %f ms, %zu corners detected (ref i7-W520: 7.166360ms, 40000)\n", t.stop()*10, fts.size());
    printf("Note, in this case, feature detection also contains the cam2world projection of the feature.\n");
    cv::Mat img_rgb = cv::Mat(img.size(), CV_8UC3);
    cv::cvtColor(img, img_rgb, CV_GRAY2RGB);
    std::for_each(fts.begin(), fts.end(), [&](Feature* i){
        cv::circle(img_rgb, cv::Point2f(i->px[0], i->px[1]), 4*(i->level+1), cv::Scalar(0,255,0), 1);
    });
    cv::imshow("ref_img", img_rgb);
    cv::waitKey(0);

    std::for_each(fts.begin(), fts.end(), [&](Feature* i){ delete i; });
}

} // namespace


int main(int argc, char** argv)
{
    if(argc != 2){
        std::cout << "usage: test_feature_detection_svo 1.png" << std::endl;
        return 1;
    }
    // printf("Loading image '%s'\n", img_name.c_str());
    cv::Mat img(cv::imread(argv[1], 0));
    assert(img.type() == CV_8UC1 && !img.empty());

    ORB_SLAM2::testCornerDetector(img);
    return 0;
}
