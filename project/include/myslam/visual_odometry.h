#ifndef VISUALODOMETRY_H
#define VISUALODOMETRY_H

#include "myslam/common_include.h"
#include "myslam/map.h"
#include <opencv/features2d/features2d.h>

namespace myslam{
class VisualOdometry
{
    public:
           typedef shared_ptr<VisualOdometry> Ptr;
           enum VOState{
            INITIALIZING =-1;
            OK=0;
            LOST
           };
           VOState state_;
           Map::Ptr map_;
           Frame::Ptr ref_;    
           Frame::Ptr curr_;       
           cv::Ptr<cv::ORB> orb_;
           
           vector<cv::Keypoint> keypoint_curr_;
           Mat descriptors_curr_;
           
           cv::FlannBaseMatcher matcher_flann_;  //比暴力匹配更快
           vector<MapPoint::Ptr> match_3dpts_;
           vector<int> match_2dkp_index_;

           SE3 T_c_r_estimated_;       //估计的当前位姿
           int num_inliers_;
           int num_lost_;

           int num_of_features_;
           double scale_factors_;
           int level_pyramid;
           float match_ratio_;      //匹配率
           int max_num_lost_;
           int min_inliers_;
         
           double key_frame_min_rot;   
           double key_frame_min_trans; //关键帧条件
           double map_point_erase_ratio; //地图剔除比值

    public:
           VisualOdometry();
           ~VisualOdometry();
           bool addFrame(Frame::Ptr frame);
         
    protected:
           void extractKeyPoints();
           void computeDescriptors(); 
           void featureMatching();
           void poseEstimationPnP(); 
           void optimizeMap();
    
           void addKeyFrame();
           void addMapPoints();
           bool checkEstimatedPose(); 
           bool checkKeyFrame();

           double getViewAngle(Frame::Ptr frame,MapPoint::Ptr point);
           
};
}
#endif












