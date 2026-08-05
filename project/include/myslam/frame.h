#indef FRAME_H
#define FRAME_H
#include "myslam/common_include.h"
#include "myslam/camera.h"

namespace myslam
{
    class MapPoint;
    class Frame{
        public:
            typedef std::shared_ptr<Frame> Ptr;
            unsigned long id_; // id 
            double time_stamp_; 
            SE3 T_c_w_; 
            Camera::Ptr camera_;
            Mat color_, depth_; 
            std::vector<cv::KeyPoint> keypoints_; //特征点
            std::vector<MapPoint*> map_points_; 
            cv::Mat descriptors_; //描述子
            Frame();            //默认构造函数
            Frame(long id, double time_stamp, SE3 T_c_w, Camera::Ptr camera, Mat color, Mat depth); //带参构造函数
            ~Frame();           //析构函数。对象销毁时自动调用

            static Frame::Ptr createFrame();       //静态函数属于类本身直接调用,创建一个新对象
            double findDepth(const cv::KeyPoint& kp);  //计算深度
            Vector3d getCamCenter() const;
            bool isInFrame(const Vector3d& pt_world); 

    };
}
#endif 