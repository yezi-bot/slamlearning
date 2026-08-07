#indef MAPPOINT_H
#define MAPPOINT_H
#include "myslam/common_include.h"
namespace myslam{
    class Frame;               //向前声明
    class MapPoint{
        public:
            typedef std::shared_ptr<MapPoint> Ptr;
            unsigned long id_; 
            Vector3d pos_; //世界坐标
            Vector3d norm_; //观测方向法向量
            cv::Mat descriptor_;
            list<Frame*> observed_frames_;
            int matched_times_;     //参与优化次数
            int visible_times_; 
            
            MapPoint();
            MapPoint(unsigned long id, Vector3d& position, Vector3d& norm, Frame* frame =nullptr,const cv::Mat& descriptor=Mat());
            inline cv::Ponit3f getPositionCV() const {
                return cv::Point3f(pos_(0,0),pos_(1,0),pos_(2,0));
            }
            static MapPoint::Ptr createMapPoint(); //创建一个新的MapPoint对象
            static MapPoint::Ptr createMapPoint( 
                 const Vector3d& pos_world, const Vector3d& norm_,const Mat& descriptor,Frame* frame );
    };
}
#endif