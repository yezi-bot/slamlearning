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
            int observed_times_; 
            int correct_times_; //参与优化次数
            MapPoint();
            MapPoint(long id, Vector3d position, Vector3d norm, cv::Mat descriptor);
            static MapPoint::Ptr createMapPoint(); //创建一个新的MapPoint对象

    };
}
#endif