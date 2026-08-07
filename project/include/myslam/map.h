#indef
#define MAP_H
#include "myslam/common_include.h"
#include "myslam/frame.h"
#include "myslam/mappoint.h"
namespace myslam{
    class Map{
        public:
            typedef std::shared_ptr<Map> Ptr;
            std::unordered_map<unsigned long, MapPoint::Ptr> map_points_;  //所有地图点    //散列表
            std::unordered_map<unsigned long, Frame::Ptr> keyframes_;  //所有关键帧
            Map(){}
            void insertKeyFrame(Frame::Ptr frame);  //插入关键帧
            void insertMapPoint(MapPoint::Ptr map_point);  //插入地图点
    };
}
#endif