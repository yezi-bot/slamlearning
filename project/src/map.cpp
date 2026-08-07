#include <myslam/map.h>

namespace myslam
{
    void Map::insertKeyFrame(Frame::Ptr frame)
    {
        if(keyframes_.find(frame->id_) == keyframes_.end())  //没有这个点就插入
        {
            keyframes_.insert(std::make_pair(frame->id_, frame));
        }
    }

    void Map::insertMapPoint(MapPoint::Ptr map_point)
    {
        if(map_points_.find(map_point->id_) == map_points_.end())
        {
            map_points_.insert(std::make_pair(map_point->id_, map_point));  //make_pair构建值对插入
        }
    }

}
