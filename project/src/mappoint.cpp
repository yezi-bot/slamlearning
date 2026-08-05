#include <myslam/mappoint.h>
#include "myslam/common_include.h"
namespace myslam
{
    MapPoint::MapPoint() : id_(-1), pos_(0,0,0), norm_(0,0,0), descriptor_(cv::Mat()) {}
    MapPoint::MapPoint(long id, Vector3d position, Vector3d norm, cv::Mat descriptor) : id_(id), pos_(position), norm_(norm), descriptor_(descriptor) {}
    MapPoint::Ptr MapPoint::createMapPoint()
    {
        static long factory_id =0;
        return make_shared<MapPoint>(factory_id++);
    }
}