#include <myslam/frame.h>

namespace myslam
{
    Frame::Frame() : id_(-1), time_stamp_(-1), camera_(nullptr) {}//构造函数初始化列表
    Frame::Frame(long id, double time_stamp, SE3 T_c_w, Camera::Ptr camera, Mat color, Mat depth):id_(id), time_stamp_(time_stamp), T_c_w_(T_c_w), camera_(camera), color_(color), depth_(depth) {}
    Frame::~Frame() {}
    Frame::Ptr Frame::createFrame()
    {
        static long factory_id =0;
        return make_shared<Frame>(factory_id++);
    }
    double findDepth(const cv::KeyPoint& kp)
    {
        int x = cvRound(kp.pt.x);
        int y = cvRound(kp.pt.y);
        ushort d = depth.ptr<ushort>(y)[x];
        if(d != 0)
            return double(d)/camera_->depth_scale_;
        else
            return -1.0;
    }

    Vector3d Frame::getCamCenter() const
    {
        return T_c_w_.inverse().translation();    //Twc和平移量
    }

    bool Frame::isInFrame(const Vector3d& pt_world)
    {
        Vector3d p_cam = camera_->world2camera(pt_world, T_c_w_);
        if(p_cam(2,0)<0) return false;
        Vector2d pixel = camera_->camera2pixel(p_cam);
        return pixel(0,0)>0 && pixel(1,0)>0 && pixel(0,0)<color_.cols && pixel(1,0)<color_.rows;
    }
}