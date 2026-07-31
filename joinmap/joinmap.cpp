#include <iostream>
#include <fstream>
using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;
#include <Eigen/Geometry>
#include <boost/format.hpp>               
#include <pcl/point_types.h>                    //点类型//
#include <pcl/io/pcd_io.h>                     //点云输入输出//
#include <pcl/visualization/cloud_viewer.h> //可视化//

int main(int argc, char **argv)
{
  vector<Mat> colorImgs, depthImgs;            //动态数组，存放mat类型的彩色图和深度图//
  vector<Eigen::Isometry3d> poses; 
  ifstream fin("./pose.txt");
  if (!fin)
  {
    cerr << "请在有pose.txt的目录下运行此程序" << endl;
    return 1;
  }
  for (int i = 0; i < 5; i++)
  {
    boost::format fmt("./%s/%d.%s"); 
    colorImgs.push_back(imread((fmt % "color" % (i + 1) % "png").str()));        //imread只能读字符串 
    depthImgs.push_back(imread((fmt % "depth" % (i + 1) % "pgm").str(), -1));  //16位，要用-1参数
    if (colorImgs.back().empty())
    {
      cerr << "彩色图读取"<<i+1<<"图片失败" << endl;
      return 1;
    }
    if (depthImgs.back().empty())
    {
      cerr << "深度图读取"<<i+1<<"图片失败" << endl;
      return 1;
    }
    cout<<"深度图类型"<<depthImgs.back().type()<<"深度图通道"<<depthImgs.back().channels()<<endl;
    double data[7] = {0};
    for (auto &d : data)
      fin >> d;
    Eigen::Quaterniond q(data[6], data[3], data[4], data[5]);   //q的顺序是wxyz
    Eigen::Isometry3d T(q);
    T.pretranslate(Eigen::Vector3d(data[0], data[1], data[2]));
    poses.push_back(T);
  }

  double cx = 325.5;
  double cy = 253.5;
  double fx = 518.0;
  double fy = 519.0;    //相机内参
  double depthScale = 1000.0; //深度值缩放成米

  typedef pcl::PointXYZRGBA PointT;                //PointT是一个点
  typedef pcl::PointCloud<PointT> PointCloud;      //PointCloud是点云
  PointCloud::Ptr pointCloud(new PointCloud); //智能指针pointCloud指向一个新的点云对象（堆内存）
  for(int i=0;i<5;++i)
  {
    Mat color = colorImgs[i];
    Mat depth = depthImgs[i];
    Eigen::Isometry3d T = poses[i];
    int height = color.rows;
    int width = color.cols;
    if (height != depth.rows || width != depth.cols)
    {
      cerr << "彩色图与深度图大小不一致" << endl;
      return 1;
    }                                           //检验分辨率
   
    for(int v=0;v<height;v++)
    {
        // 取当前行指针，安全访问
        unsigned short* depth_row = depth.ptr<unsigned short>(v);
        uchar* color_row = color.ptr<uchar>(v);
        for(int u=0;u<width;u++)
        {
            unsigned short d_raw = depth_row[u];
            if(d_raw == 0) continue;

            // 深度转米
            double z = d_raw / depthScale;
            double x = (u - cx) * z / fx;
            double y = (v - cy) * z / fy;

            Eigen::Vector3d cam_p(x,y,z);
            Eigen::Vector3d world_p = T * cam_p;

            // 取颜色
            int pixel_pos = u * 3;
            uchar b = color_row[pixel_pos];
            uchar g = color_row[pixel_pos+1];
            uchar r = color_row[pixel_pos+2];

            // 填入点云
            PointT p;
            p.x = world_p[0];
            p.y = world_p[1];
            p.z = world_p[2];
            p.b = b;
            p.g = g;
            p.r = r;
            p.a = 255;
            pointCloud->points.push_back(p);
        }
    }


   pointCloud->is_dense = false; //设置点云稠密度为false
   cout<<"点云共有"<<pointCloud->size()<<"个点."<<endl;
   pcl::io::savePCDFileBinary("map.pcd", *pointCloud); //保存点云
   
  }
}
