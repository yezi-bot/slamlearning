#include <iostream>
#include <fstream>
#include <list>
#include <vector>
#include <chrono>
#include <ctime>
#include <climits>    //最大值最小值
#include <memory>           

#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/robust_kernel.h>              //鲁棒核函数基类
#include <g2o/types/sba/types_six_dof_expmap.h>

using namespace std;
using namespace g2o;

struct Measurement
{
    Measurement ( Eigen::Vector3d p, float g ) : pos_world ( p ), grayscale ( g ) {}
    Eigen::Vector3d pos_world;
    float grayscale;
};

//针孔模型逆运算，得到相机坐标系
inline Eigen::Vector3d project2Dto3D ( int x, int y, int d, float fx, float fy, float cx, float cy, float scale )
{
    float zz = float ( d ) /scale;
    float xx = zz* ( x-cx ) /fx;
    float yy = zz* ( y-cy ) /fy;
    return Eigen::Vector3d ( xx, yy, zz );
}
inline Eigen::Vector2d project3Dto2D ( float x, float y, float z, float fx, float fy, float cx, float cy )
{
    float u = fx*x/z+cx;
    float v = fy*y/z+cy;
    return Eigen::Vector2d ( u,v );
}

//直接法算位姿
bool poseEstimationDirect ( const vector<Measurement>& measurements, cv::Mat* gray, Eigen::Matrix3f& intrinsics, Eigen::Isometry3d& Tcw );

//定义边
class EdgeSE3ProjectDirect: public BaseUnaryEdge< 1, double, VertexSE3Expmap>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW   
  EdgeSE3ProjectDirect(Eigen::Vector3d point, float fx, float fy, float cx, float cy, cv::Mat* image)
:x_world_(point), fx_(fx), fy_(fy), cx_(cx), cy_(cy), image_(image)
{}

//得到误差
  virtual void computeError()
  {
    const VertexSE3Expmap* v = static_cast<const VertexSE3Expmap*>(_vertices[0]);        //VertexSE3Expmap储存Tcw
    Eigen::Vector3d x_local = v->estimate().map(x_world_);                              //estimate得到T，map把Pw变Pc
    float x = x_local[0] / x_local[2] * fx_ + cx_;
    float y = x_local[1] / x_local[2] * fy_ + cy_;
    
    if (x < 0 || y < 0 || x >= image_->cols || y >= image_->rows)
    {
      _error(0,0) = 0.0;
      this->setLevel(1);
    }
    else
    {
      float error = getPixelValue(x, y) - _measurement;
      _error(0,0) = error;

    }
  }

  //计算雅可比
  virtual void linearizeOplus()
  {
    if (level() == 1)
    {
      _jacobianOplusXi = Eigen::Matrix<double, 1, 6>::Zero();
      return;
    }
    VertexSE3Expmap* vtx = static_cast<VertexSE3Expmap*>(_vertices[0]);
    Eigen::Vector3d xyz_trans = vtx->estimate().map(x_world_); // Pc
    double x = xyz_trans[0];
    double y = xyz_trans[1];
    double invz = 1/xyz_trans[2];
    double invz_2 = invz*invz;
    float u = x*fx_*invz + cx_;
    float v = y*fy_*invz + cy_;
    
    jacobian_uv_ksai (0,0) = - x*y*invz_2 *fx_;          // du/dωx
    jacobian_uv_ksai (0,1) = ( 1+ (x*x*invz_2 )) *fx_;   // du/dωy
    jacobian_uv_ksai (0,2) = - y*invz *fx_;              // du/dωz
    jacobian_uv_ksai (0,3) = invz *fx_;                  // du/dtx
    jacobian_uv_ksai (0,4) = 0;                          // du/dty
    jacobian_uv_ksai (0,5) = -x*invz_2 *fx_;             // du/dtz

    jacobian_uv_ksai (1,0) = - ( 1+y*y*invz_2 ) *fy_;   // dv/dωx
    jacobian_uv_ksai (1,1) = x*y*invz_2 *fy_;            // dv/dωy
    jacobian_uv_ksai (1,2) = x*invz *fy_;                // dv/dωz
    jacobian_uv_ksai (1,3) = 0;                          // dv/dtx
    jacobian_uv_ksai (1,4) = invz *fy_;                  // dv/dty
    jacobian_uv_ksai (1,5) = -y*invz_2 *fy_;             // dv/dtz  

    Eigen::Matrix<double, 1, 2> jacobian_pixel_uv;
    jacobian_pixel_uv (0,0) = (getPixelValue(u+1, v) - getPixelValue(u-1, v))/2;
    jacobian_pixel_uv (0,1) = (getPixelValue(u, v+1) - getPixelValue(u  , v-1))/2;        //中心差分
    _jacobianOplusXi = jacobian_pixel_uv * jacobian_uv_ksai;  //链式法则
  }
  virtual bool read ( istream& in ) {return false;}
  virtual bool write ( ostream& out ) const {return false;}

protected:
  inline float getPixelValue(float x, float y)
  {
     uchar* data = & image_->data[int(y) * image_->step + int(x)];  //存放xy处像素
    float xx = x - floor(x);
    float yy = y - floor(y);          //floor向下取整
    return float(
        (1-xx) * (1-yy) * data[0] +
        xx * (1-yy) * data[1] +
        (1-xx) * yy * data[image_->step] +
         xx * yy * data[image_->step+1]
    );
  }

public:
  Eigen::Vector3d x_world_;  
  float fx_=0, fy_=0, cx_=0, cy_=0;  
  cv::Mat* image_ = nullptr;  
  Eigen::Matrix<double, 2, 6> jacobian_uv_ksai; 
};   



int main ( int argc, char** argv )
{
    if ( argc != 2 )
    {
        cout<<"usage: useLK path_to_dataset"<<endl;
        return 1;
    }
    srand ( ( unsigned int ) time ( 0 ) );
    string path_to_dataset = argv[1];
    string associate_file = path_to_dataset + "/associate.txt";

    ifstream fin ( associate_file );

    string rgb_file, depth_file, time_rgb, time_depth;
    cv::Mat color, depth, gray;
    vector<Measurement> measurements;
    // 相机内参
    float cx = 325.5;
    float cy = 253.5;
    float fx = 518.0;
    float fy = 519.0;
    float depth_scale = 1000.0;
    Eigen::Matrix3f K;
    K<<fx,0.f,cx,0.f,fy,cy,0.f,0.f,1.0f;         //相机投影的内参K矩阵

    Eigen::Isometry3d Tcw = Eigen::Isometry3d::Identity();       //相机位姿

    cv::Mat prev_color;
    // 以第一个图像为参考，对后续图像和参考图像做直接法
    for ( int index=0; index<10; index++ )
    {
        cout<<"*********** loop "<<index<<" ************"<<endl;
        fin>>time_rgb>>rgb_file>>time_depth>>depth_file;
        color = cv::imread ( path_to_dataset+"/"+rgb_file );
        depth = cv::imread ( path_to_dataset+"/"+depth_file, -1 );
        if ( color.data==nullptr || depth.data==nullptr )
            continue; 
        cv::cvtColor ( color, gray, cv::COLOR_BGR2GRAY );
        if ( index ==0 )
        {
            // 对第一帧提取FAST特征点
            vector<cv::KeyPoint> keypoints;
            cv::Ptr<cv::FastFeatureDetector> detector = cv::FastFeatureDetector::create();
            detector->detect ( color, keypoints );
            for ( auto kp:keypoints )
            {
                // 去掉邻近边缘处的点
                if ( kp.pt.x < 20 || kp.pt.y < 20 || ( kp.pt.x+20 ) >color.cols || ( kp.pt.y+20 ) >color.rows )
                    continue;
                ushort d = depth.ptr<ushort> ( cvRound ( kp.pt.y ) ) [ cvRound ( kp.pt.x ) ];           //y行x列的深度
                if ( d==0 )
                    continue;
                Eigen::Vector3d p3d = project2Dto3D ( kp.pt.x, kp.pt.y, d, fx, fy, cx, cy, depth_scale );
                float grayscale = float ( gray.ptr<uchar> ( cvRound ( kp.pt.y ) ) [ cvRound ( kp.pt.x ) ] );
                measurements.push_back ( Measurement ( p3d, grayscale ) );
            }
            prev_color = color.clone();
            continue;
        }
        
      
        chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
        poseEstimationDirect ( measurements, &gray, K, Tcw );                      //直接法得到Tcw
        chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
        chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>> ( t2-t1 );
        cout<<"direct method costs time: "<<time_used.count() <<" seconds."<<endl;
        cout<<"Tcw="<<Tcw.matrix() <<endl;
        
       
        cv::Mat img_show ( color.rows*2, color.cols, CV_8UC3 );                   //cv::Mat(高度, 宽度, 类型)
        prev_color.copyTo ( img_show ( cv::Rect ( 0,0,color.cols, color.rows ) ) );       
        color.copyTo ( img_show ( cv::Rect ( 0,color.rows,color.cols, color.rows ) ) );
        for ( Measurement m:measurements )                                        //历遍所有measurements
        {
            if ( rand() > RAND_MAX/5 )     //绘制20％                                 
                continue;
            Eigen::Vector3d p = m.pos_world;
            Eigen::Vector2d pixel_prev = project3Dto2D ( p ( 0,0 ), p ( 1,0 ), p ( 2,0 ), fx, fy, cx, cy );
            Eigen::Vector3d p2 = Tcw*m.pos_world;
            Eigen::Vector2d pixel_now = project3Dto2D ( p2 ( 0,0 ), p2 ( 1,0 ), p2 ( 2,0 ), fx, fy, cx, cy );
            if ( pixel_now(0,0)<0 || pixel_now(0,0)>=color.cols || pixel_now(1,0)<0 || pixel_now(1,0)>=color.rows )
                continue;

            float b = 255*float ( rand() ) /RAND_MAX;
            float g = 255*float ( rand() ) /RAND_MAX;
            float r = 255*float ( rand() ) /RAND_MAX;
            cv::circle ( img_show, cv::Point2d ( pixel_prev ( 0,0 ), pixel_prev ( 1,0 ) ), 8, cv::Scalar ( b,g,r ), 2 );
            cv::circle ( img_show, cv::Point2d ( pixel_now ( 0,0 ), pixel_now ( 1,0 ) +color.rows ), 8, cv::Scalar ( b,g,r ), 2 );
            cv::line ( img_show, cv::Point2d ( pixel_prev ( 0,0 ), pixel_prev ( 1,0 ) ), cv::Point2d ( pixel_now ( 0,0 ), pixel_now ( 1,0 ) +color.rows ), cv::Scalar ( b,g,r ), 1 );
            //cv::circle(图像, 圆心坐标, 圆半径, 颜色, 线条粗细)
        
        }
        cv::imshow ( "result", img_show );
        cv::waitKey ( 0 );

    }
    return 0;
}

bool poseEstimationDirect ( const vector< Measurement >& measurements, cv::Mat* gray, Eigen::Matrix3f& K, Eigen::Isometry3d& Tcw )
{
    
    // 初始化g2o
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,1>> DirectBlock;  
    unique_ptr<DirectBlock::LinearSolverType> linearSolver = make_unique<g2o::LinearSolverDense< DirectBlock::PoseMatrixType > >(); //稠密求解器
    unique_ptr<DirectBlock> solver_ptr = make_unique<DirectBlock> ( move(linearSolver));     //BlockSolver 负责分块海森矩阵，内部依靠 linearSolver 真正解方程                               
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg (move(solver_ptr)); // L-M
    g2o::SparseOptimizer optimizer;                
    optimizer.setAlgorithm (solver);
    optimizer.setVerbose( true );            

    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
    pose->setEstimate ( g2o::SE3Quat ( Tcw.rotation(), Tcw.translation() ) );
    pose->setId ( 0 );
    optimizer.addVertex ( pose );

    // 添加边
    int id=1;
    for ( Measurement m: measurements )
    {
        EdgeSE3ProjectDirect* edge = new EdgeSE3ProjectDirect (              
            m.pos_world,
            K ( 0,0 ), K ( 1,1 ), K ( 0,2 ), K ( 1,2 ), gray
        );
        edge->setVertex ( 0, pose );
        edge->setMeasurement ( m.grayscale );         //观测值
        edge->setInformation ( Eigen::Matrix<double,1,1>::Identity() );       
        edge->setId ( id++ );
        optimizer.addEdge ( edge );
    }
    cout<<"edges in graph: "<<optimizer.edges().size() <<endl;
    optimizer.initializeOptimization();
    optimizer.optimize ( 30 );
    Tcw = pose->estimate();        //得到当前位姿
    return 0;
}