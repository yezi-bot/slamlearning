#include <iostream>
#include <g2o/core/base_vertex.h>  //顶点基类
#include <g2o/core/base_edge.h>    //一元边基类
#include <g2o/core/block_solver.h>   //矩阵求解器
#include <g2o/core/optimization_algorithm_levenberg.h> //LM算法
#include <g2o/core/base_unary_edge.h>
#include <g2o/solvers/dense/linear_solver_dense.h> //稠密矩阵求解器(QR)
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <cmath>  //用到exp
#include <chrono>
#include <memory>
using namespace std;

//优化顶点
class CurveFittingVertex: public g2o::BaseVertex<3, Eigen::Vector3d> //父类参数：优化变量维度，打包成向量
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW         //类内有 Eigen 固定大小矩阵，向量必须写
    virtual void setToOriginImpl() override //初始化
    {
        _estimate << 0, 0, 0;
    }

    virtual void oplusImpl(const double* update) //每次算出增量
    {
        _estimate +=Eigen ::Vector3d(update);

    }
    virtual bool read(istream& in){return true;}
    virtual bool write(ostream& out)const{return true;}
   
}; 

//一元边，表示残差        //参数：误差维度，观测值类型，连接顶点类型
class CurveFittingEdge: public g2o::BaseUnaryEdge<1,double,CurveFittingVertex> 
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CurveFittingEdge(double x):BaseUnaryEdge(),_x(x){} //初始化列表写法，一元边函数   

    void computeError() override
    {
        const CurveFittingVertex* v = static_cast<const CurveFittingVertex*>(_vertices[0]);//强制转换类型，（）里是g2o自带数组，一元边所以下标是0
        const Eigen::Vector3d abc = v->estimate();         //返回储存的abc
        _error(0,0) = _measurement - exp(abc(0,0)*_x*_x + abc(1,0)*_x + abc(2,0));
    }
    virtual bool read(istream& in){return true;}
    virtual bool write(ostream& out)const{return true;}
public:
    double _x; //x值，y值为_measurement    

};

int main(int argc, char** argv)
{
    double a=1.0, b=2.0, c=1.0; 
    int N=100; 
    double w_sigma=1.0; 
    cv::RNG rng;
    double abc[3] = {0,0,0}; 

    vector<double> x_data, y_data; 
    cout<<"generating data: "<<endl;
    for(int i=0;i<N;i++)
    {
        double x = i/100.0;
        x_data.push_back(x);
        y_data.push_back(exp(a*x*x + b*x + c) + rng.gaussian(w_sigma*w_sigma));
        cout<<x_data[i]<<" "<<y_data[i]<<endl;
    }

    //构建图优化，先设定g2o
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<3,1>> Block; //分块处理矩阵每个误差项优化变量维度为3，误差值维度为1，定义计算规则
    unique_ptr<Block::LinearSolverType> linearSolver=make_unique<g2o::LinearSolverDense<Block::PoseMatrixType>>();//创建计算工具，线性方程求解器，稠密矩阵
    unique_ptr<Block> solver_ptr(new Block(move(linearSolver))); //linearSolver算出来detax和H/b,Block对abc求偏导得到完整的H和b
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(move(solver_ptr)); //LM修正加入阻尼系数
    g2o::SparseOptimizer optimizer;     //存放abc和残差的容器
    optimizer.setAlgorithm(solver);     //容器调用LM公式
    optimizer.setVerbose(true);

    CurveFittingVertex* v = new  CurveFittingVertex ();    //初值
    v->setEstimate(Eigen::Vector3d(0,0,0)); //重复设初值，也可以设别的
    v->setId(0);             //g2o要求固定顶点
    optimizer.addVertex(v); //注册到顶点

    //加边
   for (int i =0;i<N;i++)
   {
    CurveFittingEdge* edge = new CurveFittingEdge(x_data[i]);   //new自动返回当前类的指针
    edge->setId(i);
    edge->setVertex(0,v);                 //添加点
    edge->setMeasurement(y_data[i]);     //输入观测值
    edge->setInformation(Eigen::Matrix<double,1,1>::Identity()*1/(w_sigma*w_sigma));  //噪声的逆矩阵
    optimizer.addEdge(edge);             //注册边
   }

   //优化执行
   cout<<"start optimization"<<endl;
   chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
   optimizer.initializeOptimization();
   optimizer.optimize(100);
   chrono::steady_clock::time_point t2=chrono::steady_clock::now();
   chrono::duration<double>time_used = chrono::duration_cast<chrono::duration<double>>(t2-t1);
   cout<<time_used.count()<<endl;
   





}   