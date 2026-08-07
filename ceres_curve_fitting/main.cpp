#include <iostream>
#include <ceres/ceres.h>
#include <opencv2/core/core.hpp>
#include <chrono>             //c++计算时间
using namespace std;
using namespace cv;

//代价函数
struct CURVE_FITTING_COST
{
    CURVE_FITTING_COST(double x, double y) : _x(x), _y(y) {}
    template <typename T>      //函数模板，适配double和Jet两个类型
    bool operator()(const T *const abc, T *residual) const  //运算符重载，最后const保证x和y不变
    {
        // y = exp(ax^2 + bx + c)
        residual[0] = T(_y) - ceres::exp(abc[0] * T(_x) * T(_x) + abc[1] * T(_x) + abc[2]); //残差定义
        return true;
    }
    const double _x, _y; // x,y数据
};

int main(int argc, char **argv)
{
    //产生带噪声的观测数据
    double a = 1.0, b = 2.0, c = 1.0; 
    int N = 100;                     
    double w_sigma = 1.0;           //噪声标准差
    RNG rng;                     //opencv随机数产生器
    double abc[3] = {0, 0, 0};       //参数估计值
    vector<double> x_data, y_data;     //存放带噪声的x和y
    for (int i = 0; i < N; i++)
    {
        double x = i / 100.0;
        x_data.push_back(x);
        y_data.push_back(exp(a * x * x + b * x + c) + rng.gaussian(w_sigma)); //观测值=y+标准差是1的噪声


        cout << x_data[i] << " " << y_data[i] << endl;
    }

    //构建最小二乘问题
    ceres::Problem problem;            //优化容器：problem
    for (int i = 0; i < N; i++)
    {
        problem.AddResidualBlock( //第一个参数是代价函数指针（算残差的结构体，每次输出1个残差，3个优化变量）
            new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3>(new CURVE_FITTING_COST(x_data[i], y_data[i])), 
            nullptr,          //损失函数  ，平方直接累加                                                                                      
            abc );                                                                                                      
    }

    //配置求解器
    ceres::Solver::Options options; //option是ceres的配置清单
    options.linear_solver_type = ceres::DENSE_QR; //增量方程如何求解，QR分解
    options.minimizer_progress_to_stdout = true;   //迭代过程输出

    ceres::Solver::Summary summary; //优化信息收纳到summary
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now(); //计算耗时
    ceres::Solve(options, &problem, &summary); //调用求解器，最小化总cost
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);

    cout<<summary.BriefReport() << endl; 
    for (auto i = 0; i < 3; i++)
    {
        cout << "abc[" << i << "] = " << abc[i] << endl;
    }
    return 0;
}    