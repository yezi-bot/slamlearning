#include <iostream>
#include <fstream>
#include <list>
#include <vector>
#include <chrono>
using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>

int main(int argc, char** argv)
{
    
    if (argc != 2) {
        cout << "Usage: useLK path_to_dataset" << endl;
        return 1;
    }
    string path_to_dataset = argv[1];
    string association_file = path_to_dataset + "/associate.txt";
    ifstream fin(association_file);
    string rgb_file, depth_file, time_rgb, time_depth;
    list<cv::Point2f> keypoints;
    cv::Mat color, depth, last_color;
    for (int index = 0; index < 10; index++) {
        fin >> time_rgb >> rgb_file >> time_depth >> depth_file;
        color = cv::imread(path_to_dataset + "/" + rgb_file);
        depth = cv::imread(path_to_dataset + "/" + depth_file, -1);
       
        if (index == 0) {
            vector<cv::KeyPoint> kps;        
            cv::Ptr<cv::FastFeatureDetector> detector = cv::FastFeatureDetector::create();
            detector->detect(color, kps);        //用Fast读取所有角点存入kps
            for (auto kp : kps)                  //临时变量kp
                keypoints.push_back(kp.pt);      //转成Point2f类型
            last_color = color.clone();        //保存上一帧图像作为参考图
            continue;
        }
        if (color.data == nullptr || depth.data == nullptr)
           continue;

        vector<cv::Point2f> next_keypoints;
        vector<cv::Point2f> prev_keypoints;
        for (auto kp : keypoints)                 //LK的输入输出必须是vector
            prev_keypoints.push_back(kp);
        vector<uchar> status;
        vector<float> error;

        chrono::steady_clock::time_point t1 = chrono::steady_clock::now();     //LK计算
        cv::calcOpticalFlowPyrLK(last_color, color, prev_keypoints, next_keypoints, status, error);
        chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
        chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
        cout << "LK Flow use time: " << time_used.count() << " seconds." << endl;

        //删除失败点，筛选有效点
        int i = 0;
        for (auto iter=keypoints.begin(); iter!=keypoints.end(); i++) {
            if (status[i]==0) {
               iter = keypoints.erase(iter);
               continue;
            }
            *iter = next_keypoints[i];      //更新特征点位置
            iter++;
        }
        cout<< "tracked keypoints: " << keypoints.size() << endl;
        if(keypoints.size() == 0)
        {
            cout<<"all keypoints are lost." << endl;
            break;
        }
       cv::Mat img_show = color.clone();
       for (auto kp:keypoints)
           cv::circle(img_show, kp, 10, cv::Scalar(0, 240, 0), 1);
       cv::imshow("corners", img_show);
       cv::waitKey(0);
       last_color = color;     //下一帧参考图片
    }
    return 0;
}
