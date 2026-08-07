#include "myslam/config.h"
namespace myslam
{
    std::shared_ptr<Config> Config::config_ = nullptr;  //类型，变量名，初始值

    Config::~Config()
    {
        if (file_.isOpened())
            file_.release();
    }

    void Config::setParameterFile(const std::string& filename)
    {
        if (config_ == nullptr)  
            config_ = std::shared_ptr<Config>(new Config());
        config_->file_ = cv::FileStorage(filename, cv::FileStorage::READ);  //打开yaml文件进行读写相机参数
        if (config_->file_.isOpened() == false)  //打开失败
        {
            LOG(ERROR) << "parameter file " << filename << " does not exist.";
            config_ = nullptr;
        }
    }
}