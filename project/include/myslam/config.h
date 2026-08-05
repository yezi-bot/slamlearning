#ifndef CONFIG_H
#define CONFIG_H

#include "myslam/common_include.h" 

namespace myslam 
{
class Config
{
private:
    static std::shared_ptr<Config> config_; 
    cv::FileStorage file_;           //打开yaml文件进行读写相机参数 
    Config () {}                     //放在私有区禁止外部实例化，保证只有一个config对象

public:
    ~Config();   
    static void setParameterFile( const std::string& filename ); 
    template< typename T >
    //模板函数
    static T get( const std::string& key )
    {
        return T( Config::config_->file_[key] );  //转成T类型
    }
};
}

#endif 