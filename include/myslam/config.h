#pragma once
#include "myslam/common_include.h"

namespace myslam
{
class Config
{
  private:
    static std::shared_ptr<Config> config_;
    cv::FileStorage file_;

    Config() {}

  public:
    ~Config();
    Config(Config const &) = delete;
    void operator=(Config const &) = delete;

    // set a new config file
    static void setParamFile(const std::string &filename);

    // access the params
    template <typename T>
    static T get(const std::string &key)
    {
        return T(Config::config_->file_[key]);
    }
};
} // namespace myslam