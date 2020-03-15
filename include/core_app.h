#pragma once
#include <string>
#include <opencv2/opencv.hpp>
#include <chrono>

class CoreApp
{
    
public:
    CoreApp() {};
    CoreApp(std::string host, int port);
    void start_decoder_test(const std::string& image_path, const std::string& data_path, bool use_network);
    void start_camera_test(bool use_network);
    void start_app();
    bool open_socket(std::string host, int port);
    bool get_socket_ready();

private:
    void set_timer();
    void log_timer(std::string msg);
    bool send_frame(cv::Mat& frame);

private:
    bool mSocketReady = false;
    int mSocket = 0;
    std::vector<uchar> mBuffer;
    
#ifdef _WIN32
    std::chrono::time_point<std::chrono::steady_clock> mStartTime;
#else
    std::chrono::time_point<std::chrono::system_clock> mStartTime;
#endif
};

