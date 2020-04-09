#pragma once
#include <string>
#include <opencv2/opencv.hpp>
#include <chrono>
#include "decoder.h"
#include "dm_utils.h"

class CoreApp
{
    
public:
    CoreApp() {};
    CoreApp(std::string host, int port);
    void start_decoder_test(const std::string& image_path, const std::string& data_path, bool use_network);
    void start_fpga_test(const std::string& image_path, const std::string& data_path);
    void start_camera_test(bool use_network);
    void start_app(int vsize);
    bool open_socket(std::string host, int port);
    bool get_socket_ready();

private:
    void set_timer();
    void set_timer(std::string msg);
    void log_timer(std::string msg);
    bool send_frame(cv::Mat& frame);
    void img_to_data(short* data_ptr, int size, cv::Mat& frame);
    vector<BoxLabel> generate_output(std::vector<float> mem);
    void draw_output(cv::Mat& frame, const vector<BoxLabel>& output);

private:
    bool mSocketReady = false;
    int mSocket = 0;
    std::vector<uchar> mBuffer;
    ReservedMemory mReservedMemory;
    AsipCtrl mAsipCtrl;
    
#ifdef _WIN32
    std::chrono::time_point<std::chrono::steady_clock> mStartTime;
#else
    std::chrono::time_point<std::chrono::system_clock> mStartTime;
#endif
};

