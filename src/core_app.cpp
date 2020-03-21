#include "../include/core_app.h"
#include "../include/utils.h"

#ifdef _WIN32
    #define _WINSOCK_DEPRECATED_NO_WARNINGS
    #include <winsock2.h>
#else
    #include <sys/socket.h> 
    #include <arpa/inet.h>
    #include <unistd.h>
    #include <netinet/in.h>
#endif

#include <thread>
#include <fstream>

#include <unsupported/Eigen/CXX11/Tensor>
#include <opencv2/opencv.hpp>
#include <cxxopts/cxxopts.hpp>

#include "../include/decoder.h"
#include "../include/Test.h"
#include "../include/quantizer.h"
#include "../include/utils.h"


using MatrixXf = Eigen::MatrixXf;
using string = std::string;

const int box_size = 8732; // j["box_size"].get<int>();
const int class_size = 48; // j["class_size"].get<int>();
const int layers = 6;

const int l1_size = 16 * 38 * 38;
const int c1_size = 192 * 38 * 38;
const int l2_size = 24 * 19 * 19;
const int c2_size = 288 * 19 * 19;
const int l3_size = 24 * 10 * 10;
const int c3_size = 288 * 10 * 10;
const int l4_size = 24 * 5 * 5;
const int c4_size = 288 * 5 * 5;
const int l5_size = 16 * 3 * 3;
const int c5_size = 192 * 3 * 3;
const int l6_size = 16 * 1 * 1;
const int c6_size = 192 * 1 * 1;

const std::vector<std::array<int, 3>> l_dims({
    {16, 38, 38},
    {24, 19, 19},
    {24, 10, 10},
    {24, 5, 5},
    {16, 3, 3},
    {16, 1, 1}
});

const std::vector<std::array<int, 3>> c_dims({
    {192, 38, 38},
    {288, 19, 19},
    {288, 10, 10},
    {288, 5, 5},
    {192, 3, 3},
    {192, 1, 1}
});

CoreApp::CoreApp(std::string host, int port)
{
    open_socket(host, port);
}

void CoreApp::start_decoder_test(const std::string& image_path, const std::string& data_path, bool use_network)
{
    int mem_size = l1_size + c1_size + l2_size + c2_size + l3_size + c3_size + l4_size + c4_size + l5_size + c5_size + l6_size + c6_size;
    int mem_pointer;

    std::vector<float> mem(mem_size);
    float value;
    int count = 0;

    std::ifstream infile(data_path);
    while (infile >> value) {
        mem[count] = value;
        count++;
    }

    Quantizer quant(5, 11);
    quant.unquant(&mem[0], mem_size);

    std::vector<float> mem_l;
    std::vector<float> mem_c;

    Test test;
    Decoder decoder;
    decoder.init_default_boxes300();

    //todo: speed
    set_timer();
    mem_pointer = 0;
    for (int i = 0; i < layers; i++) {
        int size_l = l_dims[i][0] * l_dims[i][1] * l_dims[i][2];
        int size_c = c_dims[i][0] * c_dims[i][1] * c_dims[i][2];

        Eigen::TensorMap<Eigen::Tensor<float, 3>> l1_tensor(&mem[mem_pointer], l_dims[i][1], l_dims[i][0], l_dims[i][2]);
        Eigen::Tensor<float, 3> l_shuffle = l1_tensor.shuffle(Eigen::array<int, 3>({ 0, 2, 1 }));

        Eigen::MatrixXf view2d_l = Eigen::Map<Eigen::MatrixXf>(l_shuffle.data(), (int)(size_l / 4), 4);
        view2d_l.transposeInPlace();
        //memcpy(&mem[0], fp, 12 * sizeof(float));
        mem_l.insert(mem_l.end(), view2d_l.data(), view2d_l.data() + size_l);

        mem_pointer += size_l;
        Eigen::TensorMap<Eigen::Tensor<float, 3>> c1_tensor(&mem[mem_pointer], c_dims[i][1], c_dims[i][0], c_dims[i][2]);
        Eigen::Tensor<float, 3> c_shuffle = c1_tensor.shuffle(Eigen::array<int, 3>({ 0, 2, 1 }));

        Eigen::MatrixXf view2d_c = Eigen::Map<Eigen::MatrixXf>(c_shuffle.data(), (int)(size_c / class_size), class_size);
        view2d_c.transposeInPlace();
        mem_c.insert(mem_c.end(), view2d_c.data(), view2d_c.data() + size_c);

        mem_pointer += size_c;
    }
    log_timer("Time shuffle 1: %ims");

    set_timer();
    Eigen::MatrixXf locations = Eigen::Map<Eigen::MatrixXf>(mem_l.data(), 4, box_size);
    Eigen::MatrixXf scores = Eigen::Map<Eigen::MatrixXf>(mem_c.data(), class_size, box_size);

    locations.transposeInPlace();
    scores.transposeInPlace();
    log_timer("Time shuffle 2: %ims");

    //auto finish = std::chrono::high_resolution_clock::now();
    

    set_timer();
    vector<BoxLabel> output = decoder.listdecode_batch(locations, scores);
    //finish = std::chrono::high_resolution_clock::now();
    log_timer("Complete Time decode batch: %ims");

    cv::Mat image;
    image = cv::imread(image_path, cv::IMREAD_COLOR);

    int width = image.cols;
    int height = image.rows;

    for (auto label : output) {
        auto p1 = cv::Point(label.coordinates[0] * width, label.coordinates[1] * height);
        auto p2 = cv::Point(label.coordinates[2] * width, label.coordinates[3] * height);
        rectangle(image, p1, p2, cv::Scalar(0, 0, 255), 2);
    }

    if (image.data) {
        if (!use_network) {
            cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE);
            imshow("Display window", image);
            cv::waitKey(0);
        }
        else
            send_frame(image);
    }
    return;
}

void CoreApp::start_camera_test(bool use_network)
{
    cv::Mat frame;
    cv::VideoCapture cap;
    cap.open("E:\\aquaman_2018.mkv");

    if (!cap.isOpened()) {
        std::cerr << "ERROR! Unable to open camera\n";
        return;
    }

    //std::vector<uchar> buf;
    int bytes = 0;
    for (;;)
    {

        cap.read(frame);
        if (frame.empty()) {
            std::cerr << "ERROR! blank frame grabbed\n";
            break;
        }
        if (use_network) {
            std::cout << "t1" << std::endl;
            send_frame(frame);
            std::cout << "t2" << std::endl;
        }
        else {
            cv::imshow("cam", frame);
            if (cv::waitKey(30) >= 0) break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
}

void CoreApp::start_app(int vsize)
{
    int img_size = 300;
    int mem_size = l1_size + c1_size + l2_size + c2_size + l3_size + c3_size + l4_size + c4_size + l5_size + c5_size + l6_size + c6_size;
    int mem_pointer;
    short* input_data = new short[img_size * img_size * 3];
    
    std::vector<float> mem_l;
    std::vector<float> mem_c;
    std::vector<float> mem(mem_size);

    Decoder decoder;
    Quantizer quant(5, 11);
    decoder.init_default_boxes300();
    
    float value;
    int count = 0;

    cv::Mat frame;
    cv::VideoCapture cap;
    cap.open("E:\\aquaman_2018.mkv");

    if (!cap.isOpened()) {
        LOG_ERROR("ERROR! Unable to open camera");
        return;
    }

    for (;;)
    {

        cap.read(frame);
        if (frame.empty()) {
            LOG_ERROR("ERROR! blank frame grabbed");
            break;
        }

        img_to_data(input_data, img_size, frame);
        // ---------- DRAM Access ----------------
        // - input_data to dram
        // - wait until core is finished
        // - data to mem 
        // ---------------------------------------
        quant.unquant(&mem[0], mem_size);


        set_timer();
        mem_pointer = 0;
        for (int i = 0; i < layers; i++) {
            int size_l = l_dims[i][0] * l_dims[i][1] * l_dims[i][2];
            int size_c = c_dims[i][0] * c_dims[i][1] * c_dims[i][2];

            Eigen::TensorMap<Eigen::Tensor<float, 3>> l1_tensor(&mem[mem_pointer], l_dims[i][1], l_dims[i][0], l_dims[i][2]);
            Eigen::Tensor<float, 3> l_shuffle = l1_tensor.shuffle(Eigen::array<int, 3>({ 0, 2, 1 }));

            Eigen::MatrixXf view2d_l = Eigen::Map<Eigen::MatrixXf>(l_shuffle.data(), (int)(size_l / 4), 4);
            view2d_l.transposeInPlace();
            mem_l.insert(mem_l.end(), view2d_l.data(), view2d_l.data() + size_l);

            mem_pointer += size_l;
            Eigen::TensorMap<Eigen::Tensor<float, 3>> c1_tensor(&mem[mem_pointer], c_dims[i][1], c_dims[i][0], c_dims[i][2]);
            Eigen::Tensor<float, 3> c_shuffle = c1_tensor.shuffle(Eigen::array<int, 3>({ 0, 2, 1 }));

            Eigen::MatrixXf view2d_c = Eigen::Map<Eigen::MatrixXf>(c_shuffle.data(), (int)(size_c / class_size), class_size);
            view2d_c.transposeInPlace();
            mem_c.insert(mem_c.end(), view2d_c.data(), view2d_c.data() + size_c);

            mem_pointer += size_c;
        }
        log_timer("Time shuffle 1: %ims");

        set_timer();
        Eigen::MatrixXf locations = Eigen::Map<Eigen::MatrixXf>(mem_l.data(), 4, box_size);
        Eigen::MatrixXf scores = Eigen::Map<Eigen::MatrixXf>(mem_c.data(), class_size, box_size);

        locations.transposeInPlace();
        scores.transposeInPlace();
        log_timer("Time shuffle 2: %ims");    


        set_timer();
        vector<BoxLabel> output = decoder.listdecode_batch(locations, scores);
        log_timer("Complete Time decode batch: %ims");
           
        int width = frame.cols;
        int height = frame.rows;
        for (auto label : output) {
            auto p1 = cv::Point(label.coordinates[0] * width, label.coordinates[1] * height);
            auto p2 = cv::Point(label.coordinates[2] * width, label.coordinates[3] * height);
            rectangle(frame, p1, p2, cv::Scalar(0, 0, 255), 2);
        }
        send_frame(frame);
    }
    
}

bool CoreApp::open_socket(std::string host, int port)
{
    struct sockaddr_in serverAddr;
    int addrLen = sizeof(struct sockaddr_in);

#ifdef _WIN32
    WSADATA Data;
    WSAStartup(MAKEWORD(2, 2), &Data); // 2.2 version
#endif
    mSocket = socket(AF_INET, SOCK_STREAM, 0);


    if (mSocket < 0) {
        std::cerr << "socket() failed" << std::endl;
        mSocketReady = false;
        return false;
    }

    serverAddr.sin_family = PF_INET;
    serverAddr.sin_addr.s_addr = inet_addr(host.c_str());
    serverAddr.sin_port = htons(port);

    if (connect(mSocket, (sockaddr*)&serverAddr, addrLen) < 0) {
        std::cerr << "connect() failed!" << std::endl;
        mSocketReady = false;
        return false;
    }

    mSocketReady = true;
    return mSocketReady;
}

bool CoreApp::get_socket_ready()
{
    return mSocketReady;
}

void CoreApp::set_timer()
{
    mStartTime = std::chrono::high_resolution_clock::now();
}

void CoreApp::log_timer(std::string msg)
{
    auto finish = std::chrono::high_resolution_clock::now();
    LOG_INFO(msg, std::chrono::duration_cast<std::chrono::milliseconds>(finish - mStartTime).count());
}

bool CoreApp::send_frame(cv::Mat& frame)
{
    int bytes = 0;

    if (frame.empty()) {
        std::cerr << "ERROR! blank frame grabbed\n";
        return false;
    }

    if(!mSocketReady) {
        std::cerr << "ERROR! No connection\n";
        return false;
    }

    cv::resize(frame, frame, cv::Size(1080, 720));
    std::vector<int> params;
    params.push_back(cv::IMWRITE_JPEG_QUALITY);
    params.push_back(60); //image quality
    imencode(".jpg", frame, mBuffer, params);

    unsigned char size_bytes[4];
    unsigned long n = mBuffer.size();

    size_bytes[0] = (n >> 24) & 0xFF;
    size_bytes[1] = (n >> 16) & 0xFF;
    size_bytes[2] = (n >> 8) & 0xFF;
    size_bytes[3] = n & 0xFF;

    std::cout << "send size 4 bytes: size " << n << std::endl;
    if ((bytes = send(mSocket, (const char*)&size_bytes, 4, 0)) < 0) {
        std::cerr << "bytes = " << bytes << std::endl;
        return false;
    }

    std::cout << "send: " << mBuffer.size() << std::endl;
    if ((bytes = send(mSocket, (const char*)mBuffer.data(), mBuffer.size(), 0)) < 0) {
        std::cerr << "bytes = " << bytes << std::endl;
        return false;
    }
    std::cout << "end" << std::endl;
}

void CoreApp::img_to_data(short* data_ptr, int size, cv::Mat& img)
{
    cv::Mat img_resize;
    cv::resize(img, img_resize, cv::Size(size, size));
    img_resize.convertTo(img_resize, CV_32FC3);
    Quantizer quant(5, 11);
    quant.normalize_c3(img_resize.ptr<float>(), 300 * 300 * 3, { 0.229, 0.224, 0.225 }, { 0.485, 0.456, 0.406 });
    quant.to_quantized_int(img_resize.ptr<float>(), 300 * 300 * 3);

    int count = 0;
    std::vector<cv::Mat> split_channel;
    cv::split(img_resize, split_channel);
    for (int y = 0; y < 300; ++y) {
        for (int c = 2; c >= 0; --c) {
            for (int x = 0; x < 300; ++x) {
                data_ptr[count] = (short)split_channel[c].at<float>(y, x);
                count++;
            }
        }
    }
}
