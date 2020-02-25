#include <iostream>
#include <nlohmann/json.hpp>
#include <fstream>
#include <chrono>
#include <fstream>


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
#include <opencv2/opencv.hpp>
#include <cxxopts/cxxopts.hpp>

#include "../include/decoder.h"
#include "../include/utils.h"
#include "../include/Test.h"
#include "../include/quantizer.h"
#include "../include/core_app.h"
#include <unsupported/Eigen/CXX11/Tensor>



using MatrixXf = Eigen::MatrixXf;
using json = nlohmann::json;
using string = std::string;

void test();
void send_cam(const char* ip, int port);
void send_frame(const char* ip, int port, cv::Mat frame);

int main(int argc, char* argv[])
{
    //test();
    string file_path = __FILE__;
    #ifdef _WIN32
        string dir_path = file_path.substr(0, file_path.rfind("\\"));
    #else
        string dir_path = file_path.substr(0, file_path.rfind("/"));
    #endif
    //todo
    //dir_path.append("/../data/data.txt");
    string data_path = dir_path;
    data_path.append("/../data/data.txt");

    //string json_path = string(dir_path).append("ssd_output.json"); // ssd_output.json");
    string image_path = string(dir_path).append("/../data/000000026564.jpg");

    cxxopts::Options options("decoder", "post processing ssd");
    options.positional_help("[optional args]").show_positional_help();
    options
        .allow_unrecognised_options()
        .add_options()
        ("h, help", "Print help")
        ("t, test", "Start decoder test with example image and data")
        ("d, data", string("data input file for test data. Default: ").append(data_path), cxxopts::value<string>())
        ("v, verbose", "Verbose \"info()\" output")
        ("host", string("IP, stream over network"), cxxopts::value<string>())
        ("p, port", string("Port, default: 8485"), cxxopts::value<int>())
        ("i, image", string("Path to image for test mode. Default: ").append(image_path), cxxopts::value<string>());

    auto result = options.parse(argc, argv);
    int port = 8485;
    string ip;
    bool network = false;

    if(result.count("host")) {
        std::cout << "network mode" << std::endl;
        network = true;
        ip = result["host"].as<string>();
        if(result.count("p")) 
            port = result["p"].as<int>();
    }

    if (result.count("h")) {
        std::cout << options.help({ "" }) << std::endl;
        exit(0);
    }

    if (result.count("v")) {
        std::cout << "Verbose ON" << std::endl;
        ::utils::verbose = true;
    }

    if (result.count("t")) {
        std::cout << "*** Decoder Test  ***" << std::endl;
        if (result.count("i"))
            image_path = result["i"].as<string>();
        if(result.count("j"))
            data_path = result["j"].as<string>();

        CoreApp app = CoreApp();
        if (network)
            app.open_socket(ip, port);

        app.start_decoder_test(image_path, data_path, network);
    }
}

void send_cam(const char* ip, int port)
{
    std::cout << "send over network" << std::endl;
    int sokt;

    struct sockaddr_in serverAddr;
    int addrLen = sizeof(struct sockaddr_in);
    sokt = socket(PF_INET, SOCK_STREAM, 0);
    if (sokt < 0) {
        std::cerr << "socket() failed" << std::endl;
    }

    serverAddr.sin_family = PF_INET;
    serverAddr.sin_addr.s_addr = inet_addr(ip);
    serverAddr.sin_port = htons(port);

    if (connect(sokt, (sockaddr*)&serverAddr, addrLen) < 0) {
        std::cerr << "connect() failed!" << std::endl;
    }

    cv::Mat frame;
    cv::VideoCapture cap;
    //cap.open(0)
    cap.open("/home/basti/data/daten/aquaman_2018.mkv");

    if (!cap.isOpened()) {
        std::cerr << "ERROR! Unable to open camera\n";
        return;
    }
    std::cout << "Start grabbing" << std::endl;
    std::vector<uchar> buf;
    int bytes = 0;
    for (;;)
    {

        cap.read(frame);
        if (frame.empty()) {
            std::cerr << "ERROR! blank frame grabbed\n";
            break;
        }
        cv::resize(frame, frame, cv::Size(1080, 720));
        std::vector<int> params;
        params.push_back(cv::IMWRITE_JPEG_QUALITY);
        params.push_back(60); //image quality
        imencode(".jpg", frame, buf, params);
        //send(sokt, buf.data(), buf.size(), 0);
        unsigned char size_bytes[4];
        unsigned long n = buf.size();

        size_bytes[0] = (n >> 24) & 0xFF;
        size_bytes[1] = (n >> 16) & 0xFF;
        size_bytes[2] = (n >> 8) & 0xFF;
        size_bytes[3] = n & 0xFF;

        std::cout << "send size 4 bytes: size " << n << std::endl;
        if ((bytes = send(sokt, (const char*)&size_bytes, 4, 0)) < 0){
            std::cerr << "bytes = " << bytes << std::endl;
            break;
        }

        std::cout << "send: " << buf.size() << std::endl;
        if ((bytes = send(sokt, (const char*)buf.data(), buf.size(), 0)) < 0){
            std::cerr << "bytes = " << bytes << std::endl;
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        //imshow("Live", frame);
        //if (waitKey(5) >= 0)
            //break;
    }
}

void send_frame(const char* ip, int port, cv::Mat frame)
{
    int sokt;

    struct sockaddr_in serverAddr;
    int addrLen = sizeof(struct sockaddr_in);
    sokt = socket(PF_INET, SOCK_STREAM, 0);
    if (sokt < 0) {
        std::cerr << "socket() failed" << std::endl;
    }

    serverAddr.sin_family = PF_INET;
    serverAddr.sin_addr.s_addr = inet_addr(ip);
    serverAddr.sin_port = htons(port);

    if (connect(sokt, (sockaddr*)&serverAddr, addrLen) < 0) {
        std::cerr << "connect() failed!" << std::endl;
    }

    std::cout << "Start grabbing" << std::endl;
    std::vector<uchar> buf;
    int bytes = 0;

    if (frame.empty()) {
        std::cerr << "ERROR! blank frame grabbed\n";
        return;
    }

    cv::resize(frame, frame, cv::Size(1080, 720));
    std::vector<int> params;
    params.push_back(cv::IMWRITE_JPEG_QUALITY);
    params.push_back(60); //image quality
    imencode(".jpg", frame, buf, params);

    unsigned char size_bytes[4];
    unsigned long n = buf.size();

    size_bytes[0] = (n >> 24) & 0xFF;
    size_bytes[1] = (n >> 16) & 0xFF;
    size_bytes[2] = (n >> 8) & 0xFF;
    size_bytes[3] = n & 0xFF;

    std::cout << "send size 4 bytes: size " << n << std::endl;
    if ((bytes = send(sokt, (const char*)&size_bytes, 4, 0)) < 0){
        std::cerr << "bytes = " << bytes << std::endl;
        return;
    }

    std::cout << "send: " << buf.size() << std::endl;
    if ((bytes = send(sokt, (const char *)buf.data(), buf.size(), 0)) < 0){
        std::cerr << "bytes = " << bytes << std::endl;
        return;
    }
}

void test() {
    int height = 2;
    int width = 2;

    float norm_mean[] = { 0.485f, 0.456f, 0.406f };
    float norm_std[] = { 0.229f, 0.224f, 0.225f };

    float data_t1[24] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120 };
    float data_t2[24] = { 100, 200, 300, 400, 500, 600, 700, 800, 900, 1000, 1100, 1200, 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000, 9000, 10000, 11000, 12000 };

    //cv::Mat cv_img = cv::Mat(height, width, CV_32FC3, data);
    //std::cout << cv::format(cv_img, cv::Formatter::FMT_PYTHON) << std::endl << std::endl;

    Eigen::TensorMap<Eigen::Tensor<float, 3>> img_tensor1((float*)data_t1, 3, 4, 2);
    Eigen::TensorMap<Eigen::Tensor<float, 3>> img_tensor2((float*)data_t2, 2, 6, 2);

    /*std::cout << img_tensor(0, 0, 0) << std::endl;
    std::cout << img_tensor(0, 0, 1) << std::endl;
    std::cout << img_tensor(0, 1, 0) << std::endl;
    std::cout << img_tensor(0, 1, 1) << std::endl;
    std::cout << "--------------" << std::endl;
    std::cout << img_tensor(0, 0, 0) << std::endl;
    std::cout << img_tensor(1, 0, 0) << std::endl;
    std::cout << img_tensor(0, 1, 0) << std::endl;
    std::cout << img_tensor(1, 1, 0) << std::endl;
    std::cout << "--------------" << std::endl;*/

    Eigen::Tensor<float, 3> te1 = img_tensor1.shuffle(Eigen::array<int, 3>({ 0, 2, 1 }));
    Eigen::Tensor<float, 3> te2 = img_tensor2.shuffle(Eigen::array<int, 3>({ 0, 2, 1 }));

    Eigen::MatrixXf view2d1 = Eigen::Map<Eigen::MatrixXf>(te1.data(), 12, 2);
    Eigen::MatrixXf view2d2 = Eigen::Map<Eigen::MatrixXf>(te2.data(), 12, 2);
    view2d1.transposeInPlace();
    view2d2.transposeInPlace();

    std::vector<float> mem;
    mem.insert(mem.end(), view2d1.data(), view2d1.data() + 24);
    mem.insert(mem.end(), view2d2.data(), view2d2.data() + 24);

    Eigen::MatrixXf endMatrix = Eigen::Map<Eigen::MatrixXf>(mem.data(), 2, 24);
    endMatrix.transposeInPlace();

    float* fp = endMatrix.data();
    for (int i = 0; i < 48; ++i)
        std::cout << fp[i] << std::endl;



    

    //Eigen::MatrixXf scores = Eigen::Map<Eigen::MatrixXf>(mem.data(), 2, 24);

    int i = 0;
    //Eigen::TensorMap<Eigen::Tensor<float, 2>> tensor2(te.data(), 3, 4);
    /*std::cout << tensor2 << std::endl;
    std::cout << "--------------" << std::endl;
    std::cout << tensor2(0, 0) << std::endl;
    std::cout << tensor2(1, 0) << std::endl;
    std::cout << tensor2(0, 1) << std::endl;
    std::cout << tensor2(1, 1) << std::endl;
    std::cout << "--------------" << std::endl;

    float* fp = te.data();
    for (int i = 0; i < 12; ++i)
        std::cout << fp[i] << std::endl;

    std::vector<float> mem(12);
    //mem.insert(mem.begin(), fp, fp + 12);
    memcpy(&mem[0], fp, 12 * sizeof(float));

    std::cout << "--------------" << std::endl;


    std::cout << mem.size() << std::endl;
    Eigen::MatrixXf test_m(3, 4);

    std::vector<std::array<int, 3>> l_dims{ { 10, 11, 12 }, { 20, 21, 22 } };
    std::cout << l_dims[0][0] << " " << l_dims[0][1] << " " << l_dims[0][1] << std::endl;


    img_tensor.chip(0, 0) = img_tensor.chip(0, 0).unaryExpr([=](float c) { return ((c / 255.0f) - norm_mean[0]) / norm_std[0]; });
    img_tensor.chip(1, 0) = img_tensor.chip(1, 0).unaryExpr([=](float c) { return ((c / 255.0f) - norm_mean[1]) / norm_std[1]; });
    img_tensor.chip(2, 0) = img_tensor.chip(2, 0).unaryExpr([=](float c) { return ((c / 255.0f) - norm_mean[2]) / norm_std[2]; });
    std::cout << img_tensor << std::endl;

    Quantizer quantizer(8, 8, 0, Quantizer::TRUNK);
    auto vec = quantizer.to_quantized_int(img_tensor.data(), img_tensor.size());

    for (auto const& value : vec)
        std::cout << value << std::endl;*/

    /*t_3d = t_3d.unaryExpr([](float c) { return c / 255.0f; });
    
    float* fp = t_3d.data();
    for (int i = 0; i < 12; ++i)
        std::cout << fp[i] << std::endl;

    std::cout << t_3d << std::endl;*/


    /*std::cout << t_3d(0, 0, 0) << std::endl;
    std::cout << t_3d(1, 1, 1) << std::endl;
    std::cout << t_3d(1, 0, 1) << std::endl;*/

    
    //auto test = t_3d.shuffle(Eigen::array<int, 3>({1, 2, 0}));
    //std::cout << test << std::endl;

}



