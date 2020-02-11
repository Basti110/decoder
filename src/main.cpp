#include <iostream>
#include <nlohmann/json.hpp>
#include <fstream>
#include <chrono>
#include <sys/socket.h> 
#include <arpa/inet.h>
#include <unistd.h>
#include <netinet/in.h>
#include <thread>
#include <opencv2/opencv.hpp>
#include <cxxopts/cxxopts.hpp>
#include "../include/decoder.h"
#include "../include/utils.h"
#include "../include/Test.h"

using MatrixXf = Eigen::MatrixXf;
using json = nlohmann::json;
using string = std::string;

cv::Mat test(const string& image_path, const string& json_path);
void cam();
void send_cam(const char* ip, int port);
void send_frame(const char* ip, int port, cv::Mat frame);

int main(int argc, char* argv[])
{
    string file_path = __FILE__;
    #ifdef _WIN32
        string dir_path = file_path.substr(0, file_path.rfind("\\"));
    #else
        string dir_path = file_path.substr(0, file_path.rfind("/"));
    #endif
    dir_path.append("/../data/");

    string json_path = string(dir_path).append("ssd_output.json");
    string image_path = string(dir_path).append("000000026564.jpg");

    cxxopts::Options options("decoder", "post processing ssd");
    options.positional_help("[optional args]").show_positional_help();
    options
        .allow_unrecognised_options()
        .add_options()
        ("h, help", "Print help")
        ("t, test", "Start decoder test with example image and data")
        ("j, json", string("Json input file for test data. Default: ").append(json_path), cxxopts::value<string>())
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
            json_path = result["j"].as<string>();
        cv::Mat frame = test(image_path, json_path);

        if (frame.data) {
            if(!network) {
                cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE);
                imshow("Display window", frame);
                cv::waitKey(0);
            }
            else 
                send_frame(ip.c_str(), port, frame);    
        }
        exit(0);
    }

    if(network) 
        send_cam(ip.c_str(), port);
    else
        cam();
}

cv::Mat test(const string& image_path, const string& json_path) {
    Test test;
    Decoder decoder;
    decoder.init_default_boxes300();

    json j;
    std::ifstream ifs;
    std::cout << json_path << std::endl;
    ifs.open(json_path, std::ifstream::in);
    ifs >> j;

    int box_size = j["box_size"].get<int>();
    int class_size = j["class_size"].get<int>();
    json locations_j = j["locations"];
    json classes_j = j["scores"];

    auto start = std::chrono::high_resolution_clock::now();
    std::vector<std::vector<float>> locations_2dv(4);
    std::vector<std::vector<float>> scores_2dv(class_size);

    for (int i = 0; i < 4; ++i) {
        locations_2dv[i] = locations_j[i].get<std::vector<float>>();
    }

    for (int i = 0; i < class_size; ++i) {
        scores_2dv[i] = classes_j[i].get<std::vector<float>>();
    }

    auto finish = std::chrono::high_resolution_clock::now();
    LOG_INFO("Time JSON to Vectors: %ims", std::chrono::duration_cast<std::chrono::milliseconds>(finish - start).count());

    start = std::chrono::high_resolution_clock::now();
    Eigen::MatrixXf locations = ::utils::vector2d_to_eigenmatrix(locations_2dv, 4, box_size);
    locations.transposeInPlace();

    Eigen::MatrixXf scores = ::utils::vector2d_to_eigenmatrix(scores_2dv, class_size, box_size);
    scores.transposeInPlace();

    finish = std::chrono::high_resolution_clock::now();
    LOG_INFO("Time Vectors to Eigen: %ims", std::chrono::duration_cast<std::chrono::milliseconds>(finish - start).count());

    start = std::chrono::high_resolution_clock::now();
    vector<BoxLabel> output = decoder.listdecode_batch(locations, scores);
    finish = std::chrono::high_resolution_clock::now();
    LOG_INFO("Complete Time decode batch: %ims", std::chrono::duration_cast<std::chrono::milliseconds>(finish - start).count());

    cv::Mat image;
    image = cv::imread(image_path, cv::IMREAD_COLOR);
    int width = image.cols;
    int height = image.rows;

    for (auto label : output) {
        auto p1 = cv::Point(label.coordinates[0] * width, label.coordinates[1] * height);
        auto p2 = cv::Point(label.coordinates[2] * width, label.coordinates[3] * height);
        rectangle(image, p1, p2, cv::Scalar(0, 0, 255), 2);
    }

    return image;
}

void cam()
{
    cv::VideoCapture cap;
    if (!cap.open(0))   
        return;
    cv::namedWindow("cam",1);
    for(;;)
    {
        cv::Mat frame;
        cap >> frame; // get a new frame from camera
        cv::imshow("cam", frame);
        if(cv::waitKey(30) >= 0) break;
    }
    return;
}

void send_cam(const char* ip, int port)
{
    std::cout << "send over network" << std::endl;
    int sokt;

    struct sockaddr_in serverAddr;
    socklen_t addrLen = sizeof(struct sockaddr_in);
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
        if ((bytes = send(sokt, &size_bytes, 4, 0)) < 0){
            std::cerr << "bytes = " << bytes << std::endl;
            break;
        }

        std::cout << "send: " << buf.size() << std::endl;
        if ((bytes = send(sokt, buf.data(), buf.size(), 0)) < 0){
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
    socklen_t addrLen = sizeof(struct sockaddr_in);
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
    if ((bytes = send(sokt, &size_bytes, 4, 0)) < 0){
        std::cerr << "bytes = " << bytes << std::endl;
        return;
    }

    std::cout << "send: " << buf.size() << std::endl;
    if ((bytes = send(sokt, buf.data(), buf.size(), 0)) < 0){
        std::cerr << "bytes = " << bytes << std::endl;
        return;
    }
}



