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
#include <unsupported/Eigen/CXX11/Tensor>



using MatrixXf = Eigen::MatrixXf;
using json = nlohmann::json;
using string = std::string;

cv::Mat test(const string& image_path, const string& json_path);
void test();
void cam();
void send_cam(const char* ip, int port);
void send_frame(const char* ip, int port, cv::Mat frame);

struct dim3 {
    int x;
    int y;
    int z;
};

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
    dir_path.append("/../data/");
    string json_path = dir_path;

    //string json_path = string(dir_path).append("ssd_output.json"); // ssd_output.json");
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

    test();
    
    /*if(network) 
        send_cam(ip.c_str(), port);
    else
        cam();*/
}

cv::Mat test(const string& image_path, const string& json_path) {
    int box_size = 8732; // j["box_size"].get<int>();
    int class_size = 48; // j["class_size"].get<int>();
    int layers = 6;

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

    std::vector<std::array<int, 3>> l_dims({
            {16, 38, 38},
            {24, 19, 19},
            {24, 10, 10},
            {24, 5, 5},
            {16, 3, 3},
            {16, 1, 1}
        });

    std::vector<std::array<int, 3>> c_dims({
        {192, 38, 38},
        {288, 19, 19},
        {288, 10, 10},
        {288, 5, 5},
        {192, 3, 3},
        {192, 1, 1}
    });

    int mem_size = l1_size + c1_size + l2_size + c2_size + l3_size + c3_size + l4_size + c4_size + l5_size + c5_size + l6_size + c6_size;
    int mem_pointer;

    std::vector<float> mem(mem_size);
    float value;
    int count = 0;
    string data_path = json_path;
    data_path.append("data.txt");
    std::ifstream infile(data_path);
    while (infile >> value) {
        mem[count] = value;
        count++;
    }

    std::vector<float> mem_l;
    std::vector<float> mem_c;
    //mem.insert(mem.begin(), fp, fp + 12);
    //memcpy(&mem[0], fp, 12 * sizeof(float));
    Test test;
    Decoder decoder;
    decoder.init_default_boxes300();

    //todo: speed
    auto start = std::chrono::high_resolution_clock::now();
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
    
    Eigen::MatrixXf locations = Eigen::Map<Eigen::MatrixXf>(mem_l.data(), 4, box_size);
    Eigen::MatrixXf scores = Eigen::Map<Eigen::MatrixXf>(mem_c.data(), class_size, box_size);

    locations.transposeInPlace();
    scores.transposeInPlace();

    auto finish = std::chrono::high_resolution_clock::now();
    LOG_INFO("Time shuffle: %ims", std::chrono::duration_cast<std::chrono::milliseconds>(finish - start).count());

    //Eigen::TensorMap<Eigen::Tensor<float, 3>> img_tensor((float*)data, 3, height, width);

    /*json j;
    std::ifstream ifs;
    string json_p = json_path;
    json_p.append("ssd_output.json");
    std::cout << json_p << std::endl;
    ifs.open(json_p, std::ifstream::in);
    ifs >> j;


    json locations_j = j["locations"];
    json classes_j = j["scores"];

    start = std::chrono::high_resolution_clock::now();
    std::vector<std::vector<float>> locations_2dv(4);
    std::vector<std::vector<float>> scores_2dv(class_size);

    for (int i = 0; i < 4; ++i) {
        locations_2dv[i] = locations_j[i].get<std::vector<float>>();
    }

    for (int i = 0; i < class_size; ++i) {
        scores_2dv[i] = classes_j[i].get<std::vector<float>>();
    }

    finish = std::chrono::high_resolution_clock::now();
    LOG_INFO("Time JSON to Vectors: %ims", std::chrono::duration_cast<std::chrono::milliseconds>(finish - start).count());

    start = std::chrono::high_resolution_clock::now();
    Eigen::MatrixXf locations2 = ::utils::vector2d_to_eigenmatrix(locations_2dv, 4, box_size);
    locations2.transposeInPlace();

    Eigen::MatrixXf scores2 = ::utils::vector2d_to_eigenmatrix(scores_2dv, class_size, box_size);
    scores2.transposeInPlace();

    finish = std::chrono::high_resolution_clock::now();
    LOG_INFO("Time Vectors to Eigen: %ims", std::chrono::duration_cast<std::chrono::milliseconds>(finish - start).count());

    for (int i = 0; i < locations.rows(); ++i) {
        for (int j = 0; j < locations.cols(); ++j) {
            float v1 = locations(i, j);
            float v2 = locations2(i, j);
            if (abs(v1 - v2) > 0.01)
                std::cout << i << ":" << j << std::endl;
        }
    }*/

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



