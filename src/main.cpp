#include <iostream>
#include <nlohmann/json.hpp>
#include <fstream>
#include <chrono>
#include <fstream>
#include <filesystem>


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
#include "../include/chunk_utils.h"
#include "../include/dm_utils.h"
#include <unsupported/Eigen/CXX11/Tensor>



using MatrixXf = Eigen::MatrixXf;
using json = nlohmann::json;
using string = std::string;

void test();
void img_to_data(std::string img_path, std::string out_path, std::string template_path);

int main(int argc, char* argv[])
{
    
    string file_path = __FILE__;
    #ifdef _WIN32
        string dir_path = file_path.substr(0, file_path.rfind("\\"));
    #else
        string dir_path = file_path.substr(0, file_path.rfind("/"));
    #endif

    AsipCtrl asip_ctrl;
    Gpio gpio;
    ReservedMemory reserved_mem;
    /*asip_ctrl.test();
    asip_ctrl.write_test();
    gpio.write_test();
    reserved_mem.write_test();
    std::cout << "FINISH" << std::endl;
    return 0;*/

    
    string json_path = dir_path + "/../data/config.json";
    string glob_path = dir_path + "/../data/configs/conv2d.glob";
    string image_path = dir_path + "/../data/imgs/20200308_170823.jpg";
    string out_path = dir_path + "/../data/img_desk.dat";
    string template_path = dir_path + "/../data/conv2d_template.dat";
    string data_path = dir_path + "/../input.dat";
    int vsize = 4;
    int port = 8485;
    string ip = "127.0.0.1";
    bool network = false;
    
    /* ----- Test Glob Select --------- */
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    ChunkContainer chunk_container;
    chunk_container.init_chunk(json_path);
    chunk_container.read_data_from_glob(glob_path, true, 0, 36);
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "Read Glob" << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "ms" << std::endl;
    bool is_complete = chunk_container.is_complete();

    if(is_complete)
        std::cout << "Glob Complete: True" << std::endl;
    else
        std::cout << "Glob Complete: False" << std::endl;

    chunk_container.write_data_on_addr(reserved_mem.get_addr());

    //test();
    //img_to_data(image_path, out_path, template_path);
   
    cxxopts::Options options("decoder", "post processing ssd");
    options.positional_help("[optional args]").show_positional_help();
    options
        .allow_unrecognised_options()
        .add_options()
        ("h, help", "Print help")
        ("t, test", "Start decoder test with example image and data")
        ("c, cam", "Test cam")
        ("d, data", string("data input file for test data. Default: ").append(data_path), cxxopts::value<string>())
        ("v, verbose", "Verbose \"info()\" output")
        ("host", string("IP, stream over network"), cxxopts::value<string>())
        ("p, port", string("Port. Default: ").append(std::to_string(port)), cxxopts::value<int>())
        ("i, image", string("Path to image for test mode. Default: ").append(image_path), cxxopts::value<string>());
        ("vsize", string("VSIZE. Default: ").append(std::to_string(vsize)), cxxopts::value<int>());

    auto result = options.parse(argc, argv);


    if(result.count("host")) {
        std::cout << "network mode" << std::endl;
        network = true;
        ip = result["host"].as<string>();
        if(result.count("p")) 
            port = result["p"].as<int>();
    }

    if (result.count("vsize"))
        vsize = result["vsize"].as<int>();


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
    else {
        CoreApp app = CoreApp();
        app.open_socket(ip, port);
        app.start_app(vsize);
        //app.start_decoder_test(image_path, data_path, network);
    }

    if (result.count("c")) {
        std::cout << "*** Camera Test  ***" << std::endl;
        CoreApp app = CoreApp();
        if (network)
            app.open_socket(ip, port);

        app.start_camera_test(network);
    }
}

string type2str(int type) {
    string r;

    uchar depth = type & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (type >> CV_CN_SHIFT);

    switch (depth) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
    }

    r += "C";
    r += (chans + '0');

    return r;
}

void img_to_data(std::string img_path, std::string out_path, std::string template_path) {
    cv::Mat cv_img = cv::imread(img_path, cv::IMREAD_COLOR);
    cv::resize(cv_img, cv_img, cv::Size(300, 300));
    cv_img.convertTo(cv_img, CV_32FC3);
    string r = type2str(cv_img.type());
    Quantizer quant(5, 11);
    quant.normalize_c3(cv_img.ptr<float>(), 300 * 300 * 3, { 0.229, 0.224, 0.225 }, { 0.485, 0.456, 0.406 });
    quant.to_quantized_int(cv_img.ptr<float>(), 300 * 300 * 3);

    
    std::filesystem::copy(template_path, out_path, std::filesystem::copy_options::overwrite_existing);

    std::ofstream outfile(out_path, std::ios_base::app | std::ios_base::out);
    std::vector<cv::Mat> split_channel;
    cv::split(cv_img, split_channel);
    for (int y = 0; y < 300; ++y) {
        for (int c = 2; c >= 0; --c) {
            for (int x = 0; x < 300; ++x) {
                //std::cout << split_channel[c].at<float>(y, x) << " ";            
                outfile << "0x" << std::hex << (short)split_channel[c].at<float>(y, x) << "\n";
            }
        }
    }
    outfile.close();
}

void test() {
    int height = 2;
    int width = 2;

    float norm_mean[] = { 0.485f, 0.456f, 0.406f };
    float norm_std[] = { 0.229f, 0.224f, 0.225f };

    float data_t1[24] = { 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 0.10, 0.11, 0.12, 0.10, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70, 0.80, 0.90, 0.100, 0.110, 0.120 };
    float data_t2[24] = { 100, 200, 300, 400, 500, 600, 700, 800, 900, 1000, 1100, 1200, 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000, 9000, 10000, 11000, 12000 };

    std::cout << std::setfill('0') << std::setw(2) << std::right << std::hex << -10 << std::endl;
    std::cout << std::hex << (short)-17 << std::endl;
    std::cout << std::hex << 10 << std::endl;
    std::cout << std::hex << 17 << std::endl;
    
    Quantizer quant(5, 11);
    cv::Mat cv_img = cv::Mat(2, 4, CV_32FC3, data_t1);
    quant.to_quantized_int(cv_img.ptr<float>(), 24);

    std::vector<cv::Mat> split_channel;
    cv::split(cv_img, split_channel);
    for (int y = 0; y < 2; ++y) {
        for (int c = 2; c >= 0; --c) {
            for (int x = 0; x < 4; ++x) {
                std::cout << split_channel[c].at<float>(y, x) << " ";
            }
        }
    }
    std::cout << cv::format(cv_img, cv::Formatter::FMT_PYTHON) << std::endl << std::endl;

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

}



