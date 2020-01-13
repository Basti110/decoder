#include <iostream>
#include <nlohmann/json.hpp>
#include <fstream>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <cxxopts/cxxopts.hpp>
#include "../include/decoder.h"
#include "../include/utils.h"
#include "../include/Test.h"

using MatrixXf = Eigen::MatrixXf;
using json = nlohmann::json;
using string = std::string;

bool test(const string& image_path, const string& json_path);
void cam();

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
        ("i, image", string("Path to image for test mode. Default: ").append(image_path), cxxopts::value<string>());

    auto result = options.parse(argc, argv);

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
        test(image_path, json_path);
        exit(0);
    }
    cam();
}

bool test(const string& image_path, const string& json_path) {
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

    if (image.data) {
        cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE);
        imshow("Display window", image);
        cv::waitKey(0);
    }
    return true;
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



