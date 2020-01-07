#include <iostream>
#include <nlohmann/json.hpp>
#include <fstream>
#include <chrono>
#include <opencv2/opencv.hpp>
#include "decoder.h"
#include "utils.h"
#include "Test.h"

using Eigen::MatrixXf;
using json = nlohmann::json;
using namespace cv;

int main()
{


    Test test;
    Decoder decoder;
    decoder.init_default_boxes300();

    /**/

    json j;
    std::ifstream ifs;
    ifs.open("C:/Users/Basti/Documents/git/pytorch-ssd/ssd_output.json", std::ifstream::in);
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
    std::cout << "Time JSON to Vectors: " << std::chrono::duration_cast<std::chrono::milliseconds>(finish - start).count() << "\n";

    start = std::chrono::high_resolution_clock::now();

    Eigen::MatrixXf locations = ::utils::vector2d_to_eigenmatrix(locations_2dv, 4, box_size);
    locations.transposeInPlace();
    
    Eigen::MatrixXf scores = ::utils::vector2d_to_eigenmatrix(scores_2dv, class_size, box_size);
    scores.transposeInPlace();

    finish = std::chrono::high_resolution_clock::now();
    std::cout << "Time Vectors to Eigen: " << std::chrono::duration_cast<std::chrono::milliseconds>(finish - start).count() << "\n";

    vector<BoxLabel> output = decoder.listdecode_batch(locations, scores);

    Mat image;
    image = imread(std::string("C:/Users/Basti/Documents/data/val2017/000000026564.jpg").c_str(), IMREAD_COLOR);
    int width = image.cols;
    int height = image.rows;

    for (auto label : output) {
        auto p1 = Point(label.coordinates[0] * width, label.coordinates[1] * height);
        auto p2 = Point(label.coordinates[2] * width, label.coordinates[3] * height);
        rectangle(image, p1, p2, Scalar(0, 0, 255), 2);
    }

    if (image.data) {
        namedWindow("Display window", WINDOW_AUTOSIZE);
        imshow("Display window", image);
        waitKey(0);
    }
}

