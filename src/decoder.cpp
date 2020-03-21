#include <numeric>
#include <math.h>
#include <chrono>
#include "../include/decoder.h"
#include "../include/utils.h"

bool Decoder::init_default_boxes300()
{
    VectorXi feat_size(6);
    feat_size << 38, 19, 10, 5, 3, 1;

    VectorXi steps(6);
    steps << 8, 16, 32, 64, 100, 300;

    VectorXi scales(7);
    scales << 21, 45, 99, 153, 207, 261, 315;

    DefaultBoxSetting settings { 
        300,
        feat_size,
        steps,
        scales,
        {{2},{2, 3},{2, 3},{2, 3},{2},{2}},
        0.1f,
        0.2f
    };

    return init_default_boxes(settings);
}

// Input DefaultBoxSetting
//  int img_size
//  VectorXi feat_size
//  VectorXi steps
//  VectorXi scales
//  vector<vector<int>> aspect_ratios
//  float scale_xy
//  float scale_wh 
bool Decoder::init_default_boxes(const DefaultBoxSetting& s)
{
    mSettings = s;
    VectorXf fk = s.steps.cast<float>().cwiseInverse() * s.img_size;
    
    vector<vector<float>> dboxes;
    vector<vector<float>> dboxes_ltrb;

    for (int i = 0; i < s.feat_size.size(); ++i) {
        float sk1 = (float)(s.scales[i]) / s.img_size;
        float sk2 = (float)(s.scales[i + 1]) / s.img_size;
        float sk3 = std::sqrt(sk1 * sk2);
        vector<std::pair<float, float>> all_sizes{ { sk1, sk1 }, { sk3, sk3 } };

        for (int alpha : s.aspect_ratios[i]) {
            float w = sk1 * std::sqrt(alpha);
            float h = sk1 / std::sqrt(alpha);
            all_sizes.push_back({ w, h });
            all_sizes.push_back({ h, w });
        }

        for (auto size : all_sizes) {
            for (int y = 0; y < s.feat_size[i]; ++y) {
                for (int x = 0; x < s.feat_size[i]; ++x) {
                    float cx = ((float)x + 0.5f) / fk[i];
                    float cy = ((float)y + 0.5f) / fk[i];
                    /*cx = std::max(0.0f, cx);
                    cx = std::min(1.0f, cx);
                    cy = std::max(0.0f, cy);
                    cy = std::min(1.0f, cy);*/
                    float l = cx - 0.5f * size.first;
                    float t = cy - 0.5f * size.second;
                    float r = cx + 0.5f * size.first;
                    float b = cy + 0.5f * size.second;
                    dboxes.push_back(vector<float>{cx, cy, size.first, size.second});
                    dboxes_ltrb.push_back(vector<float>{l, t, r, b});
                }
            }
        }
    }
    
    mDBoxes = utils::vector2d_to_eigenmatrix(dboxes, dboxes.size(), 4);
    mDBoxes_ltrb = utils::vector2d_to_eigenmatrix(dboxes_ltrb, dboxes_ltrb.size(), 4);
    mInit = true;
    return mInit;
}

vector<BoxLabel> Decoder::listdecode_batch(MatrixXf& boxes_in, MatrixXf& scores_in, float criteria, float max_output)
{
    const float treshold = 0.5;
    const float max_elements = 200;

    auto start = std::chrono::high_resolution_clock::now();
    scale_back_batch(boxes_in, scores_in);
    auto finish = std::chrono::high_resolution_clock::now();
    LOG_INFO("Time scale_back_batch: %ims", std::chrono::duration_cast<std::chrono::milliseconds>(finish - start).count());
    
    start = std::chrono::high_resolution_clock::now();
    vector<BoxLabel> output;

    for (int i = 1; i < scores_in.cols(); ++i) {
        vector<float> scores;
        MatrixXf boxes(0, boxes_in.cols());
        float score_count = 0;
        for (int j = 0; j < scores_in.rows(); ++j) {
            if (scores_in(j, i) > treshold) {
                scores.push_back(scores_in(j, i));
                boxes.conservativeResize(boxes.rows() + 1, boxes.cols());
                boxes.row(score_count) = boxes_in.row(j);
                score_count++;
            }
        }

        if (scores.size() == 0)
            continue;

        vector<int> score_idx_sorted(scores.size());
        std::iota(score_idx_sorted.begin(), score_idx_sorted.end(), 0);
        std::sort(score_idx_sorted.begin(), score_idx_sorted.end(), [&scores](int i1, int i2) {return scores[i1] < scores[i2]; });
        score_idx_sorted.resize(std::min<int>(max_elements, scores.size()));

        while (score_idx_sorted.size() > 0) {
            int idx = score_idx_sorted[score_idx_sorted.size() - 1];
            MatrixXf bboxes_sorted = utils::get_rows_from_idx_vec(boxes, score_idx_sorted);
            MatrixXf bboxes_idx = boxes.row(idx);
            vector<float> iou = calc_iou_tensor(bboxes_sorted, bboxes_idx);
            utils::remove_idx_over_criteria_in_second_vec(score_idx_sorted, iou, criteria);

            BoxLabel label;
            std::copy(bboxes_idx.data(), bboxes_idx.data() + bboxes_idx.size(), label.coordinates);
            label.label_int = i;
            label.score = scores[idx];
            output.push_back(label);
        }
    }
    finish = std::chrono::high_resolution_clock::now();
    LOG_INFO("Time compute boxes: %ims", std::chrono::duration_cast<std::chrono::milliseconds>(finish - start).count());
 	return output;
}

/*static inline float FastExpSse(float f)
{
    float e;
    __m128 x = _mm_set1_ps(f);
    __m128 a = _mm_set1_ps(12102203.2f); // (1 << 23) / ln(2)
    __m128i b = _mm_set1_epi32(127 * (1 << 23) - 486411);
    __m128  m87 = _mm_set1_ps(-87);
    // fast exponential function, x should be in [-87, 87]
    __m128 mask = _mm_cmpge_ps(x, m87);

    __m128i tmp = _mm_add_epi32(_mm_cvtps_epi32(_mm_mul_ps(a, x)), b);
    _mm_store_ps(&e, _mm_and_ps(_mm_castsi128_ps(tmp), mask));
    return e;
}*/

inline double exp1(double x) {
    x = 1.0 + x / 256.0;
    x *= x; x *= x; x *= x; x *= x;
    x *= x; x *= x; x *= x; x *= x;
    return x;
}

void Decoder::scale_back_batch(MatrixXf& boxes_in, MatrixXf& scores_in)
{
    for (int i = 0; i < boxes_in.rows(); ++i) {
        boxes_in.row(i)[0] *= mSettings.scale_xy;
        boxes_in.row(i)[1] *= mSettings.scale_xy;
        boxes_in.row(i)[2] *= mSettings.scale_wh;
        boxes_in.row(i)[3] *= mSettings.scale_wh;

        boxes_in.row(i)[0] = boxes_in.row(i)[0] * mDBoxes.row(i)[2] + mDBoxes.row(i)[0];
        boxes_in.row(i)[1] = boxes_in.row(i)[1] * mDBoxes.row(i)[3] + mDBoxes.row(i)[1];
        boxes_in.row(i)[2] = std::exp((float)boxes_in.row(i)[2]) * mDBoxes.row(i)[2];
        boxes_in.row(i)[3] = std::exp((float)boxes_in.row(i)[3]) * mDBoxes.row(i)[3];


        float l = boxes_in.row(i)[0] - 0.5f * boxes_in.row(i)[2];
        float t = boxes_in.row(i)[1] - 0.5f * boxes_in.row(i)[3];
        float r = boxes_in.row(i)[0] + 0.5f * boxes_in.row(i)[2];
        float b = boxes_in.row(i)[1] + 0.5f * boxes_in.row(i)[3];

        boxes_in.row(i)[0] = l;
        boxes_in.row(i)[1] = t;
        boxes_in.row(i)[2] = r;
        boxes_in.row(i)[3] = b;
    }

    auto start = std::chrono::high_resolution_clock::now();
    //Softmax 5ms 
    /*scores_in = scores_in.unaryExpr([](float c) 
        { 
            float e;
            __m128 t = _mm_set1_ps(c);
            t = FastExpSse(t);
            _mm_store_ps(&e, t);
            return e;
        });*/

    //Softmax 28ms
    /*for (int i = 0; i < scores_in.rows(); ++i) {
        scores_in.row(i) = scores_in.row(i).unaryExpr([](float c){ return std::exp(c); });
        float sum = scores_in.row(i).sum();
        scores_in.row(i) = scores_in.row(i).unaryExpr([sum](float c) { return c / sum; });
    }*/

    //New Softmax 2ms
    for (int i = 0; i < scores_in.rows(); ++i) {
        float sum = 0.0f;

        for (int j = 0; j < scores_in.cols(); ++j) {
            scores_in(i, j) = exp1(scores_in(i, j)); // std::exp(scores_in(i, j));
            sum += scores_in(i, j);
        }

        for (int j = 0; j < scores_in.cols(); ++j)
            scores_in(i, j) /= sum;
    }

    auto finish = std::chrono::high_resolution_clock::now();
    LOG_INFO("Time softmax: %ims", std::chrono::duration_cast<std::chrono::milliseconds>(finish - start).count());
}

//input:
//box1(N, 4)
//box2(M, 4)
//output:
//IoU(N, M)
vector<float> Decoder::calc_iou_tensor(MatrixXf& box1, MatrixXf& idx_box)
{
    if (idx_box.cols() != 4 && idx_box.rows() != 1 && box1.cols() != 4)
        LOG_ERROR("Wrong matrix size");
    
    MatrixXf box2(box1.rows(), 4);
    box2.rowwise() = idx_box.row(0);

    MatrixXf lt = box1.leftCols(2).cwiseMax(box2.leftCols(2));
    MatrixXf rb = box1.rightCols(2).cwiseMin(box2.rightCols(2));

    MatrixXf delta = rb - lt;
    delta = delta.cwiseMax(0);

    MatrixXf intersect = delta.col(0).cwiseProduct(delta.col(1));

    MatrixXf delta1 = box1.rightCols(2) - box1.leftCols(2);
    MatrixXf delta2 = box2.rightCols(2) - box2.leftCols(2);
    MatrixXf area1 = delta1.col(0).cwiseProduct(delta1.col(1));
    MatrixXf area2 = delta2.col(0).cwiseProduct(delta2.col(1));
    
    MatrixXf iou = intersect.cwiseProduct((area1 + area2 - intersect).cwiseInverse());  
    return vector<float>(iou.data(), iou.data() + iou.size());
}
