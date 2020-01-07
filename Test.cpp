#include "Test.h"
#include "decoder.h"
#include "utils.h"

void Test::run_test_utils()
{
    //Test remove idx over criteria
    {
        std::vector<int> vec({ 1, 2, 3, 4, 5 });
        vector<float> iou({ 0.7f, 0.3f, 0.5f, 0.6f, 0.1f });
        vector<int> result({ 2, 3, 5 });
        utils::remove_idx_over_criteria_in_second_vec(vec, iou, 0.5f);
        LOG_ERROR_IF(result != vec, "Test 1 failed");
    }

    //Test get_rows_from_idx_vec
    {
        MatrixXf m1(6, 4);
        m1 << 1, 1, 1, 1,
            2, 2, 2, 2,
            3, 3, 3, 3,
            4, 4, 4, 4,
            5, 5, 5, 5,
            6, 6, 6, 6;

        MatrixXf result(3, 4);
        result << 1, 1, 1, 1,
                4, 4, 4, 4,
                6, 6, 6, 6;

        std::vector<int> idx({ 0, 3, 5 });
        MatrixXf m2 = utils::get_rows_from_idx_vec(m1, idx);
        if (!result.isApprox(m2)) {
            std::cout << m2 << std::endl << std::endl;
            std::cout << result << std::endl << std::endl;
            LOG_ERROR("Test 2 failed");
        }
    }

    //Test softmax
    {
        MatrixXf m1(4, 4);
        m1 << 1, 1, 1, 1,
            2, 2, 2, 2,
            3, 3, 3, 3,
            4, 4, 4, 4;

        utils::softmax_col(m1);
        std::cout << m1 << std::endl << std::endl;
    }
}

void Test::run_test_decoder()
{
    Decoder decoder;
    //decoder.init_default_boxes300();

    {
        VectorXi feat_size(2);
        feat_size << 4, 2;

        VectorXi steps(2);
        steps << 25, 50;

        VectorXi scales(3);
        scales << 50, 75, 100;

        DefaultBoxSetting settings{
            100,
            feat_size,
            steps,
            scales,
            {{}, {}},
            0.1f,
            0.2f
        };

        decoder.init_default_boxes(settings);
    }

    MatrixXf m1(7, 4);
    m1 << 1, 3, 2, 4,
        2, 5, 5, 7,
        4, 2, 6, 4,
        5, 5, 7, 7,
        5, 1, 7, 4,
        8, 5, 10, 7,
        4, 3, 6, 6;
    std::cout << m1 << std::endl << std::endl;

    MatrixXf m2(1, 4);
    m2 << 4, 3, 6, 6;
    std::cout << m2 << std::endl << std::endl;

    std::vector<int> vec({ 1, 2, 3, 4, 5, 6 });
    vector<float> iou = decoder.calc_iou_tensor(m1, m2);
    utils::remove_idx_over_criteria_in_second_vec(vec, iou, 0.1f);
}

/*Test::Test()
{
    run_test_utils();
    run_test_decoder();
}*/
