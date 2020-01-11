#pragma once
class Test
{

public:
    Test() {
        run_test_utils();
        run_test_decoder();
    }

private:
    void run_test_utils();
    void run_test_decoder();
};