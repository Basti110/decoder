#pragma once
#include <cstdint>
#include <string>
#include <vector>

class Quantizer
{
public:
    enum RoundMode { TRUNK, NE };

public:
    Quantizer(int intbits = 5, int fracbits = 11, int gatebits = 0, RoundMode rndmode = TRUNK);
    void to_quantized_int(float* data, int size);
    void unquant(float* data, int size);
    void normalize_c3(float* data, int size, std::vector<float> std, std::vector<float> mean);

private:
    int mIntBits;
    int mFracBits;
    int mGateBits;
    int mMaxVal;
    int mMinVal;
    int mScaleFactor;
    RoundMode mRndMode;
};

