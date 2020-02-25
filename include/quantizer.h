#pragma once
#include <cstdint>
#include <string>
#include <vector>

class Quantizer
{
public:
    enum RoundMode { TRUNK, NE };

public:
    Quantizer(int intbits = 8, int fracbits = 8, int gatebits = 0, RoundMode rndmode = TRUNK);
    std::vector<int16_t> to_quantized_int(float* data, int size);

private:
    int mIntBits;
    int mFracBits;
    int mGateBits;
    int mMaxVal;
    int mMinVal;
    int mScaleFactor;
    RoundMode mRndMode;
};

