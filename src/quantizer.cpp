#include <math.h>
#include <algorithm>
#include "../include/quantizer.h"


Quantizer::Quantizer(int intbits, int fracbits, int gatebits, RoundMode rndmode)
{
    mIntBits = intbits;
    mFracBits = fracbits;
    mGateBits = gatebits;
    mRndMode = rndmode;

    mMaxVal = pow(2, (mIntBits + mFracBits - 1)) - 1; //pow(2, mIntBits - 1) - 1 + 1 - pow(2, -mFracBits);
    mMinVal = -pow(2, (mIntBits + mFracBits - 1));    //-pow(2, mIntBits - 1);
    mScaleFactor = pow(2, mFracBits);
}

std::vector<int16_t> Quantizer::to_quantized_int(float* data, int size)
{
    std::vector<int16_t> quant_integer(size);
    for (int i = 0; i < size; ++i) {
        int scaled_value;
        if(mRndMode == TRUNK) 
            scaled_value = (int)floor(data[i] * mScaleFactor);
        else
            scaled_value = (int)round(data[i] * mScaleFactor);

        quant_integer[i] = (int16_t)std::max(std::min(scaled_value, mMaxVal), mMinVal);
    }
    return quant_integer;
}
