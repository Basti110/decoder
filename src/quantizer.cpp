#include <math.h>
#include <algorithm>
#include "../include/quantizer.h"
#include <iostream>


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

// Normalize data for 3 channel. STD and MEAN in RGB format. 
// Input: [0, 255] => Transform: [0, 1] 
// Output: (Transform[channel] - mean[channel]) / std[channel] 
void Quantizer::normalize_c3(float* data, int size, std::vector<float> std, std::vector<float> mean)
{
    if (std.size() != 3)
        return;

    if (mean.size() != 3)
        return;

    if (size % 3 != 0)
        return;

    for (int i = 0; i < size; i += 3) {
        //std::cout << data[i] << " " << data[i + 1] << " " << data[i + 2] << std::endl;
        data[i] = (data[i] / 255.0f - mean[2]) / std[2];
        data[i + 1] = (data[i + 1] / 255.0f - mean[1]) / std[1];
        data[i + 2] = (data[i + 2] / 255.0f - mean[0]) / std[0];
        //std::cout << data[i] << " " << data[i + 1] << " " << data[i + 2] << std::endl << std::endl;
    }
}

void Quantizer::to_quantized_int(float* data, int size)
{
    for (int i = 0; i < size; ++i) {
        int scaled_value;
        if(mRndMode == TRUNK) 
            scaled_value = (int)floor(data[i] * mScaleFactor);
        else
            scaled_value = (int)round(data[i] * mScaleFactor);

        data[i] = (float)std::max(std::min(scaled_value, mMaxVal), mMinVal);
    }
    return;
}

void Quantizer::unquant(float* data, int size)
{
    for (int i = 0; i < size; ++i) {
        data[i] = data[i] / mScaleFactor;
    }
    return;
}
