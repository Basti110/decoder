#pragma once
#include <stdint.h>
#include <string>

class DeviceMapper {
public:
    DeviceMapper() {};
    DeviceMapper(std::string path, int size);
    bool map_device(std::string path, int size);
    bool write_test();

protected:
    std::string mDevicePath = "";
    int mFileDescriptor = -1;
    uint32_t mSize = 0;
    uint32_t* mBasePtr = nullptr;
};

class AsipCtrl : public DeviceMapper {
public:
    AsipCtrl();
    void set_command(uint8_t byte);
    void set_state(uint8_t byte);
    void set_start();
    bool read_finish();
    void set_wait();
    void set_reset();
    uint32_t read_register1();
    void test();
    
    void set_param1(uint32_t value);
    void set_param2(uint32_t value);
    void set_param3(uint32_t value);

private:
    const uint32_t mCommandMask = 0x0000FF00;
    const uint32_t mStateMask = 0x000000FF;
    const uint32_t mStartMask = 0x0001FFFF;
    const uint32_t mFinishMask = 0x0002FFFF;
    const uint32_t mWaitMask = 0x0004FFFF;
    const uint32_t mResetMask = 0x0008FFFF;
    /*const uint32_t mCommandMask = 0x000000FF;
    const uint32_t mStateMask = 0x0000FF00;
    const uint32_t mStartMask = 0x00010000;
    const uint32_t mFinishMask = 0x00020000;
    const uint32_t mWaitMask = 0x00040000;
    const uint32_t mResetMask = 0x00080000;*/
};

class Gpio : public DeviceMapper {
public:
    Gpio();
};

class ReservedMemory : public DeviceMapper {
public:
    ReservedMemory();
};
