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

class Gpio : public DeviceMapper {
public:
    Gpio();
    uint32_t get_input();
};

class AsipCtrl : public DeviceMapper {
public:
    AsipCtrl();
    void set_command(uint8_t byte);
    void set_state(uint8_t byte);
    void set_start();
    void set_finish();
    void set_wait();
    void set_reset();
    void clear_start();
    void clear_wait();
    void clear_reset();
    void clear_finish();
    bool read_interrupt();
    bool ack_interrupt();
    uint32_t read_register1();
    uint32_t read_gpio();
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
    Gpio mGpio;
    /*const uint32_t mCommandMask = 0x000000FF;
    const uint32_t mStateMask = 0x0000FF00;
    const uint32_t mStartMask = 0x00010000;
    const uint32_t mFinishMask = 0x00020000;
    const uint32_t mWaitMask = 0x00040000;
    const uint32_t mResetMask = 0x00080000;*/
};

class ReservedMemory : public DeviceMapper {
public:
    ReservedMemory();
};
