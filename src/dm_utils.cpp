#include "../include/dm_utils.h"
#include "../include/utils.h"
#include <chrono>

#ifndef _WIN32
#include <sys/stat.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>
#include <poll.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#endif

DeviceMapper::DeviceMapper(std::string path, int size)
{
    map_device(path, size);
}

bool DeviceMapper::map_device(std::string path, int size)
{
    mDevicePath = path;
    mSize = size;
#ifdef _WIN32
    return false;
#else
    std::cout << "try to open \"" << mDevicePath <<"\"" << std::endl;
    mFileDescriptor = open(mDevicePath.c_str(), O_RDWR);
    LOG_ERROR_IF_RETURN_FALSE(mFileDescriptor < 0, "Could not open Device");
    mBasePtr = (uint32_t*)mmap(0, mSize, PROT_READ | PROT_WRITE, MAP_SHARED, mFileDescriptor, 0);
    LOG_ERROR_IF_RETURN_FALSE((long)mBasePtr <= 0, "Could map Device");
    return true;
#endif
}

bool DeviceMapper::write_test()
{
    LOG_ERROR_IF_RETURN_FALSE((long)mBasePtr <= 0, "No mapped memory");
    
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    int size = (int) mSize / 4;
    for (int i = 0; i < size; ++i) {
        // std::cout << std::hex << i << std::endl;
        mBasePtr[i] = i;
    }

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "Time write data on device: " << mDevicePath << " in ";
    std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "ms" << std::endl;
    
    begin = std::chrono::steady_clock::now();

    for (int i = 0; i < size; ++i) {
        if (mBasePtr[i] != i)
            LOG_ERROR_RETURN_FALSE(utils::string_format("Write Test %s on addr %d", mDevicePath.c_str(), i));
    }
    
    end = std::chrono::steady_clock::now();
    std::cout << "Time read data of device: " << mDevicePath << " in ";
    std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "ms" << std::endl;
    return true;
}

AsipCtrl::AsipCtrl() : DeviceMapper("/dev/uio0", 4)
{

}

void AsipCtrl::set_command(uint8_t byte)
{
    uint32_t v = mBasePtr[0];
    v = v & mCommandMask;
    v = v | (uint32_t)byte;
    mBasePtr[0] = v;
}

void AsipCtrl::set_state(uint8_t byte)
{
    uint32_t v = mBasePtr[0] & mStateMask;
    v = v | (((uint32_t)byte) << 8);
    mBasePtr[0] = v;
}

void AsipCtrl::set_start()
{
    uint32_t v = mBasePtr[0] & mStartMask;
    mBasePtr[0] = v ^ (((uint32_t)1) << 16);
}

bool AsipCtrl::read_finish()
{
    return false;
}

void AsipCtrl::clear_start() {
    uint32_t v = mBasePtr[0] & mStartMask;
    mBasePtr[0] = (((uint32_t)1) << 16);
}

void AsipCtrl::clear_wait() {
    uint32_t v = mBasePtr[0] & mWaitMask;
    mBasePtr[0] = (((uint32_t)1) << 18);
}

void AsipCtrl::clear_reset() {
    uint32_t v = mBasePtr[0] & mResetMask;
    mBasePtr[0] = (((uint32_t)1) << 19);
}

void AsipCtrl::set_wait()
{
    uint32_t v = mBasePtr[0] & mWaitMask;
    mBasePtr[0] = v ^ (((uint32_t)1) << 18);
}

void AsipCtrl::set_reset()
{
    uint32_t v = mBasePtr[0] & mResetMask;
    mBasePtr[0] = v ^ (((uint32_t)1) << 19);
}

uint32_t AsipCtrl::read_register1()
{
    return mBasePtr[0];
}

void AsipCtrl::test()
{
    uint8_t command_t = 0x12;
    uint8_t state = 0x34;
    std::cout << "Register 1: " << std::hex << read_register1() << std::endl;
    set_command(command_t);
    clear_start();
    clear_wait();
    clear_reset();
    std::cout << "--- set command ---" << std::endl;
    std::cout << "Register 1: " << std::hex << read_register1() << std::endl;
    set_state(state);
    std::cout << "--- set state -----" << std::endl;
    std::cout << "Register 1: " << std::hex << read_register1() << std::endl;
    set_start();
    std::cout << "--- set start -----" << std::endl;
    std::cout << "Register 1: " << std::hex << read_register1() << std::endl;
    set_wait();
    std::cout << "--- set wait  -----" << std::endl;
    std::cout << "Register 1: " << std::hex << read_register1() << std::endl;   
    set_reset();
    std::cout << "--- set reset  -----" << std::endl;
    std::cout << "Register 1: " << std::hex << read_register1() << std::endl;
}

Gpio::Gpio() : DeviceMapper("/dev/uio1", 2)
{

}

ReservedMemory::ReservedMemory() : DeviceMapper("/dev/uio2", 0x40000000)
{
}
