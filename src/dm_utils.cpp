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

void AsipCtrl::set_finish()
{
    uint32_t v = mBasePtr[0] & mFinishMask;
    mBasePtr[0] = v ^ (((uint32_t)1) << 17);
}

bool AsipCtrl::read_finish()
{
#ifdef _WIN32
    return false;
#else
    uint32_t int_info;
    int read_size;
    read_size = read(mFileDescriptor, &int_info, sizeof(int_info));
    LOG_ERROR_IF_RETURN_FALSE(read_size < 0, "Can not read Interupt");
    std::cout << "GOT " << int_info << " interupts" << std::endl;
    return false;
#endif
}

bool AsipCtrl::write_finish()
{
#ifdef _WIN32
    return false;
#else
    uint32_t int_info = 1;
    int read_size;
    read_size = write(mFileDescriptor, &int_info, sizeof(int_info));
    LOG_ERROR_IF_RETURN_FALSE(read_size < 0, "Can not write Interupt");
    std::cout << "Wrote " << int_info << " in interupt" << std::endl;
    return false;
#endif
}

void AsipCtrl::clear_start() {
    uint32_t v = mBasePtr[0] & mStartMask; 
    mBasePtr[0] = v; //(((uint32_t)1) << 16);
}

void AsipCtrl::clear_wait() {
    uint32_t v = mBasePtr[0] & mWaitMask;
    mBasePtr[0] = v; // (((uint32_t)1) << 18);
}

void AsipCtrl::clear_reset() {
    uint32_t v = mBasePtr[0] & mResetMask;
    mBasePtr[0] = v; //(((uint32_t)1) << 19);
}

void AsipCtrl::clear_finish()
{
    uint32_t v = mBasePtr[0] & mFinishMask;
    mBasePtr[0] = v; //(((uint32_t)1) << 19);
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

uint32_t AsipCtrl::read_gpio()
{
    return mGpio.get_input();
}

void AsipCtrl::test()
{
    std::cout << "Write Finish" << std::endl;
    set_finish();
    std::cout << "Read Finish" << std::endl;
    read_finish();

    uint8_t command = 0x34;
    uint8_t state = 0x12;
    mBasePtr[0] = mBasePtr[0] & 0xF0000;
    std::cout << "Register 1: " << std::hex << read_register1() << std::endl;
    set_command(command);
    std::cout << "--- set command ---" << std::endl;
    std::cout << "Register 1: " << std::hex << read_register1() << std::endl;
    std::cout << "    GPIO 1: " << std::hex << read_gpio() << std::endl;
    set_state(state);
    std::cout << "--- set state -----" << std::endl;
    std::cout << "Register 1: " << std::hex << read_register1() << std::endl;
    std::cout << "    GPIO 1: " << std::hex << read_gpio() << std::endl;
    set_start();
    std::cout << "--- set start -----" << std::endl;
    std::cout << "Register 1: " << std::hex << read_register1() << std::endl;
    std::cout << "    GPIO 1: " << std::hex << read_gpio() << std::endl;
    set_finish();
    std::cout << "--- set finish ----" << std::endl;
    std::cout << "Register 1: " << std::hex << read_register1() << std::endl;
    std::cout << "    GPIO 1: " << std::hex << read_gpio() << std::endl;
    set_wait();
    std::cout << "--- set wait  -----" << std::endl;
    std::cout << "Register 1: " << std::hex << read_register1() << std::endl;
    std::cout << "    GPIO 1: " << std::hex << read_gpio() << std::endl;
    set_reset();
    std::cout << "--- set reset  -----" << std::endl;
    std::cout << "Register 1: " << std::hex << read_register1() << std::endl;
    std::cout << "    GPIO 1: " << std::hex << read_gpio() << std::endl;
    clear_start();
    std::cout << "--- clear start -----" << std::endl;
    std::cout << "Register 1: " << std::hex << read_register1() << std::endl;
    std::cout << "    GPIO 1: " << std::hex << read_gpio() << std::endl;
    clear_finish();
    std::cout << "--- clear finish-----" << std::endl;
    std::cout << "Register 1: " << std::hex << read_register1() << std::endl;
    std::cout << "    GPIO 1: " << std::hex << read_gpio() << std::endl;
    clear_wait();
    std::cout << "--- clear  wait -----" << std::endl;
    std::cout << "Register 1: " << std::hex << read_register1() << std::endl;
    std::cout << "    GPIO 1: " << std::hex << read_gpio() << std::endl;
    clear_reset();
    std::cout << "--- clear reset-----" << std::endl;
    std::cout << "Register 1: " << std::hex << read_register1() << std::endl;
    std::cout << "    GPIO 1: " << std::hex << read_gpio() << std::endl;

}

void AsipCtrl::set_param1(uint32_t value)
{
}

void AsipCtrl::set_param2(uint32_t value)
{
}

void AsipCtrl::set_param3(uint32_t value)
{
}

Gpio::Gpio() : DeviceMapper("/dev/uio1", 2)
{
    mBasePtr[1] = 0xffffffff;
}

uint32_t Gpio::get_input()
{
    return mBasePtr[0];
}

ReservedMemory::ReservedMemory() : DeviceMapper("/dev/uio2", 0x40000000)
{
}
