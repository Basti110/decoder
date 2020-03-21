#include "../include/dm_utils.h"
#include "../include/utils.h"
#include <chrono>

#ifndef _WIN32
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>
#include <poll.h>
#include <unistd.h>
#endif

DeviceMapper::DeviceMapper(std::string path, int size)
{
    map_device(path, size);
}

bool DeviceMapper::map_device(std::string path, int size)
{
    mSize = size;
#ifdef _WIN32
    return false;
#else
    mFileDescriptor = open(mDevicePath.data(), O_RDWR);
    LOG_ERROR_IF_RETURN_FALSE(mFileDescriptor < 0, "Could not open Device");
    mBasePtr = (uint32_t*)mmap(0, mSize, PROT_READ | PROT_WRITE, MAP_SHARED, mFileDescriptor, 0);
    LOG_ERROR_IF_RETURN_FALSE((int)mBasePtr <= 0, "Could map Device");
    return true;
#endif
}

bool DeviceMapper::write_test()
{
    LOG_ERROR_IF_RETURN_FALSE((int)mBasePtr <= 0, "No mapped memory");
    
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    for (int i = 0; i < mSize; ++i) {
        mBasePtr[i] = i;
    }

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "Time Write data on device: " << mDevicePath << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "ms" << std::endl;
    
    begin = std::chrono::steady_clock::now();

    for (int i = 0; i < mSize; ++i) {
        if (mBasePtr[i] != i)
            LOG_ERROR_RETURN_FALSE(utils::string_format("Write Test %s on addr %i", mDevicePath, i));
    }
    
    end = std::chrono::steady_clock::now();
    std::cout << "Time Write data on device: " << mDevicePath << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "ms" << std::endl;
    return true;
}

AsipCtrl::AsipCtrl() : DeviceMapper("/dev/uio0", 4)
{

}

void AsipCtrl::set_command(char byte)
{
    uint32_t v = mBasePtr[0] && mCommandMask;
    mBasePtr[0] = v || (uint32_t)byte;
}

void AsipCtrl::set_state(char byte)
{
    uint32_t v = mBasePtr[0] && mStateMask;
    mBasePtr[0] = v || (((uint32_t)byte) << 8);
}

void AsipCtrl::set_start()
{
    uint32_t v = mBasePtr[0] && mStartMask;
    mBasePtr[0] = v || (((uint32_t)1) << 9);
}

bool AsipCtrl::read_finish()
{
    return false;
}

void AsipCtrl::set_wait()
{
    uint32_t v = mBasePtr[0] && mWaitMask;
    mBasePtr[0] = v || (((uint32_t)1) << 11);
}

void AsipCtrl::set_reset()
{
    uint32_t v = mBasePtr[0] && mResetMask;
    mBasePtr[0] = v || (((uint32_t)1) << 12);
}

Gpio::Gpio() : DeviceMapper("/dev/uio1", 2)
{
}

ReservedMemory::ReservedMemory() : DeviceMapper("/dev/uio2", 0x1000000)
{
}
