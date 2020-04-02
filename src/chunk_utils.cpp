#include "../include/chunk_utils.h"
#include "../include/utils.h"
#include <fstream>
#include <sstream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;
using string = std::string;

bool ChunkContainer::init_chunk(std::string path_json, int vsize)
{
    mVSize = vsize;
    json j;
    std::ifstream ifs;
    std::cout << path_json << std::endl;
    ifs.open(path_json, std::ifstream::in);
    ifs >> j;
    int max_chunks = j.size();
    mChunks.reserve(max_chunks);

    for (int i = 0; i < max_chunks; ++i) {
        json json_chunk = j[std::to_string(i)];
        Chunk chunk;
        int pooling_size = json_chunk["length_ofmaps_pool"].get<int>();
        int conv_size = json_chunk["length_ofmaps_conv"].get<int>();
        int len_ofmap = pooling_size == 0 ? conv_size : pooling_size;
        chunk.mConfigStruct.mStartAddr = json_chunk["addr_config_struct"].get<int>(); 
        chunk.mConfigStruct.mLenght = json_chunk["length_config_struct"].get<int>();
        chunk.mFilters.mStartAddr = json_chunk["addr_filters"].get<int>();
        chunk.mFilters.mLenght = json_chunk["addr_biases"].get<int>() + json_chunk["length_biases"].get<int>() - json_chunk["addr_filters"].get<int>();
        chunk.mOfMap.mStartAddr = chunk.mConfigStruct.mStartAddr + chunk.mConfigStruct.mLenght;
        chunk.mOfMap.mLenght = len_ofmap;
        chunk.mIfMap.mStartAddr = chunk.mFilters.mStartAddr + chunk.mFilters.mLenght;
        chunk.mIfMap.mLenght = json_chunk["length_ifmaps"].get<int>();
        chunk.mChunkNumber = i;
        mChunks.push_back(chunk);
    }
    return mIsInit = true;
}

bool ChunkContainer::read_data_from_glob(std::string path_glob, bool use_ofmap, int first_chunk, int last_chunk)
{
    if (last_chunk >= mChunks.size()) {
        LOG_ERROR("Chunks not in range");
        return false;
    }

    if(!mIsInit) {
        LOG_ERROR("First initialize this container");
        return false;
    }
        
    set_chunk_active(use_ofmap, first_chunk, last_chunk);

    std::ifstream glob_file;
    glob_file.open(path_glob, std::ios::in);
    if (!glob_file.is_open()) {
        LOG_ERROR("Failed to open " + path_glob);
        return false;
    }

    // 524.288 Bytes
    int* mem_ptr;
    int max_len = 8192 * 16;
    int mem_len = 0;
    mem_ptr = new int[max_len];
    

    string line;
    int last_addr = -1;
    int total_lines = 0;
    while (std::getline(glob_file, line))
    {      
        string data_line = line.substr(1);
        int sep = data_line.find(" ");
        string addr_str = data_line.substr(0, sep);
        string value_str = data_line.substr(sep + 1);
        int addr = (int)stoul(addr_str, 0, 16);
            
        if (mem_len >= max_len || (addr != last_addr + int(mem_len / mVSize))) {
            read_data_from_mem(mem_ptr, last_addr * mVSize, mem_len);
            last_addr = addr;
            mem_len = 0;
        }

        for (int i = 0; i < mVSize; i++) {
            string word_str = value_str.substr(i * 4, 4);
            int word_value = stoi(word_str, 0, 16);
            mem_ptr[mem_len + mVSize - (i + 1)] = word_value;
        }

        mem_len += mVSize;   
        total_lines++;
    }

    if (mem_len > 0)
        read_data_from_mem(mem_ptr, last_addr * mVSize, mem_len);

    return true;
}

void ChunkContainer::write_data_on_addr(uint16_t* addr)
{
    for (Chunk& chunk : mChunks) {
        chunk.write_to_memory(addr);
    }
}

bool ChunkContainer::check_ofmap(uint16_t* ofmap, int chunk, int eps, int len)
{
    int lenght = len == -1 ? mChunks[chunk].get_ofmap_len() : len;

    LOG_ERROR_IF_RETURN_FALSE(chunk >= mChunks.size(), "Chunk not in range");
    LOG_ERROR_IF_RETURN_FALSE(mChunks.at(chunk).mOfMap.mFillPtr == mChunks.at(chunk).mOfMap.mDataPtr, "Map not filled");
    LOG_ERROR_IF_RETURN_FALSE(mChunks.at(chunk).mOfMap.mLenght != lenght, "Lenght missmatch");
    std::cout << "ifmap " << (unsigned long)ofmap << std::endl;
    std::cout << "ofmap " << (unsigned long)mChunks.at(chunk).mOfMap.mDataPtr << std::endl;
    int err = 0;
    for (int i = 0; i < lenght; ++i) {
        if (std::abs((uint16_t)(mChunks.at(chunk).mOfMap.mDataPtr[i]) - ofmap[i]) >= eps) {
            std::cout << "error at " << i << "error: " << std::abs(mChunks.at(chunk).mOfMap.mDataPtr[i] - ofmap[i]) << std::endl;
            err++;
        }
        //std::cout << mChunks.at(chunk).mOfMap.mDataPtr[i] << " " << ofmap[i] << std::endl;
    }
    std::cout << "ERRRORS: " << err << std::endl;
    return true;
}

bool ChunkContainer::is_complete()
{
    for (Chunk& chunk : mChunks) {
        if (!chunk.is_complete())
            return false;
    }
    return true;
}

Chunk& ChunkContainer::get_chunk(int chunk)
{
    std::cout << " data ptr container " << (long)mChunks.at(0).mOfMap.mDataPtr << std::endl;
    LOG_ERROR_IF(chunk > mChunks.size(), "Chunk index out of range");
    return mChunks.at(chunk);
}

bool ChunkContainer::read_data_from_mem(int* ptr, int glob_addr, int len)
{
    int cur_addr = glob_addr;
    int cur_len = len;

    for (Chunk& chunk : mChunks) {
        int read_len = chunk.address_in_range(cur_addr, len);
        if (read_len == -1)
            continue;

        chunk.read(ptr, cur_addr, cur_len);
        cur_addr += read_len;
        cur_len -= read_len;

        if (cur_len <= 0)
            break;
    }
    return true;
}

void ChunkContainer::set_chunk_active(bool use_ofmap, int first_chunk, int last_chunk)
{
    for (int i = first_chunk; i < last_chunk; ++i) {
        mChunks[i].mFilters.mDataPtr = new int[mChunks[i].mFilters.mLenght];
        mChunks[i].mFilters.mFillPtr = mChunks[i].mFilters.mDataPtr;
        mChunks[i].mFilters.mIsActive = true;

        if (use_ofmap) {
            mChunks[i].mOfMap.mDataPtr = new int[mChunks[i].mOfMap.mLenght];
            mChunks[i].mOfMap.mFillPtr = mChunks[i].mOfMap.mDataPtr;
            mChunks[i].mOfMap.mIsActive = true;
        }

        mChunks[i].mConfigStruct.mDataPtr = new int[mChunks[i].mConfigStruct.mLenght];
        mChunks[i].mConfigStruct.mFillPtr = mChunks[i].mConfigStruct.mDataPtr;
        mChunks[i].mConfigStruct.mIsActive = true;

        // Input only for first chunk
        //if (i == first_chunk) {
        mChunks[i].mIfMap.mDataPtr = new int[mChunks[i].mIfMap.mLenght];
        mChunks[i].mIfMap.mFillPtr = mChunks[i].mIfMap.mDataPtr;
        mChunks[i].mIfMap.mIsActive = true;
        //}
    }
}

void Chunk::read(int* data_ptr, int glob_addr, int len)
{
    int chunk_start = mConfigStruct.mStartAddr;
    int chunk_end = mIfMap.mStartAddr + mIfMap.mLenght;
    int data_start = glob_addr;
    int data_end = glob_addr + len;
    int cur_addr = glob_addr;
    int cur_len = len;
    // Not in Range
    if (chunk_end < data_start || data_end < chunk_start)
        return;

    // A Part is in Config Struct
    if (cur_addr < mConfigStruct.mStartAddr + mConfigStruct.mLenght && cur_addr + cur_len >= mConfigStruct.mStartAddr) {
        int r = read_map(data_ptr, cur_addr, cur_len, mConfigStruct);
        cur_addr += r;
        cur_len -= r;
    }

    // A Part is in OfMaps
    if (cur_addr < mOfMap.mStartAddr + mOfMap.mLenght && cur_addr + cur_len >= mOfMap.mStartAddr) {
        int r = read_map(data_ptr, cur_addr, cur_len, mOfMap);
        cur_addr += r;
        cur_len -= r;
    }

    // A part is in Filters
    if (cur_addr < mIfMap.mStartAddr && cur_addr + cur_len >= mFilters.mStartAddr) {
        int r = read_map(data_ptr, cur_addr, cur_len, mFilters);
        cur_addr += r;
        cur_len -= r;
    }

    // A part is in IfMaps
    if (cur_addr < chunk_end && cur_addr + cur_len >= mIfMap.mStartAddr) {
        int r = read_map(data_ptr, cur_addr, cur_len, mIfMap);
        cur_addr += r;
        cur_len -= r;
    }    

    return;
}

int Chunk::address_in_range(int addr, int len)
{
    int chunk_start = mConfigStruct.mStartAddr;
    int chunk_end = mIfMap.mStartAddr + mIfMap.mLenght;
    int data_start = addr;
    int data_end = addr + len;

    if (chunk_end < data_start)
        return -1;

    if (data_end < chunk_start)
        return -1;

    if (data_end <= chunk_end)
        return len;

    return len - (data_end - chunk_end);
}

bool Chunk::is_complete()
{
    /*std::cout << "---- Chunk " << mChunkNumber << " -----" << std::endl;
    for (int i = 0; i < mConfigStruct.mLenght; ++i)
        std::cout << i << ": " << std::hex << mConfigStruct.mDataPtr[i] << std::endl;*/

    if (mConfigStruct.mIsActive) {
        if ((long)mConfigStruct.mDataPtr + (mConfigStruct.mLenght * 4) != (long)mConfigStruct.mFillPtr)
            LOG_ERROR_RETURN_FALSE(utils::string_format("Config Struct not complete in chunk %d", mChunkNumber));
    }

    if (mOfMap.mIsActive) {
        if ((long)mOfMap.mDataPtr + (mOfMap.mLenght * 4) != (long)mOfMap.mFillPtr)
            LOG_ERROR_RETURN_FALSE(utils::string_format("OfMap not complete in chunk %d", mChunkNumber));
    }

    if (mFilters.mIsActive) {
        if ((long)mFilters.mDataPtr + (mFilters.mLenght * 4) != (long)mFilters.mFillPtr)
            LOG_ERROR_RETURN_FALSE(utils::string_format("Filters not complete in chunk %d", mChunkNumber));        
    }

    if (mIfMap.mIsActive) {
        if ((long)mIfMap.mDataPtr + (mIfMap.mLenght * 4) != (long)mIfMap.mFillPtr)
            LOG_ERROR_RETURN_FALSE(utils::string_format("IfMap not complete in chunk %d", mChunkNumber));       
    }

    return true;
}

void Chunk::write_to_memory(uint16_t* addr)
{
    std::cout << "write on chunk : " << mChunkNumber << std::endl;

    std::cout << "Adrr ConfStruct: " << mOfMap.mStartAddr << std::endl;
    if (mConfigStruct.mIsActive) {
        for (int i = 0; i < mConfigStruct.mLenght; ++i) {
            (addr + mConfigStruct.mStartAddr)[i] = (uint16_t)mConfigStruct.mDataPtr[i];
        }
    }

    std::cout << "Adrr Ofmap: " << mOfMap.mStartAddr << std::endl;
    if (mOfMap.mIsActive) {
        for (int i = 0; i < mOfMap.mLenght; ++i) {
            (addr + mOfMap.mStartAddr)[i] = (uint16_t)mOfMap.mDataPtr[i];
        }
    }

    std::cout << "Adrr Filters: " << mFilters.mStartAddr << std::endl;
    if (mFilters.mIsActive) {
        for (int i = 0; i < mFilters.mLenght; ++i) {
            (addr + mFilters.mStartAddr)[i] = (uint16_t)mFilters.mDataPtr[i];
        }
    }

    std::cout << "Adrr Ifmap: " << mIfMap.mStartAddr << std::endl;
    if (mIfMap.mIsActive) {
        for (int i = 0; i < mIfMap.mLenght; ++i) {
            (addr + mIfMap.mStartAddr)[i] = (uint16_t)mIfMap.mDataPtr[i];
        }
    }

    /*if(false)
        std::memcpy(addr + mOfMap.mStartAddr, mOfMap.mDataPtr, int(mOfMap.mLenght /
        sizeof(uint16_t)));

    std::cout << " adrr filters: " << mFilters.mStartAddr << std::endl;
    if (mFilters.mIsActive)
        std::memcpy(addr + mFilters.mStartAddr, mFilters.mDataPtr, int(mFilters.mLenght /
        sizeof(uint16_t)));

    std::cout << " adrr ifmap: " << mIfMap.mStartAddr << std::endl;
    if (mIfMap.mIsActive)
        std::memcpy(addr + mIfMap.mStartAddr, mIfMap.mDataPtr, int(mIfMap.mLenght /
        sizeof(uint16_t)));*/
}

int* Chunk::get_ofmap_ptr()
{
    std::cout << " data ptr " << (long)mOfMap.mDataPtr << std::endl;
    return mOfMap.mDataPtr;
}

int* Chunk::get_ifmap_ptr()
{
    std::cout << " data ptr " << (long)mOfMap.mDataPtr << std::endl;
    return mIfMap.mDataPtr;
}

int* Chunk::get_filter_ptr()
{
    return mFilters.mDataPtr;
}

int Chunk::get_ofmap_offset()
{
    return mOfMap.mStartAddr;
}

int Chunk::get_ofmap_len()
{
    return mOfMap.mLenght;
}

int Chunk::get_ifmap_len()
{
    return mIfMap.mLenght;
}

int Chunk::get_filter_len()
{
    return mFilters.mLenght;
}

int Chunk::read_map(int* data_ptr, int glob_addr, int len, ChunkMap& map)
{
    int* ptr = data_ptr;
    int chunk_end = map.mStartAddr + map.mLenght;
    int data_end = glob_addr + len;
    int read_len = data_end <= chunk_end ? len : len - (data_end - chunk_end);

    if (!map.mIsActive)
        return read_len;

    if (glob_addr < map.mStartAddr) {
        int diff = map.mStartAddr - glob_addr;
        ptr += diff;
        read_len -= diff;
        std::cout << mChunkNumber << std::endl;
    }

    std::memcpy(map.mFillPtr, ptr, read_len * sizeof(int));
    map.mFillPtr += read_len;

    return read_len;
}
