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
        chunk.mFilters.mStartAddr = json_chunk["addr_filters"].get<int>();
        chunk.mFilters.mLenght = json_chunk["addr_biases"].get<int>() + json_chunk["length_biases"].get<int>() - json_chunk["addr_filters"].get<int>();
        chunk.mOfMap.mStartAddr = json_chunk["addr_ofmaps_conv"].get<int>();
        chunk.mOfMap.mLenght = json_chunk["length_ofmaps_conv"].get<int>();
        chunk.mIfMap.mStartAddr = chunk.mFilters.mStartAddr + chunk.mFilters.mLenght;
        chunk.mIfMap.mLenght = json_chunk["length_ifmaps"].get<int>();
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
            
        if (mem_len >= max_len || (addr != last_addr + 1 && last_addr != -1)) {
            read_data_from_mem(mem_ptr, last_addr * mVSize, mem_len);
            mem_len = 0;
        }

        for (int i = 0; i < mVSize; i++) {
            string word_str = value_str.substr(i * 4, 4);
            int word_value = stoi(word_str, 0, 16);
            mem_ptr[mem_len + mVSize - (i + 1)] = word_value;
        }

        mem_len += mVSize;
        last_addr = addr;
        total_lines++;
    }

    if (mem_len > 0)
        read_data_from_mem(mem_ptr, last_addr, mem_len);

    return true;
}

bool ChunkContainer::read_data_from_mem(int* ptr, int glob_addr, int len)
{
    int cur_addr = glob_addr;
    int cur_len = len;
    for (auto chunk : mChunks) {
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
    for (int i = first_chunk; i <= last_chunk; ++i) {
        mChunks[i].mFilters.mDataPtr = new int[mChunks[i].mFilters.mLenght];
        mChunks[i].mFilters.mFillPtr = mChunks[i].mFilters.mDataPtr;
        mChunks[i].mFilters.mIsActive = true;

        if (use_ofmap) {
            mChunks[i].mOfMap.mDataPtr = new int[mChunks[i].mOfMap.mLenght];
            mChunks[i].mOfMap.mFillPtr = mChunks[i].mOfMap.mDataPtr;
            mChunks[i].mOfMap.mIsActive = true;
        }

        // Input only for first chunk
        if (i == first_chunk) {
            mChunks[i].mIfMap.mDataPtr = new int[mChunks[i].mIfMap.mLenght];
            mChunks[i].mIfMap.mFillPtr = mChunks[i].mIfMap.mDataPtr;
            mChunks[i].mIfMap.mIsActive = true;
        }
    }
}

void Chunk::read(int* data_ptr, int glob_addr, int len)
{
    int chunk_start = mOfMap.mStartAddr;
    int chunk_end = mIfMap.mStartAddr + mIfMap.mLenght;
    int data_start = glob_addr;
    int data_end = glob_addr + len;
    int cur_addr = glob_addr;
    int cur_len = len;

    // Not in Range
    if (chunk_end < data_start || data_end < chunk_start)
        return;


    // A Part is in OfMaps
    if (cur_addr < mFilters.mStartAddr) {
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
    int chunk_start = mOfMap.mStartAddr;
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

int Chunk::read_map(int* data_ptr, int glob_addr, int len, ChunkMap& map)
{
    int chunk_end = map.mStartAddr + map.mLenght;
    int data_end = glob_addr + len;
    int read_len = data_end <= chunk_end ? len : len - (data_end - chunk_end);

    if (!map.mIsActive)
        return read_len;

    std::memcpy(map.mFillPtr, data_ptr, read_len);
    map.mFillPtr += read_len;
    return read_len;
}
