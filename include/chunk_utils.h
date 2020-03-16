#pragma once
#include <iostream>
#include <vector>

class ChunkMap
{
public:
    void freeMap() {
        if (mIsActive && mDataPtr != nullptr)
            free(mDataPtr);
    }

    int mStartAddr = 0; // Gloabal Address in global mem
    int mLenght = 0; // Lenght of Map
    bool mIsActive = false; // Map is in use
    int *mDataPtr = nullptr; // Pointer to data
    int *mFillPtr = nullptr; // Pointer to next empty add if not read complete
};

// Chunk Class. Contains ChunkMaps .
class Chunk
{
public:
    //Read data from this address. 
    void read(int* data_ptr, int glob_addr, int len);

    //Check if data is in range.
    //Return how much data chunk must read.
    //Return -1 if der is no data in range(addr, len) for this chunk
    int address_in_range(int addr, int len);

    friend class ChunkContainer;

private:
    //Read data into a specific ChunkMap
    int read_map(int* data_ptr, int glob_addr, int len, ChunkMap& map);

private:
    ChunkMap mOfMap;
    ChunkMap mIfMap;
    ChunkMap mFilters;
};

// ChunkContainer Class. Contains Chunks.
class ChunkContainer
{
    public:
        bool init_chunk(std::string path_json, int vsize = 4);
        bool read_data_from_glob(std::string path_glob, bool use_ofmap = false, int first_chunk = 0, int last_chunk = 36);
        bool write_data_on_addr(int addr);
        bool check_ofmap(int ofmap, int len = -1);

    private:
        bool read_data_from_mem(int* ptr, int glob_addr, int len);
        void set_chunk_active(bool use_ofmap, int first_chunk, int last_chunk);

    private:
        int mVSize;
        bool mIsInit = 0;
        std::vector<Chunk> mChunks;
};

