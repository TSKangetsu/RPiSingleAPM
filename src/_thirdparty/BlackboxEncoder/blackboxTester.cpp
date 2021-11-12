#include <sys/time.h>
#include <unistd.h>
#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>
#include "./Blackbox.hpp"

uint32_t GetTimestamp()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec * (uint64_t)1000000 + tv.tv_usec;
}

unsigned int zigzagEncode(int value)
{
    return (value << 1) ^ (value >> 31);
}

std::vector<uint8_t> unsignedBtyeEncode(int value)
{
    std::vector<uint8_t> data;
    while (value > 127)
    {
        data.push_back(((uint8_t)(value | 0x80)));
        value >>= 7;
    }
    data.push_back(value);
    return data;
};

void FileInjectSTR(std::ofstream &file, const char *header)
{
    file << header;
}

void FileInjectQueue(std::ofstream &file, std::vector<uint8_t> data)
{
    for (size_t i = 0; i < data.size(); i++)
    {
        file << data[i];
    }
}

int main(int argc, char const *argv[])
{
    std::vector<BlackboxList> IList = {
        {
            .FrameName = "loopIteration",
            .FrameSigned = 0,
            .FramePredictor = 0,
            .FrameEncoder = 1,
        },
        {
            .FrameName = "time",
            .FrameSigned = 0,
            .FramePredictor = 0,
            .FrameEncoder = 1,
        },
        {
            .FrameName = "motor[0]",
            .FrameSigned = 0,
            .FramePredictor = 0,
            .FrameEncoder = 1,
        },
        {
            .FrameName = "motor[1]",
            .FrameSigned = 0,
            .FramePredictor = 0,
            .FrameEncoder = 1,
        },
        {
            .FrameName = "motor[2]",
            .FrameSigned = 0,
            .FramePredictor = 0,
            .FrameEncoder = 1,
        },
        {
            .FrameName = "motor[3]",
            .FrameSigned = 0,
            .FramePredictor = 0,
            .FrameEncoder = 1,
        },
    };

    std::vector<BlackboxList> PList = {
        {
            .FrameName = "loopIteration",
            .FrameSigned = 0,
            .FramePredictor = 6,
            .FrameEncoder = 9,
        },
        {
            .FrameName = "time",
            .FrameSigned = 0,
            .FramePredictor = 2,
            .FrameEncoder = 0,
        },
        {
            .FrameName = "motor[0]",
            .FrameSigned = 0,
            .FramePredictor = 1,
            .FrameEncoder = 1,
        },
        {
            .FrameName = "motor[1]",
            .FrameSigned = 0,
            .FramePredictor = 1,
            .FrameEncoder = 1,
        },
        {
            .FrameName = "motor[2]",
            .FrameSigned = 0,
            .FramePredictor = 1,
            .FrameEncoder = 1,
        },
        {
            .FrameName = "motor[3]",
            .FrameSigned = 0,
            .FramePredictor = 1,
            .FrameEncoder = 1,
        },
    };

    BlackboxEncoder encode({
        .IInterval = 32,
        .PInterval = "1/1",
        .FirmwareType = "Singleflight",
        .BlackBoxDataIInfo = IList,
        .BlackBoxDataPInfo = PList,
        .BlackBoxCustom = {},
    });

    int starttime = GetTimestamp();

    std::ofstream configs;
    configs.open("./bxt.txt", std::ios::out | std::ios::binary);

    FileInjectSTR(configs, encode.FullBlackboxHeader.c_str());

    for (int i = 0; i < 2000; i++)
    {
        int x = GetTimestamp() - starttime;
        FileInjectQueue(configs, encode.BlackboxPIPush({(int)i, x, i, i, i, i}));
        usleep(100);
    }

    configs.close();
    return 0;
}
