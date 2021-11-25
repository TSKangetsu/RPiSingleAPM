#pragma once

#include <iostream>

#include <deque>
#include <queue>
#include <vector>
#include <fstream>
#include <sstream>
#define BlackboxHeader "H Product:Blackbox flight data recorder by Nicholas Sherlock\n"
#define BlackboxVerion "H Data version: 2\n";
//
#define BlackboxIInt "H I interval:";
#define BlackboxPInt "H P interval:";
//
#define BlackboxIName "H Field I name:"
#define BlackboxISigned "H Field I signed:"
#define BlackboxIPredictor "H Field I predictor:"
#define BlackboxIEncoding "H Field I encoding:"
#define BlackboxPName "H Field P name:"
#define BlackboxPSigned "H Field P signed:"
#define BlackboxPPredictor "H Field P predictor:"
#define BlackboxPEncoding "H Field P encoding:"
#define BlackboxGName "H Field G name:"
#define BlackboxGSigned "H Field G signed:"
#define BlackboxGPredictor "H Field G predictor:"
#define BlackboxGEncoding "H Field G encoding:"
#define BlackboxHName "H Field H name:"
#define BlackboxHSigned "H Field H signed:"
#define BlackboxHPredictor "H Field H predictor:"
#define BlackboxHEncoding "H Field H encoding:"
#define BlackboxSName "H Field S name:"
#define BlackboxSSigned "H Field S signed:"
#define BlackboxSPredictor "H Field S predictor:"
#define BlackboxSEncoding "H Field S encoding:"
//
#define BlackboxFirmwareType "H Firmware type:"
//
struct BlackboxList
{
    const char *FrameName;
    unsigned int FrameSigned;
    unsigned int FramePredictor;
    unsigned int FrameEncoder;
};

struct BlackboxHeaderInfo
{
    int IInterval;
    const char *PInterval;

    std::string FirmwareType;
    std::vector<BlackboxList> BlackBoxDataIInfo;
    std::vector<BlackboxList> BlackBoxDataPInfo;
    std::vector<BlackboxList> BlackBoxDataGInfo;
    std::vector<BlackboxList> BlackBoxDataHInfo;
    std::vector<BlackboxList> BlackBoxDataSInfo;
    std::vector<std::string> BlackBoxCustom;
};

enum BlackboxEvent
{
    FLIGHT_LOG_EVENT_SYNC_BEEP = 0,
    FLIGHT_LOG_EVENT_AUTOTUNE_CYCLE_START = 10,  // UNUSED
    FLIGHT_LOG_EVENT_AUTOTUNE_CYCLE_RESULT = 11, // UNUSED
    FLIGHT_LOG_EVENT_AUTOTUNE_TARGETS = 12,      // UNUSED
    FLIGHT_LOG_EVENT_INFLIGHT_ADJUSTMENT = 13,
    FLIGHT_LOG_EVENT_LOGGING_RESUME = 14,
    FLIGHT_LOG_EVENT_DISARM = 15,
    FLIGHT_LOG_EVENT_FLIGHTMODE = 30, // Add new event type for flight mode status.
    FLIGHT_LOG_EVENT_LOG_END = 255
};

class BlackboxEncoder
{
public:
    std::string FullBlackboxHeader;
    inline BlackboxEncoder(BlackboxHeaderInfo);
    inline std::vector<uint8_t> BlackboxPIPush(std::vector<int> data);
    inline std::vector<uint8_t> BlackboxEPush(BlackboxEvent event, int time, int LoopIteration, int ext);
    inline std::vector<uint8_t> BlaclboxGPush(std::vector<int> data);
    inline std::vector<uint8_t> BlaclboxHPush(std::vector<int> data);
    inline std::vector<uint8_t> BlaclboxSPush(std::vector<int> data);

private:
    int PINTNUM = 1;
    int PINTDEM = 1;
    std::deque<std::vector<int>> PredictDataLast;
    BlackboxHeaderInfo HeaderInfo;

    inline static std::vector<uint8_t> dataEncode(int value, int type)
    {
        switch (type)
        {
        case 0:
            return unsignedBtyeEncode(zigzagEncode(value));
            break;

        case 1:
            return unsignedBtyeEncode(value);

        case 9:
        {
            return {};
        }

        default:
            return {};
            break;
        }
    }

    inline std::vector<int> dataPredict(std::vector<int> data, std::vector<BlackboxList> type)
    {
        if (data.size() == type.size())
        {
            std::vector<int> dataHis = data;
            for (size_t i = 0; i < data.size(); i++)
            {
                switch (type[i].FramePredictor)
                {
                case 0:
                    // do not thing..
                    break;

                case 1:
                {
                    if (PredictDataLast.size() > 0)
                    {
                        data[i] = data[i] - PredictDataLast[PredictDataLast.size() - 1][i];
                    }
                }
                break;
                case 2:
                {
                    if (PredictDataLast.size() > 2)
                    {
                        data[i] = PredictDataLast[PredictDataLast.size() - 1][i] - PredictDataLast[PredictDataLast.size() - 2][i] * 2 + PredictDataLast[PredictDataLast.size() - 3][i];
                    }
                }
                break;
                case 3:
                {
                    if (PredictDataLast.size() > 1)
                    {
                        data[i] = ((PredictDataLast[PredictDataLast.size() - 1][i] + PredictDataLast[PredictDataLast.size() - 2][i]) / 2);
                    }
                }

                case 6:
                    // do not thing..
                    break;

                default:
                    break;
                }
            }
            PredictDataLast.push_back(dataHis);
            if (PredictDataLast.size() > 3)
                PredictDataLast.pop_front();
        }

        return data;
    }

    inline static unsigned int zigzagEncode(int value)
    {
        return (value << 1) ^ (value >> 31);
    }

    inline static std::vector<uint8_t> unsignedBtyeEncode(int value)
    {
        std::vector<uint8_t> data;
        while (value > 127)
        {
            data.push_back(((uint8_t)(value | 0x80)));
            value >>= 7;
        }
        data.push_back(value);
        return data;
    }
};

BlackboxEncoder::BlackboxEncoder(BlackboxHeaderInfo Info)
{
    FullBlackboxHeader = "";
    FullBlackboxHeader += BlackboxHeader;
    FullBlackboxHeader += BlackboxVerion;
    FullBlackboxHeader += BlackboxIInt;
    FullBlackboxHeader += std::to_string(Info.IInterval);
    FullBlackboxHeader += "\n";
    FullBlackboxHeader += BlackboxPInt;
    FullBlackboxHeader += Info.PInterval;
    FullBlackboxHeader += "\n";

    std::string PINTER = Info.PInterval;
    std::string PINTERNUM = PINTER.substr(0, PINTER.find('/'));
    std::string PINTERDEM = PINTER.substr(PINTERNUM.size() + 1, PINTER.find('\0'));
    PINTNUM = std::atoi(PINTERNUM.c_str());
    PINTDEM = std::atoi(PINTERDEM.c_str());

    FullBlackboxHeader += BlackboxIName;
    for (auto sts : Info.BlackBoxDataIInfo)
    {
        FullBlackboxHeader += sts.FrameName;
        FullBlackboxHeader += ",";
    }
    // Remove Final one , expectly notice for the JS site, But blackbox-tools is work, blackboxviewer will dead with a nan, Because they fuck up the parser
    FullBlackboxHeader = FullBlackboxHeader.substr(0, FullBlackboxHeader.size() - 1);
    FullBlackboxHeader += "\n";

    FullBlackboxHeader += BlackboxISigned;
    for (auto sts : Info.BlackBoxDataIInfo)
    {
        FullBlackboxHeader += std::to_string(sts.FrameSigned);
        FullBlackboxHeader += ",";
    }
    // Remove Final one , expectly notice for the JS site, But blackbox-tools is work, blackboxviewer will dead with a nan, Because they fuck up the parser
    FullBlackboxHeader = FullBlackboxHeader.substr(0, FullBlackboxHeader.size() - 1);
    FullBlackboxHeader += "\n";

    FullBlackboxHeader += BlackboxIPredictor;
    for (auto sts : Info.BlackBoxDataIInfo)
    {
        FullBlackboxHeader += std::to_string(sts.FramePredictor);
        FullBlackboxHeader += ",";
    }
    // Remove Final one , expectly notice for the JS site, But blackbox-tools is work, blackboxviewer will dead with a nan, Because they fuck up the parser
    FullBlackboxHeader = FullBlackboxHeader.substr(0, FullBlackboxHeader.size() - 1);
    FullBlackboxHeader += "\n";

    FullBlackboxHeader += BlackboxIEncoding;
    for (auto sts : Info.BlackBoxDataIInfo)
    {
        FullBlackboxHeader += std::to_string(sts.FrameEncoder);
        FullBlackboxHeader += ",";
    }
    // Remove Final one , expectly notice for the JS site, But blackbox-tools is work, blackboxviewer will dead with a nan, Because they fuck up the parser
    FullBlackboxHeader = FullBlackboxHeader.substr(0, FullBlackboxHeader.size() - 1);
    FullBlackboxHeader += "\n";
    //=======================P FRAME INFO=============================//
    if (Info.BlackBoxDataPInfo.size() > 0)
    {
        FullBlackboxHeader += BlackboxPName;
        for (auto sts : Info.BlackBoxDataPInfo)
        {
            FullBlackboxHeader += sts.FrameName;
            FullBlackboxHeader += ",";
        }
        // Remove Final one , expectly notice for the JS site, But blackbox-tools is work, blackboxviewer will dead with a nan, Because they fuck up the parser
        FullBlackboxHeader = FullBlackboxHeader.substr(0, FullBlackboxHeader.size() - 1);
        FullBlackboxHeader += "\n";

        FullBlackboxHeader += BlackboxPSigned;
        for (auto sts : Info.BlackBoxDataPInfo)
        {
            FullBlackboxHeader += std::to_string(sts.FrameSigned);
            FullBlackboxHeader += ",";
        }
        // Remove Final one , expectly notice for the JS site, But blackbox-tools is work, blackboxviewer will dead with a nan, Because they fuck up the parser
        FullBlackboxHeader = FullBlackboxHeader.substr(0, FullBlackboxHeader.size() - 1);
        FullBlackboxHeader += "\n";

        FullBlackboxHeader += BlackboxPPredictor;
        for (auto sts : Info.BlackBoxDataPInfo)
        {
            FullBlackboxHeader += std::to_string(sts.FramePredictor);
            FullBlackboxHeader += ",";
        }
        // Remove Final one , expectly notice for the JS site, But blackbox-tools is work, blackboxviewer will dead with a nan, Because they fuck up the parser
        FullBlackboxHeader = FullBlackboxHeader.substr(0, FullBlackboxHeader.size() - 1);
        FullBlackboxHeader += "\n";

        FullBlackboxHeader += BlackboxPEncoding;
        for (auto sts : Info.BlackBoxDataPInfo)
        {
            FullBlackboxHeader += std::to_string(sts.FrameEncoder);
            FullBlackboxHeader += ",";
        }
        // Remove Final one , expectly notice for the JS site, But blackbox-tools is work, blackboxviewer will dead with a nan, Because they fuck up the parser
        FullBlackboxHeader = FullBlackboxHeader.substr(0, FullBlackboxHeader.size() - 1);
        FullBlackboxHeader += "\n";
    }

    //=======================G FRAME INFO=============================//
    if (Info.BlackBoxDataGInfo.size() > 0)
    {
        FullBlackboxHeader += BlackboxGName;
        for (auto sts : Info.BlackBoxDataGInfo)
        {
            FullBlackboxHeader += sts.FrameName;
            FullBlackboxHeader += ",";
        }
        // Remove Final one , expectly notice for the JS site, But blackbox-tools is work, blackboxviewer will dead with a nan, Because they fuck up the parser
        FullBlackboxHeader = FullBlackboxHeader.substr(0, FullBlackboxHeader.size() - 1);
        FullBlackboxHeader += "\n";

        FullBlackboxHeader += BlackboxGSigned;
        for (auto sts : Info.BlackBoxDataGInfo)
        {
            FullBlackboxHeader += std::to_string(sts.FrameSigned);
            FullBlackboxHeader += ",";
        }
        // Remove Final one , expectly notice for the JS site, But blackbox-tools is work, blackboxviewer will dead with a nan, Because they fuck up the parser
        FullBlackboxHeader = FullBlackboxHeader.substr(0, FullBlackboxHeader.size() - 1);
        FullBlackboxHeader += "\n";

        FullBlackboxHeader += BlackboxGPredictor;
        for (auto sts : Info.BlackBoxDataGInfo)
        {
            FullBlackboxHeader += std::to_string(sts.FramePredictor);
            FullBlackboxHeader += ",";
        }
        // Remove Final one , expectly notice for the JS site, But blackbox-tools is work, blackboxviewer will dead with a nan, Because they fuck up the parser
        FullBlackboxHeader = FullBlackboxHeader.substr(0, FullBlackboxHeader.size() - 1);
        FullBlackboxHeader += "\n";

        FullBlackboxHeader += BlackboxGEncoding;
        for (auto sts : Info.BlackBoxDataGInfo)
        {
            FullBlackboxHeader += std::to_string(sts.FrameEncoder);
            FullBlackboxHeader += ",";
        }
        // Remove Final one , expectly notice for the JS site, But blackbox-tools is work, blackboxviewer will dead with a nan, Because they fuck up the parser
        FullBlackboxHeader = FullBlackboxHeader.substr(0, FullBlackboxHeader.size() - 1);
        FullBlackboxHeader += "\n";
    }

    //=======================H FRAME INFO=============================//
    if (Info.BlackBoxDataHInfo.size() > 0)
    {
        FullBlackboxHeader += BlackboxHName;
        for (auto sts : Info.BlackBoxDataHInfo)
        {
            FullBlackboxHeader += sts.FrameName;
            FullBlackboxHeader += ",";
        }
        // Remove Final one , expectly notice for the JS site, But blackbox-tools is work, blackboxviewer will dead with a nan, Because they fuck up the parser
        FullBlackboxHeader = FullBlackboxHeader.substr(0, FullBlackboxHeader.size() - 1);
        FullBlackboxHeader += "\n";

        FullBlackboxHeader += BlackboxHSigned;
        for (auto sts : Info.BlackBoxDataHInfo)
        {
            FullBlackboxHeader += std::to_string(sts.FrameSigned);
            FullBlackboxHeader += ",";
        }
        // Remove Final one , expectly notice for the JS site, But blackbox-tools is work, blackboxviewer will dead with a nan, Because they fuck up the parser
        FullBlackboxHeader = FullBlackboxHeader.substr(0, FullBlackboxHeader.size() - 1);
        FullBlackboxHeader += "\n";

        FullBlackboxHeader += BlackboxHPredictor;
        for (auto sts : Info.BlackBoxDataHInfo)
        {
            FullBlackboxHeader += std::to_string(sts.FramePredictor);
            FullBlackboxHeader += ",";
        }
        // Remove Final one , expectly notice for the JS site, But blackbox-tools is work, blackboxviewer will dead with a nan, Because they fuck up the parser
        FullBlackboxHeader = FullBlackboxHeader.substr(0, FullBlackboxHeader.size() - 1);
        FullBlackboxHeader += "\n";

        FullBlackboxHeader += BlackboxHEncoding;
        for (auto sts : Info.BlackBoxDataHInfo)
        {
            FullBlackboxHeader += std::to_string(sts.FrameEncoder);
            FullBlackboxHeader += ",";
        }
        // Remove Final one , expectly notice for the JS site, But blackbox-tools is work, blackboxviewer will dead with a nan, Because they fuck up the parser
        FullBlackboxHeader = FullBlackboxHeader.substr(0, FullBlackboxHeader.size() - 1);
        FullBlackboxHeader += "\n";
    }

    //=======================S FRAME INFO=============================//
    if (Info.BlackBoxDataSInfo.size() > 0)
    {
        FullBlackboxHeader += BlackboxSName;
        for (auto sts : Info.BlackBoxDataSInfo)
        {
            FullBlackboxHeader += sts.FrameName;
            FullBlackboxHeader += ",";
        }
        // Remove Final one , expectly notice for the JS site, But blackbox-tools is work, blackboxviewer will dead with a nan, Because they fuck up the parser
        FullBlackboxHeader = FullBlackboxHeader.substr(0, FullBlackboxHeader.size() - 1);
        FullBlackboxHeader += "\n";

        FullBlackboxHeader += BlackboxSSigned;
        for (auto sts : Info.BlackBoxDataSInfo)
        {
            FullBlackboxHeader += std::to_string(sts.FrameSigned);
            FullBlackboxHeader += ",";
        }
        // Remove Final one , expectly notice for the JS site, But blackbox-tools is work, blackboxviewer will dead with a nan, Because they fuck up the parser
        FullBlackboxHeader = FullBlackboxHeader.substr(0, FullBlackboxHeader.size() - 1);
        FullBlackboxHeader += "\n";

        FullBlackboxHeader += BlackboxSPredictor;
        for (auto sts : Info.BlackBoxDataSInfo)
        {
            FullBlackboxHeader += std::to_string(sts.FramePredictor);
            FullBlackboxHeader += ",";
        }
        // Remove Final one , expectly notice for the JS site, But blackbox-tools is work, blackboxviewer will dead with a nan, Because they fuck up the parser
        FullBlackboxHeader = FullBlackboxHeader.substr(0, FullBlackboxHeader.size() - 1);
        FullBlackboxHeader += "\n";

        FullBlackboxHeader += BlackboxSEncoding;
        for (auto sts : Info.BlackBoxDataSInfo)
        {
            FullBlackboxHeader += std::to_string(sts.FrameEncoder);
            FullBlackboxHeader += ",";
        }
        // Remove Final one , expectly notice for the JS site, But blackbox-tools is work, blackboxviewer will dead with a nan, Because they fuck up the parser
        FullBlackboxHeader = FullBlackboxHeader.substr(0, FullBlackboxHeader.size() - 1);
        FullBlackboxHeader += "\n";
    }

    FullBlackboxHeader += BlackboxFirmwareType;
    FullBlackboxHeader += Info.FirmwareType;
    FullBlackboxHeader += "\n";

    for (auto sts : Info.BlackBoxCustom)
    {
        FullBlackboxHeader += sts;
    }

    HeaderInfo = Info;
};

std::vector<uint8_t> BlackboxEncoder::BlackboxPIPush(std::vector<int> data)
{
    std::vector<uint8_t> FrameBuffer;
    //
    char Head = 'I';
    if (data[0] % HeaderInfo.IInterval == 0)
        Head = 'I';
    else if ((data[0] % HeaderInfo.IInterval + PINTNUM - 1) % PINTDEM < PINTNUM)
        Head = 'P';
    else
        return {}; // No log free, size 0;

    FrameBuffer.push_back((uint8_t)Head);
    //
    if (Head == 'I' && HeaderInfo.BlackBoxDataIInfo.size() > 0)
    {
        std::vector<int> predicted = dataPredict(data, HeaderInfo.BlackBoxDataIInfo);
        for (size_t i = 0; i < data.size(); i++)
        {
            std::vector<uint8_t> tmpBuffer = dataEncode(predicted[i], HeaderInfo.BlackBoxDataIInfo[i].FrameEncoder);
            FrameBuffer.insert(FrameBuffer.end(), tmpBuffer.begin(), tmpBuffer.end());
        }
        //
        for (; !PredictDataLast.empty(); PredictDataLast.pop_front())
            ;

        for (size_t i = 0; i < 3; i++)
        {
            PredictDataLast.push_back(data);
        }
    }
    else if (Head == 'P' && HeaderInfo.BlackBoxDataPInfo.size() > 0)
    {
        std::vector<int> predicted = dataPredict(data, HeaderInfo.BlackBoxDataPInfo);
        for (size_t i = 0; i < data.size(); i++)
        {
            std::vector<uint8_t> tmpBuffer = dataEncode(predicted[i], HeaderInfo.BlackBoxDataPInfo[i].FrameEncoder);
            FrameBuffer.insert(FrameBuffer.end(), tmpBuffer.begin(), tmpBuffer.end());
        }
    }
    else
    {
        return {};
    }

    return FrameBuffer;
}

std::vector<uint8_t> BlackboxEncoder::BlackboxEPush(BlackboxEvent event, int time, int LoopIteration, int ext)
{
    std::vector<uint8_t> wdata;
    wdata.push_back(((uint8_t)'E'));
    wdata.push_back(((uint8_t)event));

    //Now serialize the data for this specific frame type
    switch (event)
    {
    case FLIGHT_LOG_EVENT_SYNC_BEEP:
        for (size_t i = 0; i < unsignedBtyeEncode(time).size(); i++)
        {
            wdata.push_back(unsignedBtyeEncode(time)[i]);
        }
        break;
    case FLIGHT_LOG_EVENT_FLIGHTMODE: // New flightmode flags write
        // TODO: Not support Yet
        // blackboxWriteUnsignedVB(data->flightMode.flags);
        // blackboxWriteUnsignedVB(data->flightMode.lastFlags);
        break;
    case FLIGHT_LOG_EVENT_DISARM:
        for (size_t i = 0; i < unsignedBtyeEncode(ext).size(); i++)
        {
            wdata.push_back(unsignedBtyeEncode(ext)[i]);
        }
        break;
    case FLIGHT_LOG_EVENT_INFLIGHT_ADJUSTMENT:
        // TODO: Not support Yet
        // if (data->inflightAdjustment.floatFlag) {
        // blackboxWrite(data->inflightAdjustment.adjustmentFunction + FLIGHT_LOG_EVENT_INFLIGHT_ADJUSTMENT_FUNCTION_FLOAT_VALUE_FLAG);
        // blackboxWriteFloat(data->inflightAdjustment.newFloatValue);
        // } else {
        // blackboxWrite(data->inflightAdjustment.adjustmentFunction);
        // blackboxWriteSignedVB(data->inflightAdjustment.newValue);
        // }
        break;
    case FLIGHT_LOG_EVENT_LOGGING_RESUME:
        for (size_t i = 0; i < unsignedBtyeEncode(LoopIteration).size(); i++)
        {
            wdata.push_back(unsignedBtyeEncode(LoopIteration)[i]);
        }
        for (size_t i = 0; i < unsignedBtyeEncode(time).size(); i++)
        {
            wdata.push_back(unsignedBtyeEncode(time)[i]);
        }
        break;
    case FLIGHT_LOG_EVENT_LOG_END:
        for (size_t i = 0; i < sizeof("End of log"); i++)
        {
            wdata.push_back(std::string("End of log", sizeof("End of log"))[i]);
        }
        wdata.push_back(0x00);
        break;
    default:
        break;
    }

    return wdata;
}

std::vector<uint8_t> BlackboxEncoder::BlaclboxGPush(std::vector<int> data)
{
    std::vector<uint8_t> FrameBuffer;
    FrameBuffer.push_back((uint8_t)'G');
    for (size_t i = 0; i < data.size(); i++)
    {
        std::vector<uint8_t> tmpBuffer = dataEncode(data[i], HeaderInfo.BlackBoxDataGInfo[i].FrameEncoder);
        FrameBuffer.insert(FrameBuffer.end(), tmpBuffer.begin(), tmpBuffer.end());
    }

    return FrameBuffer;
}

std::vector<uint8_t> BlackboxEncoder::BlaclboxHPush(std::vector<int> data)
{
    std::vector<uint8_t> FrameBuffer;
    FrameBuffer.push_back((uint8_t)'H');
    for (size_t i = 0; i < data.size(); i++)
    {
        std::vector<uint8_t> tmpBuffer = dataEncode(data[i], HeaderInfo.BlackBoxDataGInfo[i].FrameEncoder);
        FrameBuffer.insert(FrameBuffer.end(), tmpBuffer.begin(), tmpBuffer.end());
    }

    return FrameBuffer;
}

inline void FileInjectSTR(std::ofstream &file, const char *header)
{
    file << header;
}

inline void FileInjectQueue(std::ofstream &file, std::vector<uint8_t> data)
{
    for (size_t i = 0; i < data.size(); i++)
    {
        file << data[i];
    }
}