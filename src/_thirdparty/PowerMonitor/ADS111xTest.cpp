#include "ADS111x.hpp"

int main(int, char **)
{
    ADS111x adsDevice("/dev/i2c-7");

    while (true)
    {
        std::cout << adsDevice.ADS111xReadmV({
                         .Pin = 4,
                         .Range = ADS111x::SL_RangeFSR::V4_096,
                         .DataRate = ADS111x::SL_DataRateSPS::SPS_860,
                     })
                  << " ";

        std::cout << (adsDevice.ADS111xReadmV({
                          .Pin = 5,
                          .Range = ADS111x::SL_RangeFSR::V4_096,
                          .DataRate = ADS111x::SL_DataRateSPS::SPS_860,
                      }) /
                      1000.f)
                  << " ";

        std::cout << adsDevice.ADS111xReadmV({
                         .Pin = 6,
                         .Range = ADS111x::SL_RangeFSR::V4_096,
                         .DataRate = ADS111x::SL_DataRateSPS::SPS_860,
                     })
                  << " ";

        std::cout << adsDevice.ADS111xReadmV({
                         .Pin = 7,
                         .Range = ADS111x::SL_RangeFSR::V4_096,
                         .DataRate = ADS111x::SL_DataRateSPS::SPS_860,
                     })
                  << " \n";
    }
}
