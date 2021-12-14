#include <unistd.h>
#include <iostream>
#include "PowerADC.hpp"

int main(int argc, char const *argv[])
{
    ADCConfig config;
    config.MAXCurrent = 5.0;
    config.ROfShunt = 0.01;
    config.ADCBus = INA226_TIME_8MS;
    config.Shunttime = INA226_TIME_8MS;
    config.Averages = INA226_AVERAGES_16;
    config.Mode = INA226_MODE_SHUNT_BUS_CONTINUOUS;
    PowerADC adcf("/dev/i2c-1", 0x40, config);

    while (true)
    {
        ADCData data = adcf.ADCGetData();
        std::cout << data.voltage << " ";
        std::cout << data.shunt_voltage << " ";
        std::cout << data.power << " ";
        std::cout << data.current << " \n";
        sleep(1);
    }

    return 0;
}
