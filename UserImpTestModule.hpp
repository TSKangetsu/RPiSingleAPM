#pragma once
#include <thread>
#include "./src/SingleAPM.hpp"
#include "./src/_thirdparty/FlowController.hpp"

using namespace SingleAPMAPI;

class FlightControllMain : public RPiSingleAPM
{
public:
    inline FlightControllMain(){};

    inline void ServoControllInit()
    {
        servoMonitor.reset(new FlowThread(
            [&]
            {
                // 三个舵机三个开关
                // 这样子写是为了避免重复触发
                if (RF._uORB_RC_Channel_PWM[6] > 1900 && isServo6Trige)
                {
                    APMControllerServo(4, 1900);
                    isServo6Trige = false;
                }
                else if (RF._uORB_RC_Channel_PWM[6] < 1400 && !isServo6Trige)
                {
                    APMControllerServo(4, 1000);
                    isServo6Trige = true;
                }
            },
            0, 50.f));
    };

    inline void stopAndWaitForExit()
    {
        servoMonitor->FlowStopAndWait();
    }

private:
    bool isServo6Trige = false;
    std::unique_ptr<FlowThread> servoMonitor;
};