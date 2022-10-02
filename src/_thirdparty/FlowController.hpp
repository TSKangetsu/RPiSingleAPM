#pragma once
#include <chrono>
#include <thread>
#include <time.h>
#include <unistd.h>
#include <functional>
#include <sys/time.h>
#include <condition_variable>
#define LINUX_SYSTEM_SLEEP_DELAY 100

class FlowThread : public std::thread
{
public:
    inline FlowThread(std::function<void()> thread, int CPUID, float ClockingHZ);

    inline FlowThread(std::function<void()> thread) : FlowThread(thread, -1, -1){};
    inline FlowThread(std::function<void()> thread, int CPUID) : FlowThread(thread, CPUID, -1){};
    inline FlowThread(std::function<void()> thread, float ClockingHZ) : FlowThread(thread, -1, ClockingHZ){};

    inline void FlowTryStop();
    inline void FlowStopAndWait();
    inline void FlowWait() { this->join(); };

    float RunClockHz = 0;
    uint32_t TimeOut_MAX = 0;
    uint32_t TimeDT = 0;

private:
    uint32_t GetTimeStamp()
    {
        struct timespec tv;
        clock_gettime(CLOCK_MONOTONIC, &tv);
        return ((tv.tv_sec * (uint32_t)1000000 + (tv.tv_nsec / 1000)));
    }

    uint32_t TimeThreadStart = 0;
    uint32_t Time__Max = 1000000; // defaultly run 1HZ
    uint32_t TimeStart = 0;
    uint32_t Time__End = 0;
    uint32_t Time_Next = 0;
    uint32_t ThreadSpend = 0;
    uint32_t SleepOffset = 100;

    float tmpClock = 0;
    float targetClock = 0;
    int clockcount = 1;

    bool lockFlag = false;

    std::mutex copylock;
    std::condition_variable notifyWait;

    bool IsThreadRunning = false;
    std::function<void()> threadMain = [] {};
};

FlowThread::FlowThread(std::function<void()> thread, int CPUID, float ClockingHZ)
    : std::thread(
          [&]
          {
              sleep(1);
              IsThreadRunning = true;
              //
              std::unique_lock<std::mutex> lockWait{copylock};
              while (!lockFlag)
                  notifyWait.wait(lockWait);
              //
              TimeThreadStart = GetTimeStamp();
              while (IsThreadRunning)
              {
                  TimeStart = GetTimeStamp() - TimeThreadStart;
                  Time_Next = TimeStart - Time__End;
                  //
                  threadMain();
                  //
                  Time__End = GetTimeStamp() - TimeThreadStart;
                  ThreadSpend = Time__End - TimeStart;
                  //
                  if (Time__Max > 0)
                  {
                      if (ThreadSpend + Time_Next + SleepOffset > Time__Max | Time_Next < 0)
                      {
                          usleep(1);
                      }
                      else
                      {
                          usleep(Time__Max - ThreadSpend - Time_Next - SleepOffset);
                      }
                      if (ThreadSpend + Time_Next > TimeOut_MAX)
                      {
                          TimeOut_MAX = ThreadSpend;
                      }
                      //
                      if (clockcount > targetClock)
                      {
                          RunClockHz = (1.f / (tmpClock / (float)clockcount) * 1000000.f);
                          clockcount = 0;
                          tmpClock = 0;
                      }
                      tmpClock += TimeDT;
                      clockcount++;
                  }
                  Time__End = GetTimeStamp() - TimeThreadStart;
                  TimeDT = Time__End - TimeStart;
              }
          })
{
    threadMain = thread;
    //
    targetClock = ClockingHZ;
    Time__Max = ClockingHZ > 0.0 ? ((int)((1.f / ClockingHZ) * 1000000.f)) : 0;
    //
    if (CPUID >= 0)
    {
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(CPUID, &cpuset);
        int rc = pthread_setaffinity_np(this->native_handle(), sizeof(cpu_set_t), &cpuset);
    }
    //
    {
        std::unique_lock<std::mutex> lockWaitMain{copylock};
        lockFlag = true;
    }
    notifyWait.notify_all();
};

void FlowThread::FlowTryStop()
{
    IsThreadRunning = false;
};

void FlowThread::FlowStopAndWait()
{
    FlowTryStop();
    if (this->joinable())
        this->join();
};