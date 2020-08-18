#include "logger.h"
#include "synchronization.h"

Logger::Logger()
{
    logCount = 0;
    logFlushCount = 10;
}

Logger::Logger(uint8_t maxCount)
{
    logCount = 0;
    logFlushCount = maxCount;
}

void Logger::open(std::string path)
{
    logPath = path;
    logF.open(logPath);
    logF << std::fixed;
    logF.precision(2);
    logF << "Timestamp(ns),RTC,Pressure(mbar),Temperature(C)\n";
}

void Logger::logData(std::string t_rtc, float pressure, float temperature)
{
    clock_gettime(CLOCK_MONOTONIC, &T_now);
    t_nsec = as_nsec(&T_now);
    logF << t_nsec << ",";
    logF << t_rtc << ",";
    logF << pressure << ",";
    logF << temperature << "\n";
    logCount++;
    if (logCount > logFlushCount)
    {
        logF.flush();
        logCount = 0;
    }
}

void Logger::logMsg(std::string msg)
{
    clock_gettime(CLOCK_MONOTONIC, &T_now);
    t_nsec = as_nsec(&T_now);
    logF << t_nsec;
    logF << " ";
    logF << msg;
    logF << "\r\n";
    logF.flush();
}

void Logger::close()
{
    logF.close();
}
