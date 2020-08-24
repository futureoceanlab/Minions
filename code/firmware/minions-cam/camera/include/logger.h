#ifndef LOGGER_H
#define LOGGER_H

#include <string>
#include <chrono>
#include <iostream>
#include <fstream> // ofstream
#include <iostream>
#include <time.h>
#include <sys/time.h>

class Logger
{
public:
    Logger();
    Logger(uint8_t maxCount);
    void write(std::string msg);
    void logData(std::string t_rtc, float pressure, float temperature);
    void logMsg(std::string msg);
    void open(std::string path);
    void close();

private:
    uint8_t log_count;
    uint8_t log_flush_count;
    struct timespec T_now;
    long long t_nsec;
    std::string log_path;
    std::ofstream log_file;
};

#endif
