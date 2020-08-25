#include "logger.h"
#include "synchronization.h"

Logger::Logger()
{
    log_count = 0;
    log_flush_count = 10;
}

Logger::Logger(uint8_t maxCount)
{
    log_count = 0;
    log_flush_count = maxCount;
}

void Logger::open(std::string path)
{
    log_path = path;
    log_file.open(log_path);
    log_file << std::fixed;
    log_file.precision(2);
}


void Logger::write(std::string msg)
{
    log_file << msg;
    log_file.flush();
}


void Logger::logData(std::string t_rtc, float pressure, float temperature)
{
    clock_gettime(CLOCK_MONOTONIC, &T_now);
    t_nsec = asNanosec(&T_now);
    log_file << t_nsec << ",";
    log_file << t_rtc << ",";
    log_file << pressure << ",";
    log_file << temperature << "\n";
    log_count++;
    if (log_count > log_flush_count)
    {
        log_file.flush();
        log_count = 0;
    }
}

void Logger::logMsg(std::string msg)
{
    clock_gettime(CLOCK_MONOTONIC, &T_now);
    t_nsec = asNanosec(&T_now);
    log_file << t_nsec;
    log_file << " ";
    log_file << msg;
    log_file << "\r\n";
    log_file.flush();
}

void Logger::close()
{
    log_file.close();
}
