#ifndef TIMESPEC_H
#define TIMESPEC_H

#include <time.h>

// timespec加法辅助函数
inline timespec timespec_add(timespec t1, timespec t2) {
    timespec result;
    result.tv_sec = t1.tv_sec + t2.tv_sec;
    result.tv_nsec = t1.tv_nsec + t2.tv_nsec;
    
    if (result.tv_nsec >= 1000000000) {
        result.tv_sec++;
        result.tv_nsec -= 1000000000;
    }
    
    return result;
}

#endif // TIMESPEC_H
