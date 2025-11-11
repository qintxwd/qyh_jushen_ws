#ifndef _COMMON_H_
#define _COMMON_H_

#define ASSERT_TRUE_OR_EXIT(expr, msg) \
    do { \
        if (!(expr)) \
        { \
            std::cout << (msg) << " fail:" << __LINE__ << std::endl; \
            std::flush(std::cout); \
            exit(EXIT_FAILURE); \
        } \
    } while (0)

#define ASSERT_TRUE_OR_LOG(expr, msg) \
    do { \
        if (!(expr)) \
        { \
            std::cout << (msg) << " fail:" << __LINE__ << std::endl; \
            std::flush(std::cout); \
        } \
    } while (0)

#include "jktypes.h"

#define IP "10.5.5.100"

class JAKAZuRobot;

int check_error(JAKAZuRobot &robot, bool need_reset);

int check_state(JAKAZuRobot &robot);

int check_inpos(JAKAZuRobot &robot);

#endif // _COMMON_H_