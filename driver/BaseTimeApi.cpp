/******************************************************************************
 * Copyright (C)
 * File name:       BaseTimeApi.cpp
 * Version:         
 * Description:    support time and timer function
 * Other:
 * History:
 ******************************************************************************/


#include "BaseTimeApi.h"
 
#ifdef ECO_WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <time.h>
#include <sys/types.h>
#include <unistd.h>
#endif
 
/*****************************************************************************
MicroDefine And Struct
*****************************************************************************/

/*****************************************************************************
Global
*****************************************************************************/


/*****************************************************************************
Function
*****************************************************************************/


/***************************************************************
Description:
Input:
Output:
Return
****************************************************************/
void BaseTimeSleep(U32 u32SleepMS)
{
#ifdef ECO_WIN32
    Sleep(u32SleepMS);
#else
    usleep(u32SleepMS * 1000);
#endif
}
 
/***************************************************************
Description:   get system tick, (ms unit)
Input:
Output:
Return
****************************************************************/
S32 BaseGetTimeTick()
{
    S32 s32TickNow = 0;

#ifdef ECO_WIN32
    s32TickNow = GetTickCount();
#else
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);//MONOTONIC
    s32TickNow = ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}


#endif
    return s32TickNow;
}
 
 
 
