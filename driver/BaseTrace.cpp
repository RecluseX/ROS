/******************************************************************************
 * Copyright (C)
 * File name:       BaseTrace.cpp
 * Version:         
 * Description:    support trace function
 * Other:
 * History:
 ******************************************************************************/
 #ifdef ECO_WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#include <time.h>
#include "pthread.h"
#else
#include <time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <stdarg.h>
#include <pthread.h>

#endif

#include "BaseTrace.h"
#include "BaseTimeApi.h"
#include "BaseSocketApi.h"


#ifdef ECO_WIN32
#define ECOsnprintf _snprintf
#else
#define ECOsnprintf snprintf
#endif
/*****************************************************************************
MicroDefine And Struct
*****************************************************************************/
#define MAX_TRACE_MODULER_NUM  (16)
#define MAX_MODULE_NAME_LENGTH (32)

typedef struct BaseTraceInfo
{
    S32 u32LogServerSockFd;
    U32 u32LogServerAddr;
    U32 u32LogServerPort;
    U32 au32ModuleID[MAX_TRACE_MODULER_NUM];
    BaseTraceLevel_E aenTraceLevel[MAX_TRACE_MODULER_NUM];
    S8   au8ModuleName[MAX_TRACE_MODULER_NUM][MAX_MODULE_NAME_LENGTH];
}BaseTraceInfo_S;


/*****************************************************************************
Global
*****************************************************************************/
STATIC BaseTraceInfo_S gbstTraceInfo;

/** used for lock */
volatile U32 volu32LockFlag = 0;

/** used for init Trace once */
volatile S32 vols32Inited = 0;
pthread_mutex_t g_trace_mutex;

/*****************************************************************************
Function
*****************************************************************************/
string SysTime()
{
	char tmpbuf[128];
	time_t timer;
	struct tm *newtime;

	timer = time(NULL);
	newtime = localtime(&timer);

	strftime( tmpbuf, 128, "%Y-%m-%d %H:%M:%S", newtime); 
	return tmpbuf; 
}

/***************************************************************
Description:
Input:
Output:
Return
****************************************************************/
void TraceLock()
{
#if 0    
    while(1)
    {
        if(0 == volu32LockFlag)
        {
            volu32LockFlag = 1;
            break;
        }
    }
#else
    pthread_mutex_lock(&g_trace_mutex);
#endif
    return;
}

void TraceRelease()
{
#if 0
    volu32LockFlag = 0;
#else
    pthread_mutex_unlock(&g_trace_mutex);
#endif
}


/***************************************************************
Description:
Input:
Output:
Return
****************************************************************/
U32 BaseTraceInit(U32 u32LogServerAddr, U16 u16LogServerPort)
{
    U32 i;
    S32 s32SendSockfd = -1;

    if (0 != vols32Inited)
    {
        return 0;
    }

    /** reset */
    memset(&gbstTraceInfo, 0x00, sizeof(BaseTraceInfo_S));
    for(i = 0; i < MAX_TRACE_MODULER_NUM; i++)
    {
        gbstTraceInfo.au32ModuleID[i] = 0xFFFF;
    }

    /** save server info */
    gbstTraceInfo.u32LogServerAddr = u32LogServerAddr;
    gbstTraceInfo.u32LogServerPort  = (U32)u16LogServerPort;
    gbstTraceInfo.u32LogServerSockFd = -1;
    
    /** connect with logserver */
    s32SendSockfd = BaseSocketUdpSendCreate(u32LogServerAddr, u16LogServerPort);
    if(s32SendSockfd < 0)
    {
        return 1;
    }
    gbstTraceInfo.u32LogServerSockFd = s32SendSockfd;
    /**********************/

    pthread_mutex_init(&g_trace_mutex, NULL);
    vols32Inited = 1;

    return 0;
}

/***************************************************************
Description:
Input:
Output:
Return
****************************************************************/
U32 BaseTraceRelease()
{

    /** close with logserver */
    if(gbstTraceInfo.u32LogServerSockFd >= 0)
    {
#ifdef ECO_WIN32
        closesocket(gbstTraceInfo.u32LogServerSockFd);
#else
        close(gbstTraceInfo.u32LogServerSockFd);
#endif
    }
    
    /**********************/

    
    return 0;
}

/***************************************************************
Description:
Input:
Output:
Return
****************************************************************/
U32 BaseTraceRegister(const S8 *pu8ModuleName, U32 *pU32ModuleID, 
    BaseTraceLevel_E enTraceLevel)
{   
    U32 i = 0;

    if(NULL == pu8ModuleName)
    return 0;

    TraceLock();

    /** find a free register */
    while(i < MAX_TRACE_MODULER_NUM)
    {
        if (0xFFFF == gbstTraceInfo.au32ModuleID[i])
            break;
        i++;
    }

    if (i >= MAX_TRACE_MODULER_NUM)
    {
        TraceRelease();
        return 2;
    }

    gbstTraceInfo.au32ModuleID[i] = i;

    strncpy((S8 *)gbstTraceInfo.au8ModuleName[i], (S8 *)pu8ModuleName, MAX_MODULE_NAME_LENGTH);
    gbstTraceInfo.aenTraceLevel[i] = enTraceLevel;

    *pU32ModuleID = i;

    TraceRelease();

    return 0;
}

/***************************************************************
Description:
Input:
Output:
Return
****************************************************************/
void BaseTraceOut(U32 u32TraceID, BaseTraceLevel_E enTraceLev, const S8 *fmt, ...)
{
    S8 au8Tmpbuffer[4096];	
    S8 au8Tmpbuffer1[4096];	
    char au8LogLevel[16];
    va_list argptr;

    if(u32TraceID >= MAX_TRACE_MODULER_NUM)
    {
        return;
    }

    /** send to log server */
    if(enTraceLev <= gbstTraceInfo.aenTraceLevel[u32TraceID])
    {
        /** parse input */
        va_start(argptr, fmt);
        vsprintf(au8Tmpbuffer, fmt, argptr);
        va_end(argptr);
        
        memset(au8Tmpbuffer1, '\0', 1024);
        TraceLock();

		memset(au8LogLevel, 0, sizeof(au8LogLevel));

		/* BEGIN: Modified by mingming.xin, 2015/9/14 */
		TransferTraceLevel_Enum2Str(enTraceLev, au8LogLevel);
		/* END:   Modified by mingming.xin, 2015/9/14 */
#if 1
        printf("%s [%s] %s, %s", 
           SysTime().c_str(), au8LogLevel, gbstTraceInfo.au8ModuleName[u32TraceID], au8Tmpbuffer);

//#else
        /** add module name */
#ifdef ECO_WIN32
        _snprintf(au8Tmpbuffer1, 1024, "%s [%s] %s, %s", 
            SysTime().c_str(), au8LogLevel, gbstTraceInfo.au8ModuleName[u32TraceID], au8Tmpbuffer);
#else
        snprintf(au8Tmpbuffer1, 1024, "%s [%s] %s, %s", 
           SysTime().c_str(), au8LogLevel, gbstTraceInfo.au8ModuleName[u32TraceID], au8Tmpbuffer);
#endif


    /** send to log server */
    BaseSocketUdpPacketSend(gbstTraceInfo.u32LogServerSockFd,
            (U16)(gbstTraceInfo.u32LogServerPort),
            gbstTraceInfo.u32LogServerAddr,
            (U8 *)au8Tmpbuffer1, strlen(au8Tmpbuffer1));   
#endif

        TraceRelease();

    }

    return;
}


U32 BaseTraceLogServerDataProc(FILE *fp, U8 *pu8Data, U32 u32DataLen)
{
    struct tm *tblock;
    time_t cur_time;

    u32DataLen = u32DataLen;
#ifdef ECO_WIN32
    cur_time = time(NULL);    
#else
    time(&cur_time);
#endif
#if 0
    tblock=localtime(&cur_time);        
	fprintf(fp, "%4d:%2d:%2d:%2d:%2d:%2d        ",             
        tblock->tm_year + 1900, 
        tblock->tm_mon + 1, 
        tblock->tm_mday,
        tblock->tm_hour, 
        tblock->tm_min, 
        tblock->tm_sec);
#endif
    fprintf(fp, "%s", pu8Data);

    return 0;
}
U32 BaseTraceLogServerMain(U16 u16Port, U32 u32IPAddr)
{
    S32 s32Sockfd;
    U32 u32RecvLen;
    S32 s32RecvLen;
    struct sockaddr_in stFrom;
    S32 s32FromLen = sizeof(stFrom);
    fd_set            sockFdSet;
    struct timeval    timeInterval;
    S32 s32MaxSockFd, s32Ret;
    
    FILE *fp;
    U8   au8RecvData[1500];
    
    S32 s32FlushTick, s32TickNow;
    S8 u8LogFileName[4][128];
    U8 u8FileIndex;
    U32 u32FileLen;

    struct tm *tblock;
    time_t cur_time;
	
    /** create socket */
    s32Sockfd = BaseSocketUdpRecvCreate(u16Port, u32IPAddr);
    if(s32Sockfd < 0)
    {
        return 1;
    }

    /** create log file */
    memset(u8LogFileName, '\0', 4 * 128);
    
    
    cur_time = time(NULL);    
    tblock=localtime(&cur_time);    
    
    memset(u8LogFileName[0], '\0', 128);
    ECOsnprintf(u8LogFileName[0], 128, "TRACE-%d-%d-%d-%d-%d-%d.log",             
        tblock->tm_year + 1900, 
        tblock->tm_mon + 1, 
        tblock->tm_mday,
        tblock->tm_hour, 
        tblock->tm_min, 
        tblock->tm_sec);                    
    fp = fopen(u8LogFileName[0], "wb");
    if(NULL == fp)
    {
        return 2;
    }
    u8FileIndex = 1;

    /** select */
    timeInterval.tv_sec   = 0;
    timeInterval.tv_usec = 1000;
    s32MaxSockFd         = s32Sockfd + 1;

    s32FlushTick = BaseGetTimeTick();
    while(1)
    {
        FD_ZERO(&sockFdSet);
        FD_SET(s32Sockfd, &sockFdSet);
    
        /** nonblock wait for incoming */  
        s32Ret = select(s32MaxSockFd, &sockFdSet, NULL, NULL, &timeInterval);
        if(s32Ret > 0) 
        {
            if(FD_ISSET(s32Sockfd, &sockFdSet))
            {
                s32RecvLen = recvfrom(s32Sockfd, (S8 *)au8RecvData, 1500, 0, 
                    (struct sockaddr*)&stFrom, (socklen_t *)&s32FromLen);
                if(s32RecvLen > 0)
                {
                    au8RecvData[s32RecvLen] = '\0';
                    BaseTraceLogServerDataProc(fp, au8RecvData, s32RecvLen);
                }
            }
        }

        /** check file flush */
        s32TickNow = BaseGetTimeTick();
        if(s32TickNow - s32FlushTick > 10000)
        {
            /** flush file 10 seconds */
            fflush(fp);
            s32FlushTick = s32TickNow;
        }
        /** create new log file if bigger than 32MB */
        u32FileLen = ftell(fp);
        if(u32FileLen >= 32 * 1024 * 1024)
        {
            /** close old log file */
            fclose(fp);
            
            /** create log file */
            cur_time = time(NULL);    
            tblock=localtime(&cur_time);    
            memset(u8LogFileName[u8FileIndex % 4], '\0', 128);
            ECOsnprintf(u8LogFileName[u8FileIndex % 4], 128, "TRACE-%d-%d-%d-%d-%d-%d.log",             
                tblock->tm_year + 1900, 
                tblock->tm_mon + 1, 
                tblock->tm_mday,
                tblock->tm_hour, 
                tblock->tm_min, 
                tblock->tm_sec);                    
            fp = fopen(u8LogFileName[u8FileIndex % 4], "wb");
            if(NULL == fp)
            {
                return 3;
            }
            
            u8FileIndex++;
            
            /** delete log file */
            if(strlen(u8LogFileName[u8FileIndex % 4]) != 0)
            {
                ///printf("u8FileIndex = %d\n", u8FileIndex);
                ///can only do 3 files loop, why???
#ifdef ECO_WIN32
                DeleteFileA(u8LogFileName[u8FileIndex % 4]);
#else
            
                remove(u8LogFileName[u8FileIndex % 4]);
#endif
            }
            /***************/            
        }
        
        
    }

       
    return 0;
}

BaseTraceLevel_E BaseTrace_IntTranslateToEnum(int level)
{
	switch(level)
	{
		case 0: return BASE_TRACE_NO;
		case 1: return BASE_TRACE_CRIT;
		case 2: return BASE_TRACE_ERR;
		case 3: return BASE_TRACE_WARN;
		case 4: return BASE_TRACE_NOTICE;
		case 5: return BASE_TRACE_INFO;
		case 6: return BASE_TRACE_DEBUG;
		case 7: return BASE_TRACE_EMERG;
		case 8: return BASE_TRACE_ALERT;
		default: return BASE_TRACE_NO;
	}
}


int TransferTraceLevel_Enum2Str(BaseTraceLevel_E eLevel, char *au8LogLevel)
{
	if(NULL == au8LogLevel)
		return 2;

	if((eLevel < BASE_TRACE_CRIT) || (eLevel > BASE_TRACE_NO))
	{
		memcpy(au8LogLevel, "NO", strlen("NO")); 
		return 1;
	}	

	
	switch(eLevel)
	{
		case BASE_TRACE_CRIT:  
		{	
			memcpy(au8LogLevel, "CRIT", strlen("CRIT")); 
			break;
		}
		case BASE_TRACE_ERR: 
		{	
			memcpy(au8LogLevel, "ERROR", strlen("ERROR")); 
			break;
		}
		case BASE_TRACE_WARN: 
		{	
			memcpy(au8LogLevel, "WARN", strlen("WARN")); 
			break;
		}
		case BASE_TRACE_NOTICE:
		{	
			memcpy(au8LogLevel, "NOTICE", strlen("NOTICE")); 
			break;
		}
		case BASE_TRACE_INFO:
		{	
			memcpy(au8LogLevel, "INFO", strlen("INFO")); 
			break;
		}
		case BASE_TRACE_DEBUG: 
		{	
			memcpy(au8LogLevel, "DEBUG", strlen("DEBUG")); 
			break;
		}
		case BASE_TRACE_EMERG:
		{	
			memcpy(au8LogLevel, "EMERG", strlen("EMERG")); 
			break;
		}
		case BASE_TRACE_ALERT:
		{	
			memcpy(au8LogLevel, "ALERT", strlen("ALERT")); 
			break;
		}
		case BASE_TRACE_NO:
		default:
		{	
			memcpy(au8LogLevel, "NO", strlen("NO")); 
			break;
		}
	}

	return 0;
}

