#ifndef _MCU_HEAD_H_
#define _MCU_HEAD_H_

#include <pthread.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/types.h>
#include <unistd.h>
#include <sys/stat.h>
#include "BenebotTypeDefine.h"

#define SERIAL_COM_BAUDRATE	(115200)
#define DATA_BITS		(8)
#define PARITY			('N')
#define STOP_BIT		(1)

#define MAX_BUF			(64)
/*****************************************************************************
 MicroDefine And Struct
 *****************************************************************************/
class MCUBridge
{
public:
    MCUBridge();
    ~MCUBridge();

/*****************************************************************************
 Function
 *****************************************************************************/
    S32 ApplicationMCUStart(const string ttyDevPath);
    static void *ApplicationMCURun(void *arg);
    S32 ApplicationMCUMainLoop();

    S32 SendMoveForward(S32 s32Speed, S32 s32Accleration);
    S32 SendMoveBack(S32 s32Speed, S32 s32Accleration);
    S32 SendMoveLeft(S32 s32Speed, S32 s32Accleration);
    S32 SendMoveRight(S32 s32Speed, S32 s32Accleration);
    S32 SendMoveStop(S32 s32Speed, S32 s32Accleration);
    S32 SendMicroWaveCMD(U32 u32Time);
    S32 SendObliqueWaveCMD(U32 u32Time);

    S32 GYIODistanceReset(U32 u32NeedIndex);

    S32 NavyMCUMoveCtrl(S16 s16LSpeed, S16 s16RSpeed, S16 s16LAcce, S16 s16RAcce);
    bool GetOdomData(Odom_Type* data);
private:

/*****************************************************************************
 Function
 *****************************************************************************/
    S32 MCUSendMoveCtrl(S16 s16LSpeed, S16 s16RSpeed, S16 s16LAcce, S16 s16RAcce);
    S32 MCUSendMoveCtrl_1(S16 s16LSpeed, S16 s16RSpeed, S16 s16LAcce, S16 s16RAcce);

    S32 ConfigSerialCom(S32 fd, U32 baud_rate, U8 data_bits, U8 parity, U8 stop_bits);
    S32 OnMCUSerialComRead();
    S32 OnMCUSerialComWrite(U8 *data, U16 len);
    S32 OnMCUMessageReceive();


    pthread_t threadHandler;
    pthread_mutex_t MCUComMutex;   ///for com communication
    pthread_mutex_t MCUCMDMutex;   ////for command control
    pthread_mutex_t MoveCtrlMutex;   ////for command control

    pthread_mutex_t dataReadyMutex;
    bool dataReadyFlag_;
    S32 s32StopMCULoopThread;

    bool mSetWheelSpeedFlag;
    bool mSetDisIntervalFlag;
    bool mResetWheelDisFlag;

    S32 ttyFd;
    U8 idNum;
    U8 RxBuf[MAX_BUF];
    U8 mRxData[MAX_BUF];
    U8 mHeadFlag;

    /** GYRO & Distance */
    U32 u32ClientNeedIndex;
    S32 mLdistance;
    S32 mRdistance;
    U32 mGyroData;
    Odom_Type odom_;

    /** move info **/
    S16 s16LastLSpeedRsp;
    S16 s16LastRSpeedRsp;
    S16 s16LastLSpeedCMD;
    S16 s16LastRSpeedCMD;
    S16 s16LastLAcceCMD;
    S16 s16LastRAcceCMD;
    bool bMoveCtrlNeedAdd;

    S32 s32LastBompTick;  ////saving last bomp time ,ms

};

#endif
