/******************************************************************************
 * Copyright (C)
 * File name:       
 * Version:         
 * Description:    
 * Other:
 * History:
 
 ******************************************************************************/

#include "ECOAppSysTypeDefine.h"
#include "ECOAppSysGeneric.h"
#include "ApplicationMCUMain.h"

#include "BaseTimeApi.h"
#include <pthread.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/types.h>
#include <unistd.h>
#include <sys/stat.h>

/*****************************************************************************
 MicroDefine And Struct
 *****************************************************************************/


/*****************************************************************************
 Global
 *****************************************************************************/


/*****************************************************************************
 Function
 *****************************************************************************/

/////////////////// crc 8 ///////////////////////
#define GP  0x107   /* x^8 + x^2 + x + 1 */
#define DI  0x07

static S8 crc8_table[256];     /* 8-bit table */
static S32 made_table = 0;


/*
 * Should be called before any other crc function.
 */
void init_crc8()
{
    S32 i,j;
    U8 crc;
    if (!made_table)
    {
        for (i = 0; i < 256; i++)
        {
            crc = i;
            for (j = 0; j < 8; j++)
            crc = (crc << 1) ^ ((crc & 0x80) ? DI : 0);
            crc8_table[i] = crc & 0xFF;
            /* printf("table[%d] = %d (0x%X)\n", i, crc, crc); */
        }
        made_table = 1;
    }
}

/*
 * For a byte array whose accumulated crc value is stored in *crc, computes
 * resultant crc obtained by appending m to the byte array
 */
void crc8(U8 *crc, U8 m)
{
    if (!made_table)
    {
        init_crc8();
    }
    *crc = crc8_table[(*crc) ^ m];
    *crc &= 0xFF;
}

void getStrCrc8(U8 *crc, U8 *str, U32 length)
{
    U32 i;
	if((NULL == crc) || (NULL == str))
	{
		return;
	}
	
    for(i = 0; i < length; i++)
    {
        crc8(crc , *(str + i));
    }
}
/***************************************************************
Description:
Input:
Output:
Return
****************************************************************/
S32 CheckCRC(U8 *data, U32 len)
{
    U8 crc = 0;
    getStrCrc8(&crc, data, len);

    if (crc != data[len])
    {
        return -1;
    }

    return 0;
}

/***************************************************************
Description:
Input:
Output:
Return
****************************************************************/
ApplicationMCUMain::ApplicationMCUMain()
{
    pthread_mutex_init(&MCUComMutex, NULL);
    pthread_mutex_init(&MCUCMDMutex, NULL);
    pthread_mutex_init(&MoveCtrlMutex, NULL);
    
    mSetWheelSpeedFlag = false;
    mSetDisIntervalFlag = false;
    mResetWheelDisFlag = false;
    idNum = 0;

    u32ClientNeedIndex = 0;    

    s16LastLSpeedRsp = 0;
    s16LastRSpeedRsp = 0;
    s16LastLSpeedCMD = 0;
    s16LastRSpeedCMD = 0;
    bMoveCtrlNeedAdd = false;
    
}

ApplicationMCUMain::~ApplicationMCUMain()
{
    pthread_mutex_destroy(&MCUComMutex);
    pthread_mutex_destroy(&MCUCMDMutex);
    pthread_mutex_destroy(&MoveCtrlMutex);
    close(ttyFd);
}

S32 ApplicationMCUMain::ConfigSerialCom(S32 fd, U32 baud_rate, 
    U8 data_bits, U8 parity, U8 stop_bits)
{
	struct termios new_cfg, old_cfg;
	int speed;

	/* ���沢�������д��ڲ������� */
	if (tcgetattr(fd, &old_cfg) != 0) {
		perror("tcgetattr");
		return -1;
	}

	new_cfg = old_cfg;
	cfmakeraw(&new_cfg); /* ����Ϊԭʼģʽ */
	new_cfg.c_cflag &= ~CSIZE;

	/* ���ò����� */
	switch (baud_rate) {
	case 2400:
		speed = B2400;
		break;
	case 4800:
		speed = B4800;
		break;
	case 9600:
		speed = B9600;
		break;
	case 19200:
		speed = B19200;
		break;
	case 38400:
		speed = B38400;
		break;

	default:
	case 115200:
		speed = B115200;
		break;
	}
	cfsetispeed(&new_cfg, speed);
	cfsetospeed(&new_cfg, speed);

	/* ��������λ */
	switch (data_bits){
	case 7:
		new_cfg.c_cflag |= CS7;
		break;

	default:
	case 8:
		new_cfg.c_cflag |= CS8;
		break;
	}

	/* ������żУ��λ */
	switch (parity) {
	default:
	case 'n':
	case 'N':
		new_cfg.c_cflag &= ~PARENB;
		new_cfg.c_iflag &= ~INPCK;
		break;

	case 'o':
	case 'O':
		new_cfg.c_cflag |= (PARODD | PARENB);
		new_cfg.c_iflag |= INPCK;
		break;

	case 'e':
	case 'E':
		new_cfg.c_cflag |= PARENB;
		new_cfg.c_cflag &= ~PARODD;
		new_cfg.c_iflag |= INPCK;
		break;

	case 's': /* as no parity */
	case 'S':
		new_cfg.c_cflag &= ~PARENB;
		new_cfg.c_cflag &= ~CSTOPB;
		break;
	}

	/* ����ֹͣλ */
	switch (stop_bits) {
	default:
	case 1:
		new_cfg.c_cflag &= ~CSTOPB;
		break;

	case 2:
		new_cfg.c_cflag |= CSTOPB;
		break;
	}

	/* ���õȴ�ʱ�����С�����ַ� */
	new_cfg.c_cc[VTIME] = 0;
	new_cfg.c_cc[VMIN] = 1;
	tcflush(fd, TCIFLUSH); /* ����δ�����ַ� */

	/* ���������� */
	if ((tcsetattr(fd, TCSANOW, &new_cfg)) != 0) {
		perror("tcsetattr");
		return -1;
	}
	return 0;
}

/***************************************************************
Description:
Input:
Output:
Return
****************************************************************/
S32 ApplicationMCUMain::ApplicationMCUStart(const string ttyDevPath)
{
    S32 s32Ret;
    
    //BaseTraceOut(g_u32MCUTraceID, BASE_TRACE_INFO,
      //  "ApplicationMCUStart In.\n");
    /** Init UART Communication **/
    ttyFd = open((S8 *)ttyDevPath.c_str(), O_RDWR | O_NOCTTY);
    if (-1 == ttyFd)
    {
        //BaseTraceOut(g_u32MCUTraceID, BASE_TRACE_ERR,
           // "ApplicationMCUStart ApplicationMCURun open %s Error\n", (S8 *)ttyDevPath.c_str());
        return 1;
    }

    s32Ret = ConfigSerialCom(ttyFd, SERIAL_COM_BAUDRATE, DATA_BITS, PARITY, STOP_BIT);
    if (0 != s32Ret)
    {
       // BaseTraceOut(g_u32MCUTraceID, BASE_TRACE_ERR,
   //         "ApplicationMCUStart ApplicationMCURun Config RS232 Error\n");
        return 2;
    }
        
    /// Create thread for ApplicationHostRun.
    s32Ret = pthread_create((pthread_t*)&threadHandler, NULL, ApplicationMCURun, this);
    if(0 != s32Ret)
    {        
        //BaseTraceOut(g_u32MCUTraceID, BASE_TRACE_ERR,
           // "ApplicationMCUStart ApplicationMCURun Create Error.\n");
        return 3;
    }

   // BaseTraceOut(g_u32MCUTraceID, BASE_TRACE_INFO,
      //  "ApplicationMCUStart Leave.\n");
    
    return 0;
}

/***************************************************************
Description:
Input:
Output:
Return
****************************************************************/
void *ApplicationMCUMain::ApplicationMCURun(void *arg)
{
    ApplicationMCUMain *pclMCUHost;

    /// set thread parameter.
    pthread_detach(pthread_self());

    pclMCUHost = reinterpret_cast<ApplicationMCUMain *>(arg);

    //BaseTraceOut(g_u32MCUTraceID,
       // BASE_TRACE_INFO, "Thread <ApplicationMCURun> is Running ...\n\n");

    pclMCUHost->ApplicationMCUMainLoop();

    //BaseTraceOut(g_u32MCUTraceID,
       // BASE_TRACE_INFO, "Thread <ApplicationMCURun> is End !!!\n\n");

    return NULL;
}

/***************************************************************
Description:
Input:
Output:
Return
****************************************************************/
S32 ApplicationMCUMain::ApplicationMCUMainLoop()
{
    S8 command, type, response;
    s32StopMCULoopThread = 0;

    bool bflag = false;
    U32 u32WENum = 0;
    U32 u32GBNum = 0;
    bool bBompFlag = false;
    U16 u16BompBlockInfo = 0;

    s32LastBompTick = 0;
    
    while (!s32StopMCULoopThread)
    {
        if (0 != OnMCUMessageReceive())
        {
            continue;
        }
        command = 0;
        type = 0;

        command = mRxData[6];
        type = mRxData[7];

        ///if('V' == command)
        ///    printf("Get Com Data From MCU, CMD = %c, Type = %c \n", command, type);
        if(!bflag)
        {

            SendMicroWaveCMD(1000);
            SendObliqueWaveCMD(1000);
            bflag = true;
        }
        
        pthread_mutex_lock(&MCUCMDMutex);

        switch (command) {
            case 'W':
                response = 0;
                response = mRxData[8];
                switch (type) {
                case 'A':
                    printf("Type = %c, Response = %d\n", type, response);
                    
                    if (response == 0) {
                        mSetWheelSpeedFlag = true;
                    }
                    break;
                case 'D':
                    if (response == 0) {
                        mSetDisIntervalFlag = true;
                        if (mRxData[2] == 14) {
                            mLdistance = mRxData[9] | (mRxData[10] << 8) |
                                         (mRxData[11] << 16) | (mRxData[12] << 24);
                            mRdistance = mRxData[13] | (mRxData[14] << 8) |
                                         (mRxData[15] << 16) | (mRxData[16] << 24);
                        }
                    }
                    break;
                case 'E':
                    mLdistance = mRxData[8] | (mRxData[9] << 8) |
                                 (mRxData[10] << 16) | (mRxData[11] << 24);
                    mRdistance = mRxData[12] | (mRxData[13] << 8) |
                                 (mRxData[14] << 16) | (mRxData[15] << 24);
                    //save gyro distance
         //           g_pclApplicationMain->pclApplicationScheduler->GYIODistanceProc(
            //            u32ClientNeedIndex, mGyroData, mLdistance, mRdistance);
                    u32WENum++;

                    break;
                case 'H':
                    {
                        s16LastLSpeedRsp = mRxData[8] | (mRxData[9] << 8);
                        s16LastRSpeedRsp = mRxData[10] | (mRxData[11] << 8);
                        printf("LSpeed = %d, RSpeed = %d\n", s16LastLSpeedRsp, s16LastRSpeedRsp);

                        //speed notify xjtao
                        //g_pclApplicationMain->pclApplicationScheduler->SpeedNotifyProc(s16LastLSpeedRsp, s16LastRSpeedRsp);

						if(bMoveCtrlNeedAdd)
                        {
                            MCUSendMoveCtrl(s16LastLSpeedCMD, s16LastRSpeedCMD, 
                                s16LastLAcceCMD, s16LastRAcceCMD);
                        }                        
                    }
                case 'F':
                    if (response == 0) {
                        mResetWheelDisFlag = true;
                    }
                    break;
                default:
                    break;
            }
                break;
            case 'G':
                if('B' == type)
                {
                    mGyroData = mRxData[8] | (mRxData[9] << 8);
                    u32GBNum++;        
                    ////printf("distance = %d, %d, mGyroData = %d\n", mLdistance, mRdistance, mGyroData);
                }
                break;
            case 'U':
                if('B' == type)
                {
                    U16 u16Dis1, u16Dis2, u16Dis3;
                    u16Dis1 = mRxData[8] + (mRxData[9]<<8);
                    u16Dis2 = mRxData[10] + (mRxData[11]<<8);
                    u16Dis3 = mRxData[12] + (mRxData[13]<<8);
              //      g_pclApplicationMain->pclApplicationScheduler->WaveSensorDistanceProc(u16Dis1, u16Dis2, u16Dis3);
                    //printf("%d, %d, %d, %d, %d\n", u16Dis1, u16Dis2, u16Dis3, u32WENum, u32GBNum);
                }
                break;
            case 'V':
                if('C' == type)
                {
                    U16 u16Dis1;
                    u16Dis1 = mRxData[8] + (mRxData[9]<<8);

            //		g_pclApplicationMain->pclApplicationScheduler->ObliqueInfoProc(u16Dis1);
                    ////printf("%d, %d, %d\n", u16Dis1, u32WENum, u32GBNum);
                }
                break;                
            case 'B':
                if('B' == type)
                {
                    u16BompBlockInfo  = mRxData[8] + (mRxData[9]<<8);
                    bBompFlag = true;
                    printf("GetBompInfo [0x%04x]\n", u16BompBlockInfo );
                }
                break;   
			case 'H':
				if('A' == type) 
				{
					U8 u8HumanInfo;
					u8HumanInfo = mRxData[8];
        //			g_pclApplicationMain->pclApplicationScheduler->HumanSensorInfoProc(u8HumanInfo);
				}
            default:
                break;
        }
        
        pthread_mutex_unlock(&MCUCMDMutex);


        if(bBompFlag)
        {
            bBompFlag = false;   ////reset            
            s32LastBompTick = BaseGetTimeTick();
            if(g_pclApplicationMain->pclAppConfig->stSysConfig.u32BompActionFlag)
            {
                ////Start Break Stop, only for test 
                SendMoveStop(0, 1000);
            }

            /** Send Info to Application **/
            g_pclApplicationMain->pclApplicationScheduler->BompActionProc(u16BompBlockInfo);
        }


    }

    return 0;
}
/***************************************************************
Description:
Input:
Output:
Return
****************************************************************/
S32 ApplicationMCUMain::OnMCUSerialComRead()
{
    U16 rcount;
    U8 *rx = RxBuf;
    S32 ret;
    rcount = 0;	
    
    memset(RxBuf, 0, MAX_BUF);
    mHeadFlag = 0;
    
    do {
        if (rcount == MAX_BUF - 1) {
            rcount = 0;
            memset(RxBuf, 0, MAX_BUF);
            rx = RxBuf;
            //printf("rcount == MAX_BUF\n");
            continue;
        }
 
        ret = read(ttyFd, rx++, 1);
        //printf("ret = %d, %x\n", ret, RxBuf[rcount]);
        
        if (RxBuf[0] == 0xaa) {
            mHeadFlag++;
        } else {
            mHeadFlag = 0;
            rx = RxBuf;
            continue;
        }

        if (mHeadFlag == 2) {
            if (RxBuf[1] != 0xcc) {
                mHeadFlag = 0;
                RxBuf[0] = 0;
                rx = RxBuf;
                continue;
            } else {
            }
        }

        if ((RxBuf[rcount] == '\r') && (rcount > 10)) {
            if (RxBuf[rcount - 1] == '\n') {
                break;
            }
        }
        rcount++;
        if (rcount > 30) {
            return -1;
        }
    } while (1);
    
    return (rcount + 1);
}
/***************************************************************
Description:
Input:
Output:
Return
****************************************************************/
S32 ApplicationMCUMain::OnMCUSerialComWrite(U8 *data, U16 len)
{
    U32 i;
    int ret;
    pthread_mutex_lock(&MCUComMutex);
    for (i = 0; i < len; i++)
    {
        ret = write(ttyFd, &data[i], 1);
        if(ret != 1)
            {
                printf("OnMCUSerialComWriteErr, ret = %d\n", ret);
            }
    }
    
    pthread_mutex_unlock(&MCUComMutex);

    return 0;
}
/***************************************************************
Description:
Input:
Output:
Return
****************************************************************/
S32 ApplicationMCUMain::OnMCUMessageReceive()
{
    S32 s32Ret, len;

    memset(RxBuf, 0, MAX_BUF);
    memset(mRxData, 0, sizeof(mRxData));
    
    s32Ret = OnMCUSerialComRead();
    if (-1 == s32Ret)
    {
        return 1;		
    }

    len = RxBuf[2] | (RxBuf[3] << 8);
	if((8 > len) || (MAX_BUF < len))
	{
		return 2;
	}
	
    memcpy(mRxData, RxBuf, len);
    if (CheckCRC(mRxData + 4, len - 7))
    {
        return 3;
    }

    return 0;
}

/***************************************************************
Description:
Input:
Output:
Return
****************************************************************/
S32 ApplicationMCUMain::MCUSendMoveCtrl(S16 s16LSpeed, 
    S16 s16RSpeed, S16 s16LAcce, S16 s16RAcce)
{
    bool bChangedFlag = false;
    S32 s32Tick = BaseGetTimeTick();
    
#define SPEED_MOVE_THRESHOLD 300
#define SPEED_STOP_THRESHOLD 400

    /** check last bomp tick **/
    if(s32Tick - s32LastBompTick < 3000)
    {
        printf("Move Ctrl Cancel when Bomp coming\n");
        return 0;
    }
    
#if 0
    MCUSendMoveCtrl_1(s16LSpeed, s16RSpeed, s16LAcce, s16RAcce);
    return 0;
#else    
    pthread_mutex_lock(&MoveCtrlMutex);

    /*** saving last command **/
    s16LastLSpeedCMD = s16LSpeed;
    s16LastRSpeedCMD = s16RSpeed;
    s16LastLAcceCMD = s16LAcce;
    s16LastRAcceCMD = s16RAcce;
        
    ///BaseTraceOut(g_u32MCUTraceID,
       // BASE_TRACE_DEBUG, "MCUSendMoveCtrl, [%d], Last[%d], [%d], Now [%d], [%d] \n",
        //bMoveCtrlNeedAdd, s16LastLSpeedRsp, s16LastRSpeedRsp, s16LSpeed, s16RSpeed);

    if((abs(s16LastLSpeedRsp) < SPEED_MOVE_THRESHOLD) 
        && (abs(s16LastRSpeedRsp) < SPEED_MOVE_THRESHOLD))
    {
        bMoveCtrlNeedAdd = false;
        bChangedFlag = true;
        MCUSendMoveCtrl_1(s16LSpeed, s16RSpeed, s16LAcce, s16RAcce);
        //BaseTraceOut(g_u32MCUTraceID, BASE_TRACE_DEBUG, "MCUSendMoveCtrl 1.\n");
        goto MCUSendMoveCtrl_END;
    }
    
    /**** move between different direction *****/    
    if(abs(s16LastLSpeedRsp + s16LSpeed) < (abs(s16LastLSpeedRsp) + abs(s16LSpeed)))
    {
        bMoveCtrlNeedAdd = true;
        bChangedFlag = true;
        if(abs(s16LastLSpeedRsp) < SPEED_STOP_THRESHOLD)
        {
            SendMoveStop(0, 1000);
           // BaseTraceOut(g_u32MCUTraceID, BASE_TRACE_DEBUG, "MCUSendMoveCtrl 21.\n");
        }
        else
        {
            SendMoveStop(0, 600);            
            //BaseTraceOut(g_u32MCUTraceID, BASE_TRACE_DEBUG, "MCUSendMoveCtrl 22.\n");
        }        
        goto MCUSendMoveCtrl_END;
    }
    
    
    if(abs(s16LastRSpeedRsp + s16RSpeed) < (abs(s16LastRSpeedRsp) + abs(s16RSpeed)))
    {
        bMoveCtrlNeedAdd = true;
        bChangedFlag = true;        
        if(abs(s16LastRSpeedRsp) < SPEED_STOP_THRESHOLD)
        {
            //BaseTraceOut(g_u32MCUTraceID, BASE_TRACE_DEBUG, "MCUSendMoveCtrl 31.\n");
            SendMoveStop(0, 1000);
        }
        else
        {
            //BaseTraceOut(g_u32MCUTraceID, BASE_TRACE_DEBUG, "MCUSendMoveCtrl 32.\n");
            SendMoveStop(0, 600);            
        }
        goto MCUSendMoveCtrl_END;
    }

    
MCUSendMoveCtrl_END:    
    if(!bChangedFlag)
    {
        bMoveCtrlNeedAdd = false;        
        MCUSendMoveCtrl_1(s16LSpeed, s16RSpeed, s16LAcce, s16RAcce);
        //BaseTraceOut(g_u32MCUTraceID, BASE_TRACE_DEBUG, "MCUSendMoveCtrl 4.\n");
    }
    pthread_mutex_unlock(&MoveCtrlMutex);
#endif
    return 0;
}


/***************************************************************
Description:
Input:
Output:
Return
****************************************************************/
S32 ApplicationMCUMain::MCUSendMoveCtrl_1(S16 s16LSpeed, 
    S16 s16RSpeed, S16 s16LAcce, S16 s16RAcce)
{
    U8 crc = 0;
    S32 len;
    U8 SetSpeed[20] = {0xaa, 0xcc, 20, 0, 0, 1, 'W', 'A', 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0x0A, 0x0D};

    printf("MCUSendMoveCtrl, %d, %d, %d, %d\n", s16LSpeed, s16RSpeed, s16LAcce, s16RAcce);
    
    SetSpeed[5] = idNum++;
    SetSpeed[8] = 1;
    SetSpeed[9] = s16LSpeed & 0xFF;
    SetSpeed[10] = (s16LSpeed >> 8) & 0xFF;
    SetSpeed[11] = s16RSpeed & 0xFF;
    SetSpeed[12] = (s16RSpeed >> 8) & 0xFF;
    SetSpeed[13] = s16LAcce & 0xFF;
    SetSpeed[14] = (s16LAcce >> 8) & 0xFF;
    SetSpeed[15] = s16RAcce & 0xFF;
    SetSpeed[16] = (s16RAcce >> 8) & 0xFF;
    len = (SetSpeed[2] | SetSpeed[3] << 8);
    getStrCrc8(&crc, SetSpeed + 4, len - 7);
    SetSpeed[17] = crc;
    
    OnMCUSerialComWrite(SetSpeed, len);

    return 0;
}
/***************************************************************
Description:
Input:
Output:
Return
****************************************************************/
S32 ApplicationMCUMain::NavyMCUMoveCtrl(S16 s16LSpeed, 
    S16 s16RSpeed, S16 s16LAcce, S16 s16RAcce)
{
    if(s16LSpeed || s16RSpeed)
    {
        MCUSendMoveCtrl_1(s16LSpeed, s16RSpeed, s16LAcce, s16RAcce);
    }
    else
    {
        SendMoveStop(s16LSpeed, s16LAcce);
    }

    return 0;
}

/***************************************************************
Description:
Input:
Output:
Return
****************************************************************/
S32 ApplicationMCUMain::SendMoveForward(S32 s32Speed, S32 s32Accleration)
{
    S16 s16Speed, s16Acce;
    
    s16Speed = s32Speed;
    s16Acce = s32Accleration;
    
    MCUSendMoveCtrl(s16Speed, s16Speed, s16Acce, s16Acce);

    return 0;
}

/***************************************************************
Description:
Input:
Output:
Return
****************************************************************/
S32 ApplicationMCUMain::SendMoveBack(S32 s32Speed, S32 s32Accleration)
{
    S16 s16Speed, s16Acce;
    
    s16Speed = -s32Speed;
    s16Acce = s32Accleration;

    MCUSendMoveCtrl(s16Speed, s16Speed, s16Acce, s16Acce);

    return 0;
}

/***************************************************************
Description:
Input:
Output:
Return
****************************************************************/
S32 ApplicationMCUMain::SendMoveLeft(S32 s32Speed, S32 s32Accleration)
{
    S16 s16LSpeed, s16LAcce;
    S16 s16RSpeed, s16RAcce;
    
    s16LSpeed = -s32Speed;
    s16RSpeed = s32Speed;    
    ///s16LAcce = s32Accleration;
    ///s16RAcce = s32Accleration;
    s16LAcce = 300;
    s16RAcce = 300;     
    MCUSendMoveCtrl(s16LSpeed, s16RSpeed, s16LAcce, s16RAcce);
    

    return 0;
}
/***************************************************************
Description:
Input:
Output:
Return
****************************************************************/
S32 ApplicationMCUMain::SendMoveRight(S32 s32Speed, S32 s32Accleration)
{
    S16 s16LSpeed, s16LAcce;
    S16 s16RSpeed, s16RAcce;
    
    s16LSpeed = s32Speed;
    s16RSpeed = -s32Speed;
    ///s16LAcce = s32Accleration;
    ///s16RAcce = s32Accleration;
    s16LAcce = 300;
    s16RAcce = 300;    
    MCUSendMoveCtrl(s16LSpeed, s16RSpeed, s16LAcce, s16RAcce);

    return 0;
}
/***************************************************************
Description:
Input:
Output:
Return
****************************************************************/
S32 ApplicationMCUMain::SendMoveStop(S32 s32Speed, S32 s32Accleration)
{
    U8 crc = 0;
    S32 len = 0;
    U8 SetSpeed[22] = {0xaa, 0xcc, 14, 0, 0, 1, 'W', 'A', 0, 0, 0, 1, 0x0A, 0x0D};

    S16 s16Speed, s16Acce;
    
    s16Speed = s32Speed;
    s16Acce = s32Accleration;
    
    printf("SendMoveStop, [%d] \n", s32Accleration);
    
    SetSpeed[5] = idNum++;
    if(s32Accleration > 900)
    {
        SetSpeed[8] = 3;   ////break stop 
    }
    else
    {
        SetSpeed[8] = 2;   ////slow stop
    }
    SetSpeed[9] = s16Acce & 0xFF;
    SetSpeed[10] = (s16Acce >> 8) & 0xFF;    
    len = (SetSpeed[2] | SetSpeed[3] << 8);
    getStrCrc8(&crc, SetSpeed + 4, len - 7);
    SetSpeed[11] = crc;
    
    OnMCUSerialComWrite(SetSpeed, len);
    return 0;
}
/***************************************************************
Description:
Input:
Output:
Return
****************************************************************/
S32 ApplicationMCUMain::SendMicroWaveCMD(U32 u32Time)
{
    U8 crc = 0;
    S32 len, timeOut = 0;
    U8 SetOpenDataBuf[12] = {0xaa, 0xcc, 12, 0, 0, 1, 'U', 'C', 1, 0, 0xA, 0xD};
    U8 SetTimeDataBuf[13] = {0xaa, 0xcc, 12, 0, 0, 1, 'U', 'A', 0, 0, 0, 0xA, 0xD};
    
    len = (SetOpenDataBuf[2] | SetOpenDataBuf[3] << 8);
    getStrCrc8(&crc, SetOpenDataBuf + 4, len - 7);
    SetOpenDataBuf[9] = crc;
    
    OnMCUSerialComWrite(SetOpenDataBuf, len);
    
    SetTimeDataBuf[8] = u32Time& 0xFF;
    SetTimeDataBuf[9] = (u32Time >> 8) & 0xFF;
    len = (SetTimeDataBuf[2] | SetTimeDataBuf[3] << 8);
    getStrCrc8(&crc, SetTimeDataBuf + 4, len - 7);
    SetTimeDataBuf[10] = crc;
    OnMCUSerialComWrite(SetTimeDataBuf, len);    

    return 0;
}

/***************************************************************
Description:
Input:
Output:
Return
****************************************************************/
S32 ApplicationMCUMain::SendObliqueWaveCMD(U32 u32Time)
{
    U8 crc = 0;
    S32 len, timeOut = 0;
    U8 SetOpenDataBuf[12] = {0xaa, 0xcc, 12, 0, 0, 1, 'V', 'A', 1, 0, 0xA, 0xD};
    U8 SetTimeDataBuf[13] = {0xaa, 0xcc, 12, 0, 0, 1, 'V', 'B', 0, 0, 0, 0xA, 0xD};
    
    len = (SetOpenDataBuf[2] | SetOpenDataBuf[3] << 8);
    getStrCrc8(&crc, SetOpenDataBuf + 4, len - 7);
    SetOpenDataBuf[9] = crc;
    
    OnMCUSerialComWrite(SetOpenDataBuf, len);
    
    SetTimeDataBuf[8] = u32Time& 0xFF;
    SetTimeDataBuf[9] = (u32Time >> 8) & 0xFF;
    len = (SetTimeDataBuf[2] | SetTimeDataBuf[3] << 8);
    getStrCrc8(&crc, SetTimeDataBuf + 4, len - 7);
    SetTimeDataBuf[10] = crc;
    OnMCUSerialComWrite(SetTimeDataBuf, len);

    return 0;
}

/***************************************************************
Description:
Input:
Output:
Return
****************************************************************/
S32 ApplicationMCUMain::GYIODistanceReset(U32 u32NeedIndex)
{
    U8 crc = 0;
    S32 len, timeOut = 0;
    U8 SetCommandBuf[11] = {0xaa, 0xcc, 11, 0, 0, 1, 'W', 'F', 0, 0x0A, 0x0D};

    u32NeedIndex = u32NeedIndex;
    
    len = (SetCommandBuf[2] | SetCommandBuf[3] << 8);
    getStrCrc8(&crc, SetCommandBuf + 4, len - 7);
    SetCommandBuf[8] = crc;
    
    OnMCUSerialComWrite(SetCommandBuf, len);

    mResetWheelDisFlag = false;
    u32ClientNeedIndex = u32NeedIndex;
    
    return 0;
}
