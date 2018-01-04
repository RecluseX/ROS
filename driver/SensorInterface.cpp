/******************************************************************************
 * Copyright (C)
 * File name:
 * Version:
 * Description:
 * Other:
 * History:

 ******************************************************************************/

#include <poll.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h> 
#include <errno.h> 
#include <sys/poll.h>
#include <sys/un.h> 
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h> 
#include <sys/socket.h> 
#include <netinet/in.h>

#include "SensorInterface.h"
#include "BaseTimeApi.h"
#include <cmath>
/*****************************************************************************
 MicroDefine And Struct
 *****************************************************************************/
using  namespace std;
  U16 SingularFilter(CIRCLE_BUFFER**  cir_buffer)
    {
    	int  i, j;
         U16  gyro_group[ FILTER_N ] = {0};

         U16  size = (*cir_buffer)->pop(gyro_group);

          U16*  rst  = &(*cir_buffer)->buffer_[(*cir_buffer)->index_new_];

         if ( size  <  4 )
        	 return *rst;
       //   printf("RST : %d",*rst);
         //将周期360 到0 的不连续点去掉
           for (j = size-1  ; j >  0; j--) {
        	U16 p0 = gyro_group[j];
        	U16 p1 = gyro_group[j-1];
            if(std::abs(p0) < 100  &&  std::abs((float)p1-36000) < 100)
            {
                  for( i = j ; i <=  size-1  ; ++i)
                  {
                	  gyro_group[i] += 36000;
                  }
                  break;
            }
            else if (  std::abs((float)p0 - 36000) < 100  && std::abs(float(p1)) < 100 )
            		{
                          for(i = 0 ;  i <  j ; ++i)
                          {
                        	  gyro_group[i] += 36000;
                          }
                          break;
            		}
	  }//end of for
     //   printf("size %d, \n",size);

        // 去掉数列中的噪声点


       	U32  average = 0 ;
       	U32 sum = 0;
      	U16 half = size/2;

      	U32 half0 = 0 ;
      	U32 half1 = 0 ;
           for( j = 0 ;  j < half  ; ++j)
           {
        	   half0 += gyro_group[j];
           }
      //     printf("half0 : %d",half0);
           for(int j = half ; j < size-1  ; ++j)
        	   half1 += gyro_group[j];
           sum = half0 + half1;

           average = sum/(double)(size-1);

           if(std::abs((float)gyro_group[size-1] - (float)average) > 500)//新加入的点是奇异点
           {
              	double aver_half0 = 0 ;
              	double  aver_half1 = 0 ;
              	double  quater = (size/2.0 -1)/2;
              	aver_half0 = (float)half0/(float)half ;

              	if(size - 1 - half > 0)
              	aver_half1 =(float)half1/float(size-1 - half) ;
              	else {
              		aver_half1 = 0 ;
              		return *rst;
              	}
       //       	printf("sum :%d, average :%d aver_half0 :%f    aver_half1 :%f\n",sum,average,aver_half0, aver_half1);
              	double  k = (aver_half1 - aver_half0)/(float(size-1)/2.0);
         //     	printf("kkkkkkkkk%f\n",k);
              	float rst_final  =   aver_half0  + k*(size -1  - quater);

        //      	printf("*rst_final : %f",rst_final);
              	if(rst_final > 36000)  rst_final   -=  36000;
              	*rst  = (U16)rst_final;
              	printf("对奇异点进行处理  %d；%d,  %d\n", *rst ,gyro_group[size-1] ,average);
           }
    return *rst;
}

/*****************************************************************************
 Global
 *****************************************************************************/
bool  Fuzing_lds_gyro_output(LDSMeasurementFrame_S * sensorCollector, LaserFrameData_S*  fuzedLDS)
  {
	 if(0x000003FF != sensorCollector->u32NodeFlag)
		 return false;
	// LaserFrameData_S*  pRst = *fuzedLDS;
	// printf("one frame comming time  :%d\n",( sensorCollector->astLDSNodeSet[0].s32GetTimeTick));
//  printf("one frame time duration :%d\n",(sensorCollector->astLDSNodeSet[1].s32GetTimeTick - sensorCollector->astLDSNodeSet[0].s32GetTimeTick));
#if 1
	 fuzedLDS->s32Timestamp = (sensorCollector->astLDSNodeSet[0].s32GetTimeTick );//- 1.0*(sensorCollector->astLDSNodeSet[1].s32GetTimeTick - sensorCollector->astLDSNodeSet[0].s32GetTimeTick) );//+ sensorCollector->astLDSNodeSet[9].s32GetTimeTick)/2.0;
	 for(int i = 0 ; i <  LDSFRAME_NODE_NUMBER ; ++i)
	 {
		 U16 numOfRange = sensorCollector->astLDSNodeSet[i].u16Data_count;
		 LDSMeasurementNode_S  node = sensorCollector->astLDSNodeSet[i];
		 for(int j = 0 ; j <  144; ++j)
		 {
			 U16 index = float(j)*float(numOfRange)/144.0 + 0.5;
			 fuzedLDS->Distance[j+ i*144] = node.u16Distance[index];
		 }
	 }
//	 printf("data fuzzing success !!!!!!!!!!!\n");
#else
	 for(int i = 0 ; i < LDSFRAME_NODE_NUMBER ; ++i)
	 {

	 }
#endif
	  return true;
  }
/*****************************************************************************
 Function
 *****************************************************************************
/***************************************************************
Description:
Input:
Output:
Return
****************************************************************/
SensorInterfaceAPI::SensorInterfaceAPI()
{
    s32LDSFd = -1;
    s32IPCFd = -1;
    
    /** Init Streamer Parse  for Map */
 //   InitStreamParser(&stPktParser);
 //   memset(au8MessageBuffer, 0, ECOBENEBOT3_MSG_MAXLEN);

    ////reset data for gyro
   // bGetGyioDisFlag = false;//xjtao
 //   u32ResetGYIODistanceReqIndex = -1;
 //   pthread_mutex_init(&GyioDisMutex, NULL);

    ////reset lds recv handler
    memset(&stLDSUnpackFrame, 0, sizeof(LDSMeasurementFrame_S));
    pthread_mutex_init(&LDSMutex, NULL);
    memset(&stLDSHandler, 0, sizeof(LDSDataUnPackHandler_S));

  //   gyroFilterBuf = new CIRCLE_BUFFER(FILTER_N);
  //   mLastGyroSetData = 0;
 //    mLastGyroGetData = 0 ;

     m_threadID = -1 ;
     m_isTerminal = false;
     pthread_mutex_init(&m_locker,NULL);// = PTHREAD_MUTEX_INITIALIZER;


}
SensorInterfaceAPI::~SensorInterfaceAPI()
{
   // pthread_mutex_destroy(&GyioDisMutex);
    
    pthread_mutex_destroy(&LDSMutex);
    while(vecLDSRawFrame.size() > 0)
    {
        vector<LDSMeasurementFrame_S *>::iterator iter1;
        iter1 = vecLDSRawFrame.begin();
        delete (LDSMeasurementFrame_S*)(*iter1);
        vecLDSRawFrame.erase(iter1);
    }
    vecLDSRawFrame.clear();

    pthread_mutex_lock(&m_locker);
    m_isTerminal = true;
    pthread_mutex_unlock(&m_locker);


    if(m_threadID != -1)
    pthread_join(m_threadID,NULL);

    pthread_mutex_destroy(&m_locker);

}

/***************************************************************
Description:
Input:
Output:
Return
****************************************************************/
S32 SensorInterfaceAPI::LDSInit(string strDevName)
{
    S32 s32Ret;
    struct termios new_cfg, old_cfg;
   
    /** Init UART Communication **/
    s32LDSFd = open((S8 *)strDevName.c_str(), O_RDWR | O_NOCTTY);
    if (-1 == s32LDSFd)
    {
        printf("LDSInit LDS open %s Error\n", (S8 *)strDevName.c_str());
        return 1;
    }


    /* ���沢�������д��ڲ������� */
    if (tcgetattr(s32LDSFd, &old_cfg) != 0) {
        perror("tcgetattr");
        return -1;
    }

    new_cfg = old_cfg;
    cfmakeraw(&new_cfg); /* ����Ϊԭʼģʽ */
    new_cfg.c_cflag &= ~CSIZE;
#if 1    
    /* ���ò����� */
    cfsetispeed(&new_cfg, B230400);
    cfsetospeed(&new_cfg, B230400);
#else
    /* ���ò����� */
    cfsetispeed(&new_cfg, B115200);
    cfsetospeed(&new_cfg, B115200);    
#endif
    /* ��������λ */
    new_cfg.c_cflag |= CS8;

    /* ������żУ��λ */
    new_cfg.c_cflag &= ~PARENB;
    new_cfg.c_iflag &= ~INPCK;

    /* ����ֹͣλ */
    new_cfg.c_cflag &= ~CSTOPB;


    /* ���õȴ�ʱ�����С�����ַ� */
    new_cfg.c_cc[VTIME] = 0;
    new_cfg.c_cc[VMIN] = 1;
    tcflush(s32LDSFd, TCIFLUSH); /* ����δ�����ַ� */

    /* ���������� */
    if ((tcsetattr(s32LDSFd, TCSANOW, &new_cfg)) != 0) {
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
S32 SensorInterfaceAPI::IPCInit(string strDevName)
{
    S32 ret;
    S32 sock; 
    struct  sockaddr_un servaddr; 
        
    if  ((sock = socket(PF_UNIX, SOCK_STREAM,  0)) <  0)
    {
        printf( "IPCClientBlockWorker:: socket error\n" );
        return -1;
    }
    
    memset(&servaddr,  0 ,  sizeof (servaddr)); 
    servaddr.sun_family = AF_UNIX; 
    strcpy(servaddr.sun_path,  (S8 *)strDevName.c_str()); 

    if  (connect(sock, ( struct  sockaddr *)&servaddr,  sizeof (servaddr)) <  0 ) 
    {
        printf( "IPCClientBlockWorker:: connect error\n" );
        return -2;
    }

    s32IPCFd = sock;
    
    return 0;
}
/***************************************************************
Description:
Input:
Output:
Return
****************************************************************/
S32 SensorInterfaceAPI::SensorInterfaceInit( string strLDSName)
{
    S32 s32Ret;
    
    /** Init Communication **/
    s32Ret = LDSInit(strLDSName);    
    if(s32Ret)
    {
        printf("*****************************   LDSInit Error  ********************************\n");
        return -1;
    }
    
    /// Create thread for IPCCommunication Worker.
    m_threadID = pthread_create((pthread_t*)&threadHandler, NULL, WorkerRun, this);
    if(0 != m_threadID)
    {        
        printf("SensorInterfaceInit  pthread_create Error\n");
        return 1;
    }
    
    return 0;
}

/***************************************************************
Description:
Input:
Output:
Return
****************************************************************/
void *SensorInterfaceAPI::WorkerRun(void *arg)
{
    SensorInterfaceAPI *pclHandler;

    /// set thread parameter.
   // pthread_detach(pthread_self());

    printf("Thread <SensorInterfaceAPI::WorkerRun> is Running ...\n\n");
    
    pclHandler = reinterpret_cast<SensorInterfaceAPI *>(arg);
    
    pclHandler->MainLoop();

    return NULL;
}
/***************************************************************
Description:
Input:
Output:
Return
****************************************************************/
S32 SensorInterfaceAPI::MainLoop()
{
#define IPC_MAX_BUFFER_LEN 1024
#define LDS_MAX_BUFFER_LEN 200// 128 113
    int ret;
    int timeout;
    unsigned char  recvbuf[ IPC_MAX_BUFFER_LEN ] = { 0 }; 
    int readLen;
    struct pollfd fds[2]; 

    if(s32LDSFd)
    {
        fds[0].fd = s32LDSFd;
        fds[0].events = POLLIN | POLLERR;           //  POLLIN��ʾ�����ݿɶ�, POLLERR��ʾ����
        fds[0].revents = 0;
    }
    else
    {
        fds[0].fd = -1;
    }
    
    if(s32IPCFd)
    {
        fds[1].fd = s32IPCFd;
        fds[1].events = POLLIN | POLLERR;           //  POLLIN��ʾ�����ݿɶ�, POLLERR��ʾ����
        fds[1].revents = 0;        
    }
    else
    {
        fds[1].fd = -1;
    }
    timeout = 10;
    
    while( 1 )
    {
  //  	  pthread_mutex_lock(&m_locker);
    	  if(m_isTerminal == true)
    		  break;
   // 	    pthread_mutex_unlock(&m_locker);

        /** **/
        if(s32LDSFd < 0)
        {            
            fds[0].fd = -1;
            /** reconnect to lds **/
            /*************/
        }
        if(s32IPCFd < 0)
        {            
            fds[1].fd = -1;
            /** reconnect to application **/
            /*************/
        }        
        
        ret = poll( fds, 2, timeout);
        if ( ret < 0 )
        {
            printf("poll failure!\n");            
            break;
        }
        if(0 == ret)
        {
            continue;
        }

        ////data coming
        if( (fds[0].revents & POLLERR) ||(fds[0].revents & POLLRDHUP) )                     //  ���ݳ���
        {
            printf("LDS Disconnect\n");
            close(s32LDSFd);
            s32LDSFd = -1;
        }
        else if( fds[0].revents & POLLIN )                      //  �ͻ����׽��������ݿ�д
        {
            int readLen;
            memset(recvbuf,  0 ,  LDS_MAX_BUFFER_LEN); 
            readLen = read(s32LDSFd, recvbuf,  LDS_MAX_BUFFER_LEN);
            if( readLen <= 0 )
            {
                 ////close and reset
                close( s32LDSFd);
                s32LDSFd = -1;
            }
            else
            {
                /** Get Message From Remote **/
            //	printf("readLen:%d\n",readLen);
                LDSDataProcess(recvbuf, readLen);
            }
        }

        if( (fds[1].revents & POLLERR) ||(fds[1].revents & POLLRDHUP) )                     //  ���ݳ���
        {
            printf("IPC Disconnect\n");
            close(s32IPCFd);
            s32IPCFd = -1;
        }
        else if( fds[1].revents & POLLIN )                      //  �ͻ����׽��������ݿ�д
        {
            int readLen;
            memset(recvbuf,  0 ,  IPC_MAX_BUFFER_LEN); 
            readLen = read(s32IPCFd, recvbuf,  IPC_MAX_BUFFER_LEN);
            if( readLen <= 0 )
            {
                 ////close and reset
                close( s32IPCFd);
                s32IPCFd = -1;
            }
            else
            {
                /** Get Message From Remote **/
         //       IPCDataProcess(recvbuf, readLen);
            }
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
S32 SensorInterfaceAPI::LDSDataProcess(U8 *pu8RecvData, U32 u32RecvLen)
{
    int pos1, pos2;
    unsigned char *ptmp;
    int i;
    bool bStartFlag = false;
    int frameLen = 0;
    int ret = 0;
    LDSDataUnPackHandler_S *pstHandler = &stLDSHandler;
    bool bGetOneNode = false;

 //   printf("1RecvLDSDate Len = %d\n", u32RecvLen);
#if 0
{
    static FILE *stfp = NULL;
    if(NULL == stfp)
    {
        stfp = fopen("saveldsdata", "wb");
    }
    if(NULL != stfp)
    {
        fwrite(pu8RecvData, 1, u32RecvLen, stfp);            
    }
}
#endif

    pos1 = 0; 
    pos2 = 0;

    if(pstHandler->buflen + u32RecvLen >= LDSDATAUNPACK_BUFLEN)
    {
        printf("data overflow\n");
        pstHandler->buflen = 0;
        return 1;
    }
    
    memcpy(pstHandler->databuf + pstHandler->buflen, pu8RecvData, u32RecvLen);
    pstHandler->buflen += u32RecvLen;
    if(pstHandler->buflen < 200)//10
    {
        return 2;
    }    
    i = 0;
    bStartFlag = false;
    pos1 = pos2 = 0;
    ptmp = pstHandler->databuf;
    while(i < (pstHandler->buflen - 1))
    {
        if((0xce == ptmp[i]) && (0xfa == ptmp[i+1]))
        {
            if(!bStartFlag)
            {
                pos1 = i;
                bStartFlag = true;
            }
            else
            {
                pos2 = i;
                break;
            }
        }
        i++;
    }

    if(pos2 > 0)
    {
        memset(&stLDSUnpackNode, 0, sizeof(LDSMeasurementNode_S));
        
        ////printf("Get node pos 0x%08x, 0x%08x\n", pos1, pos2);
        ptmp = pstHandler->databuf + pos1;
        frameLen = pos2 - pos1;
        ret = LDSDataFrameAnalyse(ptmp, frameLen, &stLDSUnpackNode);
        if(0 == ret)
        {
            bGetOneNode = true;
            stLDSUnpackNode.s32GetTimeTick = BaseGetTimeTick();
        }
        else
        {
            printf("LDSDataFrameAnalyse Erro\n");  
        }
        memmove(pstHandler->databuf, pstHandler->databuf + pos2, pstHandler->buflen - pos2);
        pstHandler->buflen = pstHandler->buflen - pos2;
    }

    if(bGetOneNode)
    {
        S32 s32Index;
        LDSMeasurementNode_S *pstNode;
        s32Index = stLDSUnpackNode.u16Start_angle / 360;
        ///printf("Get Node Index = %d\n", s32Index);
        if(s32Index >= LDSFRAME_NODE_NUMBER)
        {
            printf("Index Error\n");
            return 3;
        }
        
        if(0 == s32Index)
        {   
            ////reset
            memset(&stLDSUnpackFrame, 0, sizeof(LDSMeasurementFrame_S));
        }        
        
        /** copy to frame buffer **/        
        pstNode = &(stLDSUnpackFrame.astLDSNodeSet[s32Index]);
        memcpy(pstNode, &stLDSUnpackNode, 
            sizeof(LDSMeasurementNode_S));
        stLDSUnpackFrame.u32NodeFlag |= 1<<s32Index;

        ////last node in one frame
        if((LDSFRAME_NODE_NUMBER-1) == s32Index)
        {
    /*        printf("stLDSUnpackFrame.u32NodeFlag = 0x%08x, size = %d\n",
                stLDSUnpackFrame.u32NodeFlag, vecLDSRawFrame.size());*/
        
            /** check data */
            if(0x000003FF == stLDSUnpackFrame.u32NodeFlag)
            {
                LDSMeasurementFrame_S *pstFrame;

                pthread_mutex_lock(&LDSMutex);
                
                ////check buffer size
                if(vecLDSRawFrame.size() >= SENSORINTERFACE_LDS_VECTORFRAMESIZE)
                {
                    vector<LDSMeasurementFrame_S *>::iterator iter1;
                    iter1 = vecLDSRawFrame.begin();
                    delete (LDSMeasurementFrame_S*)(*iter1);
                    vecLDSRawFrame.erase(iter1);
                }
        
                /** add to frame vector **/
                pstFrame = new LDSMeasurementFrame_S;
                memcpy(pstFrame, &stLDSUnpackFrame, sizeof(LDSMeasurementFrame_S));
                vecLDSRawFrame.push_back(pstFrame);
                /*********************/
                
                pthread_mutex_unlock(&LDSMutex);                
            }
            else
            {
                printf("Error, not recv full data\n");
            }
            ////reset
            memset(&stLDSUnpackFrame, 0, sizeof(LDSMeasurementFrame_S));
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
/*
Description:
Input:
Output:
Return
****************************************************************/
S32 SensorInterfaceAPI::LDSDataFrameAnalyse(U8 *pdata, S32 datalen,
    LDSMeasurementNode_S *pstNode)
{
    int nodenumber;
    int startangle;
    int datasum = 0;
    int dataall = 0;
    int i;
    int distance;
    int dataanalyseneed;
    if(datalen < 10)
    {
        return -1;
    }
    
    nodenumber = pdata[2] + (pdata[3]<<8);
    startangle = pdata[4] + (pdata[5]<<8);
    dataanalyseneed = 6 + nodenumber * 2 + 2;
   // printf("dataLen :%d,%d\n",datalen,dataanalyseneed);

    if(datalen != dataanalyseneed)
    {
        printf("datalen error, = %d, %d\n", datalen, dataanalyseneed);
        return 1;
    }
    pstNode->u16Sync = 0xFACE;
    pstNode->u16Data_count = nodenumber;
    pstNode->u16Start_angle = startangle;
    
    for(i = 0; i < nodenumber; i++)
    {
        distance = pdata[6+2*i] + (pdata[6+2*i+1]<<8);
        dataall += distance;
        
        distance = distance&0x1FFF;

        pstNode->u16Distance[i] = distance;
    }
    dataall += nodenumber + startangle;
    datasum = pdata[6 + 2*nodenumber] + (pdata[6+2*nodenumber+1]<<8);
    if((dataall%65536) != datasum /*&& ((dataall - datasum) != 65536)*/)
    {
        printf("checkerror!  angle = %d, number = %d, check %d, %d, sub[%d]\n", 
            startangle, nodenumber, dataall, datasum, dataall - datasum);
        return 2;
    }
    return 0;
}
/***************************************************************
Description:
Input:
Output:
Return
****************************************************************/
/*
Input:
Output:
Return
****************************************************************/
/***************************************************************
Description:
Input:
Output:
Return
****************************************************************/

/***************************************************************
Description:
Input:
Output:
Return
****************************************************************/
S32 SensorInterfaceAPI::GetOneLDSData(LaserFrameData_S *pstLaserFrame, 
        OtherSensorData_S *pstSensorInfo)
{    
    vector<LDSMeasurementFrame_S *>::iterator iterlds;
    LDSMeasurementFrame_S *pstMeasurementInfo;
    LDSMeasurementFrame_S stOutFrame;
    
//return 1;

    if(0 == vecLDSRawFrame.size())
    {
        return 1;
    }
   // printf("GetOneLDSData Tick = %d\n", BaseGetTimeTick());
    
    memset(&stOutFrame, 0, sizeof(LDSMeasurementFrame_S));
    
    ////get lds data from frame vector
    pthread_mutex_lock(&LDSMutex);
    
    iterlds = vecLDSRawFrame.begin();   ///get first one
    pstMeasurementInfo = *iterlds;
    memcpy(&stOutFrame, pstMeasurementInfo, sizeof(LDSMeasurementFrame_S));
    delete pstMeasurementInfo;    
    vecLDSRawFrame.erase(iterlds);
    
    pthread_mutex_unlock(&LDSMutex);


    /** get gyio ... sensor data from vector **/    
  //  GetLDSMatchSensorData(&stOutFrame);
    

    /*** LDS Data Process ***/
#if 0
{
    int i;
    for(i = 0; i < LDSFRAME_NODE_NUMBER; i++)
    {
        printf("%d, %d\n", 
            stOutFrame.astLDSNodeSet[i].s32GetTimeTick, 
            stOutFrame.astOtherSensorSet[i].s32Timestamp);
    }
}
#endif

   if( Fuzing_lds_gyro_output(&stOutFrame, pstLaserFrame) == false)
   {
	   pstLaserFrame = NULL;
	   printf(" there is no valid data \n");
	   return 3;
   }
//   printf(" +++++++++++++++++++++++++++++++++ \n");
    /************************/
    
    return 0U;
}

/***************************************************************
Description:
Input:
Output:
Return
****************************************************************/
S32 SensorInterfaceAPI::GetLatestLDSData(LaserFrameData_S *pstLaserFrame, 
        OtherSensorData_S *pstSensorInfo)
{    
    vector<LDSMeasurementFrame_S *>::iterator iterlds;    
    LDSMeasurementFrame_S *pstMeasurementInfo;
    LDSMeasurementFrame_S stOutFrame;

  //  return 1;
    
    if(0 == vecLDSRawFrame.size())
    {
        return 1;
    }
    memset(&stOutFrame, 0, sizeof(LDSMeasurementFrame_S));
    
    ////get lds data from frame vector
    pthread_mutex_lock(&LDSMutex);

    iterlds = vecLDSRawFrame.end();   ///get lastest one
    pstMeasurementInfo = *iterlds;
    memcpy(&stOutFrame, pstMeasurementInfo, sizeof(LDSMeasurementFrame_S));

    ////clear lds frame
    for(iterlds = vecLDSRawFrame.begin(); iterlds != vecLDSRawFrame.end(); iterlds++)
    {
        pstMeasurementInfo = *iterlds;
        delete pstMeasurementInfo;
    }
    vecLDSRawFrame.clear();
    
    pthread_mutex_unlock(&LDSMutex);


    /** get gyio ... sensor data from vector **/
    //GetLDSMatchSensorData(&stOutFrame);

    return 1;
    /*** LDS Data Process ***/
/*    if( Fuzing_lds_gyro_output(&stOutFrame, &pstLaserFrame) == false)
    {
 	   pstLaserFrame = NULL;
    }*/
    /************************/
    
    return 0;    
}

/***************************************************************
Description:
Input:
Output:
Return
****************************************************************/
S32 SensorInterfaceAPI::GetSensorData(S32 s32ExpectTime, OtherSensorData_S *pstSensorInfo)
{

    //printf("GetGYRODISTANCE %d, %d\n", s32ExpectTime, pstSensorInfo->s32Timestamp);
    
    return 0;
}
