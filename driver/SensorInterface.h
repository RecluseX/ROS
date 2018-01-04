/******************************************************************************
 * Copyright (C)
 * File name:       
 * Version:         
 * Description:    
 * Other:
 * History:
 
 ******************************************************************************/
#ifndef _SENSORINTERFACE_API_H_
#define _SENSORINTERFACE_API_H_
#include <stdio.h>
#include<string>
#include<vector>
#include <pthread.h>

typedef char               S8;              /*8-bit*/
typedef short              S16;            /*16-bit*/
typedef int                S32;            /*32-bit*/

typedef unsigned char         U8;              /*8-bit*/
typedef unsigned short        U16;            /*16-bit*/
typedef unsigned int          U32;            /*32-bit*/

typedef float                 FLOAT;
/*****************************************************************************
 MicroDefine And Struct
 *****************************************************************************/
using std::string;
using std::vector;
 typedef struct OtherSensorData
{
    S32 s32Timestamp;
    U32 u32Angle;
    S32 s32LDistance;
    S32 s32RDistance;
    S32 s32IMU;
}OtherSensorData_S;

typedef struct LDSMeasurementNode
{
    S32 s32GetTimeTick; ///ms
    U16 u16Sync;
    U16 u16Data_count;
    U16 u16Start_angle;
    U16 u16Distance[150];  //sometimes the quantities of data are more than 144, so I set it to 80. 
    U16 u16Checksum;
}LDSMeasurementNode_S;

/*         xjtao*/
#define LDSFRAME_NODE_NUMBER 10
#define FILTER_N 8
typedef struct LDSMeasurementFrame
{
    U32 u32NodeFlag;   ////bit0 ~bit9
    LDSMeasurementNode_S astLDSNodeSet[LDSFRAME_NODE_NUMBER];
    OtherSensorData_S astOtherSensorSet[LDSFRAME_NODE_NUMBER];
}LDSMeasurementFrame_S;

typedef struct LaserFrameData
{
    S32 s32Timestamp;
    S32 s32FrameIndex;
    float Distance[1440];
}LaserFrameData_S;



#define LDSDATAUNPACK_BUFLEN (4*1024)
typedef struct LDSDataUnPackHandler
{
    U8 databuf[LDSDATAUNPACK_BUFLEN];
    S32 buflen;    
}LDSDataUnPackHandler_S;

typedef struct CIRCLE_BUFFER{
	CIRCLE_BUFFER(U16 size)
	{
		buffer_len_ = size;
		buffer_ = new U16(size);
        reset();

	}
	~CIRCLE_BUFFER()
	{
		delete buffer_;
	}
	void reset()
	{
		memset(buffer_,0,sizeof(U16) * buffer_len_);
		isInitialized_ = false;
		index_add_ = 0 ;
		index_new_ = 0 ;
	}
	//环形队列
	void addNew(U16  data)
	{
        buffer_[index_add_++] = data;
        index_new_ = index_add_ - 1;
        if(index_add_  >=  buffer_len_)
        {
        	index_add_ = 0 ;
        	isInitialized_ = true;
        }
	}
	//按照FIFO顺序返回队列
	U16  pop(U16* buf)
	{
		 if(isInitialized_ == false)
		 {
			 memcpy(buf,buffer_,sizeof(U16)*index_add_);
			 return index_add_;
		 }
          memcpy(buf, buffer_+index_add_ ,sizeof(U16)* (buffer_len_  - index_add_));
          memcpy(buf+ (buffer_len_  - index_add_) ,buffer_, sizeof(U16) * index_add_) ;
          return buffer_len_;

	}
	U16* buffer_;
	U16 buffer_len_;

	U16 index_add_;
	U16 index_new_;
	bool isInitialized_ ;

}Circular_Buffer;

class SensorInterfaceAPI
{
public:
    SensorInterfaceAPI();
    ~SensorInterfaceAPI();
 /*****************************************************************************
 Function
 *****************************************************************************/   
    S32 LDSInit(string strDevName);
    S32 IPCInit(string strDevName);
    S32 SensorInterfaceInit( string strLDSName);
    S32 GetOneLDSData(LaserFrameData_S *pstLaserFrame, OtherSensorData_S *pstSensorInfo);    
    S32 GetLatestLDSData(LaserFrameData_S *pstLaserFrame, OtherSensorData_S *pstSensorInfo);    
    S32 GetSensorData(S32 s32ExpectTime, OtherSensorData_S *pstSensorInfo);
    
private:
/*****************************************************************************
 Function
 *****************************************************************************/
    static void *WorkerRun(void *arg);
    S32 MainLoop();
    S32 LDSDataProcess(U8 *pu8RecvData, U32 u32RecvLen);
    S32 IPCDataProcess(U8 *pu8RecvData, U32 u32RecvLen);
    S32 LDSDataFrameAnalyse(U8 *pdata, S32 datalen, LDSMeasurementNode_S *pstNode);
    S32 ClientMsgProcess(U8 *pu8MessageBuf, U32 u32MessageLen);
    
    S32 GetLDSMatchSensorData(LDSMeasurementFrame_S *pstLDSFrame);    

/*****************************************************************************
 
 *****************************************************************************/    
    pthread_t threadHandler;
    S32 s32LDSFd;
    S32 s32IPCFd;

#define SENSORINTERFACE_GYRO_VECTORBUFSIZE 10    

    /** for lds uart recv */
    pthread_mutex_t LDSMutex;    
    LDSDataUnPackHandler_S stLDSHandler;
    LDSMeasurementNode_S stLDSUnpackNode;
    LDSMeasurementFrame_S stLDSUnpackFrame;
#define SENSORINTERFACE_LDS_VECTORFRAMESIZE 5        
    vector<LDSMeasurementFrame_S *> vecLDSRawFrame;    

    pthread_t m_threadID ;
    bool m_isTerminal;
    pthread_mutex_t m_locker;
 };

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



#endif


