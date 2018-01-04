/******************************************************************************
 * Copyright (C)
 * File name:       BaseTrace.h
 * Version:         
 * Description:    support trace function
 * Other:
 * History:
 ******************************************************************************/
#ifndef _BASETRACE_H_
#define _BASETRACE_H_

#include "BenebotTypeDefine.h"
#include <string>
using namespace std;
/*****************************************************************************
MicroDefine And Struct
*****************************************************************************/
typedef enum BaseTraceLevel
{
  	BASE_TRACE_CRIT = 0x0001, 
	BASE_TRACE_ERR, 
	BASE_TRACE_WARN, 
	BASE_TRACE_NOTICE,
	BASE_TRACE_INFO,
	BASE_TRACE_DEBUG, 
	BASE_TRACE_EMERG,
	BASE_TRACE_ALERT,
	BASE_TRACE_NO
} BaseTraceLevel_E;

/*****************************************************************************
Global
*****************************************************************************/


/*****************************************************************************
Function
*****************************************************************************/
string SysTime();
/***************************************************************
Description:
Input:
Output:
Return
****************************************************************/
U32 BaseTraceInit(U32 u32LogServerAddr, U16 u16LogServerPort);
/***************************************************************
Description:
Input:
Output:
Return
****************************************************************/
U32 BaseTraceRelease();
/***************************************************************
Description:
Input:
Output:
Return
****************************************************************/
U32 BaseTraceRegister(const S8 *pu8ModuleName, U32 *pu32ModuleID, 
    BaseTraceLevel_E enTraceLevel);

/***************************************************************
Description:
Input:
Output:
Return
****************************************************************/
void BaseTraceOut(U32 u32TraceID, BaseTraceLevel_E enTraceLev, const S8 *fmt, ...);
    
/***************************************************************
Description:
Input:
Output:
Return
****************************************************************/
U32 BaseTraceLogServerMain(U16 u16Port, U32 u32IPAddr);


/***************************************************************
Description:
Input:
Output:
Return
****************************************************************/
BaseTraceLevel_E BaseTrace_IntTranslateToEnum(int level);
int TransferTraceLevel_Enum2Str(BaseTraceLevel_E eLevel, char *au8LogLevel);

#endif 
