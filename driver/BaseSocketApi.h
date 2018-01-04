/******************************************************************************
 * Copyright (C)
 * File name:       BaseSocketApi.h
 * Version:         
 * Description:    support socket base function
 * Other:
 * History:
 ******************************************************************************/

#ifndef _BASESOCKETAPI_H_
#define _BASESOCKETAPI_H_


#include "ECOAppSysTypeDefine.h"

#ifdef ECO_WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <sys/socket.h>
#include <netinet/in.h>
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

#ifdef ECO_WIN32
/***************************************************************
Description:
Input:
Output:
Return
****************************************************************/
bool InitWinsock(void);
#endif

/***************************************************************
Description:
Input:
Output:
Return
****************************************************************/
S32 BaseSocketUdpSendCreate(U16 u16Port, U32 u32IpAddr);

/***************************************************************
Description:
Input:
Output:
Return
****************************************************************/
S32 BaseSocketUdpRecvCreate(U16 u16Port, U32 u32IpAddr, U16 *Pu16Port_OUT = NULL);

/***************************************************************
Description:
Input:
Output:
Return
****************************************************************/
U32 BaseSocketUdpPacketSend(S32 s32Sockfd, U16 u16Port, 
        U32 u32DstAddr, U8 *pu8Data, U32 u32DataLen);

/***************************************************************
Description:
Input:
Output:
Return
****************************************************************/
S32 BaseSocketTcpClientCreate(U16 u16LocalPort, U16 u16DstPort, U32 u32DstIpAddr);

/***************************************************************
Description:
Input:
Output:
Return
****************************************************************/
S32 BaseSocketTcpServerCreate(U16 u16Port, U32 u32IpAddr);

/***************************************************************
Description:
Input:
Output:
Return
****************************************************************/
U32 BaseSocketTcpPacketSend(S32 s32Sockfd, U8 *pu8Data, U32 u32DataLen);

/***************************************************************
Description:
Input:
Output:
Return
****************************************************************/
U32 BaseSocketGetLocalIP(U32 *pu32IPAddr);

/***************************************************************
Description:
Input:
Output:
Return
****************************************************************/

//S32 BaseSocketGetLocalPort(U16 *u16LocalPort);


/***************************************************************
Description:
Input:
Output:
Return
****************************************************************/
S32 BaseSocketTcpClient_1_Create(U16 *pu16LocalPort, S32 s32LocalIP);

/***************************************************************
Description:
Input:
Output:
Return
****************************************************************/
S32 BaseSocketTcpClient_2_Connect(S32 s32Sockfd, U16 u16DstPort, U32 u32DstIpAddr);


#endif 
