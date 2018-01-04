/******************************************************************************
 * Copyright (C)
 * File name:       BaseSocketApi.cpp
 * Version:         
 * Description:    support socket base function
 * Other:
 * History:
 ******************************************************************************/

#include "ECOAppSysTypeDefine.h"
#include "BaseSocketApi.h"

#ifdef ECO_WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#include <mstcpip.h>
#else
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <ifaddrs.h>
#include <unistd.h>
#include <netdb.h>
#include <signal.h>
#endif

/*****************************************************************************
MicroDefine And Struct
*****************************************************************************/
#define SOCKET_SEND_BUFSIZE 	(2 * 1024 * 1024)
#define SOCKET_RECV_BUFSIZE 	(2 * 1024 * 1024)

/***************************************************************
Description:
Input:
Output:
Return
****************************************************************/
#ifdef ECO_WIN32
bool InitWinsock(void)
{
    S32 		s32Error;
    WORD 	VersionRequested;
    WSADATA 	WsaData;

    VersionRequested = MAKEWORD(2,2);

    s32Error = WSAStartup(VersionRequested,&WsaData); //start WinSock2
    if(s32Error!=0)
    {
        return false;
    }
    else
    {
        if(LOBYTE(WsaData.wVersion)!=2||HIBYTE(WsaData.wHighVersion)!=2)
        {
            WSACleanup();
            return false;
        }
    }

    return true;
}
#endif

/***************************************************************
Description:
Input:
Output:
Return
****************************************************************/
S32 BaseSocketUdpSendCreate(U16 u16Port, U32 u32IpAddr)
{
    S32 s32SockFd;
    S32 s32Ret;
    S32 s32BufSize;
    
    //u16Port = u16Port;
    //u32IpAddr = u32IpAddr;

    s32SockFd = socket(AF_INET,SOCK_DGRAM,0);
    if(s32SockFd < 0)
    {
        printf("UDP: Create socket send error! [Line: %d]\n", __LINE__);
    }
    else
    {

    }

    return s32SockFd;

}

/***************************************************************
Description:
Input:          Input Host Byte Order port and ip.  port = 0 || ip = 0 Auto.
Output:     if Pu16Port_OUT not NULL, output this socket bind port in Host Byte Order.
Return
****************************************************************/
S32 BaseSocketUdpRecvCreate(U16 u16Port, U32 u32IpAddr, U16 *Pu16Port_OUT)
{
    S32 s32SockFd;
    S32 s32Ret;
    S32 s32BufSize;
    S32 s32Flag = 1;
    struct sockaddr_in stLocal;

    //u16Port = u16Port;
    //u32IpAddr = u32IpAddr;

    s32SockFd = socket(AF_INET, SOCK_DGRAM, 0);
    if(s32SockFd < 0)
    {
        printf("UDP: Create socket recv error! [Line: %d]\n", __LINE__);
    }
    else
    {
#if 1
        /// Set udp send and receive buffer size.   2M
        s32BufSize = SOCKET_SEND_BUFSIZE;
        s32Ret = setsockopt(s32SockFd, SOL_SOCKET, SO_SNDBUF, 
            (S8 *)&s32BufSize, sizeof(s32BufSize));
        if(s32Ret)
        {
            printf("UDP: setsockopt set send buffer size failed! [Line: %d]\n", __LINE__);
        }

        s32BufSize = SOCKET_RECV_BUFSIZE;
        s32Ret = setsockopt(s32SockFd, SOL_SOCKET, SO_RCVBUF, 
            (S8 *)&s32BufSize, sizeof(s32BufSize));
        if(s32Ret)
        {
            printf("UDP: setsockopt set recv buffer size failed! [Line: %d]\n", __LINE__);
        }

#ifdef ECO_WIN32
        /// Set udp receive model was nonblock.
        int net_timeout = 500;  /// nonblock receive.
        setsockopt(s32SockFd, SOL_SOCKET, SO_RCVTIMEO, (char *)&net_timeout,sizeof(int));
        if(s32Ret)
        {
            printf("UDP: setsockopt set nonblock receive failed! [Line: %d]\n", __LINE__);
        }
#else
        struct timeval timeout = {0, 500000};
        s32Ret = setsockopt(s32SockFd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
        if(s32Ret)
        {
            printf("UDP: setsockopt set nonblock receive failed! [Line: %d]\n", __LINE__);
        }
#endif

#endif


        U32 u32TmpIP   = htonl(u32IpAddr);
        //U32 u32TmpIP   = u32IpAddr;
        U16 u16TmpPort = htons(u16Port);

    	memset(&stLocal, 0, sizeof(struct sockaddr_in));

    	stLocal.sin_family      = AF_INET;
        if (0 == u32TmpIP)
    	    stLocal.sin_addr.s_addr = INADDR_ANY;
        else
            stLocal.sin_addr.s_addr = u32TmpIP;
    	stLocal.sin_port        = u16TmpPort;
    	s32Ret = bind(s32SockFd, (struct sockaddr *)&stLocal, sizeof(stLocal));
    	if(s32Ret < 0)
    	{
            printf("UDP: bind err. [Line: %d]\n", __LINE__);
    		return s32Ret;
    	}

        if (Pu16Port_OUT != NULL)
        {
            if (u16TmpPort == 0)
            {
                struct sockaddr_in stClientAddr;
                #ifdef ECO_WIN32
                S32 s32ClientAddrLength = sizeof(sockaddr_in);
                #else
                socklen_t s32ClientAddrLength = sizeof(sockaddr_in);
                #endif
                getsockname(s32SockFd, (struct sockaddr *)&stClientAddr, &s32ClientAddrLength);
                u16TmpPort = stClientAddr.sin_port;
            }
            *Pu16Port_OUT = u16TmpPort;
        }
    }

    return s32SockFd;
}


/***************************************************************
Description:
Input:
Output:
Return
****************************************************************/
U32 BaseSocketUdpPacketSend(S32 s32Sockfd, U16 u16Port, 
        U32 u32DstAddr, U8 *pu8Data, U32 u32DataLen)
{
    struct sockaddr_in stServer;
    U32 u16SendLen;
    S32 s32Len =sizeof(stServer);

    stServer.sin_family = AF_INET;
    stServer.sin_port = htons(u16Port); 
    stServer.sin_addr.s_addr = htonl(u32DstAddr);

    u16SendLen = sendto(s32Sockfd, (S8 *)pu8Data, u32DataLen, 0, 
            (struct sockaddr*)&stServer, s32Len);
    if(u16SendLen != u32DataLen)
    {
        printf("UDP: sendto err. [send length %d -- data length %d].\n", u16SendLen, u32DataLen);
    }

    return u16SendLen;    
}

/***************************************************************
Description:
Input:
Output:
Return
****************************************************************/
S32 BaseSocketTcpClientCreate(U16 u16LocalPort, U16 u16DstPort, U32 u32DstIpAddr)
{
    S32 s32Sockfd;
    S32 s32Ret;
    struct sockaddr_in stServer;
    S32 s32BufSize;
    S32 s32Len = sizeof(s32BufSize);
    S32 s32Flag = 1;

    s32Sockfd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
#ifdef ECO_WIN32
    if (INVALID_SOCKET == s32Sockfd)
#else
    if(s32Sockfd < 0)
#endif
    {
        printf("Create TCP socket Err. [Line: %d]\n", __LINE__);
        return s32Sockfd;
    }

    stServer.sin_family = AF_INET; 
    stServer.sin_port = htons(u16LocalPort);
    stServer.sin_addr.s_addr=htonl(INADDR_ANY);  
    s32Ret = bind(s32Sockfd, (struct sockaddr *)&stServer, sizeof(stServer));
#ifdef ECO_WIN32
    if (SOCKET_ERROR == s32Ret)
#else
    if(s32Ret < 0)
#endif
    {
        printf("TCP Socket bind Err. [Line: %d]\n", __LINE__);
#ifdef ECO_WIN32        
        closesocket(s32Sockfd);       
#else
        close(s32Sockfd);
#endif
        return s32Ret;
    }
    
#if 1
  
    s32BufSize = SOCKET_RECV_BUFSIZE;
    s32Ret = setsockopt(s32Sockfd, SOL_SOCKET, SO_RCVBUF, 
        (S8 *)&s32BufSize, sizeof(s32BufSize));
    if(s32Ret)
    {
        printf("TCP: setsockopt set receive buffer size failed! [Line: %d]\n", __LINE__);
    }

#endif

    stServer.sin_family = AF_INET; 
    stServer.sin_port = htons(u16DstPort);
    stServer.sin_addr.s_addr=htonl(u32DstIpAddr);        
    s32Ret = connect(s32Sockfd, (struct sockaddr *)&stServer, sizeof(stServer));
	
#ifdef ECO_WIN32
    if (s32Ret == SOCKET_ERROR)
#else
    if(s32Ret < 0)
#endif
    {
        printf("TCP: Socket connect Err. [Line: %d]\n", __LINE__);
#ifdef ECO_WIN32
        closesocket(s32Sockfd);       
#else
        close(s32Sockfd);
#endif
        return s32Ret;
    }


    return s32Sockfd;
}

/***************************************************************
Description:
Input:
Output:
Return
****************************************************************/
S32 BaseSocketTcpServerCreate(U16 u16Port, U32 u32IpAddr)
{
    S32 s32Sockfd;
    S32 s32Ret;
    struct sockaddr_in stServer;
    S32 s32BufSize;
    S32 s32Flag = 1;

    //u16Port = u16Port;
    //u32IpAddr = u32IpAddr;

    s32Sockfd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
#ifdef ECO_WIN32
    if (INVALID_SOCKET ==s32Sockfd)
#else
    if(s32Sockfd < 0)
#endif
    {
        printf("Create TCP socket Err. [Line: %d]\n", __LINE__);
        return s32Sockfd;
    }

    stServer.sin_family = AF_INET; 
    stServer.sin_port = htons(u16Port);
    stServer.sin_addr.s_addr=htonl(INADDR_ANY);  

    s32Ret = bind(s32Sockfd, (struct sockaddr *)&stServer, sizeof(stServer));
#ifdef ECO_WIN32
    if (SOCKET_ERROR == s32Ret)
#else
    if(s32Ret < 0)
#endif
    {
        printf("TCP Socket connect Err. [Line: %d]\n", __LINE__);
        return s32Ret;
    }
#if 0
#ifdef ECO_WIN32
    /// enable KeepAlive.
    S32 s32KeepAlive = 1;
    s32Ret = setsockopt(s32Sockfd, SOL_SOCKET, SO_KEEPALIVE, (char*)&s32KeepAlive, sizeof(s32KeepAlive));
    if (s32Ret == SOCKET_ERROR)
    {
        printf("Enable KeepAlive fail.\n");
        return s32Ret;
    }

    /// set KeepAlive param.
    tcp_keepalive alive_in = {0};
    tcp_keepalive alive_out = {0};
    alive_in.keepalivetime = 5000;      // Idle 10s then start the first detection.
    alive_in.keepaliveinterval = 3000;  // Interval 3s send the next detection.
    alive_in.onoff = TRUE;

    unsigned long ulBytesReturn = 0;

    s32Ret = WSAIoctl(s32Sockfd, SIO_KEEPALIVE_VALS, &alive_in, sizeof(alive_in),
    &alive_out, sizeof(alive_out), &ulBytesReturn, NULL, NULL);
    if (s32Ret == SOCKET_ERROR)
    {
        printf("Set KeepAlive param fail.\n");
        return s32Ret;
    }
#else

#endif
#endif

#if 1
    /// Set send and receive buffer size.
    s32BufSize = SOCKET_SEND_BUFSIZE;
    s32Ret = setsockopt(s32Sockfd, SOL_SOCKET, SO_SNDBUF, 
        (S8 *)&s32BufSize, sizeof(s32BufSize));
    if(s32Ret)
    {
        printf("TCP: setsockopt set send buffer size failed! [Line: %d]\n", __LINE__);
    }

    s32BufSize = SOCKET_RECV_BUFSIZE;
    s32Ret = setsockopt(s32Sockfd, SOL_SOCKET, SO_RCVBUF, 
        (S8 *)&s32BufSize, sizeof(s32BufSize));
    if(s32Ret)
    {
        printf("TCP: setsockopt set recv buffer size failed! [Line: %d]\n", __LINE__);
    }
#endif


    s32Ret = listen(s32Sockfd, 10);
#ifdef ECO_WIN32
    if (SOCKET_ERROR == s32Ret)
#else
    if(s32Ret < 0)
#endif
    {
        printf("TCP: Socket listen Err. [Line: %d]\n", __LINE__);
        return s32Ret;
    }

    return s32Sockfd;
}

/***************************************************************
Description:
Input:
Output:
Return
****************************************************************/
U32 BaseSocketTcpPacketSend(S32 s32Sockfd, U8 *pu8Data, U32 u32DataLen)
{
    U32 u32SendLen;
    /// If the opposite client closed socket, this send function will cause a SIGPIPE signal and the system default deal way is quit process.
#ifndef ECO_WIN32
    signal(SIGPIPE, SIG_IGN);
#endif
    u32SendLen = send(s32Sockfd, (S8 *)pu8Data, u32DataLen, 0);
    if(u32SendLen != u32DataLen)
    {
        printf("TCP: send error ERR = %d. [Line: %d]\n", u32SendLen, __LINE__);
    }

    return u32SendLen;
}
/***************************************************************
Description:
Input:
Output:
Return:             /// Net Byte Order
****************************************************************/
U32 BaseSocketGetLocalIP(U32 *pu32IPAddr)
{
    if (NULL == pu32IPAddr) return 1;
    
    S8 s8LocalIp[1280];
    memset(&s8LocalIp, 0, sizeof(s8LocalIp));
    
    #ifdef ECO_WIN32
    WSADATA wsa_data;
    if (WSAStartup( MAKEWORD(2,0), &wsa_data ) == 0)
    {
        S8 s8HostName[255];
        if (gethostname(s8HostName, sizeof(s8HostName)) == 0)
        {
            struct hostent *pstHostInfo;
            if ((pstHostInfo = gethostbyname(s8HostName)) != NULL)
            {
                U32 u32Cnt = 0;
                while (pstHostInfo->h_addr_list[u32Cnt] != NULL)
                {
                    if (AF_INET == pstHostInfo->h_addrtype)
                    {
                        strcpy(s8LocalIp, inet_ntoa(*(struct in_addr *)pstHostInfo->h_addr_list[u32Cnt]));
						if ( ( *pu32IPAddr = inet_addr(s8LocalIp) ) == INADDR_NONE ) *pu32IPAddr = 0;
                        return 0;
                    }
					u32Cnt++;
                }
            }
        }
        WSACleanup();
    }
    #else
    struct ifaddrs *if_addr_struct, *ifaddr;
    if (0 != getifaddrs(&ifaddr)) return 0;
	for (if_addr_struct = ifaddr; if_addr_struct != NULL; if_addr_struct = if_addr_struct->ifa_next)
	{
		if (strncmp(if_addr_struct -> ifa_name, "lo", 2) == 0) continue;	/// loop address
		if (if_addr_struct -> ifa_addr == NULL) continue;					/// null
		if (if_addr_struct -> ifa_addr-> sa_family != AF_INET) continue;	/// not IPV4
		getnameinfo( if_addr_struct->ifa_addr, sizeof(struct sockaddr_in),
                     s8LocalIp, NI_MAXHOST, NULL, 0, NI_NUMERICHOST);
		if (s8LocalIp[0] == 0) continue;
		//s8LocalIp = *(int *)&(((sockaddr_in *)if_addr_struct->ifa_addr)->sin_addr);
        *pu32IPAddr = *(int *)&(((sockaddr_in *)if_addr_struct->ifa_addr)->sin_addr);
		return 0;
	}
    freeifaddrs(ifaddr);
    #endif
    
    return 2;
}

/***************************************************************
Description:
Input:
Output:
Return
****************************************************************/
S32 BaseSocketTcpClient_1_Create(U16 *pu16LocalPort, S32 s32LocalIP)
{
    S32 s32Sockfd;
    S32 s32Ret;
    struct sockaddr_in stServer;
    S32 s32BufSize;
 
    s32Sockfd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
#ifdef ECO_WIN32
    if (INVALID_SOCKET == s32Sockfd)
#else
    if(s32Sockfd < 0)
#endif
    {
        printf("TCP: Create TCP socket Err. [Line: %d]\n", __LINE__);
        return s32Sockfd;
    }
#if 0
#ifdef ECO_WIN32
    /// enable KeepAlive.
    S32 s32KeepAlive = 1;
    s32Ret = setsockopt(s32Sockfd, SOL_SOCKET, SO_KEEPALIVE, (char*)&s32KeepAlive, sizeof(s32KeepAlive));
    if (s32Ret == SOCKET_ERROR)
    {
        //printf("Enable KeepAlive fail.\n");
        return s32Ret;
    }

    /// set KeepAlive param.
    tcp_keepalive alive_in = {0};
    tcp_keepalive alive_out = {0};
    alive_in.keepalivetime = 10000;      // Idle 10s then start the first detection.
    alive_in.keepaliveinterval = 3000;  // Interval 3s send the next detection.
    alive_in.onoff = TRUE;

    unsigned long ulBytesReturn = 0;

    s32Ret = WSAIoctl(s32Sockfd, SIO_KEEPALIVE_VALS, &alive_in, sizeof(alive_in),
    &alive_out, sizeof(alive_out), &ulBytesReturn, NULL, NULL);
    if (s32Ret == SOCKET_ERROR)
    {
        //printf("Set KeepAlive param fail.\n");
        return s32Ret;
    }
#else

#endif
#endif

#if 1

    s32BufSize = SOCKET_RECV_BUFSIZE;
    s32Ret = setsockopt(s32Sockfd, SOL_SOCKET, SO_RCVBUF, 
        (S8 *)&s32BufSize, sizeof(s32BufSize));
    if(s32Ret)
    {
        printf("TCP: setsockopt set recv buffer size failed! [Line: %d]\n", __LINE__);
    }

    /// Set receive timeout.
#ifdef ECO_WIN32
    int timeout = 3000;
    s32Ret = setsockopt(s32Sockfd, SOL_SOCKET, SO_RCVTIMEO, (S8 *)&timeout, sizeof(timeout));
    if(s32Ret)
    {
        printf("TCP: setsockopt set recv timeout failed! [Line: %d]\n", __LINE__);
    }
#else
    struct timeval timeout = {3, 0};///3s
    s32Ret = setsockopt(s32Sockfd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    if(s32Ret)
    {
        printf("TCP: setsockopt set recv timeout failed! [Line: %d]\n", __LINE__);
    }

#endif

#endif
    return s32Sockfd;

}

/***************************************************************
Description:
Input:
Output:
Return
****************************************************************/
S32 BaseSocketTcpClient_2_Connect(S32 s32Sockfd, U16 u16DstPort, U32 u32DstIpAddr)
{

    S32 s32Ret;
    struct sockaddr_in stServer;
    
    stServer.sin_family = AF_INET; 
    stServer.sin_port = htons(u16DstPort);
    stServer.sin_addr.s_addr=htonl(u32DstIpAddr);
    /// Default connect timeout is 75 second.
#ifndef ECO_WIN32
    signal(SIGALRM, SIG_IGN);	// ignore the alarm signal.
    alarm(5);
    s32Ret = connect(s32Sockfd, (struct sockaddr *)&stServer, sizeof(stServer));
    alarm(0);
#else
    s32Ret = connect(s32Sockfd, (struct sockaddr *)&stServer, sizeof(stServer));
#endif

#ifdef ECO_WIN32
    if (s32Ret == SOCKET_ERROR)
#else
    if(s32Ret < 0)
#endif
    {
        printf("TCP: Socket connect Err. [Line: %d]\n", __LINE__);
#ifdef ECO_WIN32
        closesocket(s32Sockfd);       
#else
        close(s32Sockfd);
#endif
        return s32Ret;
    }


    return s32Sockfd;
}
