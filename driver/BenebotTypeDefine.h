#ifndef _BENEBOT_TYPE_DEFINE_H
#define _BENEBOT_TYPE_DEFINE_H
#include <stdio.h>
#include <string>
#include <vector>
#include <stdlib.h>
typedef  int  S32;
typedef unsigned char U8;
typedef unsigned int U32;
typedef unsigned short U16;
typedef short S16;
typedef char S8;
using std::string;
using std::vector;

typedef struct ODOMDATA{
    int LDist;
    int RDist;
    int Gyro;
	float vL;
	float vR;
	float wz;
    int timeStamp;
} Odom_Type;

#endif
