/**
 * @file FreeDStructures.h
 * @author Diogo Sousa
 * @brief 
 * @version 1.0
 * @date 2021-08-11
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef FREEDSTRUCTURES_H
#define FREEDSTRUCTURES_H
#include <cstring>
#include "TrackingStructures.h"

typedef const struct FreeDOperators
{
    const unsigned int pan = 32768;
    const unsigned int tilt = 32768;
    const unsigned int roll = 32768;
    const unsigned int x = 64000;
    const unsigned int y = 64000;
    const unsigned int z = 64000;
    const unsigned int zoom = 524288;
    const unsigned int focus = 524288;
};

FreeDOperators operators;

#pragma pack(1)
typedef struct CameraData
{
    unsigned char Header;
    unsigned char CameraId;
    unsigned char Pan[3];
    unsigned char Tilt[3];
    unsigned char Roll[3];
    unsigned char x[3];
    unsigned char y[3];
    unsigned char z[3];
    unsigned char zoom[3];
    unsigned char focus[3];
    short UserDefined;
    unsigned char Checksum;
};

static void FourBytesToThreeBigEndian(unsigned char value4Bytes[4], unsigned char value3bytes[3])
{
    value3bytes[0] = value4Bytes[2];
    value3bytes[1] = value4Bytes[1];
    value3bytes[2] = value4Bytes[0];
}

static void ConvertToFreeDFormat(unsigned char bytes[3], float value, unsigned int multiplier)
{
    unsigned char value_in_bytes[4];
    int aux = (int)(value * multiplier);
    memcpy(value_in_bytes, &aux, sizeof(int));
    FourBytesToThreeBigEndian(value_in_bytes, bytes);
}

static void convertPoseToCameraData(poseData data, unsigned int zoom, unsigned int focus, CameraData *toSend)
{
    ConvertToFreeDFormat(toSend->Pan, data.eulerOfRotation.pan, operators.pan);
    ConvertToFreeDFormat(toSend->Tilt, data.eulerOfRotation.tilt, operators.tilt);
    ConvertToFreeDFormat(toSend->Roll, data.eulerOfRotation.roll, operators.roll);
    ConvertToFreeDFormat(toSend->x, data.translation.x, operators.x);
    ConvertToFreeDFormat(toSend->y, data.translation.y, operators.y);
    ConvertToFreeDFormat(toSend->z, data.translation.z, operators.z);
    ConvertToFreeDFormat(toSend->zoom, zoom, operators.zoom);
    ConvertToFreeDFormat(toSend->focus, focus, operators.focus);
}

#endif