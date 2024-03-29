/**
 * @file FreeDStructures.h
 * @author Diogo Sousa
 * @brief FreeD protocol helpers for R3 engine
 * @version 1.0
 * @date 2021-08-11
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef FREEDSTRUCTURES_H
#define FREEDSTRUCTURES_H

//My headers
#include "TrackingStructures.h"
#include "QuaternionHelpers.h"

struct FreeDOperators
{
    const unsigned int pan = 32768;
    const unsigned int tilt = 32768;
    const unsigned int roll = 32768;
    const unsigned int w = 32768;
    const unsigned int x = 64000;
    const unsigned int y = 64000;
    const unsigned int z = 64000;
    const unsigned int zoom = 524288;
    const unsigned int focus = 524288;
};
static FreeDOperators freeDOperators;

#pragma pack(1)
struct CameraData
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
    unsigned char W[3];
};

static void FourBytesToThreeBigEndian(unsigned char value4Bytes[4], unsigned char value3bytes[3])
{
    value3bytes[0] = value4Bytes[2];
    value3bytes[1] = value4Bytes[1];
    value3bytes[2] = value4Bytes[0];
}

static int ThreeBytesToSignedInt(unsigned char byte0, unsigned char byte1, unsigned char byte2)
{
    int value = (int)(byte0 << 16 | byte1 << 8 | byte2);
    if (value & (0x8 << 20))
    {
        value |= 0xFF << 24;
    }
    return value;
}

static void ConvertToFreeDFormat(unsigned char bytes[3], float value, unsigned int multiplier)
{
    unsigned char value_in_bytes[4];
    int aux = (int)(value * multiplier);
    memcpy(value_in_bytes, &aux, sizeof(int));
    FourBytesToThreeBigEndian(value_in_bytes, bytes);
}

static void convertPoseToCameraData(Vector3 &position, Quaternion &rotation, unsigned int zoom, unsigned int focus, CameraData *toSend)
{
    ConvertToFreeDFormat(toSend->Pan, rotation.y, freeDOperators.pan);
    ConvertToFreeDFormat(toSend->Tilt, rotation.x, freeDOperators.tilt);
    ConvertToFreeDFormat(toSend->Roll, rotation.z, freeDOperators.roll);
    ConvertToFreeDFormat(toSend->W, rotation.w, freeDOperators.w);
    ConvertToFreeDFormat(toSend->x, position.x, freeDOperators.x);
    ConvertToFreeDFormat(toSend->y, position.y, freeDOperators.y);
    ConvertToFreeDFormat(toSend->z, position.z, freeDOperators.z);
    ConvertToFreeDFormat(toSend->zoom, zoom, freeDOperators.zoom);
    ConvertToFreeDFormat(toSend->focus, focus, freeDOperators.focus);
}

#endif