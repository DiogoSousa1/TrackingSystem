/**
 * @file EngineClient.h
 * @author Diogo Sousa
 * @brief Client to connect to R3 engine using FreeD protocol
 * @version 0.1
 * @date 2021-08-11
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef ENGINECLIENT_H
#define ENGINECLIENT_H
#include "TrackingStructures.h"
class EngineClient
{
public:
    EngineClient();
    virtual ~EngineClient();
    bool sendToEngine(poseData dataToSend);

private:

};


#endif