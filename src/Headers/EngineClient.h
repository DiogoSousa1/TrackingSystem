/**
 * @file EngineClient.h
 * @author Diogo Sousa
 * @brief Client to connect to R3 engine using FreeD protocol
 * @version 1.0
 * @date 2021-08-11
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef ENGINECLIENT_H
#define ENGINECLIENT_H
#include <sys/socket.h>
#include <iostream>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

//My headers
#include "TrackingStructures.h"
#include "FreeDHelpers.h"

using namespace std;

class EngineClient
{
public:
    EngineClient(string ip, string port);
    virtual ~EngineClient();
    bool sendToEngine(PoseData dataToSend);

private:
    int socketDescriptor;
    sockaddr_in address;
};

#endif