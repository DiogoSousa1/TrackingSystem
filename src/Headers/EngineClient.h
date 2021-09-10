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

//My headers
#include "TrackingStructures.h"
#include "FreeDHelpers.h"
using namespace std;

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef _WINDOWS_
#include <winsock2.h>
#include <Ws2tcpip.h>
#endif
#pragma comment(lib, "Ws2_32.lib")

#endif

#ifdef __unix__
#include <sys/socket.h>
#include <iostream>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#endif

class EngineClient
{
public:
    EngineClient(string ip, string port);
    virtual ~EngineClient();
    bool sendToEngine(Vector3 &position, Quaternion &rotation, unsigned int zoom, unsigned int focus);

private:
    unsigned int socketDescriptor;
    sockaddr_in address;
#ifdef _WIN32
    void LoadWSA();
    WSADATA wsa;
#endif
};

#endif