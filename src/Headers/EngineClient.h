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

#ifdef __unix__
#include <sys/socket.h>
#include <iostream>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
//linux socket connection
class EngineClient
{
public:
    EngineClient(string ip, string port);
    virtual ~EngineClient();
    bool sendToEngine(Vector3& position, Quaternion& rotation, unsigned int zoom, unsigned int focus);

private:
    int socketDescriptor;
    sockaddr_in address;
};

#endif
#ifdef _WIN32
#include <winsock2.h>
#include <Ws2tcpip.h>
#pragma comment(lib, "Ws2_32.lib")
//windows socket connection

class EngineClient
{
public:
    EngineClient(string ip, string port);
    virtual ~EngineClient();
    bool sendToEngine(Vector3& position, Quaternion& rotation, unsigned int zoom, unsigned int focus);

private:
    void LoadWSA();
    socket socketDescriptor;
    sockaddr_in address;
    WSADATA wsa;
};
#endif
#endif