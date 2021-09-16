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
#include <winsock2.h>
#include <Ws2tcpip.h>
#pragma comment(lib, "Ws2_32.lib")
#endif

#ifdef __unix__
#include <sys/socket.h>
#include <iostream>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#endif

class NetworkClient
{
public:
    NetworkClient(string sendIP, string sendPort, string receiveIP, string receivePort);
    virtual ~NetworkClient();
    bool sendToEngine(Vector3 &position, Quaternion &rotation, unsigned int zoom, unsigned int focus);

private:
    //for sending camera data to engine
    unsigned int sendSocketDescriptor;
    sockaddr_in sendAddress;
    //for receiving camera raw data
    unsigned int receiveSocketDescriptor;
    sockaddr_in receiveAddress;
    void initializeSocket(string IP, string Port, bool isReceiver);

#ifdef _WIN32
    void LoadWSA();
    WSADATA wsa;
#endif
};

#endif