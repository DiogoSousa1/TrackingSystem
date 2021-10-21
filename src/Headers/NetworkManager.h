/**
 * @file NetworkManager.h
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


#include <thread>

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



class NetworkManager
{
public:
    //constructors and destructors
    NetworkManager(string sendIP, string sendPort, string receivePort);
    virtual ~NetworkManager();

    //send tracking data to destination
    bool sendTrackingData(Vector3 &position, Quaternion &rotation, unsigned int zoom, unsigned int focus);
    //starts receiving data 
    void receiveLensData(int* zoom, int* focus);

    unsigned int receiveSocketDescriptor;
private:
    //for sending camera data to engine
    unsigned int sendSocketDescriptor;
    sockaddr_in sendAddress;

    //for receiving camera raw data
    sockaddr_in receiveAddress;

    //initialize sockets to send and receive data
    void initializeSockets(string sendIP, string sendPort, string receivePort);

#ifdef _WIN32
    void LoadWSA();
    WSADATA wsa;
#endif
};

#endif