/**
 * @file TrackingDevice.h
 * @author Diogo Sousa (diogo.sousa@wtvision.com)
 * @brief Represents a t265 intel tracking device with tag detection properties
 * @version 1.0
 * @date 2021-08-24
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef TRACKINGDEVICE_H_
#define TRACKINGDEVICE_H_

#include <librealsense2/rs.hpp>
#include <librealsense2/rs.h>
#include <librealsense2/rsutil.h>

//My headers
#include "NetworkClient.h"
#include "TagManager.h"

class TrackingDevice
{
public:
    //Constructors and destructors
    TrackingDevice(NetworkClient &engine_client);
    ~TrackingDevice();

    /**
     * @brief Starts the t265 pipeline and initializes tracking with tag detection 
     * 
     * @param tagSize tag size in meters
     * @param relativePosition - camera extrinsics between tracking device and broadcast camera
     * @param relativeRotation - camera extrinsics between tracking device and broadcast camera
     */
    void startTracking(const float tagSize, Vector3 &relativePosition, Quaternion &relativeRotation);
    /**
     * @brief Sends a stop request
     * 
     */
    void stopTracking();

private:
    NetworkClient client;
    bool stop;
};

#endif