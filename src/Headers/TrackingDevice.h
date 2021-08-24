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
#include "EngineClient.h"
#include "TagManager.h"

class TrackingDevice
{
public:
    //Constructors and destructors
    TrackingDevice(EngineClient& engine_client);
    ~TrackingDevice();
    void startTracking(const float tagSize);
    void stopTracking();

private:
    EngineClient client;
    bool stop;
};

#endif