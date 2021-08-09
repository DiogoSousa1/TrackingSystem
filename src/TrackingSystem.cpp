//============================================================================
// Name        : TrackingSystem.cpp
// Author      : Barata
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#ifndef REALSENSE2_H
#define REALSENSE2_H
#include <librealsense2/rs.hpp>
#include <librealsense2/rs.h>
#include <librealsense2/rsutil.h>
#endif
#include <apriltag/apriltag.h>
#include "Headers/TagManager.h"

using namespace std;

int main()
{
	rs2::pipeline pipe;
	rs2::config cfg;
	cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
	cfg.enable_stream(RS2_STREAM_FISHEYE, 1, RS2_FORMAT_Y8);
	cfg.enable_stream(RS2_STREAM_FISHEYE, 2, RS2_FORMAT_Y8);
	return 0;
}
