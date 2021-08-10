//============================================================================
// Name        : TrackingSystem.cpp
// Author      : Barata
// Version     : 1.0
// Copyright   : Your copyright notice
// Description : Tracking System, Ansi-style
//============================================================================

#include <iostream>
#include <librealsense2/rs.hpp>
#include <librealsense2/rs.h>
#include <librealsense2/rsutil.h>
#include <apriltag/apriltag.h>

//My headers
#include "Headers/TagManager.h"

using namespace std;

//TODO: Test if tag detection works!!

//entry point for tracking application
int main()
{
	rs2::pipeline pipe;
	rs2::config cfg;
	cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
	cfg.enable_stream(RS2_STREAM_FISHEYE, 1, RS2_FORMAT_Y8);
	cfg.enable_stream(RS2_STREAM_FISHEYE, 2, RS2_FORMAT_Y8);
	const int fisheye_sensor_idx = 1;
	cout << "Starting pipeline...";
	rs2::pipeline_profile profile = pipe.start(cfg);
	rs2::stream_profile fisheyeStream = profile.get_stream(RS2_STREAM_FISHEYE, fisheye_sensor_idx);
	rs2_extrinsics tagPose = {0};
	rs2_intrinsics fisheye_intrinsics = fisheyeStream.as<rs2::video_stream_profile>().get_intrinsics();
	rs2_extrinsics body_toFisheye_extrinsics = fisheyeStream.get_extrinsics_to(profile.get_stream(RS2_STREAM_POSE));
	rs2_pose cameraLastKnownPose;
	const double tagSize = 0.144; //tag size in meters;

	Tag_Manager tagManager = Tag_Manager(body_toFisheye_extrinsics, fisheye_intrinsics, tagSize);

	while (true)
	{

		rs2::frameset frame = pipe.wait_for_frames();
		rs2::video_frame fisheyeFrame = frame.get_fisheye_frame(fisheye_sensor_idx);
		unsigned long long frame_Number = fisheyeFrame.get_frame_number();
		rs2::pose_frame poseFrame = frame.get_pose_frame();
		
		//only do tag detector between 6 frames
		if (frame_Number % 6 == 0 && tagManager.allTagsDetected.totalTagsDetected == 0)
		{

			fisheyeFrame.keep();
			tagManager.detect((unsigned char *)fisheyeFrame.get_data());
			//TODO: rotate relative pose to coordinate system given by tag yaw
			if (tagManager.allTagsDetected.totalTagsDetected > 0)
			{
				cameraLastKnownPose = poseFrame.get_pose_data();

			}
		}

		//TODO: calculate new position and rotation of the camera based on the position and rotation of april tag detected

		std::cout << "Detected origin tag\n";
	}

	tagManager.~Tag_Manager();
	pipe.stop();

	return 0;
}
