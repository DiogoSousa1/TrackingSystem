//============================================================================
// Name        : TrackingSystem.cpp
// Author      : Barata
// Version     : 1.0
// Copyright   : wTVIsion stuff
// Description : Tracking System, Ansi-style
//============================================================================

#include <iostream>
#include <librealsense2/rs.hpp>
#include <librealsense2/rs.h>
#include <librealsense2/rsutil.h>
#include <apriltag/apriltag.h>
//My headers
#include "Headers/TagManager.h"
#include "Headers/EngineClient.h"
using namespace std;

//entry point for tracking application
int main()
{

	string ip = "192.168.1.70";
	string port = "6301";
	rs2::pipeline pipe;
	rs2::config cfg;
	Matrix3 coordinateTransform;
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

	const double tagSize = 0.144; //tag size in meters;

	Tag_Manager tagManager = Tag_Manager(body_toFisheye_extrinsics, fisheye_intrinsics, tagSize);
	EngineClient client = EngineClient(ip, port);
	while (true)
	{

		rs2::frameset frame = pipe.wait_for_frames();
		rs2::video_frame fisheyeFrame = frame.get_fisheye_frame(fisheye_sensor_idx);
		unsigned long long frame_Number = fisheyeFrame.get_frame_number();
		rs2_pose cameraLastKnownPose;
		rs2::pose_frame poseFrame = frame.get_pose_frame();
		rs2_pose lastPose = poseFrame.get_pose_data();

		//only do tag detector between 6 frames
		if (frame_Number % 6 == 0)
		{

			fisheyeFrame.keep();

			if (tagManager.detect((unsigned char *)fisheyeFrame.get_data(), &lastPose))
			{
				//DEBUG
				stringstream stream;
				stream << "Tags detected\n CameraRotation:\ntilt:" << tagManager.allTagsDetected.tagsCameraPositions[0].eulerRotation.tilt << "\n";
				stream << "pan: " << tagManager.allTagsDetected.tagsCameraPositions[0].eulerRotation.pan << "\n";
				stream << "roll: " << tagManager.allTagsDetected.tagsCameraPositions[0].eulerRotation.roll << "\n";
				stream << "Camera Pos:\nx: " << tagManager.allTagsDetected.tagsCameraPositions[0].position.x << "\n";
				stream << "y: " << tagManager.allTagsDetected.tagsCameraPositions[0].position.y << "\n";
				stream << "z: " << tagManager.allTagsDetected.tagsCameraPositions[0].position.z << "\n";
				stream << "World Pos:\nx: " << tagManager.allTagsDetected.tagsWorldPositions[0].position.x << "\n";
				stream << "y: " << tagManager.allTagsDetected.tagsWorldPositions[0].position.y << "\n";
				stream << "z: " << tagManager.allTagsDetected.tagsWorldPositions[0].position.z << "\n";
				stream << "World rotation:\ntilt: " << tagManager.allTagsDetected.tagsWorldPositions[0].eulerRotation.tilt << "\n";
				stream << "pan:" << tagManager.allTagsDetected.tagsWorldPositions[0].eulerRotation.pan << "\n";
				stream << "roll:" << tagManager.allTagsDetected.tagsWorldPositions[0].eulerRotation.roll << "\n";
				cout << stream.str();
				cameraLastKnownPose = poseFrame.get_pose_data();
			}
			else
				cout << "No tag detected\n";
		}

		if (tagManager.allTagsDetected.totalTagsDetected > 0)
		{
			PoseData tagPose = tagManager.allTagsDetected.tagsCameraPositions[0];

			PoseData enginePose;

			//Calculate new translation based on the tag position relative to camera

			client.sendToEngine(enginePose);
		}

		//TODO: calculate new position and rotation of the camera based on the position and rotation of april tag detected
		
	}

	tagManager.~Tag_Manager();
	pipe.stop();
	client.~EngineClient();
	return 0;
}
