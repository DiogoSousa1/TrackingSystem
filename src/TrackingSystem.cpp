//============================================================================
// Name        : TrackingSystem.cpp
// Author      : Barata
// Version     : 1.0
// Copyright   : wTVIsion stuff
// Description : Tracking System, Ansi-style
//============================================================================

#include <iostream>
#include <fstream>
#include <librealsense2/rs.hpp>
#include <librealsense2/rs.h>
#include <librealsense2/rsutil.h>
#include <apriltag/apriltag.h>
#include <unistd.h>
#include <fcntl.h>

//My headers
#include "Headers/TagManager.h"
#include "Headers/EngineClient.h"
using namespace std;

static void readInput(int pipe)
{
	cout << "Receiving input here..." << endl;
	char command;
	while (command != 's')
	{
		command = getchar();
		getchar();
		cout << "U wrote " << command << "\n";
		write(pipe, &command, sizeof(char));
	}
	cout << "Closing app..." << endl;
	exit(0);
}

//entry point for tracking application
int main()
{
	//? create fork to read input and use pipe ?
	int p[2];
	if (pipe(p) == 0)
	{
		cout << "Pipe created..." << endl;
		fcntl(p[0], F_SETFL, O_NONBLOCK);
	}
	else
	{
		return -1;
	}

	if (fork() != 0)
	{
		close(p[1]);

		string ip = "192.168.1.70";
		string port = "6301";
		rs2::pipeline camPipeline;
		rs2::config cfg;
		cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
		cfg.enable_stream(RS2_STREAM_FISHEYE, 1, RS2_FORMAT_Y8);
		cfg.enable_stream(RS2_STREAM_FISHEYE, 2, RS2_FORMAT_Y8);
		const int fisheye_sensor_idx = 1;

		//write standart output to out.txt file
		remove("out.txt");
		int out = open("out.txt", O_RDWR | O_CREAT | O_NONBLOCK, S_IRWXU);
		dup2(out, 1);

		cout << "Starting pipeline..." << endl;
		rs2::pipeline_profile profile = camPipeline.start(cfg);
		rs2::stream_profile fisheyeStream = profile.get_stream(RS2_STREAM_FISHEYE, fisheye_sensor_idx);
		rs2_extrinsics tagPose = {0};
		rs2_intrinsics fisheye_intrinsics = fisheyeStream.as<rs2::video_stream_profile>().get_intrinsics();
		rs2_extrinsics body_toFisheye_extrinsics = fisheyeStream.get_extrinsics_to(profile.get_stream(RS2_STREAM_POSE));

		const double tagSize = 0.144; //tag size in meters;

		Tag_Manager tagManager = Tag_Manager(body_toFisheye_extrinsics, fisheye_intrinsics, tagSize);
		EngineClient client = EngineClient(ip, port);
		bool stop = false;
		while (!stop)
		{

			rs2::frameset frame = camPipeline.wait_for_frames();
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
					stream << "Tags detected\nCameraRotation:\ntilt:" << tagManager.allTagsDetected.tagsCameraPositions[0].eulerRotation.tilt << "\n";
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

					cout << stream.str() << endl;
					cameraLastKnownPose = poseFrame.get_pose_data();
				}
			}

			if (tagManager.allTagsDetected.totalTagsDetected > 0)
			{

				//TODO: calculate new position and rotation of the camera based on the position and rotation of april tag detected
				PoseData tagWorldPose = tagManager.allTagsDetected.tagsWorldPositions[0];
				PoseData tagCameraPose = tagManager.allTagsDetected.tagsCameraPositions[0];
				Matrix3 coordinateTransform = tagWorldPose.rotationMatrix * rotateX(degreesToRadians(90.0f));
				cout << "Coordinate transformation: \n";
				printEulers(convertMatrixToEuler(coordinateTransform));
				PoseData enginePose = {0};
				enginePose.position = transform((lastPose.translation - tagWorldPose.position), coordinateTransform);
				Matrix3 cameraRotation = Invert(coordinateTransform * quaternionToMatrix(lastPose.rotation));
				enginePose.rotationMatrix = cameraRotation;
				enginePose.eulerRotation = convertMatrixToEuler(enginePose.rotationMatrix);
				//enginePose.eulerRotation = EulerAngles{.tilt = 0.0f, .pan = 0.0f, .roll = 0.0f};
				//Calculate new translation based on the tag position relative to camera
				client.sendToEngine(enginePose);
			}
			else
			{
				cout << "Waiting for tag detection..." << endl;
			}
			char commandSent = (char)0;

			if (read(p[0], &commandSent, sizeof(char)) != -1)
			{
				cout << "command received: " << commandSent << endl;
			}

			if (commandSent == 's')
			{
				stop = true;
			}
		}

		camPipeline.stop();
		return 0;
	}
	else
	{
		close(p[0]);
		readInput(p[1]);
	}
}
