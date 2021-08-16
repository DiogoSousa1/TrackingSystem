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

#define PROJECT_PATH_LOGS "/home/barata/Documentos/Tracking_System/TrackingSystem/logs/"
#define DEFAULT_LOG_FILE "out.log"
#define concat(first, second) first second

#define FULL_PATH_OUT_LOG concat(PROJECT_PATH_LOGS, DEFAULT_LOG_FILE)
using namespace std;

static void readInput(int pipe)
{
	cout << "Receiving input here..." << endl;
	char command;
	while (command != 's')
	{
		command = getchar();
		//read \n after char
		getchar();
		write(pipe, &command, sizeof(char));
	}
	cout << "Closing app..." << endl;
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

	if (fork())
	{
		close(p[1]);

		string ip = "192.168.1.70";
		string port = "6301";
		rs2::pipeline camPipeline;
		rs2::config cfg;
		cfg.enable_all_streams();

		const int fisheye_sensor_idx = 1;

		//write standard output to out.txt file
		remove(FULL_PATH_OUT_LOG);
		int out = open(FULL_PATH_OUT_LOG, O_RDWR | O_CREAT | O_NONBLOCK, S_IRWXU);
		dup2(out, 1);
		int curFileOffset;
		cout << "Starting pipeline..." << endl;
		//To keep log file small store the curfile offset then rewrite data on the same bytes (old data is useless to debug)
		curFileOffset = lseek(1, 0, SEEK_CUR);

		rs2::pipeline_profile profile = camPipeline.start(cfg);
		rs2::stream_profile fisheyeStream = profile.get_stream(RS2_STREAM_FISHEYE, fisheye_sensor_idx);
		rs2_extrinsics tagPose = {0};
		rs2_intrinsics fisheye_intrinsics = fisheyeStream.as<rs2::video_stream_profile>().get_intrinsics();
		rs2_extrinsics body_toFisheye_extrinsics = fisheyeStream.get_extrinsics_to(profile.get_stream(RS2_STREAM_POSE));

		const double tagSize = 0.144; //tag size in meters;

		stringstream stream;
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
			cout << "Tracker confidence: " << lastPose.tracker_confidence << "\n";
			//only do tag detector between 6 frames
			if (frame_Number % 6 == 0)
			{
				fisheyeFrame.keep();

				if (tagManager.detect((unsigned char *)fisheyeFrame.get_data(), &lastPose))
				{

					cameraLastKnownPose = lastPose;
				}
			}

			if (tagManager.allTagsDetected.totalTagsDetected > 0)
			{

				//TODO: calculate new position and rotation of the camera based on the position and rotation of april tag detected
				//! rotation not implemented

				PoseData tagWorldPose = tagManager.allTagsDetected.tagsWorldPositions[0];
				PoseData tagCameraPose = tagManager.allTagsDetected.tagsCameraPositions[0];
				Matrix3 coordinateTransform = tagWorldPose.rotationMatrix; //* rotateX(degreesToRadians(90.0f));
				PoseData enginePose = {0};
				enginePose.position = transform((lastPose.translation - tagWorldPose.position), coordinateTransform);
				enginePose.position.z *= -1.0f;
				Matrix3 cameraRotation = Invert(coordinateTransform) * quaternionToMatrix(lastPose.rotation);
				enginePose.rotationMatrix = cameraRotation;
				enginePose.eulerRotation = convertMatrixToEuler(enginePose.rotationMatrix);

				cout << "Sending to engine:\n";
				printPoseData(enginePose);

				client.sendToEngine(enginePose);
			}
			else
			{
				cout << "Waiting for tag detection..." << endl;
			}

			lseek(1, curFileOffset, SEEK_SET);

			//read commands from pipe
			char commandSent = (char)0;

			read(p[0], &commandSent, sizeof(char));

			if (commandSent == 's')
			{
				stop = true;
			}
		}

		camPipeline.stop();
		close(p[0]);
		return 0;
	}
	else
	{
		//child code
		close(p[0]);
		readInput(p[1]);
		close(p[1]);
		exit(0);
	}
}
