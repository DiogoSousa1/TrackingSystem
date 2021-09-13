//============================================================================
// Name        : TrackingSystem.cpp
// Author      : Barata
// Version     : 1.0
// Copyright   : wTVIsion stuff
// Description : Tracking System, Ansi-style
//============================================================================

#include <iostream>
#include <fstream>
#include <fcntl.h>
#include <thread>

#ifdef _WIN32
#include <io.h>
#endif
#pragma warning(disable : 4996)

//My headers
#include "Headers/TagManager.h"
#include "Headers/TrackingDevice.h"
#include "Headers/EngineClient.h"

#define PROJECT_PATH_LOGS "./"
#define DEFAULT_LOG_FILE "out2.log"
#define concat(first, second) first second

#define DEFAULT_IP "192.168.5.249"
#define DEFAULT_PORT "6301"

#define FULL_PATH_OUT_LOG concat(PROJECT_PATH_LOGS, DEFAULT_LOG_FILE)

using namespace std;

//entry point for tracking application
int main()
{

	string ip = DEFAULT_IP;
	string port = DEFAULT_PORT;
	cout << "Insert the translation between broadcast camera and tracking device in format \"x y z\":\n";
	Vector3 relativePosition;
	EulerAngles angles;
	Quaternion relativeRotation;
	cin >> relativePosition.x >> relativePosition.y >> relativePosition.z;
	cout << "Insert the rotation between broadcast camera and tracking device in format \"pan tilt roll\":" << endl;
	cin >> angles.pan >> angles.tilt >> angles.roll;
	relativeRotation = convertEulerToQuaternion(angles);
	relativePosition = rotateVector(relativePosition, invertQuaternion(relativeRotation));
	cout << "Receiving input here...\nPress s to shutdown app!" << endl;

	//write standard output to out.log file
	remove(FULL_PATH_OUT_LOG);
	int out = open(FULL_PATH_OUT_LOG, O_RDWR | O_CREAT, S_IRWXU);
	int sout = dup(1);
	dup2(out, 1);

	EngineClient client = EngineClient(ip, port);
	TrackingDevice device = TrackingDevice(client);

	thread t([&]
			 {
				 const float tagSize = 0.144f;
				 device.startTracking(tagSize, relativePosition, relativeRotation);
			 });

	bool stop = false;
	while (!stop)
	{
		//read commands
		char commandSent = getchar();

		if (commandSent == 's')
		{
			stop = true;
			device.stopTracking();
		}
	}

	t.join();
	dup2(sout, 1);
	cout << "Closing App..." << endl;
	return 0;
}
