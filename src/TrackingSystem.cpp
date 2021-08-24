//============================================================================
// Name        : TrackingSystem.cpp
// Author      : Barata
// Version     : 1.0
// Copyright   : wTVIsion stuff
// Description : Tracking System, Ansi-style
//============================================================================

#include <iostream>
#include <fstream>
#include <apriltag/apriltag.h>
#include <unistd.h>
#include <fcntl.h>
#include <thread>

//My headers
#include "Headers/TagManager.h"
#include "Headers/TrackingDevice.h"
#include "Headers/EngineClient.h"

#define PROJECT_PATH_LOGS "/home/barata/Documentos/Tracking_System/TrackingSystem/logs/"
#define DEFAULT_LOG_FILE "out.log"
#define concat(first, second) first second

#define DEFAULT_IP "192.168.1.70"
#define DEFAULT_PORT "6301"

#define FULL_PATH_OUT_LOG concat(PROJECT_PATH_LOGS, DEFAULT_LOG_FILE)
using namespace std;

static void readInput(int pipe)
{
	cout << "Receiving input here...\nPress s to exit app!" << endl;
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

	string ip = DEFAULT_IP;
	string port = DEFAULT_PORT;
	cout << "Receiving input here...\nPress s to shutdown app!" << endl;

	//write standard output to out.txt file
	remove(FULL_PATH_OUT_LOG);
	int out = open(FULL_PATH_OUT_LOG, O_RDWR | O_CREAT | O_NONBLOCK, S_IRWXU);
	int sout = dup(1);
	dup2(out, 1);

	int curFileOffset;
	EngineClient client = EngineClient(ip, port);
	TrackingDevice device = TrackingDevice(client);
	thread t([&device]
			 {
				 const float tagSize = 0.144f;
				 device.startTracking(tagSize);
			 });

	bool stop = false;
	while (!stop)
	{
		//read commands from pipe
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
