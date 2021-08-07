//============================================================================
// Name        : TrackingSystem.cpp
// Author      : Barata
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include <librealsense2/rs.hpp>
#include <apriltag/apriltag.h>
#include "TagManager.h"

using namespace std;



int main() {
	rs2::pipeline pipe;
	Tag_Manager tag = Tag_Manager();
	pipe.start();
	cout << "Hello world!";
	return 0;
}
