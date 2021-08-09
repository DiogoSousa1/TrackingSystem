/*
 * TagManager.h
 *
 *  Created on: 06/08/2021
 *      Author: barata
 */

#ifndef TAGMANAGER_H_
#define TAGMANAGER_H_

#ifndef APRILTAG_h
#define APRILTAG_H
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#include <apriltag/apriltag_pose.h>
#endif

#ifndef REALSENSE2_H
#define REALSENSE2_H
#include <librealsense2/rs.h>
#include <librealsense2/rsutil.h>
#endif

#include <math.h>
#include <iostream>

class Tag_Manager
{
public:
	Tag_Manager(const rs2_extrinsics extrinsics, const rs2_intrinsics &intrisics, float tagSize);
	virtual ~Tag_Manager();
	rs2_extrinsics detect(unsigned char *grayImage, const rs2_pose *camera_pose);

private:
	apriltag_detector *tag_detector;
	apriltag_family_t *tag;
	apriltag_detection_info_t *info;
	rs2_intrinsics camera_intrinsics;
};

#endif /* TAGMANAGER_H_ */
