/**
 * @file TagManager.h
 * @author Diogo Sousa
 * @brief 
 * @version 1.0
 * @date 2021-08-09
 * 
 * @copyright Copyright (c) 2021
 * 
 */
/* TAGMANAGER_H */
#ifndef TAGMANAGER_H_
#define TAGMANAGER_H_

#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#include <apriltag/apriltag_pose.h>
#include <librealsense2/rs.h>
#include <librealsense2/rsutil.h>

#include <iostream>

//My headers
#include "MathHelper.h"
#include "GeometryHelpers.h"
#include "TagStructures.h"

class Tag_Manager
{
public:

	//constructor and destructors
	Tag_Manager(const rs2_extrinsics extrinsics, const rs2_intrinsics &intrisics, float tagSize);
	virtual ~Tag_Manager();

	//detect tag position relative to camera
	bool detect(unsigned char *grayImage, TagStructure* tags);

private:

	apriltag_detector *tag_detector;
	apriltag_family_t *tag;
	apriltag_detection_info_t *info;
	rs2_intrinsics camera_intrinsics;
};

#endif /* TAGMANAGER_H_ */
