#include "Headers/TrackingDevice.h"

TrackingDevice::TrackingDevice(EngineClient &engine_client) : client(engine_client)
{
    stop = false;
}
TrackingDevice::~TrackingDevice()
{
}

void TrackingDevice::startTracking(const float tagSize)
{
    //Initialization of t265 pipeline
    rs2::pipeline camPipeline;
    rs2::config cfg;
    cfg.enable_all_streams();

    const int fisheye_sensor_idx = 1;
    rs2::pipeline_profile profile = camPipeline.start(cfg);
    rs2::stream_profile fisheyeStream = profile.get_stream(RS2_STREAM_FISHEYE, fisheye_sensor_idx);
    //get intrinsics and extrinsics for coordinate transformations between t265 pose and fisheye lens coord system
    rs2_intrinsics fisheye_intrinsics = fisheyeStream.as<rs2::video_stream_profile>().get_intrinsics();

    rs2_extrinsics body_toFisheye_extrinsics = fisheyeStream.get_extrinsics_to(profile.get_stream(RS2_STREAM_POSE));

    //creates new tag manager to use during tracking
    Tag_Manager tagManager = Tag_Manager(body_toFisheye_extrinsics, fisheye_intrinsics, tagSize);

    //transformation of coordinate system to tag world
    Matrix3 coordinateTransform;
    Quaternion worldRotation;
    
    //pose data to fill and send to engine
    Vector3 position;
    Quaternion rotation;

    while (!stop)
    {

        rs2::frameset frame = camPipeline.wait_for_frames();
        rs2::video_frame fisheyeFrame = frame.get_fisheye_frame(fisheye_sensor_idx);
        unsigned long long frame_Number = fisheyeFrame.get_frame_number();
        rs2::pose_frame poseFrame = frame.get_pose_frame();
        rs2_pose lastPose = poseFrame.get_pose_data();

        cout << "Tracker confidence: " << lastPose.tracker_confidence << "\n";
        
        //only do tag detector between 6 frames
        if (frame_Number % 6 == 0)
        {
            fisheyeFrame.keep();

            //Detect tags in image
            tagManager.detect((unsigned char *)fisheyeFrame.get_data(), &lastPose);
        }

        //if already detected a tag
        if (tagManager.allTagsDetected.totalTagsDetected > 0)
        {

            //get pose data of first tag detected
            PoseData tagWorldPose = tagManager.allTagsDetected.tagsWorldPositions[0];
            PoseData tagCameraPose = tagManager.allTagsDetected.tagsCameraPositions[0];
            cout << "Tag pose in camera:\n";
            printPoseData(tagCameraPose);

            cout << "--------------------------\n\nTag pose in world:\n";
            printPoseData(tagWorldPose);
            printMatrix3(tagWorldPose.rotationMatrix);

            //calculate the  between the camera world and tags coord system
            //need rotations to align y with the tag's normal
            coordinateTransform = transpose(tagWorldPose.rotationMatrix) * rotateX(degreesToRadians(90.0f));

            cout << "World coordinate transformation:\n";
            printEulers(convertMatrixToEuler(coordinateTransform));
            //convert rotation of coordinate system to quaternion
            worldRotation = convertMatrix3ToQuaternion(coordinateTransform);

            //invert z axis
            coordinateTransform.m13 *= -1.0f;
            coordinateTransform.m23 *= -1.0f;
            coordinateTransform.m33 *= -1.0f;

            //vector transformation made using matrix algebra
            //transform the camera coordinate relative to tag's world with tag in origin
            position = transformCoordinate((lastPose.translation - tagWorldPose.position), coordinateTransform);

            //compute camera in tag's world rotation
            //Rotation of camera calculated using quaternion algebra
            rotation = invertQuaternion(lastPose.rotation) * invertQuaternion(worldRotation);

            cout << "------------------------------\n\nSending to engine:\n";
            printVector3(position);
            printEulers(convertQuaternionToEuler(rotation));
            client.sendToEngine(position, rotation, 1, 1);
        }
        else
        {
            cout << "Waiting for tag detection..." << endl;
        }

        //keep out file compact
        lseek(1, 0, SEEK_SET);
    }

    camPipeline.stop();
}

void TrackingDevice::stopTracking()
{
    stop = true;
}