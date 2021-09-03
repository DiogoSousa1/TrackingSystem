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

        //if already detected a tag
        if (tagManager.allTagsDetected.totalTagsDetected > 0)
        {

            //get the first tag detected
            PoseData tagWorldPose = tagManager.allTagsDetected.tagsWorldPositions[0];
            PoseData tagCameraPose = tagManager.allTagsDetected.tagsCameraPositions[0];
            cout << "Tag pose in camera:\n";
            printPoseData(tagCameraPose);

            cout << "--------------------------\n\nTag pose in world:\n";
            printPoseData(tagWorldPose);

            //calculate the  between the camera world and tags coord system
            //need rotations to align y with the tag's normal

            Quaternion coordinateTransform = tagWorldPose.rotation * rotateQuaternionX(degreesToRadians(-90.0f));

            cout << "World coordinate transformation:\n";

            printEulers(convertQuaternionToEuler(coordinateTransform));

            PoseData enginePose = {0};
            //transform the camera coordinate relative to tag's world with tag in origin
            enginePose.position = rotateVector((lastPose.translation - tagWorldPose.position), coordinateTransform);
            enginePose.position.z *= -1.0f;

            //compute camera in tag's world rotation
            Quaternion cameraRotation = coordinateTransform * invert(lastPose.rotation);
            enginePose.eulerRotation = convertQuaternionToEuler(cameraRotation);
            cout << "------------------------------\n\nSending to engine:\n";
            printPoseData(enginePose);

            client.sendToEngine(enginePose);
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