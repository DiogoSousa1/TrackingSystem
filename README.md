# TrackingSystem

<h1>Intel realsense tracking system with tag detection</h1>
<h2>Requirements:</h2>

<ul>
  <li><a href="https://www.intelrealsense.com/tracking-camera-t265/">T265 intel realsense tracking camera model</a></li>
  <li><a href="https://github.com/AprilRobotics/apriltag">AprilTag library</a></li>
  <li><a href="https://github.com/IntelRealSense/librealsense">Librealsense2</a> installed</li>
</ul>


<h3>Known issues and objectives:</h3>
<ul>
<li>Use quaternions to all calculations of rotation</li>
</ul>

<h3>Working branches</h3>
<p>Main branch uses matrix calculations for rotations and sends quaternions to engine to avoid gimbal lock</p>
<p>Quaternion branch uses only quaternion calculations to represent rotation (not working properly)</p>


<h2>Purpose</h2>
The basic purpose of the system is to send tracking data from data given by the intel camera, a tag is detected and the coordinate system is calculated around that tag.


<h1>Disclaimer</h1>
This repo was for a private project to implement a tracking solution, since intel discontinued the support for the specific camera in use the project was cancelled :(.
