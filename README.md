# TrackingSystem
ul {
  list-style: none;
}
ul li:before {
  content
}
<h1>Intel realsense tracking system with tag detection</h1>
<h2>Requirements:</h2>

<ul>
  <li><a href="https://www.intelrealsense.com/tracking-camera-t265/">T265 intel realsense tracking camera model</a></li>
  <li>Linux based system</li>
  <li><a href="https://github.com/AprilRobotics/apriltag">AprilTag library</a> installed</li>
  <li><a href="https://github.com/IntelRealSense/librealsense">Librealsense2</a> installed</li>
</ul>

This setup was made using vs code.
This system is being created for wTVision and this rep should be deleted soon!

<h3>Known issues and objectives:</h3>
<ul>
<li>Receive tag and rotation and apply to all the points given by camera</li>
<li>ASAP linux support for this project in wTVision</li>
<li>Check rotation matrix of tag given by aprilrobotics library, may be needed an initial transformation for coordinate system</li>
<li>Connection using socket api not working ✓</li>
</ul>