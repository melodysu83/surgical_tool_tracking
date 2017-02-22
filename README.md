# editted by: February 22, 2017
This is the prototype of how the code may look like.
(1) Three threads: the console thread (handle keyboard events, as fast as possible, but displays at 1Hz),
                   the raven thread (1000Hz, as required from spec), 
                   the image thread (handles getting image frames from usb_cam and push it back as a ROS topic).

(2) Massive modification from last time. Now it is more topic specific and user friendly (kind of?).

(3) Video log: This is how it works right now.
[![Alt text for your video](https://i.ytimg.com/vi/H7MriIoE_Go/2.jpg?time=1487799573332)](https://youtu.be/H7MriIoE_Go)




# editted by: February 13, 2017
This is inspried by:
http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
where we can transform data from ROS topic "/usb_cam/image_raw" to OpenCv image format "Mat", process it using normal OpenCv, and then push it back to another ROS topic.

The conversion is made by cv_bridge.
which is a very convinient tool that's already built in.

Currently I'm simply drawing a red circle on the image then push it back to another ROS topic, but more processing will be done in the future (aiming at surgical tool tracking).


To run everything, go to "myproject/src/launch/myproject.launch"
this will integrate everything together.
Also, no need to call "roscore" on a separate terminal, "roslaunch" takes care of the problem.

Few things to be aware of when playing with usb_cam:

(1) >> rosrun usb_cam usb_cam_node does not shows the obtained image frame to a window
    >> roslaunch usb_cam/launch/usb_cam-test.launch does!
    So do what is best of the situation:)
    
(2) calibrate the camera in order to get camera_info.yaml
follow steps in this link: http://wiki.ros.org/camera_calibration
to be specific, for a mono camera: http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration
                for a stereo camera: http://wiki.ros.org/camera_calibration/Tutorials/StereoCalibration
Remember to edit the camera_info_URL in the "usb_cam_node.cpp" file

(3) Now, finally, be sure the device name you're having, normally its "/dev/video0"
But if it's not, be sure to change it from the default. 
Changes should be made in the launch file.


Getting started with usb_cam:
http://answers.ros.org/question/197651/how-to-install-a-driver-like-usb_cam/

$ mkdir -p ~/catkin-ws/src

$ cd ~/catkin-ws/src

$ git clone https://github.com/bosch-ros-pkg/usb_cam.git

$ cd ..

$ catkin_make

$ source ~/catkin-ws/devel/setup.bash
after the above you can verify using ..

$ roscd usb_cam
roscore on a new terminal $ roscore
and then in the terminal where you $ source 'd your usb_cam code run:

$ rosrun usb_cam usb_cam_node

or

$ roslaunch usb_cam/launch/usb_cam-test.launch
Make sure you have your camera connected, before running the above command!!! you view the captured image on rviz,

$ rosrun rviz rviz
