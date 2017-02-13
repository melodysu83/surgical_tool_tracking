# editted by: February 13, 2017
This is inspried by:
http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
where we can transform data from ROS topic "/usb_cam/image_raw" to OpenCv image format "Mat", process it using normal OpenCv, and then push it back to another ROS topic.

The conversion is made by cv_bridge.
which is a very convinient tool that's already built in.

Currently I'm simply drawing a red circle on the image then push it back to another ROS topic, but more processing will be done in the future (aiming at surgical tool tracking).

