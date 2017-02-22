#include <ros/ros.h>
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include <tf/transform_datatypes.h>
#include <stdio.h>
#include <iomanip>
#include <ros/transport_hints.h>
#include <sstream>
#include <pthread.h>
#include <termios.h>
#include <queue>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "./raven_2/raven_automove.h"
#include "./raven_2/raven_state.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

#define RAVEN_INDIGO   1
#define RAVEN_KINETIC  2
#define LEFT_ARM 0
#define RIGHT_ARM 1
#define RAVEN_LOOP_RATE 1000     // in Hz
#define IMAGE_LOOP_RATE 30       // in Hz

enum IMAGE_STATUS_LIST{
	EMPTY_IMAGE,       // 0
	START_LOADING,     // 1
	DONE_LOADING,      // 2
	START_PROCESSING,  // 3
	DONE_PROCESSING,   // 4
	START_PUBLISHING,  // 5
	DONE_PUBLISHING    // 6
};

using namespace std;

class MyTracker_MotionPlanner
{
	private:	
		int ArmType;
		tf::Vector3 Current_Pos;    // current raven position
		tf::Vector3 Delta_Pos;
		tf::Quaternion Current_Ori; // current raven rotation
		tf::Quaternion Delta_Ori;

	public:
		MyTracker_MotionPlanner();
		~MyTracker_MotionPlanner();
		tf::Transform ComputeNullMotion();

		bool set_ArmType(int);
		bool set_Current_Pos(boost::array<int, 6>);
		bool set_Current_Ori(boost::array<float, 18>);
};
