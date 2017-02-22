#include "MyTracker_Controller.h"


int main(int argc, char **argv)
{
	// raven_version_handler TODO: 	
	//--------------------------------------------------------
	int version = RAVEN_INDIGO;    // if you are using indigo_raven
	//version = RAVEN_KINETIC;  // if you are using kinetic_raven
	
	// Initialize ROS node
	ros::init(argc, argv, "Surgical_Tool_Tracker");
	// initialize the system
	MyTracker_Controller ctrl;
	ctrl.initial(version,argc,argv);
	// start the console_thread and ros_thread
	ctrl.start_thread();
	// trigger ROS publish and subscribe update
	ros::spin();

	// join the console_thread and ros_thread
	ctrl.join_thread();
	
	exit(1);
}
