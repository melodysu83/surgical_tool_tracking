#include "MyTracker_Controller.h"

MyTracker_Controller::MyTracker_Controller():it_(nh_)
{
	LEFT_MOTION.set_ArmType(LEFT_ARM);
	LEFT_MOTION.set_ArmType(RIGHT_ARM);
}


MyTracker_Controller::~MyTracker_Controller()
{
}


void MyTracker_Controller::initial(int version, int argc, char** argv)  
{	
	init_sys(version);
	if(!init_ros(argc,argv))
	{	
		ROS_ERROR("Fail to initialize ROS. Exiting!");
		exit(1);
	}
}


void MyTracker_Controller::init_sys(int version)
{
	this->RAVEN_SUB_COUNT = 0;
	this->RAVEN_PUB_COUNT = 0;
	this->IMAGE_SUB_COUNT = 0;
	this->IMAGE_PUB_COUNT = 0;

        this->RAVEN_VERSION = version;
	this->IMAGE_STATUS = EMPTY_IMAGE;
}


bool MyTracker_Controller::init_ros(int argc, char** argv)
{
	while(!ros::ok())
	{
		cout<<"Wait for ROS to initialize"<<endl;
	}
	// Define message based on Raven version
	string msg_name;
	if (RAVEN_VERSION == RAVEN_INDIGO)
	{
		cout<< "Raven version in use: Indigo Release"<<endl;
        	msg_name = "ravenstate";
	}
	else if (RAVEN_VERSION == RAVEN_KINETIC)
	{
		cout<< "Raven version in use: Kinetic Release"<<endl;
		msg_name = "raven_state";
	}
	else
	{
		cout<< "Raven version in use: [Error] Invalid!"<<endl;
	}
	// Setup Raven publish/subscribe relation
	raven_pub_ = nh_.advertise<raven_2::raven_automove>("raven_automove",1);	
	raven_sub_ = nh_.subscribe(msg_name,1,&MyTracker_Controller::ravenCb,this);

	// Setup Image publish/subscribe relation
	image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, &MyTracker_Controller::imageCb, this);
	image_pub_ = it_.advertise("/Surgical_Tool_Tracker/output_video", 1);
	return true;
}


void MyTracker_Controller::start_thread()
{
	pthread_create(&console_thread,NULL,MyTracker_Controller::static_console_process,this);
	pthread_create(&ros_image_thread,NULL,MyTracker_Controller::static_ros_image_process,this);
	pthread_create(&ros_raven_thread,NULL,MyTracker_Controller::static_ros_raven_process,this);

}


void MyTracker_Controller::join_thread()
{
	pthread_join(console_thread,NULL);
	pthread_join(ros_image_thread,NULL);
	pthread_join(ros_raven_thread,NULL);
}


void *MyTracker_Controller::console_process()
{
	int theKey;
	ros::Time time;
	time = time.now();
	while (ros::ok())
  	{
		theKey = getKey();
		if(theKey!=-1)
			cout<<"the key"<<theKey<<endl;
		
		if((time.now()-time).toSec() > 1)
		{
			console_display();
			time = time.now();
		}
	}
	
}


void *MyTracker_Controller::ros_image_process()
{
	static ros::Rate loop_rate(IMAGE_LOOP_RATE);
	while (ros::ok())
  	{
		if(cv_ptr_SEG)
		{
			//(1) find surgical tool
			process_image_segmenting();

			//(2) publish segmented image
			imagePb();

			//(3) release cvbridge object
			cv_ptr_RAW.reset();
			cv_ptr_SEG.reset();
		}
		
		//(5) wait for next publish
		ros::spinOnce();
		loop_rate.sleep();
		
	}
}



void *MyTracker_Controller::ros_raven_process()
{
	static ros::Rate loop_rate(RAVEN_LOOP_RATE);
	while (ros::ok())
  	{	
		//(1) update raven state (for future computation uses)
		LEFT_MOTION.set_Current_Pos(CURR_RAVEN_STATE.pos);
		LEFT_MOTION.set_Current_Ori(CURR_RAVEN_STATE.ori);

		RIGHT_MOTION.set_Current_Pos(CURR_RAVEN_STATE.pos);
		RIGHT_MOTION.set_Current_Ori(CURR_RAVEN_STATE.ori);		

		//(3) prepare user command
		process_raven_command();

		//(4) publish raven command
		ravenPb();

		//(5) wait for next publish
		ros::spinOnce();
		loop_rate.sleep();
	}
}


void * MyTracker_Controller::static_console_process(void* classRef)
{
	return ((MyTracker_Controller *)classRef)->console_process();
}


void * MyTracker_Controller::static_ros_image_process(void* classRef)
{
	return ((MyTracker_Controller *)classRef)->ros_image_process();
}


void * MyTracker_Controller::static_ros_raven_process(void* classRef)
{
	return ((MyTracker_Controller *)classRef)->ros_raven_process();
}


void MyTracker_Controller::ravenCb(const raven_2::raven_state msg)
{
	// (1) save the updated raven_state 
	CURR_RAVEN_STATE.runlevel = msg.runlevel;
	CURR_RAVEN_STATE.sublevel = msg.sublevel;
	CURR_RAVEN_STATE.last_seq = msg.last_seq;
	CURR_RAVEN_STATE.dt = msg.dt;

	for(int i=0; i<2; i++)
	{
		CURR_RAVEN_STATE.type[i] = msg.type[i];
		CURR_RAVEN_STATE.grasp_d[i] = msg.grasp_d[i];
	}

	for(int i=0; i<6; i++)
	{
		CURR_RAVEN_STATE.pos[i] = msg.pos[i];
		CURR_RAVEN_STATE.pos_d[i] = msg.pos_d[i];
	}

	for(int i=0; i<18; i++)
	{
		CURR_RAVEN_STATE.ori[i] = msg.ori[i];
		CURR_RAVEN_STATE.ori_d[i] = msg.ori_d[i];
	}
	// (2) update recieved data count
	RAVEN_SUB_COUNT ++;
}


void MyTracker_Controller::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
	if(!cv_ptr_SEG && (IMAGE_STATUS == EMPTY_IMAGE || IMAGE_STATUS == DONE_PUBLISHING))
	{
		IMAGE_STATUS = START_LOADING;
		try
		{
			cv_ptr_RAW = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
			cv_ptr_SEG = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
		IMAGE_STATUS = DONE_LOADING;
	}
	IMAGE_SUB_COUNT ++;
}

void MyTracker_Controller::ravenPb()
{
	static raven_2::raven_automove msg_raven_automove;	

	// (1) wrap up the new command	
	msg_raven_automove.hdr.stamp = msg_raven_automove.hdr.stamp.now(); //hdr

	tf::transformTFToMsg(TF_INCR[LEFT_ARM], msg_raven_automove.tf_incr[LEFT_ARM]);   //tf_incr
	tf::transformTFToMsg(TF_INCR[RIGHT_ARM], msg_raven_automove.tf_incr[RIGHT_ARM]);

	// (2) send new command
	raven_pub_.publish(msg_raven_automove);

	// (3) update published data count
	RAVEN_PUB_COUNT ++;
}


void MyTracker_Controller::imagePb() 
{
	// (1) check if ready to publish	
	if(IMAGE_STATUS == DONE_PROCESSING)
	{
		IMAGE_STATUS = START_PUBLISHING;

		// (2) send new command
		try
		{	
			image_pub_.publish(cv_ptr_SEG->toImageMsg());
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
		
		IMAGE_STATUS = DONE_PUBLISHING;
	}

	// (3) update published data count
	IMAGE_PUB_COUNT ++;
}


void MyTracker_Controller::process_image_segmenting()
{
	if(IMAGE_STATUS == DONE_LOADING)
	{
		IMAGE_STATUS = START_PROCESSING;
		//Todo:
		//(1) project raven state to 2D ? -->bounding box
		
		//(2) segment image
		
		//(3) draw?
		if (cv_ptr_SEG->image.rows > 60 && cv_ptr_SEG->image.cols > 60)
		{
			cv::circle(cv_ptr_SEG->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
		}
		
		IMAGE_STATUS = DONE_PROCESSING;
	}
	
}

void MyTracker_Controller::process_raven_command()
{
	//Todo:
	// TF_INCR[LEFT_ARM] and TF_INCR[RIGHT_ARM] will be published
	// based on keyboard and LEFT_MOTION, RIGHT_MOTION, RAVEN_CURRENT_STATE	
	TF_INCR[LEFT_ARM] = LEFT_MOTION.ComputeNullMotion(); 
	TF_INCR[RIGHT_ARM] = RIGHT_MOTION.ComputeNullMotion(); 
	
}


int MyTracker_Controller::getKey() 
{
    	int character;
    	struct termios orig_term_attr;
    	struct termios new_term_attr;

    	// set the terminal to raw mode 
    	tcgetattr(fileno(stdin), &orig_term_attr);
    	memcpy(&new_term_attr, &orig_term_attr, sizeof(struct termios));
    	new_term_attr.c_lflag &= ~(ECHO|ICANON);
    	new_term_attr.c_cc[VTIME] = 0;
    	new_term_attr.c_cc[VMIN] = 0;
    	tcsetattr(fileno(stdin), TCSANOW, &new_term_attr);

    	// read a character from the stdin stream without blocking 
    	//   returns EOF (-1) if no character is available 
    	character = fgetc(stdin);

   	// restore the original terminal attributes 
    	tcsetattr(fileno(stdin), TCSANOW, &orig_term_attr);

    	return character;
}

void MyTracker_Controller::console_display()
{
	cout<<"raven subscribe number: "<<RAVEN_SUB_COUNT <<endl;
	cout<<"raven publish number: "<<RAVEN_PUB_COUNT <<endl;
	cout<<"image subscribe number: "<<IMAGE_SUB_COUNT <<endl;
	cout<<"image publish number: "<<IMAGE_PUB_COUNT <<endl;
	cout<<endl;
}
