#include "MyTracker_MotionPlanner.h"

class MyTracker_Controller
{
	private:
		int RAVEN_VERSION;
		IMAGE_STATUS_LIST IMAGE_STATUS;

		int RAVEN_SUB_COUNT;
		int RAVEN_PUB_COUNT;
		int IMAGE_SUB_COUNT;
		int IMAGE_PUB_COUNT;

		pthread_t console_thread;
		pthread_t ros_image_thread;
		pthread_t ros_raven_thread;
	
		ros::NodeHandle nh_; 
		ros::Publisher   raven_pub_;
		ros::Subscriber  raven_sub_;
		
		image_transport::ImageTransport it_;
		image_transport::Subscriber image_sub_;
		image_transport::Publisher image_pub_;

		raven_2::raven_state CURR_RAVEN_STATE;
		tf::Transform TF_INCR[2];

		cv_bridge::CvImagePtr cv_ptr_RAW;
		cv_bridge::CvImagePtr cv_ptr_SEG;

		MyTracker_MotionPlanner LEFT_MOTION;
		MyTracker_MotionPlanner RIGHT_MOTION;

	public:
		MyTracker_Controller();
		~MyTracker_Controller();

		void initial(int, int, char**);
		void init_sys(int);
		bool init_ros(int, char**);

		void start_thread();
		void join_thread();
		void *console_process(void);
		void *ros_image_process(void);
		void *ros_raven_process(void);
		static void *static_console_process(void*);
		static void *static_ros_image_process(void*);
		static void *static_ros_raven_process(void*);

		void ravenPb();
		void ravenCb(const raven_2::raven_state);
		void imagePb();
		void imageCb(const sensor_msgs::ImageConstPtr&);

		void process_image_segmenting();
		void process_raven_command();	

		int getKey();
		void console_display();
};

