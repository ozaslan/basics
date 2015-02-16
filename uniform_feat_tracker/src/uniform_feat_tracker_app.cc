#include "uniform_feat_tracker.hh"
#include "uniform_feat_extractor.hh"  

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

ros::Subscriber image_subs;
ros::Publisher  image_publ;

cv_bridge::CvImagePtr image_msg;
UniformFeatureExtractor unif_feat_extractor("fast", 3, 100, Size2i(4, 3), 9, true);
UniformFeatureTracker unif_feat_tracker(100, 13);

int  setup_messaging_interface(ros::NodeHandle &n);
void image_callback(const sensor_msgs::Image &msg);
int  publish_image();

bool debug_mode = false;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "uniform_feat_tracker_app");
	ros::NodeHandle n("~");
	
	setup_messaging_interface(n);
	
	ros::spin();
	
	return 0;
}

int setup_messaging_interface(ros::NodeHandle &n)
{
	if(debug_mode)	{
		ROS_INFO("UNIF FEAT TRACKER APP : setting up messaging interface.");
		ROS_INFO(" --- Listening  : /cam0/image");
		ROS_INFO(" --- Publishing : /uniform_flows");
	}
		
	image_subs  = n.subscribe("/cam0/image", 10, image_callback, ros::TransportHints().tcpNoDelay());
	image_publ  = n.advertise<sensor_msgs::Image>("/uniform_flows", 10);

	return 0;
}

int publish_image()
{
	if(debug_mode)
		ROS_INFO("UNIF FEAT TRACKER APP : Published image to /uniform_flows");

	image_publ.publish(image_msg->toImageMsg());

	return 0;
}

void image_callback(const sensor_msgs::Image &msg)
{
	static int frame_cnt = 0;
	if(debug_mode)
		ROS_INFO("UNIF FEAT TRACKER APP : Got Image message!");	

	try	{
		image_msg = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	unif_feat_tracker.track_features(image_msg->image, unif_feat_extractor);
	unif_feat_tracker.plot_flow(image_msg->image, true, true);

	publish_image();
	frame_cnt++;
}
