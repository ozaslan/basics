#include "uniform_feat_tracker.hh"
#include "uniform_feat_extractor.hh"  

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

ros::Subscriber image_subs;
ros::Publisher  image_publ;

string image_mask_path = "";
cv_bridge::CvImagePtr image_msg;
UniformFeatureExtractor unif_feat_extractor("fast", 2, 500, Size2i(6, 7), 5, true);
UniformFeatureTracker unif_feat_tracker(500, 3);

int  setup_messaging_interface(ros::NodeHandle &n);
void image_callback(const sensor_msgs::Image &msg);
void process_inputs(const ros::NodeHandle &n);
int  publish_image();

bool debug_mode = false;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "uniform_feat_tracker_app");
	ros::NodeHandle n("~");
	
  process_inputs(n);
	setup_messaging_interface(n);
	
	ros::spin();
	
	return 0;
}

void process_inputs(const ros::NodeHandle &n)
{     
  n.param("debug_mode", debug_mode, true);

  n.param("image_mask_path", image_mask_path, string("no_mask_path"));
}


int setup_messaging_interface(ros::NodeHandle &n)
{
	if(debug_mode)	{
		ROS_INFO("UNIF FEAT TRACKER APP : setting up messaging interface.");
		ROS_INFO(" --- Listening  : /cam3/image");
		ROS_INFO(" --- Publishing : /uniform_flows");
	}
		
	image_subs  = n.subscribe("/cam3/image", 10, image_callback, ros::TransportHints().tcpNoDelay());
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

  cv::Mat gray_img, color_img = image_msg->image * 1;
  vector<cv::Mat> channels;

  cv::split(color_img, channels); 
  cv::equalizeHist(channels[0], channels[0]);
  cv::equalizeHist(channels[1], channels[1]);
  cv::equalizeHist(channels[2], channels[2]);
  cv::merge(channels, color_img);


  if(color_img.channels() == 3)
    cv::cvtColor(color_img, gray_img, CV_BGR2GRAY);
  else if(color_img.channels() == 4)
    cv::cvtColor(color_img, gray_img, CV_BGRA2GRAY);
  else
    gray_img = color_img;

  cv::GaussianBlur(gray_img, gray_img, cv::Size(13, 13), 5, 5, BORDER_DEFAULT );

  cv::Mat mask2;
  cv::threshold(gray_img, mask2, 150, 255, cv::THRESH_BINARY);

  cv::adaptiveThreshold(gray_img, gray_img, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 55, -2.5);

  cv::Mat mask = cv::imread(image_mask_path);
  if(mask.channels() != 1)
    cv::cvtColor(mask, mask, CV_BGR2GRAY);

  mask = cv::min(mask, mask2);
  cv::namedWindow("mask");
  cv::imshow("mask", mask);
  cv::namedWindow("gray_img");
  cv::imshow("gray_img", gray_img);

  cv::waitKey(1);

	unif_feat_tracker.track_features(gray_img, unif_feat_extractor, mask);
	unif_feat_tracker.plot_flow(image_msg->image, true, true);

	publish_image();
	frame_cnt++;
}
