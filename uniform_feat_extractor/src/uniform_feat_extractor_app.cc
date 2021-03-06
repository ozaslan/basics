#include "uniform_feat_extractor.hh" 

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

ros::Subscriber image_subs;
ros::Publisher  image_publ;

string image_mask_path = "";
cv_bridge::CvImagePtr image_msg;
UniformFeatureExtractor unif_feat_detector("fast", 3, 600, Size2i(5, 4), 3, true);

int  setup_messaging_interface(ros::NodeHandle &n);
void image_callback(const sensor_msgs::Image &msg);
int  publish_image();
void process_inputs(const ros::NodeHandle &n);

bool debug_mode = false;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "uniform_feat_extractor_app");
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
    ROS_INFO("UNIF FEAT EXTRACTOR APP : setting up messaging interface.");
    ROS_INFO(" --- Listening  : /cam3/image");
    ROS_INFO(" --- Publishing : /uniform_feats");
  }

  image_subs  = n.subscribe("/cam3/image", 10, image_callback, ros::TransportHints().tcpNoDelay());
  image_publ  = n.advertise<sensor_msgs::Image>("/uniform_feats", 10);

  return 0;
}

int publish_image()
{
  if(debug_mode)
    ROS_INFO("UNIF FEAT EXTRACTOR APP : Published image to /uniform_feats");

  image_publ.publish(image_msg->toImageMsg());

  return 0;
}

void image_callback(const sensor_msgs::Image &msg)
{
  static int frame_cnt = 0;
  if(debug_mode)
    ROS_INFO("UNIF FEAT EXTRACTOR APP : Got Image message!");	

  try	{
    image_msg = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)	{
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  static vector<Point2f> features;
  std::random_shuffle(features.begin(), features.end());
  features.resize(features.size() * 0);

  cv::Mat mask = cv::imread(image_mask_path);
  if(mask.channels() != 1)
    cv::cvtColor(mask, mask, CV_BGR2GRAY);

  unif_feat_detector.extract_features(image_msg->image, features, mask);
  unif_feat_detector.plot_features(image_msg->image, features, true);
  publish_image();

  if(frame_cnt % 200 == 0){
    unif_feat_detector.set_detector("fast");
    unif_feat_detector.set_threshold(9);
    cout << "FAST " << endl;
  } else if(frame_cnt % 200 == 270){
    unif_feat_detector.set_detector("harris");
    unif_feat_detector.set_threshold(0.01);
    cout << "Harris " << endl;
  } else if(frame_cnt % 200 == 240){
    unif_feat_detector.set_detector("gftt");
    unif_feat_detector.set_threshold(0.01);
    cout << "GFTT" << endl;
  }

  frame_cnt++;
}


