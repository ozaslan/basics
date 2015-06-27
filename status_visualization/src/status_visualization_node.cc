#include<vector>
#include<iostream>

#include<ros/ros.h>
#include<com_msgs/RC.h>
#include<com_msgs/Status.h>
#include<sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>

#include<rc_proc.hh>

ros::Subscriber rc_subs;
ros::Subscriber status_subs;
ros::Publisher  panel_publ;

double refresh_rate;
bool debug_mode;

RCProc rc_proc;
double voltage;
double current;
vector<float> cpu_load;

int process_inputs(const ros::NodeHandle &n);
int setup_messaging_interface(ros::NodeHandle &n);
int loop(const ros::NodeHandle &n);
void rc_callback(const com_msgs::RC &msg);
void status_callback(const com_msgs::Status &msg);
void publish_panel();

/*
   - process the inputs
   - setup messaging interface
   - setup callbacks
   - loop
 */

int main(int argc, char* argv[]){

	ros::init(argc, argv, "status_visualization_node");
	ros::NodeHandle n("~");

	process_inputs(n);

	setup_messaging_interface(n);

	loop(n);        

	return 1;
}

int process_inputs(const ros::NodeHandle &n)
{
	string rc_config_path;

	n.param("refresh_rate"    , refresh_rate    ,  25.0);
	n.param("debug_mode"      , debug_mode      , false);
	n.param("rc_config_path"  , rc_config_path  , string(""));

	// RC Related params	
	rc_proc.load_params(rc_config_path);
	rc_proc.normalize() = true;

	ROS_INFO(" ---------------- STATUS VISUALIZATION ------------------");
	ROS_INFO("[debug_mode] ----------- : [%s]", debug_mode ? "TRUE" : "FALSE");
	ROS_INFO("[refresh_rate] --------- : [%.3lf]", refresh_rate);
	ROS_INFO("[rc_config_path] ------- : [%s]" , rc_config_path.c_str());
	rc_proc.print_params();
	ROS_INFO(" ------------------------------------------------");

	return 0;
}

int setup_messaging_interface(ros::NodeHandle &n){
	rc_subs     = n.subscribe("rc"    , 10,       rc_callback, ros::TransportHints().tcpNoDelay());
	status_subs = n.subscribe("status", 10,   status_callback, ros::TransportHints().tcpNoDelay());
	panel_publ  = n.advertise<sensor_msgs::Image>("panel", 10);

	return 0;
}

int loop(const ros::NodeHandle &n)
{
	ros::Rate r(refresh_rate); 

	while(ros::ok()){
		ros::spinOnce();
		publish_panel();
		r.sleep();
	}

	return 0;
}

void rc_callback(const com_msgs::RC &msg){
	if(debug_mode)
		ROS_INFO("RC NAV : Got RC message!");
	double rc_x, rc_y, rc_z, rc_psi;

	rc_proc.process(msg, rc_x, rc_y, rc_z, rc_psi);
}

void status_callback(const com_msgs::Status &msg){
	if(debug_mode)
		ROS_INFO("RC NAV : Got Status message!");

	voltage = msg.voltage;
	current = msg.current;
	cpu_load = msg.cpu_load;
}

void publish_panel(){
	//static cv::Mat panel()
}
