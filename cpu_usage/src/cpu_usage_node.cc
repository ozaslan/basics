#include <unistd.h>

#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>

#include <boost/lexical_cast.hpp>
#include <boost/regex.hpp>
#include <boost/date_time.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include<ros/ros.h>
#include<cpu_usage/CPULoad.h>
#include "cpu_usage.hh"

using namespace std;

double sample_count;
double refresh_rate;
bool   debug_mode;

CpuUsage cpu_loads;
ros::Publisher cpu_load_publ;

int process_inputs(const ros::NodeHandle &n);
int setup_messaging_interface(ros::NodeHandle &n);
int loop(const ros::NodeHandle &n);
void publish_cpu_usage();

/*
   - process the inputs
   - setup messaging interface
   - setup callbacks
   - loop
 */

int process_inputs(const ros::NodeHandle &n)
{
  n.param("refresh_rate", refresh_rate,  25.0);
  n.param("sample_count", sample_count,   5.0);
  n.param("debug_mode"  ,   debug_mode,  true);

  ROS_INFO(" ---------------- CPU USAGE ----------------------");
  ROS_INFO("[debug_mode] ----------- : [%s]"   , debug_mode ? "TRUE" : "FALSE");
  ROS_INFO("[refresh_rate] --------- : [%.3lf]", refresh_rate);
  ROS_INFO("[sample_count] --------- : [%.1lf]", sample_count);
  ROS_INFO(" -------------------------------------------------");

  cpu_loads = CpuUsage(sample_count);

  return 0;
}

int setup_messaging_interface(ros::NodeHandle &n){
  if(debug_mode){
    ROS_INFO("CPU USAGE : Setting up messaging interface");
    ROS_INFO("CPU USAGE : Publishing to : /cpu_load topic name");
  }

  cpu_load_publ  = n.advertise<cpu_usage::CPULoad>("cpu_load", 10);

  return 0;
}

int loop(const ros::NodeHandle &n)
{
  ros::Rate r(refresh_rate); 

  while(ros::ok()){
    ros::spinOnce();
    publish_cpu_usage();
    r.sleep();
  }

  return 0;
}


int main(int argc, char* argv[]){

  ros::init(argc, argv, "cpu_usage_node");
  ros::NodeHandle n("~");

  process_inputs(n);

  setup_messaging_interface(n);

  loop(n);        

  return 1;
}


void publish_cpu_usage(){
  if(debug_mode){
    ROS_INFO("CPU USAGE : Publish CPU Usage");
  }

  static cpu_usage::CPULoad msg;
  msg.header.stamp = ros::Time::now();
  msg.header.seq++;
  msg.header.frame_id = "time";

  msg.loads = cpu_loads.get_usage();

  cpu_load_publ.publish(msg);
}
