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

using namespace std;

double sample_count;
double refresh_rate;
bool   debug_mode;

ros::Publisher cpu_load_publ;

std::vector<long long> get_idle();
std::vector<float> get_load();

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

//This function reads /proc/stat and returns the idle value for each cpu in a vector
std::vector<long long> get_idle() {

  cout << "Q0" << endl;

  //Virtual file, created by the Linux kernel on demand
  std::ifstream in( "/proc/stat" );

  std::vector<long long> result;

  //cout << in.rdbuf() << endl;

  //This might broke if there are not 8 columns in /proc/stat
  boost::regex reg("cpu(\\d+) (\\d+) (\\d+) (\\d+) (\\d+) (\\d+) (\\d+) (\\d+) (\\d+) (\\d+) (\\d+)");

  cout << "Q1" << endl;

  std::string line;
  while ( std::getline(in, line) ) {

    boost::smatch match;
    if ( boost::regex_match( line, match, reg ) ) {

      long long idle_time = boost::lexical_cast<long long>(match[5]);

      result.push_back( idle_time );
    }
  }
  return result;
}

//This function returns the avarege load in the next interval_seconds for each cpu in a vector
//get_load() halts this thread for interval_seconds
std::vector<float> get_load() {
  static vector<boost::posix_time::ptime> time_stamps(sample_count);
  static vector<vector<long long> > idle_times(sample_count);

  // Erase the oldest time stamp and record the latest one
  time_stamps.erase(time_stamps.begin());
  time_stamps.push_back(boost::date_time::microsec_clock<boost::posix_time::ptime>::universal_time());

  // The same for the 'idle_times'
  idle_times.erase(idle_times.begin());
  idle_times.push_back(get_idle());

  //boost::posix_time::ptime current_time_1 = boost::date_time::microsec_clock<boost::posix_time::ptime>::universal_time();
  //std::vector<long long> idle_time_1 = get_idle();

  //sleep(interval_seconds);

  //boost::posix_time::ptime current_time_2 = boost::date_time::microsec_clock<boost::posix_time::ptime>::universal_time();
  //std::vector<long long> idle_time_2 = get_idle();

  //We have to measure the time, beacuse sleep is not accurate
  const float total_seconds_elapsed = float((time_stamps.back() - time_stamps.front()).total_milliseconds()) / 1000.f;

  std::vector<float> cpu_loads;

  for ( unsigned i = 0; i < idle_times.front().size(); ++i ) {
    //This might get slightly negative, because our time measurment is not accurate
    const float load = 1.f - float(idle_times.back()[i] - idle_times.front()[i])/(100.f * total_seconds_elapsed);
    cpu_loads.push_back( 100 * load );

  }

  return cpu_loads;
}

void publish_cpu_usage(){
  if(debug_mode){
    ROS_INFO("CPU USAGE : Publish CPU Usage");
  }

  static cpu_usage::CPULoad msg;
  msg.header.stamp = ros::Time::now();
  msg.header.seq++;
  msg.header.frame_id = "time";

  msg.loads = get_load();

  cpu_load_publ.publish(msg);
}
