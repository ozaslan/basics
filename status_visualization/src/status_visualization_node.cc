#include<vector>
#include<string>
#include<iostream>

#include<ros/ros.h>
#include<com_msgs/RC.h>
#include<com_msgs/Status.h>
#include<sensor_msgs/Image.h>
#include<opencv2/opencv.hpp>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/image_encodings.h>
#include<image_transport/image_transport.h>

#include<rc_proc.hh>

ros::Subscriber rc_subs;
ros::Subscriber status_subs;
ros::Publisher  panel_publ;

cv::Mat panel;

string title;
double refresh_rate;
double critical_voltage1,
       critical_voltage2,
       critical_voltage3;
bool debug_mode;

RCProc rc_proc;
vector<double> rc_vals;

double voltage;
double current;
vector<float> cpu_load;
cv::Rect roi_title,
  roi_sticks,
  roi_voltage,
  roi_current,
  roi_cpu;

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
  string panel_path;

  n.param("refresh_rate"     , refresh_rate  ,  25.0);
  n.param("debug_mode"       , debug_mode    , false);
  n.param("rc_config_path"   , rc_config_path, string(""));
  n.param("panel_path"       , panel_path    , string(""));
  n.param("title"            , title         , string("Title"));
  n.param("critical_voltage1", critical_voltage1, 0.0);
  n.param("critical_voltage2", critical_voltage2, 0.0);
  n.param("critical_voltage3", critical_voltage3, 0.0);

  // RC Related params	
  rc_proc.load_params(rc_config_path);
  rc_proc.normalize() = true;

  /*
  // Generate the panel template path
  string panel_path = panel_path;
  std::size_t found = panel_path.rfind(".");
  if (found != std::string::npos)
  panel_path.replace (found, 1, "_template.");
   */

  ROS_INFO(" ---------------- STATUS VISUALIZATION ------------------");
  ROS_INFO("[debug_mode] -------------- : [%s]", debug_mode ? "TRUE" : "FALSE");
  ROS_INFO("[refresh_rate] ------------ : [%.3lf]", refresh_rate);
  ROS_INFO("[rc_config_path] ---------- : [%s]" , rc_config_path.c_str());
  ROS_INFO("[panel_path] -------------- : [%s]" , panel_path.c_str());
  ROS_INFO("[title] ------------------- : [%s]" , title.c_str());
  ROS_INFO("critical_volgate[1, 2, 3] - : [%.3lf, %.3lf, %.3lf]", critical_voltage1, critical_voltage2, critical_voltage3);
  rc_proc.print_params();
  ROS_INFO(" ------------------------------------------------");

  rc_vals.resize(4);
  std::fill(rc_vals.begin(), rc_vals.end(), 0);
  // ------------------------------------------------------------------- //

  // Load the panel and panel template
  panel = cv::imread(panel_path);

  // Extract the plotting region from the panel template
  cv::Mat temp;
  vector<vector<cv::Point> > contours;
  vector<cv::Vec4i> hierarchy;

  inRange(panel, cv::Scalar(0, 0, 255), cv::Scalar(0, 0, 255), temp);
  findContours(temp, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
  roi_title = cv::boundingRect( cv::Mat(contours[0]) );

  inRange(panel, cv::Scalar(0, 255, 0), cv::Scalar(0, 255, 0), temp);
  findContours(temp, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
  roi_voltage = cv::boundingRect( cv::Mat(contours[0]) );

  inRange(panel, cv::Scalar(255 , 0, 0), cv::Scalar(255 , 0, 0), temp);
  findContours(temp, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
  roi_cpu = cv::boundingRect( cv::Mat(contours[0]) );

  inRange(panel, cv::Scalar(0, 255, 255), cv::Scalar(0, 255, 255), temp);
  findContours(temp, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
  roi_current = cv::boundingRect( cv::Mat(contours[0]) );

  inRange(panel, cv::Scalar(255 , 255, 0), cv::Scalar(255, 255, 0), temp);
  findContours(temp, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
  roi_sticks = cv::boundingRect( cv::Mat(contours[0]) );

  cout << roi_title << endl;
  cout << roi_voltage << endl;
  cout << roi_cpu << endl;
  cout << roi_sticks << endl;
  cout << roi_current << endl;

  // ------------------------------------------------------------------- //

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

  rc_proc.process(msg, rc_vals);
}

void status_callback(const com_msgs::Status &msg){
  if(debug_mode)
    ROS_INFO("RC NAV : Got Status message!");

  voltage = msg.voltage;
  current = msg.current;
  cpu_load.resize(1);
  cpu_load[0] = msg.cpu_load;

  cpu_load.clear();

  FILE *fip = popen("cat /proc/stat", "r");
  char line[9999];
  while (fgets(line, sizeof(line), fip)){
    stringstream ss(line);
    vector<string> items;
    items.reserve(33);
    string item;
    while (std::getline(ss, item, ' ')) {
      items.push_back(item);
    }

    if(items[0].substr(0, 3) == "cpu"){
      int cpu_no = items[0][3] - '0';
      if(cpu_no >= 0){
        double n, u, b, i;
        n = atof(items[1].c_str());
        u = atof(items[2].c_str());
        b = atof(items[3].c_str());
        i = atof(items[4].c_str());
        cpu_load.push_back((double)(n + u + b) / (n + u + b + i) * 100);
      }
    }
  }

  pclose(fip);

}

void publish_panel(){
  static cv::Mat temp_patch, temp_panel;
  if(temp_panel.cols != panel.cols || temp_panel.rows != panel.rows)
    temp_panel = panel * 0;
  temp_panel = cv::Scalar(255, 255, 255);

  // ---------------- Title ------------------------------------------ //

  temp_patch = temp_panel(roi_title);
  cv::rectangle(temp_panel, roi_title, cv::Scalar(255, 255, 255), -1);
  putText(temp_patch, title, cv::Point(10, 30), cv::FONT_HERSHEY_PLAIN | cv::FONT_ITALIC, 2, cv::Scalar(0, 0, 255), 2, 8, false);


  // ---------------- Voltage ---------------------------------------- //

  string integer, decimal;
  std::ostringstream ss;
  ss << voltage;
  std::size_t found = ss.str().find(".");
  if (found == std::string::npos){
    integer = ss.str() + ".";
    decimal = "000";
  } else {
    integer = ss.str().substr(0, found) + ".";
    decimal = ss.str().substr(found + 1);
    while(decimal.size() < 3)
      decimal += "0";

  }

  temp_patch = temp_panel(roi_voltage);

  cv::Scalar voltage_color;
  if(voltage < critical_voltage3){
    int sec = ros::Time::now().toSec();
    if(sec % 2 == 0)
      voltage_color = cv::Scalar(0, 0, 255);
    else
      voltage_color = cv::Scalar(255, 255, 255);
  } else if(voltage < critical_voltage2) {
    double ratio = 1 / (critical_voltage2 - critical_voltage3);
    double g = ratio * (voltage - critical_voltage3) * 255;
    voltage_color = cv::Scalar(0, g, 255);
  } else if(voltage < critical_voltage1) {
    double ratio = 1 / (critical_voltage1 - critical_voltage2);
    double r = ratio * (voltage - critical_voltage2) * 255;
    voltage_color = cv::Scalar(0, 255, 255 - r);
  } else {
    voltage_color = cv::Scalar(0, 255, 0);
  }

  cv::rectangle(temp_panel, roi_voltage, voltage_color, -1);
  putText(temp_patch, "Voltage", cv::Point(5, 50), cv::FONT_HERSHEY_PLAIN | cv::FONT_ITALIC, 4, cv::Scalar(255, 0, 0), 2, 8, false);
  putText(temp_patch, integer + decimal + " V.", cv::Point(0, 110), cv::FONT_HERSHEY_PLAIN, 3.8, cv::Scalar(255, 0, 0), 5, 8, false);

  // ------------ Sticks --------------------------------------------- // 

  cv::Rect roi_right_stick,
    roi_left_stick;
  roi_right_stick.width  = min(roi_sticks.width, roi_sticks.height);
  roi_right_stick.height = roi_right_stick.width;
  roi_left_stick.width   = roi_right_stick.width;
  roi_left_stick.height  = roi_right_stick.width;

  roi_right_stick.y = roi_sticks.y;
  roi_left_stick.y = roi_sticks.y;
  roi_right_stick.x = roi_sticks.x + roi_right_stick.width;
  roi_left_stick.x = roi_sticks.x;

  cv::circle(temp_panel, cv::Point(roi_right_stick.x + roi_right_stick.width  / 2,
        roi_right_stick.y + roi_right_stick.height / 2), 
      roi_right_stick.width / 2, cv::Scalar(200, 200, 200), -1);
  cv::circle(temp_panel, cv::Point( roi_left_stick.x +  roi_left_stick.width  / 2,
        roi_left_stick.y +  roi_left_stick.height / 2), 
      roi_left_stick.width / 2, cv::Scalar(200, 200, 200), -1);

  cv::circle(temp_panel, cv::Point(roi_right_stick.x + roi_right_stick.width  / 2,
        roi_right_stick.y + roi_right_stick.height / 2), 
      roi_right_stick.width / 2, cv::Scalar(200, 0, 0), 1);
  cv::circle(temp_panel, cv::Point( roi_left_stick.x +  roi_left_stick.width  / 2,
        roi_left_stick.y +  roi_left_stick.height / 2), 
      roi_left_stick.width / 2, cv::Scalar(200, 0, 0), 1);

  temp_patch = temp_panel(roi_right_stick);
  cv::line(temp_patch,  cv::Point(roi_right_stick.width / 2,
        roi_right_stick.height / 2),
      cv::Point((rc_vals[1] + 1) * roi_right_stick.width  / 2,
        (rc_vals[0] + 1) * roi_right_stick.height / 2), 
      cv::Scalar(255, 255, 255), 9);
  cv::circle(temp_patch, cv::Point((rc_vals[1] + 1) * roi_right_stick.width  / 2,
        (rc_vals[0] + 1) * roi_right_stick.height / 2), 
      10, cv::Scalar(0, 255, 0), -1);
  cv::circle(temp_patch, cv::Point((rc_vals[1] + 1) * roi_right_stick.width  / 2,
        (rc_vals[0] + 1) * roi_right_stick.height / 2), 
      5, cv::Scalar(255, 255, 255), -1);

  temp_patch = temp_panel(roi_left_stick);
  cv::line(temp_patch,  cv::Point(roi_left_stick.width / 2,
        roi_left_stick.height / 2),
      cv::Point((rc_vals[3] + 1) * roi_left_stick.width  / 2,
        (rc_vals[2] + 1) * roi_left_stick.height / 2), 
      cv::Scalar(255, 255, 255), 9);
  cv::circle(temp_patch, cv::Point((rc_vals[3] + 1) * roi_left_stick.width  / 2,
        (rc_vals[2] + 1) * roi_left_stick.height / 2), 
      10, cv::Scalar(0, 255, 0), -1);
  cv::circle(temp_patch, cv::Point((rc_vals[3] + 1) * roi_left_stick.width  / 2,
        (rc_vals[2] + 1) * roi_left_stick.height / 2), 
      5, cv::Scalar(255, 255, 255), -1);

  // ------------ CPU ------------------------------------------------ // 

  if(cpu_load.size() != 0){

    cv::rectangle(temp_panel, roi_cpu, cv::Scalar(230, 230, 230), -1);

    cv::Rect roi_cpu_bar;
    roi_cpu_bar.width  = roi_cpu.width / cpu_load.size();

    for(int i = 0 ; i < (int)cpu_load.size() ; i++){
      roi_cpu_bar.x = roi_cpu.x + i * roi_cpu_bar.width;
      roi_cpu_bar.height = roi_cpu.height * cpu_load[i] / 100.0;
      roi_cpu_bar.y = roi_cpu.y + roi_cpu.height - roi_cpu_bar.height;
      cv::rectangle(temp_panel, roi_cpu_bar, cv::Scalar(255, 0, 0), -1);
      cout << roi_cpu_bar << endl;
    }
  }

  cv::rectangle(temp_panel, roi_current, cv::Scalar(  0,   0, 0), 1);
  cv::rectangle(temp_panel, roi_voltage, cv::Scalar(  0,   0, 0), 1);
  cv::rectangle(temp_panel, roi_sticks , cv::Scalar(  0,   0, 0), 1);
  cv::rectangle(temp_panel, roi_title  , cv::Scalar(  0,   0, 0), 1);
  cv::rectangle(temp_panel, roi_cpu    , cv::Scalar(  0,   0, 0), 1);

  //cv::namedWindow("a");
  //cv::imshow("a", temp_panel);
  //cv::waitKey(1);

  static cv_bridge::CvImage panel_msg;
  panel_msg.header.stamp = ros::Time::now();
  panel_msg.header.seq++;
  panel_msg.header.frame_id = "panel";
  panel_msg.encoding = "bgr8"; 
  panel_msg.image = temp_panel;
  panel_publ.publish(panel_msg.toImageMsg()); 

}
