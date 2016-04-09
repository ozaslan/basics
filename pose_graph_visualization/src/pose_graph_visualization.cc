#include <ros/ros.h>
#include<nav_msgs/Path.h>
#include<nav_msgs/Odometry.h>
#include<tf/transform_broadcaster.h>
#include<visualization_msgs/Marker.h>
#include<visualization_msgs/MarkerArray.h>
#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/PoseWithCovarianceStamped.h>

#include <Eigen/Dense> 
#include <Eigen/Eigenvalues> 

#include <vector>
#include <string>
#include <iostream>

#include <utils.hh>
#include "pose_graph_visualization/PoseGraph.h"

using namespace std;

/*
  This node listens to PoseGraph a topic which has
  a PoseArray and Connectivity Matrix. It outputs
  two marker arrays one of which is for the poses
  and the other for the edges between poses.
*/

double refresh_rate;
bool   debug_mode;
Eigen::Vector4d edge_color;
double axis_length;
double edge_thickness;

ros::Publisher  pose_array_publ,
                edge_array_publ;
ros::Subscriber pose_graph_subs;

pose_graph_visualization::PoseGraph pose_graph_msg;
visualization_msgs::Marker pose_array_msg;
visualization_msgs::Marker edge_array_msg;

void publish_pose_array();
void publish_edge_array();

void pose_graph_callback(const pose_graph_visualization::PoseGraph& msg);

void process_inputs(const ros::NodeHandle &n);
void setup_messaging_interface(ros::NodeHandle &n);
void loop(const ros::NodeHandle &n);

int main(int argc, char* argv[]){

	ros::init(argc, argv, "pose_graph_visualization_node");
	ros::NodeHandle n("~");

	process_inputs(n);
	setup_messaging_interface(n);
	loop(n);	

	return 1;
}

void process_inputs(const ros::NodeHandle &n){

	n.param("refresh_rate", refresh_rate, 100.0);
	n.param("debug_mode", debug_mode, false);

	n.param("edge_color_r"  , edge_color(0), 1.0);
	n.param("edge_color_g"  , edge_color(1), 0.0);
	n.param("edge_color_b"  , edge_color(2), 1.0);
	n.param("edge_color_a"  , edge_color(3), 1.0);
	n.param("axis_length"   , axis_length  , 1.0);
	n.param("edge_thickness", edge_thickness, 1.0);

	ROS_INFO(" ---------- POSE GRAPH VISUALIZATION ------------");
	ROS_INFO("[refresh_rate] ------ : [%.3f - (doesn't run on fixed rate)]", refresh_rate);
	ROS_INFO("[debug_mode] -------- : [%s]", debug_mode ? "TRUE" : "FALSE");
	ROS_INFO("[edge_color (rgba)] - : [%.3f, %.3f, %.3f, %.3f]", edge_color(0), edge_color(1), edge_color(2), edge_color(3));
	ROS_INFO("[edge_thickness] ---- : [%.3f]", edge_thickness);
	ROS_INFO("[axis_length] ------- : [%.3f]", axis_length);
	ROS_INFO(" ----------------------------------------");
}

void setup_messaging_interface(ros::NodeHandle &n)
{
  pose_array_publ = n.advertise<visualization_msgs::Marker>("pose_graph/poses", 10);
  edge_array_publ = n.advertise<visualization_msgs::Marker>("pose_graph/edges", 10);
  pose_graph_subs = n.subscribe("pose_graph", 10, pose_graph_callback, ros::TransportHints().tcpNoDelay());
}

void loop(const ros::NodeHandle &n)
{
	ros::spin();
}	

void pose_graph_callback(const pose_graph_visualization::PoseGraph& msg){
	if(debug_mode)
		ROS_INFO("POSE GRAPH VISUALIZATION : Got PoseGraph Data");

  pose_graph_msg = msg;
	publish_pose_array();
	publish_edge_array();
}

void publish_pose_array(){
	if(debug_mode)
		ROS_INFO("POSE GRAPH VISUALIZATION : Publish Pose Data");

  static bool initialized = false;

  pose_array_msg.header.stamp = ros::Time::now();
  pose_array_msg.header.frame_id = "world";
  pose_array_msg.header.seq++;

  if(initialized == false){
    pose_array_msg.ns = "pose_graph/edges";
    pose_array_msg.id = 0;
    pose_array_msg.type = visualization_msgs::Marker::LINE_LIST;
    pose_array_msg.action = visualization_msgs::Marker::ADD;
    pose_array_msg.pose.position.x = 0;
    pose_array_msg.pose.position.y = 0;
    pose_array_msg.pose.position.z = 0;
    pose_array_msg.pose.orientation.w = 1;
    pose_array_msg.pose.orientation.x = 0;
    pose_array_msg.pose.orientation.y = 0;
    pose_array_msg.pose.orientation.z = 0;
    pose_array_msg.color.r = 1;
    pose_array_msg.color.g = 1;
    pose_array_msg.color.b = 1;
    pose_array_msg.color.a = 1;
    pose_array_msg.scale.x = .05;
    pose_array_msg.scale.y = .05;
    pose_array_msg.scale.z = .05;
    pose_array_msg.lifetime = ros::Duration(0);
    pose_array_msg.frame_locked = true;
  }

  initialized = true;

  int num_edges = pose_graph_msg.edges.size() / 2;
  pose_array_msg.points.resize(6 * num_edges);
  pose_array_msg.colors.resize(6 * num_edges);

  Eigen::Matrix3d dcm;

  for(int i = 0 ; i < num_edges ; i++){
    geometry_msgs::Pose pose = pose_graph_msg.poses[i];
    dcm = axis_length * utils::trans::pose2se3(pose).topLeftCorner<3, 3>();
 
    pose_array_msg.points[6 * i + 0].x = pose.position.x;
    pose_array_msg.points[6 * i + 0].y = pose.position.y;
    pose_array_msg.points[6 * i + 0].z = pose.position.z;
    pose_array_msg.points[6 * i + 1].x = pose.position.x + dcm(0, 0);
    pose_array_msg.points[6 * i + 1].y = pose.position.y + dcm(1, 0);
    pose_array_msg.points[6 * i + 1].z = pose.position.z + dcm(2, 0);
    pose_array_msg.points[6 * i + 2].x = pose.position.x;
    pose_array_msg.points[6 * i + 2].y = pose.position.y;
    pose_array_msg.points[6 * i + 2].z = pose.position.z;
    pose_array_msg.points[6 * i + 3].x = pose.position.x + dcm(0, 1);
    pose_array_msg.points[6 * i + 3].y = pose.position.y + dcm(1, 1);
    pose_array_msg.points[6 * i + 3].z = pose.position.z + dcm(2, 1);
    pose_array_msg.points[6 * i + 4].x = pose.position.x;
    pose_array_msg.points[6 * i + 4].y = pose.position.y;
    pose_array_msg.points[6 * i + 4].z = pose.position.z;
    pose_array_msg.points[6 * i + 5].x = pose.position.x + dcm(0, 2);
    pose_array_msg.points[6 * i + 5].y = pose.position.y + dcm(1, 2);
    pose_array_msg.points[6 * i + 5].z = pose.position.z + dcm(2, 2);

    pose_array_msg.colors[6 * i + 0].r = 1;
    pose_array_msg.colors[6 * i + 0].g = 0;
    pose_array_msg.colors[6 * i + 0].b = 0;
    pose_array_msg.colors[6 * i + 0].a = 1;
    pose_array_msg.colors[6 * i + 1].r = 1;
    pose_array_msg.colors[6 * i + 1].g = 0;
    pose_array_msg.colors[6 * i + 1].b = 0;
    pose_array_msg.colors[6 * i + 1].a = 1;
    pose_array_msg.colors[6 * i + 2].r = 0;
    pose_array_msg.colors[6 * i + 2].g = 1;
    pose_array_msg.colors[6 * i + 2].b = 0;
    pose_array_msg.colors[6 * i + 2].a = 1;
    pose_array_msg.colors[6 * i + 3].r = 0;
    pose_array_msg.colors[6 * i + 3].g = 1;
    pose_array_msg.colors[6 * i + 3].b = 0;
    pose_array_msg.colors[6 * i + 3].a = 1;
    pose_array_msg.colors[6 * i + 4].r = 0;
    pose_array_msg.colors[6 * i + 4].g = 0;
    pose_array_msg.colors[6 * i + 4].b = 1;
    pose_array_msg.colors[6 * i + 4].a = 1;
    pose_array_msg.colors[6 * i + 5].r = 0;
    pose_array_msg.colors[6 * i + 5].g = 0;
    pose_array_msg.colors[6 * i + 5].b = 1;
    pose_array_msg.colors[6 * i + 5].a = 1;
  }

	pose_array_publ.publish(pose_array_msg);
}

void publish_edge_array(){
	if(debug_mode)
		ROS_INFO("POSE GRAPH VISUALIZATION : Publish Edge Data");

  static bool initialized = false;

  edge_array_msg.header.stamp = ros::Time::now();
  edge_array_msg.header.frame_id = "world";
  edge_array_msg.header.seq++;

  if(initialized == false){
    edge_array_msg.ns = "pose_graph/edges";
    edge_array_msg.id = 0;
    edge_array_msg.type = visualization_msgs::Marker::LINE_LIST;
    edge_array_msg.action = visualization_msgs::Marker::ADD;
    edge_array_msg.pose.position.x = 0;
    edge_array_msg.pose.position.y = 0;
    edge_array_msg.pose.position.z = 0;
    edge_array_msg.pose.orientation.w = 1;
    edge_array_msg.pose.orientation.x = 0;
    edge_array_msg.pose.orientation.y = 0;
    edge_array_msg.pose.orientation.z = 0;
    edge_array_msg.color.r = edge_color(0);
    edge_array_msg.color.g = edge_color(1);
    edge_array_msg.color.b = edge_color(2);
    edge_array_msg.color.a = edge_color(3);
    edge_array_msg.scale.x = .03;
    edge_array_msg.scale.y = .03;
    edge_array_msg.scale.z = .03;
    edge_array_msg.lifetime = ros::Duration(0);
    edge_array_msg.frame_locked = true;
  }

  initialized = true;

  int num_edges = pose_graph_msg.edges.size() / 2;
  edge_array_msg.points.resize(2 * num_edges);

  for(int i = 0 ; i < num_edges ; i++){
    int id1, id2;
    id1 = pose_graph_msg.edges[2 * i];
    id2 = pose_graph_msg.edges[2 * i + 1];
    geometry_msgs::Pose pose1 = pose_graph_msg.poses[id1];
    geometry_msgs::Pose pose2 = pose_graph_msg.poses[id2];
    edge_array_msg.points[2 * i].x = pose1.position.x;
    edge_array_msg.points[2 * i].y = pose1.position.y;
    edge_array_msg.points[2 * i].z = pose1.position.z;
    edge_array_msg.points[2 * i + 1].x = pose2.position.x;
    edge_array_msg.points[2 * i + 1].y = pose2.position.y;
    edge_array_msg.points[2 * i + 1].z = pose2.position.z;
  }

	edge_array_publ.publish(edge_array_msg);
}
