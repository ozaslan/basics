#include <ros/ros.h>
#include<nav_msgs/Path.h>
#include<nav_msgs/Odometry.h>
#include<tf/transform_broadcaster.h>
#include<visualization_msgs/Marker.h>
#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/PoseWithCovarianceStamped.h>

#include <Eigen/Dense> 
#include <Eigen/Eigenvalues> 

#include <vector>
#include <string>
#include <iostream>

#include <utils.hh>

using namespace std;

double refresh_rate;
bool   debug_mode;
double max_path_len;
double mesh_scale;
double cov_scale;

tf::TransformBroadcaster *tf_br;
ros::Publisher mesh_publ,
	pose_publ,
	path_publ,
	cov_publ,
	vel_publ;
ros::Subscriber odom_subs;

string mesh_resource;
string frame_id;
Eigen::Vector4d cov_color,
				vel_color,
				mesh_color;

nav_msgs::Path path_msg;
nav_msgs::Odometry odom_msg;
geometry_msgs::PoseWithCovarianceStamped pose_msg;
visualization_msgs::Marker mesh_msg;
visualization_msgs::Marker vel_msg;
visualization_msgs::Marker cov_msg;

void publish_mesh();
void publish_path();
void publish_covariance();
void publish_tf();
void publish_velocity();
void publish_pose();

void odom_callback(const nav_msgs::Odometry& msg);

void process_inputs(const ros::NodeHandle &n);
void setup_messaging_interface(ros::NodeHandle &n);
void loop(const ros::NodeHandle &n);

int main(int argc, char* argv[]){

	ros::init(argc, argv, "odom_visualization_node");
	ros::NodeHandle n("~");

	process_inputs(n);
	setup_messaging_interface(n);
	loop(n);	

	return 1;
}

void process_inputs(const ros::NodeHandle &n){

	n.param("refresh_rate", refresh_rate, 100.0);
	n.param("debug_mode", debug_mode, false);

	n.param("mesh_resource", mesh_resource	 , string("package://odom_visualization/meshes/ins_khex.stl"));
	n.param("frame_id"	, frame_id, string("world"));
	n.param("mesh_color_r", mesh_color(0), 1.0);
	n.param("mesh_color_g", mesh_color(1), 0.0);
	n.param("mesh_color_b", mesh_color(2), 0.0);
	n.param("mesh_color_a", mesh_color(3), 1.0);
	n.param("cov_color_r" , cov_color(0) , 1.0);
	n.param("cov_color_g" , cov_color(1) , 0.0);
	n.param("cov_color_b" , cov_color(2) , 0.0);
	n.param("cov_color_a" , cov_color(3) , 1.0);
	n.param("vel_color_r" , vel_color(0) , 1.0);
	n.param("vel_color_g" , vel_color(1) , 0.0);
	n.param("vel_color_b" , vel_color(2) , 0.0);
	n.param("vel_color_a" , vel_color(3) , 1.0);
	n.param("max_path_len", max_path_len , 1000.0);
	n.param("mesh_scale"  , mesh_scale   , 1.0);
	n.param("cov_scale"   , cov_scale    , 1.0);

	ROS_INFO(" ---------- ODOM VISUALIZATION ------------");
	ROS_INFO("[refresh_rate] ------ : [%.3f - (doesn't run on fixed rate)]", refresh_rate);
	ROS_INFO("[debug_mode] -------- : [%s]", debug_mode ? "TRUE" : "FALSE");
	ROS_INFO("[mesh_resource] ----- : [%s]", mesh_resource.c_str());
	ROS_INFO("[max_path_len] ------ : [%.3f]", max_path_len);
	ROS_INFO("[mesh, cov]_scale --- : [%.3f, %.3f]", mesh_scale, cov_scale);
	ROS_INFO("[mesh_color (rgba)] - : [%.3f, %.3f, %.3f, %.3f]", mesh_color(0), mesh_color(1), mesh_color(2), mesh_color(3));
	ROS_INFO("[cov_color (rgba)] -- : [%.3f, %.3f, %.3f, %.3f]", cov_color(0), cov_color(1), cov_color(2), cov_color(3));
	ROS_INFO("[vel_color (rgba)] -- : [%.3f, %.3f, %.3f, %.3f]", vel_color(0), vel_color(1), vel_color(2), vel_color(3));
	ROS_INFO(" ----------------------------------------");
}

void setup_messaging_interface(ros::NodeHandle &n)
{
	path_publ = n.advertise<nav_msgs::Path>("path", 10);
	mesh_publ = n.advertise<visualization_msgs::Marker>("mesh", 10);
	pose_publ = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose", 10);
	cov_publ  = n.advertise<visualization_msgs::Marker>("cov", 10);
	vel_publ  = n.advertise<visualization_msgs::Marker>("vel", 10);
	odom_subs = n.subscribe("odom", 10, odom_callback, ros::TransportHints().tcpNoDelay()); 
	tf_br = new (std::nothrow) tf::TransformBroadcaster[1];
}

void loop(const ros::NodeHandle &n)
{
	ros::spin();
}	

void odom_callback(const nav_msgs::Odometry &msg){
	if(debug_mode)
		ROS_INFO("ODOM VISUALIZATION : Got Odometry Data");

	odom_msg = msg;

	publish_mesh();
	publish_path();
	publish_covariance();
	publish_tf();
	publish_velocity();
	publish_pose();
}

void publish_mesh(){
	if(debug_mode)
		ROS_INFO("ODOM VISUALIZATION : Publish Mesh Data");

	mesh_msg.header.stamp = ros::Time::now();
	mesh_msg.header.frame_id = frame_id;
	mesh_msg.header.seq++;
	mesh_msg.ns = "odom_visualization";
	mesh_msg.id = 0;
	mesh_msg.type = visualization_msgs::Marker::MESH_RESOURCE;
	mesh_msg.action = visualization_msgs::Marker::ADD;
	mesh_msg.pose = odom_msg.pose.pose;
	mesh_msg.scale.x = mesh_scale;
	mesh_msg.scale.y = mesh_scale;
	mesh_msg.scale.z = mesh_scale;
	mesh_msg.color.r = mesh_color(0);
	mesh_msg.color.g = mesh_color(1);
	mesh_msg.color.b = mesh_color(2);
	mesh_msg.color.a = mesh_color(3);
	mesh_msg.lifetime = ros::Duration(0);
	mesh_msg.frame_locked = true;
	mesh_msg.mesh_resource = mesh_resource;

	mesh_publ.publish(mesh_msg);
}

void publish_path(){
	if(debug_mode)
		ROS_INFO("ODOM VISUALIZATION : Publish Path Data");

	path_msg.header.stamp = ros::Time::now();
	path_msg.header.frame_id = frame_id;
	path_msg.header.seq++;

	geometry_msgs::PoseStamped pose_stamped;
	pose_stamped.header = odom_msg.header;
	pose_stamped.pose = odom_msg.pose.pose;
	path_msg.poses.push_back(pose_stamped);

	
	if((int)path_msg.poses.size() > max_path_len){
		path_msg.poses.erase(path_msg.poses.begin(), path_msg.poses.begin() + (path_msg.poses.size() - max_path_len));
	}
	

	path_publ.publish(path_msg);
}

void publish_covariance(){
	if(debug_mode)
		ROS_INFO("ODOM VISUALIZATION : Publish Covariance Visuals");

	cov_msg.header.stamp = ros::Time::now();
	cov_msg.header.frame_id = frame_id;
	cov_msg.header.seq++;
	cov_msg.ns = "cov";
	cov_msg.id = 0;
	cov_msg.type = visualization_msgs::Marker::SPHERE;
	cov_msg.action = visualization_msgs::Marker::ADD;
	cov_msg.pose = odom_msg.pose.pose;

	// Eigen value decomposition to get the axis scales
	Eigen::Matrix3d cov;
	for(int r = 0 ; r < 3 ; r++)
		for(int c = 0 ; c < 3 ; c++)
			cov(r, c) = odom_msg.pose.covariance[r * 6 + c];

	SelfAdjointEigenSolver<Matrix3d> es(cov);

	Eigen::Matrix3d evecs = es.eigenvectors();
	Eigen::Vector3d evals = es.eigenvalues();

  double det = evecs.determinant();
  if(det < 0)
    evecs.topLeftCorner<3, 1>() *= -1;

  //cout << "COW = [" << cov << "];" << endl;
  //cout << "evecs = [" << evecs << "];" << endl;
  //cout << "evals = [" << evals << "];" << endl;

	Eigen::Vector4d quat = utils::trans::dcm2quat(evecs);

	cov_msg.pose.orientation.w = quat(0);
	cov_msg.pose.orientation.x = quat(1);
	cov_msg.pose.orientation.y = quat(2);
	cov_msg.pose.orientation.z = quat(3);
	
	cov_msg.scale.x = evals(0) * cov_scale;
	cov_msg.scale.y = evals(1) * cov_scale;
	cov_msg.scale.z = evals(2) * cov_scale;
	cov_msg.color.r = cov_color(0);
	cov_msg.color.g = cov_color(1);
	cov_msg.color.b = cov_color(2);
	cov_msg.color.a = cov_color(3);
	cov_msg.lifetime = ros::Duration(0);
	cov_msg.frame_locked = true;
	cov_msg.points.resize(1);

	cov_publ.publish(cov_msg);
}

void publish_tf(){
	if(debug_mode)
		ROS_INFO("ODOM VISUALIZATION : Publish tf");

	tf::Transform transform;
	transform.setOrigin( tf::Vector3( odom_msg.pose.pose.position.x,
				odom_msg.pose.pose.position.y,
				odom_msg.pose.pose.position.z) );
	tf::Quaternion q(odom_msg.pose.pose.orientation.x,
			odom_msg.pose.pose.orientation.y,
			odom_msg.pose.pose.orientation.z,
			odom_msg.pose.pose.orientation.w);
	transform.setRotation(q);
	tf_br->sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_id, "robot"));
}

void publish_velocity(){
	if(debug_mode)
		ROS_INFO("ODOM VISUALIZATION : Publish Velocity");

	vel_msg.header.stamp = ros::Time::now();
	vel_msg.header.frame_id = frame_id;
	vel_msg.header.seq++;
	vel_msg.ns = "vel";
	vel_msg.id = 0;
	vel_msg.type = visualization_msgs::Marker::ARROW;
	vel_msg.action = visualization_msgs::Marker::ADD;
	vel_msg.pose = odom_msg.pose.pose;

	// -------------------------------------------------------------------------- //
	// Generate a rotation matrix that aligns the x-axis with the velocity vector //
	Eigen::Vector3d basis[3];
	basis[0](0) = odom_msg.twist.twist.linear.x;
	basis[0](1) = odom_msg.twist.twist.linear.y;
	basis[0](2) = odom_msg.twist.twist.linear.z;
	double vel_norm = basis[0].norm();

	basis[0] /= basis[0].norm() + 1e-6;
	basis[1] = basis[0];
	basis[1](0) += 1;
	basis[1] = basis[0].cross(basis[1]);
	basis[1] /= basis[1].norm() + 1e-6;
	basis[2] = basis[0].cross(basis[1]);
	basis[2] /= basis[2].norm() + 1e-6;

	Eigen::Matrix3d dcm;
	dcm.block<3, 1>(0, 0) = basis[0];
	dcm.block<3, 1>(0, 1) = basis[1];
	dcm.block<3, 1>(0, 2) = basis[2];
	Eigen::Vector4d quat = utils::trans::dcm2quat(dcm);
	// -------------------------------------------------------------------------- //

	vel_msg.pose.orientation.w = quat(0);
	vel_msg.pose.orientation.x = quat(1);
	vel_msg.pose.orientation.y = quat(2);
	vel_msg.pose.orientation.z = quat(3);

	vel_msg.scale.x = vel_norm;
	vel_msg.scale.y = .05;
	vel_msg.scale.z = .05;
	vel_msg.color.r = vel_color(0);
	vel_msg.color.g = vel_color(1);
	vel_msg.color.b = vel_color(2);
	vel_msg.color.a = vel_color(3);
	vel_msg.lifetime = ros::Duration(0);
	vel_msg.frame_locked = true;

	vel_msg.points.clear();

	vel_publ.publish(vel_msg);
}

void publish_pose(){
	if(debug_mode)
		ROS_INFO("ODOM VISUALIZATION : Publish Pose");

	pose_msg.header.stamp = ros::Time::now();
	pose_msg.header.frame_id = frame_id;
	pose_msg.header.seq++;

	pose_msg.pose = odom_msg.pose;

	pose_publ.publish(pose_msg);
}

