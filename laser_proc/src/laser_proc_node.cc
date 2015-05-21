/*
   This node listens to a laser scanner, processes the data
   for the given parameter/filter set and publishes as LaserScan.
   If configured, this publishes transformed 3D point as pointcloud.
   The source of transformation can be sensor relative position,
   odometry or Imu.
 */

#include "laser_proc.hh"

#include <list>
#include <string>
#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Core>        
#include <Eigen/Dense>          
#include <Eigen/Geometry>

#include <utils.hh>
#include <calib_params.hh>
#include <laser_proc/ProcLaserScan.h>

Eigen::Matrix4d trans;	// SE(3) transformation matrix due to
// either imu, odometry or pose.

// It is the API programmer's responsibility to 
// provide only one of imu/odom/pose for proper 
// behavior of the node. The programmer should 
// remap other topics to some other string to prevent
// multiple transformation sources. (One alternative
// is to check a parameter. However this increases
// the number of parameters.)
ros::Subscriber imu_subs,
	lidar_subs, 
	odom_subs,
	pose_subs;

ros::Publisher	lidar_publ,
	proc_lidar_publ,
	vis_publ,
	proj_pc_publ,
	pc_publ;

// Calibration parameter of the lidar.
LidarCalibParams	lidar_calib_params;
LaserProc			laser_procc;

// All of the output messages are published which include
// lidar, point cloud and visualization markers.
sensor_msgs::LaserScan			in_lidar_msg, out_lidar_msg;
sensor_msgs::Imu				imu_msg;
nav_msgs::Odometry				odom_msg;
geometry_msgs::Pose				pose_msg;
visualization_msgs::MarkerArray vis_msg;
laser_proc::ProcLaserScan		proc_lidar_msg;
sensor_msgs::PointCloud			pc_msg;
sensor_msgs::PointCloud			proj_pc_msg;

void imu_callback	(const sensor_msgs::Imu &msg);
void lidar_callback	(const sensor_msgs::LaserScan &msg);
void odom_callback	(const nav_msgs::Odometry &msg);
void pose_callback	(const geometry_msgs::Pose &msg);

void publish_vis_msgs();
void publish_lidar_msg();
void publish_proc_lidar_msg();
void publish_pc_msg();
void publish_projected_pc_msg();
void plot_polar_visualization();

void process_inputs(const ros::NodeHandle &n);
void setup_messaging_interface(ros::NodeHandle &n);
void loop(const ros::NodeHandle &n);


double	refresh_rate;
bool	debug_mode;
string	calib_file;
bool	median_filter;
int		median_filter_window;
bool	downsample;
double	downsample_thres;
bool	remove_occlusion;
double	remove_occlusion_thres;
int		remove_occlusion_window;
bool	remove_slant_edges;
double	remove_slant_edges_thres;
int		remove_slant_edges_window;
bool	cluster;
int		cluster_min_skips;
double	cluster_range_jump;
int		cluster_min_cluster_size;
bool	rate_linearity;
int		rate_linearity_window;
bool	extract_lines;
double	extract_lines_min_len;
int		extract_lines_min_pts;
double	extract_lines_epsilon;
bool	project;
bool	polar_visualization;

int main(int argc, char* argv[]){

	ros::init(argc, argv, "laser_proc_node");
	ros::NodeHandle n("~");

	trans = Eigen::Matrix4d::Identity();

	process_inputs(n);
	setup_messaging_interface(n);
	loop(n);	

	return 1;
}

void process_inputs(const ros::NodeHandle &n)
{
	n.param("refresh_rate", refresh_rate, 40.0);
	n.param("debug_mode", debug_mode, false);

	n.param("calib_file", calib_file, string(""));
	n.param("median_filter", median_filter, false);
	n.param("median_filter_window", median_filter_window, 3);
	n.param("downsample", downsample, false);
	n.param("downsample_thres", downsample_thres, 0.05);
	n.param("remove_occlusion", remove_occlusion, false);
	n.param("remove_occlusion_thres", remove_occlusion_thres, 0.50);
	n.param("remove_occlusion_window", remove_occlusion_window, 3);
	n.param("remove_slant_edges", remove_slant_edges, false);
	n.param("remove_slant_edges_thres", remove_slant_edges_thres, 1.4835);
	n.param("remove_slant_edges_window", remove_slant_edges_window, 3);
	n.param("cluster", cluster, false);
	n.param("cluster_min_skips", cluster_min_skips, 3);
	n.param("cluster_range_jump", cluster_range_jump, 0.50);
	n.param("cluster_min_cluster_size", cluster_min_cluster_size, 3);
	n.param("rate_linearity", rate_linearity, false);
	n.param("rate_linearity_window", rate_linearity_window, 3);
	n.param("extract_lines", extract_lines, false);
	n.param("extract_lines_min_len", extract_lines_min_len, 0.10);
	n.param("extract_lines_min_pts", extract_lines_min_pts, 3);
	n.param("extract_lines_epsilon", extract_lines_epsilon, 0.08);
	n.param("project", project, false);
	n.param("polar_visualization", polar_visualization, false);

	ROS_INFO(" ---------- LASER PROC ------------");
	ROS_INFO("[refresh_rate] ------------- : [%.3f]", refresh_rate);
	ROS_INFO("[debug_mode] --------------- : [%s]"  , debug_mode ? "TRUE" : "FALSE");
	ROS_INFO("[calib_file] ----------------: [%s]"  , calib_file.c_str());
	ROS_INFO("[median_filter] ------------ : [%s]"  , median_filter ? "TRUE" : "FALSE");
	ROS_INFO("[median_filter_window] ----- : [%d]"  , median_filter_window);
	ROS_INFO("[downsample] --------------- : [%s]"  , downsample ? "TRUE" : "FALSE");
	ROS_INFO("[downsample_thres] --------- : [%.3f]", downsample_thres);
	ROS_INFO("[remove_occlusion] --------- : [%s]"  , remove_occlusion ? "TRUE" : "FALSE");
	ROS_INFO("[remove_occlusion_thres] --- : [%.3f]", remove_occlusion_thres);
	ROS_INFO("[remove_occlusion_window] -- : [%d]"  , remove_slant_edges_window);
	ROS_INFO("[remove_slant_edges] ------- : [%s]"  , remove_slant_edges ? "TRUE" : "FALSE");
	ROS_INFO("[remove_slant_edges_thres -- : [%.3f]", remove_slant_edges_thres);
	ROS_INFO("[remove_slant_edges_window]  : [%d]"  , remove_slant_edges_window);
	ROS_INFO("[cluster] ------------------ : [%s]"  , cluster ? "TRUE" : "FALSE");
	ROS_INFO("[cluster_min_skips] -------- : [%d]"  , cluster_min_skips);
	ROS_INFO("[cluster_range_jump] ------- : [%.3f]", cluster_range_jump);
	ROS_INFO("[cluster_min_cluster_size] - : [%d]"  , cluster_min_cluster_size);
	ROS_INFO("[rate_linearity] ----------- : [%s]"  , rate_linearity ? "TRUE" : "FALSE");
	ROS_INFO("[rate_linearity_window] ---- : [%d]"  , rate_linearity_window);
	ROS_INFO("[extract_lines] ------------ : [%s]"  , extract_lines ? "TRUE" : "FALSE");
	ROS_INFO("[extract_lines_min_len] ---- : [%.3f]", extract_lines_min_len);
	ROS_INFO("[extract_lines_min_pts] ---- : [%d]"  , extract_lines_min_pts);
	ROS_INFO("[extract_lines_epsilon] ---- : [%.3f]", extract_lines_epsilon);
	ROS_INFO("[project] ------------------ : [%s]"  , project ? "TRUE" : "FALSE");
	ROS_INFO(" ----------------------------------------");

	lidar_calib_params.load(calib_file);
	ROS_INFO("Lidar Calib Params :");

	lidar_calib_params.print();
}

void setup_messaging_interface(ros::NodeHandle &n)
{
	imu_subs		= n.subscribe("imu"	, 10, imu_callback	, ros::TransportHints().tcpNoDelay()); 
	lidar_subs  = n.subscribe("scan", 10, lidar_callback, ros::TransportHints().tcpNoDelay()); 
	odom_subs		= n.subscribe("odom", 10, odom_callback	, ros::TransportHints().tcpNoDelay()); 
	pose_subs		= n.subscribe("pose", 10, pose_callback	, ros::TransportHints().tcpNoDelay()); 

	lidar_publ		    = n.advertise<sensor_msgs::LaserScan>("lidar", 10);
	pc_publ			      = n.advertise<sensor_msgs::PointCloud>("pc", 10);
	proj_pc_publ	    = n.advertise<sensor_msgs::PointCloud>("proj_pc", 10);
	proc_lidar_publ   = n.advertise<laser_proc::ProcLaserScan>("proc_lidar", 10);
	vis_publ		      = n.advertise<visualization_msgs::MarkerArray>("vis", 10);
}

void loop(const ros::NodeHandle &n)
{
	ros::Rate r(refresh_rate);

	while (n.ok())
	{
		r.sleep();
		ros::spinOnce();	
	}
}

void imu_callback(const sensor_msgs::Imu &msg){
	if(debug_mode)
		ROS_INFO("ROOF LOCALIZER : Got IMU Data");

	imu_msg = msg;

	imu_msg.orientation.x *= -1;
	imu_msg.orientation.y *= -1;

	trans.topLeftCorner<3, 3>() = utils::trans::imu2dcm(imu_msg, false); 
	trans.topRightCorner<3, 1>().fill(0);
	trans(3, 3) = 1;
}

void lidar_callback(const sensor_msgs::LaserScan &msg){
	if(debug_mode)
		ROS_INFO("ROOF LOCALIZER : Got Bottom Lidar Data");

	in_lidar_msg = msg;

	laser_procc.update_data(in_lidar_msg, lidar_calib_params);
	if(median_filter == true)
		laser_procc.median_filter(median_filter_window);
	if(rate_linearity == true)
		laser_procc.rate_linearity(rate_linearity_window);
	if(downsample == true)
		laser_procc.downsample(downsample_thres);
	if(remove_occlusion == true)
		laser_procc.remove_occlusions(remove_occlusion_thres, remove_occlusion_window);
	if(remove_slant_edges == true)
		laser_procc.remove_slant_edges(remove_slant_edges_thres, remove_slant_edges_window);
	if(cluster == true)
		laser_procc.cluster(cluster_min_skips, cluster_range_jump, cluster_min_cluster_size);
	if(extract_lines == true)
		laser_procc.extract_lines(extract_lines_min_len, extract_lines_min_pts, extract_lines_epsilon);

	laser_procc.transform(trans, false);

	publish_vis_msgs();
	publish_lidar_msg();
	publish_proc_lidar_msg();
	publish_pc_msg();
	if(polar_visualization == true)
		plot_polar_visualization();

	if(project == true){
		laser_procc.project();	
		publish_projected_pc_msg();
	}
}

void odom_callback(const nav_msgs::Odometry &msg){
	if(debug_mode)
		ROS_INFO("ROOF LOCALIZER : Got Odometry Data");

	odom_msg = msg;

	trans = utils::trans::odom2se3(odom_msg);
}

void pose_callback	(const geometry_msgs::Pose &msg){
	if(debug_mode)
		ROS_INFO("ROOF LOCALIZER : Got Pose Data");

	pose_msg = msg;

	trans = utils::trans::pose2se3(pose_msg);
}

void publish_vis_msgs(){

	laser_procc.get_RVIZ_markers(vis_msg);

	for(int i = 0 ; i < (int)vis_msg.markers.size() ; i++){
		vis_msg.markers[i].ns = "";
		vis_msg.markers[i].header.seq++;
		vis_msg.markers[i].header.stamp = ros::Time::now();
		vis_msg.markers[i].header.frame_id = "world";
		vis_msg.markers[i].lifetime = ros::Duration(1.5 / refresh_rate);
	}

	vis_publ.publish(vis_msg);
}

void publish_lidar_msg(){

	out_lidar_msg.header.seq++;
	out_lidar_msg.header.stamp = ros::Time::now();
	out_lidar_msg.header.frame_id = "world";
	out_lidar_msg.angle_min = in_lidar_msg.angle_min;
	out_lidar_msg.angle_max = in_lidar_msg.angle_max;
	out_lidar_msg.angle_increment = in_lidar_msg.angle_increment;
	out_lidar_msg.range_min = in_lidar_msg.range_min;
	out_lidar_msg.range_max = in_lidar_msg.range_max;
	const vector<double> &ranges = laser_procc.get_ranges();
	out_lidar_msg.ranges = vector<float>(ranges.begin(), ranges.end());

	lidar_publ.publish(out_lidar_msg);
}

void publish_proc_lidar_msg(){

	proc_lidar_msg.header.seq++;
	proc_lidar_msg.header.stamp = ros::Time::now();
	proc_lidar_msg.header.frame_id = "world";

	proc_lidar_msg.scan = out_lidar_msg;
	const vector<int> mask = laser_procc.get_mask();
	proc_lidar_msg.mask.resize(mask.size());
	for(int i = 0 ; i < (int)mask.size() ; i++)
		proc_lidar_msg.mask[i] = mask[i];

	const vector<double> linearity = laser_procc.get_linearity_rates();
	proc_lidar_msg.linearity.resize(linearity.size());
	for(int i = 0 ; i < (int)mask.size() ; i++)
		proc_lidar_msg.linearity[i] = linearity[i];

	const Eigen::Matrix3d fim = laser_procc.get_fim();
	for(int i = 0 ; i < 9 ; i ++)
		proc_lidar_msg.fim[i] = fim(i/3, i%3);

	// Programmer can get the 3D points through the point cloud message.
	// The 2D points can be reconstructed in the client node.
	proc_lidar_msg.points_3d.clear();
	proc_lidar_msg.points_2d.clear();

	const vector<pair<int, int> >& line_idxs = laser_procc.get_line_indexes();
	proc_lidar_msg.line_idxs.resize(line_idxs.size() * 2);
	for(int i = 0 ; i < (int)line_idxs.size() ; i++){
		proc_lidar_msg.line_idxs[2*i + 0] = line_idxs[i].first;
		proc_lidar_msg.line_idxs[2*i + 1] = line_idxs[i].second;
	}

	proc_lidar_publ.publish(proc_lidar_msg);
}

void publish_pc_msg(){
	const vector<Eigen::Vector3d>& rays_3d = laser_procc.get_3d_points();
	const vector<int> mask = laser_procc.get_mask();
	const vector<double> linearity = laser_procc.get_linearity_rates();

	pc_msg.header.seq++;
	pc_msg.header.stamp = ros::Time::now();
	pc_msg.header.frame_id = "world";

	pc_msg.points.clear();
	pc_msg.points.reserve(rays_3d.size());
	pc_msg.channels.resize(1);
	pc_msg.channels[0].values.clear();
	pc_msg.channels[0].values.reserve(rays_3d.size());
	pc_msg.channels[0].name = "linearity";

	for(int i = 0 ; i < (int)rays_3d.size() ; i++){
		if(mask[i] == false)
			continue;
		pc_msg.channels[0].values.push_back(linearity[i] < 0.5 ? linearity[i] : 0.5);
		geometry_msgs::Point32 pt;
		pt.x = rays_3d[i](0);
		pt.y = rays_3d[i](1);
		pt.z = rays_3d[i](2);
		pc_msg.points.push_back(pt);
		//cout << "range[" << i << "] = " << rays_3d[i].norm() << " linearity[" << i << "] = " << linearity[i] << endl;
	}

	pc_publ.publish(pc_msg);
}

void publish_projected_pc_msg(){
	const vector<Eigen::Vector3d>& rays_3d = laser_procc.get_3d_points();
	const vector<int> mask = laser_procc.get_mask();
	const vector<double> linearity = laser_procc.get_linearity_rates();

	proj_pc_msg.header.seq++;
	proj_pc_msg.header.stamp = ros::Time::now();
	proj_pc_msg.header.frame_id = "world";

	proj_pc_msg.points.clear();
	proj_pc_msg.points.reserve(rays_3d.size());
	proj_pc_msg.channels.resize(1);
	proj_pc_msg.channels[0].values.clear();
	proj_pc_msg.channels[0].values.reserve(rays_3d.size());
	proj_pc_msg.channels[0].name = "linearity";

	for(int i = 0 ; i < (int)rays_3d.size() ; i++){
		if(mask[i] == false)
			continue;
		proj_pc_msg.channels[0].values.push_back(linearity[i] < 0.5 ? linearity[i] : 0.5);
		geometry_msgs::Point32 pt;
		pt.x = rays_3d[i](0);
		pt.y = rays_3d[i](1);
		pt.z = rays_3d[i](2);
		proj_pc_msg.points.push_back(pt);
		//cout << "range[" << i << "] = " << rays_3d[i].norm() << " linearity[" << i << "] = " << linearity[i] << endl;
	}

	proj_pc_publ.publish(proj_pc_msg);

}


// This function plots in a separate OpenCV window the ranges with cluster
// colors and linearity rates.
void plot_polar_visualization(){
	const vector<double> &ranges = laser_procc.get_ranges();
	const vector<double> &linearity_rates = laser_procc.get_linearity_rates();
	const vector<int>	 &mask = laser_procc.get_mask();

	double min_range  = in_lidar_msg.range_min;
	double max_range  = in_lidar_msg.range_max;
	int    num_ranges = ranges.size();

	// Allocate space for the plot. This should include space for the
	// plot and the margins.
	//int margin = 10;	
	int width = num_ranges; // One pixel per range
	int height = (max_range - min_range) * 10; // One pixel per 10cm.

	static cv::Mat plot1 = cv::Mat(height, width, CV_8UC3);
	plot1.setTo(cv::Scalar(245, 199, 118));

	for(int i = 0 ; i < num_ranges ; i++){
		if(mask[i] <= 0)
			continue;
		for(int j = -5; j <= 5 ; j++)
			cv::circle(plot1, cv::Point(i, height - 10 * ranges[i] + j), 1, cv::Scalar( 255 * linearity_rates[i],
						255 * linearity_rates[i],
						255 * linearity_rates[i]));
		std_msgs::ColorRGBA color = vis_msg.markers[mask[i]].color;
		for(int j = -3; j <= 3 ; j++)
			cv::circle(plot1, cv::Point(i, height - 10 * ranges[i] + j), 1, cv::Scalar(255 * color.b, 255 * color.g, 255 * color.r));
	}

	cv::namedWindow("Polar Plot");
	cv::imshow("Polar Plot", plot1);
	cv::waitKey(1);
}
