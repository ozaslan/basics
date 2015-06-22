#include "rc_proc.hh"

bool rc_proc::process(const com_msgs::RC &rc_msg, double &x, double &y, double &z, double &psi){

	double _dead_zone_x;                     
	double _dead_zone_y;                     
	double _dead_zone_z;                     
	double _dead_zone_psi;                   
	double _min_x;                           
	double _max_x;                           
	double _min_y;                           
	double _max_y;                           
	double _min_z;                           
	double _max_z;                           
	double _min_psi;                         
	double _max_psi;                         
	bool   _normalize;        

	int mid_x   = (max_x   + min_x  ) / 2;
	int mid_y   = (max_y   + min_y  ) / 2;
	int mid_z   = (max_z   + min_z  ) / 2;
	int mid_psi = (max_psi + min_psi) / 2;

	double x_stick_val   = rc_msg.right_fb - mid_x;
	double y_stick_val   = rc_msg.right_rl - mid_y;
	double z_stick_val   =  rc_msg.left_fb - mid_z;
	double psi_stick_val =  rc_msg.left_rl - mid_psi;

	// ---
	if(x_stick_val > dead_zone_x)
		x_stick_val -= dead_zone_x;
	else if(x_stick_val < -dead_zone_x)
		x_stick_val += dead_zone_x;
	else
		x_stick_val = 0;
	// ---
	if(y_stick_val > dead_zone_y)
		y_stick_val -= dead_zone_y;
	else if(y_stick_val < -dead_zone_y)
		y_stick_val += dead_zone_y;
	else
		y_stick_val = 0;
	// ---
	if(z_stick_val > dead_zone_z)
		z_stick_val -= dead_zone_z;
	else if(z_stick_val < -dead_zone_z)
		z_stick_val += dead_zone_z;
	else
		z_stick_val = 0;
	// ---
	if(psi_stick_val > dead_zone_psi)
		psi_stick_val -= dead_zone_psi;
	else if(psi_stick_val < -dead_zone_psi)
		psi_stick_val += dead_zone_psi;
	else
		psi_stick_val = 0;

	if(_normalize == true){
		x_stick_val   /= max_x - mid_x - dead_zone_x;
		y_stick_val   /= max_y - mid_y - dead_zone_y;
		z_stick_val   /= max_z - mid_z - dead_zone_z;
		psi_stick_val /= max_psi - mid_psi - dead_zone_psi;
	}

	return true;
}

bool rc_proc::process(const com_msgs::RC &rc_msg, vector<double> &proc_vals){
	proc_vals.resize(4);
	return process(rc_msg, proc_vals[0], proc_vals[1], proc_vals[2], proc_vals[3]);
}
