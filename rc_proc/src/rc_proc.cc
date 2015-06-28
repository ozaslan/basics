#include "rc_proc.hh"
#include "yaml-cpp/yaml.h"

bool RCProc::load_params(const string &path){
	YAML::Node node = YAML::LoadFile(path.c_str());

	_dead_zone_x   = node["dead_zone_x"].as<double>();
	_dead_zone_y   = node["dead_zone_y"].as<double>();
	_dead_zone_z   = node["dead_zone_z"].as<double>();
	_dead_zone_psi = node["dead_zone_psi"].as<double>();
	_min_x = node["min_x"].as<double>();
	_max_x = node["max_x"].as<double>();
	_min_y = node["min_y"].as<double>();
	_max_y = node["max_y"].as<double>();
  _min_z = node["min_z"].as<double>();
	_max_z = node["max_z"].as<double>();
  _min_psi = node["min_psi"].as<double>();
	_max_psi = node["max_psi"].as<double>();
  _normalize = node["normalize"].as<bool>();
  _reverse_x = node["reverse_x"].as<bool>();
  _reverse_y = node["reverse_y"].as<bool>();
  _reverse_z = node["reverse_z"].as<bool>();
  _reverse_psi = node["reverse_psi"].as<bool>();

	return true;
}

bool RCProc::print_params(){
  cout << "dead_zone_[x, y, z, psi] = [" << _dead_zone_x << ", " << _dead_zone_y << ", " << _dead_zone_z << ", " << _dead_zone_psi << "]" << endl;
  cout << "[min, max]_x     = [" << _min_x   << ", " << _max_x   << "]" << endl;
  cout << "[min, max]_y     = [" << _min_y   << ", " << _max_y   << "]" << endl;
  cout << "[min, max]_z     = [" << _min_z   << ", " << _max_z   << "]" << endl;
  cout << "[min, max]_psi   = [" << _min_psi << ", " << _max_psi << "]" << endl;
  cout << "[normalize]      = [" << (_normalize ? "TRUE" : "FALSE") << "]" << endl;
  cout << "reverse_[x, y]   = [" << (_reverse_x ? "TRUE" : "FALSE") << ", " << (_reverse_y ? "TRUE" : "FALSE") << " ]" << endl;
  cout << "reverse_[z, psi] = [" << (_reverse_z ? "TRUE" : "FALSE") << ", " << (_reverse_psi ? "TRUE" : "FALSE") << " ]" << endl;
  return true;
}

bool RCProc::process(const com_msgs::RC &rc_msg, double &x, double &y, double &z, double &psi){
	
  int mid_x   = (_max_x   + _min_x  ) / 2;
	int mid_y   = (_max_y   + _min_y  ) / 2;
	int mid_z   = (_max_z   + _min_z  ) / 2;
	int mid_psi = (_max_psi + _min_psi) / 2;

  /*
  cout << "mid_x   = " << mid_x   << endl;
  cout << "mid_y   = " << mid_y   << endl;
  cout << "mid_z   = " << mid_z   << endl;
  cout << "mid_psi = " << mid_psi << endl;

  cout << "rc_msg = [" << rc_msg.right_fb << ", " << rc_msg.right_rl << ", " << rc_msg.left_fb << ", " << rc_msg.left_rl << "]" << endl;
  */

	double x_stick_val   = (double)rc_msg.right_fb - mid_x;
	double y_stick_val   = (double)rc_msg.right_rl - mid_y;
	double z_stick_val   = (double)rc_msg.left_fb - mid_z;
	double psi_stick_val = (double)rc_msg.left_rl - mid_psi;

  /*
  cout << "x_stick_val   = " <<   x_stick_val << endl;
  cout << "y_stick_val   = " <<   y_stick_val << endl;
  cout << "z_stick_val   = " <<   z_stick_val << endl;
  cout << "psi_stick_val = " << psi_stick_val << endl;
  */

	// ---
	if(x_stick_val > _dead_zone_x)
		x_stick_val -= _dead_zone_x;
	else if(x_stick_val < -_dead_zone_x)
		x_stick_val += _dead_zone_x;
	else
		x_stick_val = 0;
	// ---
	if(y_stick_val > _dead_zone_y)
		y_stick_val -= _dead_zone_y;
	else if(y_stick_val < -_dead_zone_y)
		y_stick_val += _dead_zone_y;
	else
		y_stick_val = 0;
	// ---
	if(z_stick_val > _dead_zone_z)
		z_stick_val -= _dead_zone_z;
	else if(z_stick_val < -_dead_zone_z)
		z_stick_val += _dead_zone_z;
	else
		z_stick_val = 0;
	// ---
	if(psi_stick_val > _dead_zone_psi)
		psi_stick_val -= _dead_zone_psi;
	else if(psi_stick_val < -_dead_zone_psi)
		psi_stick_val += _dead_zone_psi;
	else
		psi_stick_val = 0;

  /*
  cout << "after filtering :" << endl;
  cout << "x_stick_val   = " <<   x_stick_val << endl;
  cout << "y_stick_val   = " <<   y_stick_val << endl;
  cout << "z_stick_val   = " <<   z_stick_val << endl;
  cout << "psi_stick_val = " << psi_stick_val << endl;
  */

	if(_normalize == true){
		x_stick_val   /= _max_x   - mid_x   - _dead_zone_x;
		y_stick_val   /= _max_y   - mid_y   - _dead_zone_y;
		z_stick_val   /= _max_z   - mid_z   - _dead_zone_z;
		psi_stick_val /= _max_psi - mid_psi - _dead_zone_psi;
	}

  x   = x_stick_val   * (_reverse_x ? -1 : 1);
  y   = y_stick_val   * (_reverse_y ? -1 : 1);
  z   = z_stick_val   * (_reverse_z ? -1 : 1);
  psi = psi_stick_val * (_reverse_psi ? -1 : 1);

  /*
  cout << "after normalizing :" << endl;
  cout << "x_stick_val   = " <<   x_stick_val << endl;
  cout << "y_stick_val   = " <<   y_stick_val << endl;
  cout << "z_stick_val   = " <<   z_stick_val << endl;
  cout << "psi_stick_val = " << psi_stick_val << endl;
  */

	return true;
}

bool RCProc::process(const com_msgs::RC &rc_msg, vector<double> &proc_vals){
	proc_vals.resize(4);
	return process(rc_msg, proc_vals[0], proc_vals[1], proc_vals[2], proc_vals[3]);
}
