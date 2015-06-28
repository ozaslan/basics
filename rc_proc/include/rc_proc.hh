#ifndef __RC_PROC_HH__
#define __RC_PROC_HH__

#include <com_msgs/RC.h>
#include <string>

using namespace std;

class RCProc{
private:
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
	bool   _reverse_x, _reverse_y, _reverse_z, _reverse_psi;
public:
	inline double & dead_zone_x  (){return _dead_zone_x;}
	inline double & dead_zone_y  (){return _dead_zone_y;}
	inline double & dead_zone_z  (){return _dead_zone_z;}
	inline double & dead_zone_psi(){return _dead_zone_psi;}
	inline double & min_x(){return _min_x;}
	inline double & max_x(){return _max_x;}
	inline double & min_y(){return _min_y;}
	inline double & max_y(){return _max_y;}
	inline double & min_z(){return _min_z;}
	inline double & max_z(){return _max_z;}
	inline double & min_psi(){return _min_psi;}
	inline double & max_psi(){return _max_psi;}
	inline bool   & normalize(){return _normalize;}
	inline bool   & reverse_x(){return _reverse_x;}
	inline bool   & reverse_y(){return _reverse_y;}
	inline bool   & reverse_z(){return _reverse_z;}
	inline bool   & reverse_psi(){return _reverse_psi;}

	// This function loads the RC params from the config file at 'config_path'.
	// If the filte cannot be parsed, it sets the params to their default values.
	bool load_params(const string &config_path);
	// This function prints the parameters.
	bool print_params();
	// This function gets an RC message, filters the values.
	bool process(const com_msgs::RC &rc_msg, double &x, double &y, double &z, double &psi);
	// This function is the same as the other 'process(...)' function except
	// it accepts a vector to keep the filtered values.
	bool process(const com_msgs::RC &rc_msg, vector<double> &proc_vals);
	
};



#endif

