#ifndef __RC_PROC_HH__
#define __RC_PROC_HH__

#include <com_msgs/RC.h>

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

	// This function gets an RC message, filters the values.
	bool process(const com_msgs::RC &rc_msg, double &x, double &y, double &z, double &psi);
	// This function is the same as the other 'process(...)' function except
	// it accepts a vector to keep the filtered values.
	bool process(const com_msgs::RC &rc_msg, vector<double> &proc_vals);
	
};



#endif

