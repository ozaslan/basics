#ifndef __CPU_USAGE_HH__
#define __CPU_USAGE_HH__

#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>

#include <boost/lexical_cast.hpp>
#include <boost/regex.hpp>
#include <boost/date_time.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include<cpu_usage/CPULoad.h>

using namespace std;

class CpuUsage{
private:
	int _sample_count;	

	vector<vector<long long> >		 _idle_times;
	vector<boost::posix_time::ptime> _time_stamps;
	vector<float> _cpu_usage;

	vector<long long> _get_idle();

public:
	CpuUsage(int sample_count = 5);
	const vector<float> & get_usage();
};




#endif
