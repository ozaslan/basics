#include "cpu_usage.hh"
#include <utils.hh>

CpuUsage::CpuUsage(int sample_count){
	ASSERT(sample_count > 1, "sample_count > 1")
	_sample_count = sample_count;
	_time_stamps.resize(_sample_count);
	_idle_times.resize(_sample_count);
}

//This function reads /proc/stat and returns the idle value for each cpu in a vector
vector<long long> CpuUsage::_get_idle() {
	//Virtual file, created by the Linux kernel on demand
	std::ifstream in("/proc/stat");

	std::vector<long long> result;

	//This might broke if there are not 8 columns in /proc/stat
	boost::regex reg("cpu(\\d+) (\\d+) (\\d+) (\\d+) (\\d+) (\\d+) (\\d+) (\\d+) (\\d+) (\\d+) (\\d+)");

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
const vector<float> & CpuUsage::get_usage() { 
	// Erase the oldest time stamp and record the latest one
	_time_stamps.erase(_time_stamps.begin());
	_time_stamps.push_back(boost::date_time::microsec_clock<boost::posix_time::ptime>::universal_time());

	// The same for the 'idle_times'
	_idle_times.erase(_idle_times.begin());
	_idle_times.push_back(_get_idle());

	//We have to measure the time, beacuse sleep is not accurate
	const float total_seconds_elapsed = float((_time_stamps.back() - _time_stamps.front()).total_milliseconds()) / 1000.f;

	_cpu_usage.clear();

	for ( unsigned i = 0; i < _idle_times.front().size(); ++i ) {
		//This might get slightly negative, because our time measurment is not accurate
		const float load = 1.f - float(_idle_times.back()[i] - _idle_times.front()[i])/(100.f * total_seconds_elapsed);
		_cpu_usage.push_back( 100 * (load < 0 ? 0 : load));
	}

	return _cpu_usage;
}

