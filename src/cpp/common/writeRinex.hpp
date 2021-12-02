
#ifndef __RINEXOUTPUT_H__
#define __RINEXOUTPUT_H__

#include "observations.hpp"
#include "enums.h"
#include "snx.hpp"

#include <boost/algorithm/string/replace.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/log/trivial.hpp>
#include <boost/filesystem.hpp>

#include <algorithm>
#include <fstream>
#include <string>
#include <math.h>
#include <list>
#include <map>


using std::string;
using std::list;
using std::pair;
using std::map;

struct RinexOutput
{
	string fileName;
	long headerObsPos	= 0;
	long headerTimePos	= 0;
	
	Sinex_stn_snx_t snx;
	
	map<E_Sys, list<pair<E_ObsCode, E_ObsDesc>>> codesPerSys;
};

void recordRinexObservations(
	RinexOutput&	rinexOutput, 
	ObsList&		obsList);


#endif
