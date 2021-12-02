#ifndef WRITECLK_HPP
#define WRITECLK_HPP

// Needed as StationMap is a kind of typedef.
#include "station.hpp"

#include <string>

using std::string;

struct GTime;
struct KFState;
class E_Ephemeris;

void tryPrepareFilterPointers(
	KFState&		kfState, 
	StationMap*		stationMap_ptr);

void outputClocks(
	string			filename,
	E_Ephemeris		clkDataRecSrc,
	E_Ephemeris		clkDataSatSrc,
	GTime			tsync,	
	KFState&		kfState,
	StationMap*		stationMap_ptr);

#endif
