#ifndef __WRITEPOSCLKSP3_HPP
#define __WRITEPOSCLKSP3_HPP

#include <string>

using std::string;

struct GTime;
class E_Ephemeris;

void outputSp3(
	string&			filename,
	E_Ephemeris		posClkDataSrc,
	GTime			time);
#endif
