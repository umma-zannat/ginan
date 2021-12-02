#include "writeClock.hpp"

#include <fstream>
#include <tuple>
#include <list>
#include <map>



#include "GNSSambres.hpp"
#include "navigation.hpp"
#include "acsConfig.hpp"
#include "constants.hpp"
#include "station.hpp"
#include "algebra.hpp"
#include "preceph.hpp"
#include "enums.h"

/* macro defintions */
#define             VERSION             3.00
using std::map;

struct ClockEntry
{
	string		id		= "";		// Either station of satellite.
	string		monid	= "";		// Monument identification, receiver.
	Vector3d	recPos	= {};		// Receiver position.
	double	 	clock	= 0;		// Mean clock delta reference.
	double	 	sigma	= 0;		// Standard deviation.
	bool	 	isRec	= true;		// If true is receiver clock data.
};

typedef std::list<ClockEntry> ClockList;


void outputRinexClocksBody(
	string&		filename,	    ///< Path to output file.
	ClockList&	clkList,	    ///< List of data to print.
	GTime		tsync)		    ///< Epoch time.
{
	double ep[6] = {};
	time2epoch(tsync, ep);
	
	std::ofstream clockFile(filename, std::ofstream::app);
	
	if (!clockFile)
	{
		BOOST_LOG_TRIVIAL(error) << "Error opening " << filename << " for RINEX clock file.";
		return;
	}

	for (auto& clkVal : clkList)
	{
		string dataType;
		if (clkVal.isRec)	dataType = "AR";	// Result for receiver clock.
		else				dataType = "AS";	// Result for satellite clock.

		int numData = 2; // Number of data values is 2, clock and sigma.
		tracepdeex(0,clockFile,"%2s %-4s %4d%3d%3d%3d%3d%10.6f%3d   %19.12E %19.12E\n",
			dataType, clkVal.id, 
			(int) ep[0],
			(int) ep[1],
			(int) ep[2],
			(int) ep[3],
			(int) ep[4],
				ep[5],
			numData,
			clkVal.clock,
			clkVal.sigma);
	}
}

void getKalmanSatClks(
	ClockList&	clkValList,
	KFState&	kfState)
{
	for (auto& [key,index] : kfState.kfIndexMap)
	{
		if (key.type != KF::SAT_CLOCK)
		{
			continue;
		}
		
		double clk		= 0;
		double variance	= 0;
		kfState.getKFValue(key, clk, &variance);

		ClockEntry clkVal;
		clkVal.id		= key.Sat.id();
		clkVal.clock	= clk / CLIGHT;
		clkVal.sigma	= sqrt(variance);
		clkVal.isRec	= false;
		
		clkValList.push_back(clkVal);
	}
}


void getKalmanRecClks(
	ClockList&	clkValList,
	ClockEntry&	referenceRec,
	KFState& 	kfState)
{
	int firstBiasGroup = 0;
	for (auto& [key, index] : kfState.kfIndexMap)
	{
		if  ( (  key.type		== KF::REC_SYS_BIAS
			  || key.type		== KF::REC_CLOCK)
			&&(  firstBiasGroup == 0
			  || firstBiasGroup == key.num))
		{
			firstBiasGroup = key.num;
			
			double clk		= 0;
			double variance	= 0;
			kfState.getKFValue(key, clk, &variance);

			ClockEntry clkVal;
			clkVal.id		= key.str;
			clkVal.clock	= clk / CLIGHT;
			clkVal.sigma	= sqrt(variance);
			clkVal.isRec	= true;

			if (key.station_ptr == nullptr)
			{
				BOOST_LOG_TRIVIAL(error) << "Kalman RINEX file clock entry has no reference to receiver.";
// 				continue;
			}
			else
			{
				clkVal.monid  = key.station_ptr->snx.monuid;
				clkVal.recPos = key.station_ptr->snx.pos;
			}
			
			clkValList.push_back(clkVal);
		}

		if	(  key.type		== KF::REC_SYS_BIAS
			|| key.type		== KF::REC_CLOCK)
		{
			// Enter details for reference receiver if available.
			referenceRec.id		= key.str;
			referenceRec.isRec	= true;
			
			if (key.station_ptr)
			{
				referenceRec.monid	= key.station_ptr->snx.monuid;
				referenceRec.recPos	= key.station_ptr->snx.pos;
			}
		}
	}
}

void getPreciseRecClks(
	ClockList&  	clkValList,
	StationMap*		stationMap_ptr,
	GTime 			tsync)
{
	if (stationMap_ptr == nullptr)
	{
		return;
	}
	
	auto& stationMap = *stationMap_ptr;
	
	for (auto& [id, rec] : stationMap)
	{
		double dt;
		double variance;
		int ret = pephclk(tsync, rec.id, nav, dt, &variance);
		if (ret != 1)
		{
			BOOST_LOG_TRIVIAL(warning) 
			<< "Station : " << rec.id
			<< ", precise clock entry not calculated.";
			
			continue;
		}

		ClockEntry clkVal;
		clkVal.id		= rec.id;
		clkVal.clock	= dt;
		clkVal.sigma	= sqrt(variance);
		clkVal.isRec	= true;
		clkVal.monid	= rec.snx.monuid;
		clkVal.recPos	= rec.snx.pos;
		
		clkValList.push_back(clkVal);
	}
}

void getSatClksFromEph(
	ClockList&  	clkValList,
	GTime 			time,
	E_Ephemeris		ephType)
{
	for (auto& [satId, satNav] : nav.satNavMap)
	{
		SatSys Sat;
		Sat.fromHash(satId);

		if (acsConfig.process_sys[Sat.sys] == false)
		{
			continue;
		}

		// Create a dummy observation
		Obs obs;
		obs.Sat = Sat;
		
		obs.satNav_ptr = &nav.satNavMap[obs.Sat]; // for satpos_ssr()

		int pass = satpos(std::cout, time, time, obs, ephType, E_OffsetType::COM, nav, nullptr, false);
		if (pass == false)
		{
			BOOST_LOG_TRIVIAL(warning)
			<< "Satellite : " << Sat.id()
			<< ",  clock entry not calculated.";
			
			continue;
		}
		
		ClockEntry clkVal;
		clkVal.id		= Sat.id();
		clkVal.clock	= obs.dtSat[0];
		clkVal.sigma	= sqrt(obs.ephVar);
		clkVal.isRec	= false;
		
		clkValList.push_back(clkVal);
	}
}

void outputRinexClocksHeader(
	string&			filename,			///< Path of tile to output to
	ClockList&		clkValList,			///< List of clock values to output
	ClockEntry&		referenceRec,		///< Entry for the reference receiver
	GTime			tsync)				///< Epoch time
{
	std::ofstream clockFile(filename, std::ofstream::app);
	
	if (!clockFile)
	{
		BOOST_LOG_TRIVIAL(error) << "Error opening " << filename << " for RINEX clock file.";
		return;
	}

	auto pos = clockFile.tellp();
	if (pos != 0)
		return;

	double ep[6] = {};
	time2epoch(tsync, ep);

	char syschar = 0;
	for (int i = 0; i < E_Sys::NUM_SYS; i++)
	{
		if (acsConfig.process_sys[i])
		{
			SatSys Sat = SatSys(E_Sys::_from_integral(i));
			if (syschar == 0)
				syschar = Sat.sysChar();
			else
			{
				syschar = 'M';
				break;
			}
		}
	}
	if (syschar == 0)
	{
		BOOST_LOG_TRIVIAL(error) << "Writing RINEX clock file no systems in process_sys.";
		return;
	}

	string clkRefStation = referenceRec.id;
	
	int num_recs = 0;
	for (auto clkVal : clkValList)
	{
		if (clkVal.isRec)
		{
			num_recs++;
		}
	}
	
	tracepdeex(0,clockFile,"%9.2f           C                   %c                   %s\n",
		VERSION,syschar,
		"RINEX VERSION / TYPE");

	tracepdeex(0,clockFile,"%-20s%-20s%4d%02d%02d %02d%02d%02d %4s%s\n",
		acsConfig.analysis_program,
		acsConfig.analysis_agency,
		(int)ep[0],
		(int)ep[1],
		(int)ep[2],
		(int)ep[3],
		(int)ep[4],
		(int)ep[5],
		"LCL","PGM / RUN BY / DATE");

	tracepdeex(0,clockFile,"%-60s%s\n","",																	"SYS / # / OBS TYPES");
	tracepdeex(0,clockFile,"%-60s%s\n","",																	"TIME SYSTEM ID");
	tracepdeex(0,clockFile,"%6d    %2s    %2s%-42s%s\n",           2,"AS","AR","",							"# / TYPES OF DATA");
	tracepdeex(0,clockFile,"%-60s%s\n","",																	"STATION NAME / NUM");
	tracepdeex(0,clockFile,"%-60s%s\n","",																	"STATION CLK REF");
	tracepdeex(0,clockFile,"%-3s  %-55s%s\n", acsConfig.analysis_agency.c_str(), acsConfig.analysis_center.c_str(),			"ANALYSIS CENTER");
	tracepdeex(0,clockFile,"%6d%54s%s\n",1,"",																"# OF CLK REF");

	// Note clkRefStation can be a zero length string.
	tracepdeex(0,clockFile,"%-4s %-20s%35s%s\n", "", clkRefStation,"",										"ANALYSIS CLK REF");
	tracepdeex(0,clockFile,"%6d    %-50s%s\n", num_recs, "IGS14",											"# OF SOLN STA / TRF");
	// MM This line causes the clock combination software to crash to removing
	//tracepdeex(0,clockFile,"%-60s%s\n",acsConfig.rinex_comment,												"COMMENT");

	/* output receiver id and coordinates */

	for (auto& clkVal : clkValList)
	{
		if (clkVal.isRec == false)
		{
			continue;
		}
			
		string idStr  = clkVal.id	.substr(0,4);
		string monuid = clkVal.monid.substr(0,20);
		
		tracepdeex(0,clockFile,"%-4s ",idStr);
		tracepdeex(0,clockFile,"%-20s",monuid);
		tracepdeex(0,clockFile,"%11.0f %11.0f %11.0f%s\n",
				clkVal.recPos(0) * 1000,
				clkVal.recPos(1) * 1000,
				clkVal.recPos(2) * 1000,
				"SOLN STA NAME / NUM");
	}


	int num_sats = 0;
	if (acsConfig.process_sys[E_Sys::GPS])	num_sats += NSATGPS;
	if (acsConfig.process_sys[E_Sys::GLO])	num_sats += NSATGLO;
	if (acsConfig.process_sys[E_Sys::GAL])	num_sats += NSATGAL;
	if (acsConfig.process_sys[E_Sys::CMP])	num_sats += NSATCMP;
		
	/* output satellite PRN*/
	int k = 0;		
	tracepdeex(0,clockFile,"%6d%54s%s\n",num_sats,"","# OF SOLN SATS");
	if (acsConfig.process_sys[E_Sys::GPS])	for (int prn = 1; prn <= NSATGPS; prn++)	{k++;	SatSys s(E_Sys::GPS,prn);	tracepdeex(0,clockFile,"%3s ",	s.id().c_str());	if (k % 15 == 0) tracepdeex(0,clockFile,"%s\n","PRN LIST");}
	if (acsConfig.process_sys[E_Sys::GLO])	for (int prn = 1; prn <= NSATGLO; prn++)	{k++;	SatSys s(E_Sys::GLO,prn);	tracepdeex(0,clockFile,"%3s ",	s.id().c_str());	if (k % 15 == 0) tracepdeex(0,clockFile,"%s\n","PRN LIST");}
	if (acsConfig.process_sys[E_Sys::GAL])	for (int prn = 1; prn <= NSATGAL; prn++)	{k++;	SatSys s(E_Sys::GAL,prn);	tracepdeex(0,clockFile,"%3s ",	s.id().c_str());	if (k % 15 == 0) tracepdeex(0,clockFile,"%s\n","PRN LIST");}
	if (acsConfig.process_sys[E_Sys::CMP])	for (int prn = 1; prn <= NSATCMP; prn++)	{k++;	SatSys s(E_Sys::CMP,prn);	tracepdeex(0,clockFile,"%3s ",	s.id().c_str());	if (k % 15 == 0) tracepdeex(0,clockFile,"%s\n","PRN LIST");}
	/*finish the line*/						while (k % 15 != 0)							{k++;								tracepdeex(0,clockFile,"%3s ",	"");				if (k % 15 == 0) tracepdeex(0,clockFile,"%s\n","PRN LIST");}

	tracepdeex(0,clockFile,"%-60s%s\n","","END OF HEADER");
}


void tryPrepareFilterPointers(
	KFState&		kfState, 
	StationMap*		stationMap_ptr)
{
	if (stationMap_ptr == nullptr)
	{
		return;
	}
	
	auto& stationMap = *stationMap_ptr;
	
	map<KFKey, short> replacementKFIndexMap;
	for (auto& [key, index] : kfState.kfIndexMap)
	{
		KFKey kfKey = key;
		
		if	(  kfKey.station_ptr == nullptr
			&& kfKey.str.empty() == false)
		{
			auto it = stationMap.find(kfKey.str);
			if (it == stationMap.end())
			{
				continue;
			}
			
			auto& [id, station]	= *it;
			kfKey.station_ptr	= &station;
		}
		
		replacementKFIndexMap[kfKey] = index;
	}
	
	kfState.kfIndexMap = replacementKFIndexMap;
}

void outputClocks(
	string			filename,
	E_Ephemeris		clkDataRecSrc,
	E_Ephemeris		clkDataSatSrc,
	GTime			time,
	KFState&		kfState,
	StationMap*		stationMap_ptr)
{
	ClockList  clkValList;
	ClockEntry referenceRec;
	referenceRec.isRec = false;

	switch (clkDataSatSrc)
	{
		case +E_Ephemeris::NONE:																	break;
		case +E_Ephemeris::KALMAN:			getKalmanSatClks	(clkValList, kfState);				break;
		case +E_Ephemeris::PRECISE:			//fallthrough
		case +E_Ephemeris::BROADCAST:		//fallthrough
		case +E_Ephemeris::SSR:				getSatClksFromEph	(clkValList, time, clkDataSatSrc);	break;
		default:	BOOST_LOG_TRIVIAL(error) << "Unknown / Undefined clock data source.";			return;
	}

	switch (clkDataRecSrc)
	{
		case +E_Ephemeris::NONE:																		break;
		case +E_Ephemeris::KALMAN:			getKalmanRecClks	(clkValList, referenceRec, kfState);	break;
		case +E_Ephemeris::PRECISE:			getPreciseRecClks	(clkValList, stationMap_ptr, tsync);	break;
		case +E_Ephemeris::SSR:				//fallthrough
		case +E_Ephemeris::BROADCAST:		//fallthrough
		default:	BOOST_LOG_TRIVIAL(error) << "Printing receiver clocks for " << clkDataRecSrc._to_string() << " not implemented.";	return;
	}

	outputRinexClocksHeader	(filename, clkValList, referenceRec,	tsync);
	outputRinexClocksBody	(filename, clkValList,					tsync);
}
