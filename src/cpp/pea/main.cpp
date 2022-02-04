
// #pragma GCC optimize ("O0")

#include <sys/time.h>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <memory>
#include <chrono>
#include <thread>
#include <string>

#include "omp.h"

using namespace std::literals::chrono_literals;
using std::this_thread::sleep_for;
using std::chrono::system_clock;
using std::chrono::time_point;
using std::string;

#include <boost/log/utility/setup/console.hpp>
#include <boost/log/trivial.hpp>
#include <boost/filesystem.hpp>


#include "ntripCasterService.hpp"
#include "minimumConstraints.hpp"
#include "acsNtripBroadcast.hpp"
#include "networkEstimator.hpp"
#include "peaCommitVersion.h"
#include "writeRinexNav.hpp"
#include "writeRinexObs.hpp"
#include "algebraTrace.hpp"
#include "rtsSmoothing.hpp"
#include "ntripSocket.hpp"
#include "corrections.hpp"
#include "streamTrace.hpp"
#include "writeClock.hpp"
#include "GNSSambres.hpp"
#include "instrument.hpp"
#include "acsConfig.hpp"
#include "acsStream.hpp"
#include "testUtils.hpp"
#include "biasSINEX.hpp"
#include "ionoModel.hpp"
#include "ephemeris.hpp"
#include "writeSp3.hpp"
#include "station.hpp"
#include "summary.hpp"
#include "antenna.hpp"
#include "satStat.hpp"
#include "fileLog.hpp"
#include "common.hpp"
#include "orbits.hpp"
#include "gTime.hpp"
#include "mongo.hpp"
#include "debug.hpp"
#include "enums.h"
#include "ppp.hpp"
#include "snx.hpp"
#include "trop.h"
#include "vmf3.h"


nav_t		nav		= {};
int			epoch	= 1;
GTime		tsync	= GTime::noTime();


void removeUnprocessed(
	ObsList&	obsList)
{
	for (auto& obs : obsList)
	{
		if (acsConfig.process_sys[obs.Sat.sys] == false)
		{
			obs.excludeSystem = true;
		}
	}
}

bool fileChanged(
	string filename)
{
	bool valid = checkValidFile(filename);
	if (valid == false)
	{
		return false;
	}

	auto modifyTime = boost::filesystem::last_write_time(filename);
	
	auto it = acsConfig.configModifyTimeMap.find(filename);
	if (it == acsConfig.configModifyTimeMap.end())
	{
		//the first time this file has been read, 
		//update then return true
		acsConfig.configModifyTimeMap[filename] = modifyTime;
		
		return true;
	}
	
	auto& [dummy, readTime] = *it;
	if (readTime != modifyTime)
	{
		//has a different modification time, update then return true
		readTime = modifyTime;
	
		return true;
	}
	
	//has been read with this time before
	return false;
}

void removeInvalidFiles(
	vector<string>& files)
{
	for (auto it = files.begin(); it != files.end(); )
	{
		auto& filename = *it;
		bool valid = checkValidFile(filename);
		if (valid == false)
		{
			it = files.erase(it);
		}
		else
		{
			it++;
		}
	}
}


void reloadInputFiles()
{
	removeInvalidFiles(acsConfig.atxfiles);
	for (auto& atxfile : acsConfig.atxfiles)
	{
		if (fileChanged(atxfile) == false)
		{
			continue;
		}

		BOOST_LOG_TRIVIAL(info)
		<< "Loading ATX file " << atxfile;

		bool pass = readantexf(atxfile, nav);
		if (pass == false)
		{
			BOOST_LOG_TRIVIAL(error)
			<< "Unable to load ATX from file " << atxfile;

			continue;
		}
	}

	removeInvalidFiles(acsConfig.erpfiles);
	for (auto& erpfile : acsConfig.erpfiles)
	{
		if (fileChanged(erpfile) == false)
		{
			continue;
		}

		BOOST_LOG_TRIVIAL(info)
		<< "Loading ERP file " << erpfile;

		readerp(erpfile, &nav.erp);
	}

	removeInvalidFiles(acsConfig.orbfiles);
	removeInvalidFiles(acsConfig.sp3files);
	if (acsConfig.orbfiles.empty() == false)
	{
		bool updated = false;

		/* orbit info from orbit file */
		for (auto& orbfile : acsConfig.orbfiles)
		{
			if (fileChanged(orbfile) == false)
			{
				continue;
			}

			updated = true;

			BOOST_LOG_TRIVIAL(info)
			<< "Reading in the orbit file" << orbfile << std::endl;

			readorbit(orbfile);
		}

		if (updated)
		{
			orb2sp3(nav);
		}
	}
	else
	{
		for (auto& sp3file : acsConfig.sp3files)
		{
			if (fileChanged(sp3file) == false)
			{
				continue;
			}

			BOOST_LOG_TRIVIAL(info)
			<< "Loading SP3 file " << sp3file << std::endl;

			readSp3ToNav(sp3file, &nav, 0);
		}
	}

	removeInvalidFiles(acsConfig.navfiles);
	for (auto& navfile : acsConfig.navfiles)
	{
		if (fileChanged(navfile) == false)
		{
			continue;
		}

		BOOST_LOG_TRIVIAL(info)
		<< "Loading NAV file " << navfile;

		FileRinexStream rinexStream(navfile);

		rinexStream.parse();
	}

	removeInvalidFiles(acsConfig.clkfiles);
	for (auto& clkfile : acsConfig.clkfiles)
	{
		if (fileChanged(clkfile) == false)
		{
			continue;
		}

		/* CLK file - RINEX 3 */
		BOOST_LOG_TRIVIAL(info)
		<< "Loading CLK file " << clkfile;

		FileRinexStream rinexStream(clkfile);

		rinexStream.parse();
	}

	removeInvalidFiles(acsConfig.dcbfiles);
	for (auto& dcbfile : acsConfig.dcbfiles)
	{
		if (fileChanged(dcbfile) == false)
		{
			continue;
		}

		/* DCB file */
		BOOST_LOG_TRIVIAL(info)
		<< "Loading DCB file " << dcbfile;

		readdcb(dcbfile, &nav);
	}

	removeInvalidFiles(acsConfig.bsxfiles);
	for (auto& bsxfile : acsConfig.bsxfiles)
	{
		if (fileChanged(bsxfile) == false)
		{
			continue;
		}

		/* BSX file*/
		BOOST_LOG_TRIVIAL(info)
		<< "Loading BSX file " << bsxfile;

		readBiasSinex(bsxfile);
	}

	removeInvalidFiles(acsConfig.ionfiles);
	for (auto& ionfile : acsConfig.ionfiles)
	{
		if (fileChanged(ionfile) == false)
		{
			continue;
		}

		BOOST_LOG_TRIVIAL(info)
		<< "Loading ION file " << ionfile;

		readtec(ionfile, &nav);
	}

	static bool once = true;
	removeInvalidFiles(acsConfig.snxfiles);
	for (auto& snxfile : acsConfig.snxfiles)
	{
		if (fileChanged(snxfile) == false)
		{
			continue;
		}

		BOOST_LOG_TRIVIAL(info)
		<< "Loading SNX file " <<  snxfile << std::endl;

		bool fail = read_sinex(snxfile, once);
		if (fail)
		{
			BOOST_LOG_TRIVIAL(error)
			<< "Unable to load SINEX file " << snxfile;

			continue;
		}

		once = false;
	}
}

void createNewTraceFile(
	const string				id,
	boost::posix_time::ptime	logptime,
	string  					new_path_trace,
	string& 					old_path_trace,
	bool						outputHeader,
	bool						outputConfig)
{	
	replaceString(new_path_trace, "<STATION>", id);
	replaceTimes (new_path_trace, logptime);

	// Create the trace file if its a new filename, otherwise, keep the old one
	if (new_path_trace == old_path_trace)
	{
		//the filename is the same, keep using the old ones
		return;
	}

	old_path_trace = new_path_trace;

	BOOST_LOG_TRIVIAL(debug)
	<< "\tCreating new file for " << id << " at " << old_path_trace;

	std::ofstream trace(old_path_trace);
	if (!trace)
	{
		BOOST_LOG_TRIVIAL(error)
		<< "Could not create file for " << id << " at " << old_path_trace;

		return;
	}

	// Trace file head
	if (outputHeader)
	{
		trace << "station    : " << id << std::endl;
		trace << "start_epoch: " << acsConfig.start_epoch			<< std::endl;
		trace << "end_epoch  : " << acsConfig.end_epoch				<< std::endl;
		trace << "trace_level: " << acsConfig.trace_level			<< std::endl;
		trace << "pea_version: " << GINAN_COMMIT_VERSION			<< std::endl;
// 		trace << "rts_lag    : " << acsConfig.pppOpts.rts_lag		<< std::endl;
	}

	if (outputConfig)
	{
		dumpConfig(trace);
	}
}

map<string, string> fileNames;

/** Create new empty trace files only when required when the filename is changed
 */
void createTracefiles(
	map<string, Station>&	stationMap,
	Network&				net)
{
	string dummyStr;
	GTime logtime = tsync.roundTime(acsConfig.trace_rotate_period);
	
	boost::posix_time::ptime	logptime	= boost::posix_time::from_time_t(logtime.time);
	
	if (acsConfig.output_trace)
	for (auto& [id, rec] : stationMap)
	{
		createNewTraceFile(id,			logptime,	acsConfig.trace_filename,							rec.traceFilename,						true,	acsConfig.output_config);
	}

	if	(	acsConfig.output_trop_sinex
		&&	acsConfig.process_user)
	for (auto& [id, rec] : stationMap)
	{
		createNewTraceFile(id,			logptime,	acsConfig.trop_sinex_filename,						rec.tropFilename,						false,	false);
		
		if	(  acsConfig.process_rts
			&& acsConfig.pppOpts.rts_lag)
		{
			createNewTraceFile(id,		logptime,	acsConfig.trop_sinex_filename + SMOOTHED_SUFFIX,	rec.rtsTropFilename,					false,	false);
		}
	}
	
	if	(  acsConfig.output_trace
		&& acsConfig.process_rts
		&& acsConfig.pppOpts.rts_lag)
	for (auto& [id, rec] : stationMap)
	{
		createNewTraceFile(id,			logptime,	acsConfig.pppOpts.rts_filename + FORWARD_SUFFIX,	rec.rtk.pppState.rts_forward_filename,	false,	false);
		createNewTraceFile(id,			logptime,	acsConfig.pppOpts.rts_filename,						rec.rtk.pppState.rts_filename,			true,	acsConfig.output_config);
	}

	if	(  acsConfig.output_trace
		&& acsConfig.process_rts
		&& acsConfig.netwOpts.rts_lag)
	{
		createNewTraceFile(net.id,		logptime,	acsConfig.netwOpts.rts_filename + FORWARD_SUFFIX,	net.kfState.rts_forward_filename,		false,	false);
		createNewTraceFile(net.id,		logptime,	acsConfig.netwOpts.rts_filename,					net.kfState.rts_filename,				true,	acsConfig.output_config);
	}

	if (acsConfig.output_summary)
	{
		createNewTraceFile(net.id,		logptime,	acsConfig.summary_filename,							net.traceFilename,						true,	acsConfig.output_config);
	}

	if (acsConfig.output_log)
	{
		createNewTraceFile("",			logptime,	acsConfig.log_filename,								FileLog::path_log,						false,	false);
	}
	
	if	( acsConfig.output_trop_sinex
		&&acsConfig.process_network)
	{	
		createNewTraceFile(net.id,		logptime,	acsConfig.trop_sinex_filename,						net.tropFilename,						false,	false);
		
		if	( acsConfig.process_rts
			&&acsConfig.netwOpts.rts_lag)
		{
			createNewTraceFile(net.id,	logptime,	acsConfig.trop_sinex_filename + SMOOTHED_SUFFIX,	net.rtsTropFilename,					false,	false);
		}
	}

	if (acsConfig.output_rinex_obs)
	for (auto& [id, rec] : stationMap)
	{
		auto filenameMap = getSysOutputFilenames(acsConfig.rinex_obs_filename,	tsync, id);
		for (auto& [filename, dummy] : filenameMap)
		{
			createNewTraceFile(id,		logptime,	filename,											fileNames[filename],					false,	false);
		}
	}

	if (acsConfig.output_rinex_nav)
	{
		auto filenameMap = getSysOutputFilenames(acsConfig.rinex_nav_filename,	tsync);
		for (auto& [filename, dummy] : filenameMap)
		{
			createNewTraceFile("",		logptime,	filename,											fileNames[filename],					false,	false);
		}
	}

	if (acsConfig.output_orbits)
	{
		auto filenameMap = getSysOutputFilenames(acsConfig.orbits_filename,		tsync);
		for (auto& [filename, dummy] : filenameMap)
		{
			createNewTraceFile(net.id,	logptime,	filename,											fileNames[filename],					false,	false);
		}
	}

	if (acsConfig.output_clocks)
	{
		auto filenameMap = getSysOutputFilenames(acsConfig.clocks_filename,		tsync);
		for (auto& [filename, dummy] : filenameMap)
		{
			createNewTraceFile(net.id,	logptime,	filename,											fileNames[filename],					false,	false);
		}

		if	(  acsConfig.process_rts
			&& acsConfig.netwOpts.rts_lag)
		for (auto& [filename, dummy] : filenameMap)
		{
			createNewTraceFile(net.id, 	logptime,	filename + SMOOTHED_SUFFIX, 						fileNames[filename + SMOOTHED_SUFFIX],	false,	false);
		}
	}

	for (auto& [id, upStream_ptr] : outStreamManager.ntripUploadStreams)
	{
		NtripBroadcaster::NtripUploadClient& upStream = *upStream_ptr;
		upStream.ntripTrace.level_trace = level_trace;
		createNewTraceFile(id + "-UP",	logptime,	acsConfig.trace_filename,							upStream.traceFilename,					true,	acsConfig.output_config);
	}
}

void avoidCollisoins(
	StationMap&		stationMap)
{
	for (auto& [id, rec] : stationMap)
	{
		auto trace = getTraceFile(rec);
		
		acsConfig.getRecOpts(id);
		
		//prepare and connect navigation objects to the observations
		for (auto& obs : rec.obsList)
		{
			obs.satNav_ptr					= &nav.satNavMap[obs.Sat];
			obs.satNav_ptr->eph_ptr 		= seleph<Eph>	(trace, tsync, obs.Sat, -1, nav);
			obs.satNav_ptr->geph_ptr 		= seleph<Geph>	(trace, tsync, obs.Sat, -1, nav);
			obs.satNav_ptr->pephList_ptr	= &nav.pephMap[obs.Sat];
			obs.mount 						= rec.id;
			updatenav(obs);

			obs.satStat_ptr					= &rec.rtk.satStatMap[obs.Sat];

			acsConfig.getSatOpts(obs.Sat);
		}
		
		// if the vmf3 option has been selected:
		if	(acsConfig.tropOpts.vmf3dir.empty() == false)
		{
			BOOST_LOG_TRIVIAL(debug)
			<< "VMF3 option chosen";
			
			double ep[6];
			time2epoch(tsync, ep);
			
			double jd = ymdhms2jd(ep);

			// read vmf3 grid information
			rec.vmf3.m = 1;
			udgrid(acsConfig.tropOpts.vmf3dir.c_str(), rec.vmf3.vmf3g, jd, rec.mjd0, rec.vmf3.m);
		}
	}
}

void initialiseStation(
	string		id,
	Station&	rec)
{
	BOOST_LOG_TRIVIAL(debug)
	<< "Initialising station " << id;

	rec.id = id;

	// Read the BLQ file
	bool found = false;
	for (auto& blqfile : acsConfig.blqfiles)
	{
		found = readblq(blqfile, id.c_str(), rec.rtk.opt.otlDisplacement[0]);

		if (found)
		{
			break;
		}
	}

	if (found == false)
	{
		BOOST_LOG_TRIVIAL(warning)
		<< "No BLQ for " << id;
	}

	if (acsConfig.process_user)
	{
		rec.rtk.pppState.max_filter_iter	= acsConfig.pppOpts.max_filter_iter;
		rec.rtk.pppState.max_prefit_remv	= acsConfig.pppOpts.max_prefit_remv;
		rec.rtk.pppState.inverter			= acsConfig.pppOpts.inverter;
		rec.rtk.pppState.output_residuals	= acsConfig.output_residuals;

		rec.rtk.pppState.rejectCallbacks.push_back(countSignalErrors);
		rec.rtk.pppState.rejectCallbacks.push_back(deweightMeas);
	}

	if	( acsConfig.process_rts
		&&acsConfig.pppOpts.rts_lag)
	{
		rec.rtk.pppState.rts_lag = acsConfig.pppOpts.rts_lag;
	}
}

/** Perform operations for each station
 * This function occurs in parallel with other stations - ensure that any operations on global maps do not create new entries, as that will destroy the map for other processes.
 * Variables within the rec object are ok to use, but be aware that pointers from the within the receiver often point to global variables.
 * Prepare global maps by accessing the desired elements before calling this function.
 */
void mainOncePerEpochPerStation(
	Station&	rec,
	double*		orog,
	gptgrid_t&	gptg,
	bool&		emptyEpoch)
{
	TestStack ts(rec.id);
	
	if (rec.ready == false)
	{
		return;
	}
	
	emptyEpoch = false;

	auto trace = getTraceFile(rec);

	trace << std::endl << "------=============== Epoch " << epoch << " =============-----------" << std::endl;

	BOOST_LOG_TRIVIAL(debug)
	<< "\tRead " << rec.obsList.size()
	<< " observations for station " << rec.id;

	//calculate statistics
	{
		if (rec.firstEpoch	== GTime::noTime())		{	rec.firstEpoch	= rec.obsList.front().time;		}
														rec.lastEpoch	= rec.obsList.front().time;
		rec.epochCount++;
		rec.obsCount += rec.obsList.size();

		for (auto& obs				: rec.obsList)
		for (auto& [ft, sigList]	: obs.SigsLists)
		for (auto& sig				: sigList)
		{
			rec.codeCount[sig.code]++;
		}

		for (auto& obs				: rec.obsList)
		{
			rec.satCount[obs.Sat.id()]++;
		}
	}

	if (acsConfig.process_preprocessor)
	{
		removeUnprocessed(rec.obsList);
		
		obsVariances(rec.obsList);
	
		sinexPerEpochPerStation(tsync, rec);

		/* linear combinations */
		for (auto& obs : rec.obsList)
		{
			obs.satStat_ptr->lc_pre = obs.satStat_ptr->lc_new;
			obs.satStat_ptr->lc_new = {};
		}
		obs2lcs		(trace,	rec.obsList);

		/* cycle slip/clock jump detection and repair */

		detectslips	(trace,	rec.obsList);
		detectjump	(trace,	rec.obsList, acsConfig.elevation_mask, rec.cj);

		for (auto& obs			: rec.obsList)
		for (auto& [ft, Sig]	: obs.Sigs)
		{
			if (obs.satStat_ptr->sigStatMap[ft].slip.any)
			{
				rec.slipCount++;
				break;
			}
		}

		GTime prevTime = rec.rtk.sol.time;

		//do a spp on the observations
		sppos(trace, rec.obsList, rec.rtk.sol);
		
		if (prevTime.time != 0)
		{
			rec.rtk.tt = rec.rtk.sol.time - prevTime;
		}
	}

	if (acsConfig.process_ionosphere)
	{
		bool sppUsed;
		selectAprioriSource(rec, sppUsed);

		if (rec.aprioriPos.norm())
			update_receivr_measr(trace, rec);
	}

	/* exclude measurements of eclipsing satellite (block IIA) */
	if (acsConfig.reject_eclipse)
	{
		testeclipse(rec.obsList);
	}

	if (acsConfig.process_user)
	{
		pppos(trace, rec.rtk, rec.obsList, rec);

		if	(  rec.rtk.sol.stat 			!= SOLQ_NONE
			&& acsConfig.ambrOpts.WLmode	!= +E_ARmode::OFF)
		{
			KFState kfARcopy = rec.rtk.pppState;

			int nfixed = enduserAmbigResl(trace, rec.obsList, kfARcopy);
// 			if (nfixed > 0)
// 			{
// 				trace << std::endl << "-------------- AR PPP solution ----------------------" << std::endl;
// 				pppoutstat(trace, kfARcopy, false, SOLQ_FIX, rec.rtk.sol.numSats);
// 				trace << std::endl << "------------ AR PPP solution end --------------------" << std::endl;
// 			}
		}

		if (acsConfig.output_ppp_sol)
		{
			outputPPPSolution(rec);
		}
	}

	if (acsConfig.process_network)
	for (auto once : {1})
	{
		// If there is no antenna information skip processing this station
		if (rec.snx.anttype.empty())
		{
			BOOST_LOG_TRIVIAL(warning)
			<< "\tNo Antenna Information for " << rec.id
			<< " skipping this station";

			break;
		}

		// if the vmf3 option has been selected, use it, otherwise send null
		vmf3_t*	rtkVmf3_ptr = nullptr;
		double*	rtkOrog_ptr = nullptr;
		if	(!acsConfig.tropOpts.vmf3dir.empty())
		{
			rtkVmf3_ptr = &rec.vmf3;
			rtkOrog_ptr = orog;
		}

		/* observed minus computed for each satellites */
		pppomc(trace, rec.rtk, rec.obsList, gptg, rec.cj, rec, rtkVmf3_ptr, rtkOrog_ptr);
	}

	if	( (acsConfig.process_rts)
		&&(acsConfig.pppOpts.rts_lag > 0)
		&&(epoch >= acsConfig.pppOpts.rts_lag))
	{
		KFState rts = RTS_Process(rec.rtk.pppState);
	}

#	ifdef ENABLE_MONGODB
	if	( acsConfig.process_user
		&&acsConfig.output_mongo_states)
	{
		mongoStates(rec.rtk.pppState);
	}
#	endif

	if (acsConfig.output_rinex_obs)
	{
		writeRinexObs(rec.id, rec.snx, tsync, rec.obsList);
	}
}

void mainPerEpochPostProcessingAndOutputs(
	Network&		net,
	StationMap&		stationMap)
{
	TestStack ts("post");
	
	auto netTrace = getTraceFile(net);

	if	( acsConfig.process_user
		&&acsConfig.output_trop_sinex)
	for (auto& [id, rec]	: stationMap)
	{
		KFState dummy;
		outputTropSinex(rec.tropFilename, tsync, stationMap,	dummy, id);
	}
	
	if	( acsConfig.process_ppp
		||acsConfig.process_network)
	{
		// Output clock product body part
		if	(acsConfig.output_clocks)
		{
			auto filenameSysMap = getSysOutputFilenames(acsConfig.clocks_filename, tsync);

			for (auto [filename, sysMap] : filenameSysMap)
			{
				outputClocks(filename, acsConfig.clocks_receiver_source, acsConfig.clocks_satellite_source, tsync, sysMap, net.kfState, &stationMap);
			}
		}
	
		if	( acsConfig.output_AR_clocks == false
			||ARsol_ready() == false)
		{
			net.kfState.outputStates(netTrace);
		}

#		ifdef ENABLE_MONGODB
		if (acsConfig.output_mongo_states)
		{
			mongoStates(net.kfState);
		}
#		endif

		/* Instantaneous AR */
		if (acsConfig.output_AR_clocks)
		{
			KFState KF_ARcopy = net.kfState;

			networkAmbigResl(netTrace, stationMap, KF_ARcopy);

			if (ARsol_ready())
			{
				KF_ARcopy.outputStates(netTrace);
			}

#			ifdef ENABLE_MONGODB
			if (acsConfig.output_mongo_states)
			{
				mongoStates(KF_ARcopy, "_AR");
			}
#			endif
		
			if (ARsol_ready())
			{
				auto filenameSysMap = getSysOutputFilenames(acsConfig.clocks_filename, tsync);

				for (auto [filename, sysMap] : filenameSysMap)
				{
					outputClocks(filename, E_Ephemeris::KALMAN, E_Ephemeris::KALMAN, tsync, sysMap, KF_ARcopy, &stationMap);
				}
			}
		}

		if	(  acsConfig.process_rts
			&& acsConfig.netwOpts.rts_lag > 0 
			&& epoch >= acsConfig.netwOpts.rts_lag)
		{
			KFState rts = RTS_Process(net.kfState, false, &stationMap, net.clockFilename, net.tropFilename);
		}
	
		if (acsConfig.ssrOpts.calculate_ssr)
		{
			std::set<SatSys> sats;	// List of satellites visible in this epoch
			for (auto& [id, rec]	: stationMap)
			for (auto& obs			: rec.obsList)
			{
				sats.insert(obs.Sat);
			}
			
			calcSsrCorrections(netTrace, net.kfState, sats, tsync);

			if (acsConfig.ssrOpts.upload_to_caster)
				outStreamManager.sendMessages(true);
			else
				rtcmEncodeToFile(epoch);
		}
	}
	
	if (acsConfig.output_orbits)
	{
		outputSp3(tsync, acsConfig.orbits_data_source, &net.kfState);
	}

	if (acsConfig.output_persistance)
	{
		outputPersistanceNav();
	}
	
#	ifdef ENABLE_MONGODB
	if (acsConfig.output_mongo_states)
	{
		outputApriori(stationMap);
	}
#	endif
	
	writeNetworkTraces(stationMap);
}

void mainOncePerEpoch(
	Network&		net,
	StationMap&		stationMap,
	GTime			tsync)
{
	Instrument	instrument	("1/Ep");
	TestStack	ts			("1/Ep");
	
	//load any changes from the config
	acsConfig.parse();
	
	avoidCollisoins(stationMap);
	
	//reload any new or modified files
	reloadInputFiles();

	createTracefiles(stationMap, net);
	
	auto netTrace = getTraceFile(net);
	
	netTrace << std::endl << "------=============== Epoch " << epoch << " =============-----------" << std::endl;
	
	//initialise mongo if not already done
#	ifdef ENABLE_MONGODB
	{
		mongoooo();
	}
#	endif

	//try to get svns of all used satellites
	for (auto& [satId, satNav] : nav.satNavMap)
	{
		SatSys Sat;
		Sat.fromHash(satId);
		PhaseCenterData* pcvsat = findAntenna(Sat.id(), tsync, nav);
		if (pcvsat)
		{
			Sat.setSvn(pcvsat->svn);
		}
	}
		
	//do per-station pre processing
	bool emptyEpoch = true;
#	ifdef ENABLE_PARALLELISATION
#	ifndef ENABLE_UNIT_TESTS
		Eigen::setNbThreads(1);
#		pragma omp parallel for
#	endif
#	endif
	for (int i = 0; i < stationMap.size(); i++)
	{
		auto rec_ptr_iterator = stationMap.begin();
		std::advance(rec_ptr_iterator, i);

		auto& [id, rec] = *rec_ptr_iterator;
		mainOncePerEpochPerStation(rec,nav.orography, nav.gptg, emptyEpoch);
	}
	Eigen::setNbThreads(0);

	if	(emptyEpoch)
	{
		BOOST_LOG_TRIVIAL(warning)
		<< "Epoch " << epoch << " has no observations";
	}
	
#	ifdef ENABLE_MONGODB
	if (acsConfig.output_mongo_measurements)
	{
		mongoMeasSatStat_all(stationMap);
	}
#	endif

	if (acsConfig.ssrOpts.calculate_ssr)
	{
		initSsrOut();
	}

	if (acsConfig.process_ppp)
	{
		PPP(netTrace, stationMap, net.kfState, nav.gptg, nav.orography);
	}

	if (acsConfig.output_rinex_nav)
	{
		writeRinexNav();
	}

	if (acsConfig.process_network)
	{
		networkEstimator(netTrace, stationMap, net.kfState, tsync);
	}

	if (1)	//acsConfig.ambiguityResolution
	{
		//ambiguityResolution(netTrace, net.kfState);
	}
	
	if (acsConfig.process_ionosphere)
	{
		update_ionosph_model(netTrace, stationMap, tsync);
	}

	if (acsConfig.delete_old_ephemerides)
	{
		cullOldEphs(tsync);
		cullOldSSRs(tsync);
	}

	if (acsConfig.check_plumbing)
	{
		plumber();
	}
	
	mainPerEpochPostProcessingAndOutputs(net, stationMap);
	
	TestStack::saveData();
	TestStack::testStatus();
}

void mainPostProcessing(
	Network&				net,
	map<string, Station>&	stationMap)
{
	auto netTrace = getTraceFile(net);
	
	KFState KF_ARcopy;
	if (acsConfig.output_AR_clocks)
	{
		KF_ARcopy = net.kfState;
	}
	
	if	(  acsConfig.process_ionosphere
		&& acsConfig.output_ionex)
	{
		ionex_file_write(netTrace, tsync, true);
	}

	if	( acsConfig.process_network
		&&acsConfig.process_minimum_constraints)
	{
		BOOST_LOG_TRIVIAL(info)
		<< std::endl
		<< "---------------PROCESSING NETWORK WITH MINIMUM CONSTRAINTS ------------- " << std::endl;

		minimum(netTrace, net.kfState);
		net.kfState.outputStates(netTrace);

		if (acsConfig.output_AR_clocks)
		{
			minimum(netTrace, KF_ARcopy);
			KF_ARcopy.outputStates(netTrace);
		}
	}

	if (acsConfig.output_sinex)
	{
		sinexPostProcessing(tsync, stationMap, net.kfState);
	}
	
	if (acsConfig.output_persistance)
	{
		BOOST_LOG_TRIVIAL(info)
		<< "Storing persistant states to continue processing...";

		outputPersistanceStates(stationMap, net.kfState);
	}

	if (acsConfig.process_network)
	{
		if	(acsConfig.output_AR_clocks)	outputOrbit(KF_ARcopy);
		else								outputOrbit(net.kfState);
	}

	if	(acsConfig.process_rts)
	{
		if (acsConfig.pppOpts.rts_lag < 0)
		for (auto& [id, rec] : stationMap)
		{
			BOOST_LOG_TRIVIAL(info)
			<< std::endl
			<< "---------------PROCESSING PPP WITH RTS------------------------- " << std::endl;

			RTS_Process(rec.rtk.pppState,	true, &stationMap, "",					rec.rtsTropFilename);
		}

		if (acsConfig.netwOpts.rts_lag < 0)
		{
			BOOST_LOG_TRIVIAL(info)
			<< std::endl
			<< "---------------PROCESSING NETWORK WITH RTS--------------------- " << std::endl;

			RTS_Process(net.kfState,		true, &stationMap, net.rtsClockFilename, net.rtsTropFilename);
		}

		if (acsConfig.ionFilterOpts.rts_lag < 0)
		{
			BOOST_LOG_TRIVIAL(info)
			<< std::endl
			<< "---------------PROCESSING IONOSPHERE WITH RTS------------------ " << std::endl;

			RTS_Process(iono_KFState,		true, &stationMap);
		}
	}

	if (acsConfig.output_bias_sinex)
	{
		bias_io_opt biaopt;
		biaopt.OSB_biases = acsConfig.ambrOpts.writeOSB;
		biaopt.DSB_biases = acsConfig.ambrOpts.writeDSB;
		biaopt.SSR_biases = false;
		biaopt.SAT_biases = acsConfig.ambrOpts.writeSATbias;
		biaopt.REC_biases = acsConfig.ambrOpts.writeRecBias;
		biaopt.COD_biases = true;
		biaopt.PHS_biases = true;

		writeBiasSinex(netTrace, tsync, net.biasSINEXFilename, biaopt);
	}

	if 	(  acsConfig.ambrOpts.WLmode != +E_ARmode::OFF
		|| acsConfig.ambrOpts.NLmode != +E_ARmode::OFF)
	{
		net_sect_out(netTrace);
	}

	TestStack::printStatus(true);
	Instrument::printStatus();

	outputSummaries(netTrace, stationMap);
}

int main(
	int		argc, 
	char**	argv)
{
	tracelevel(5);

	boost::log::core::get()->set_filter (boost::log::trivial::severity >= boost::log::trivial::info);
	boost::log::add_console_log(std::cout, boost::log::keywords::format = "%Message%");

	BOOST_LOG_TRIVIAL(info)
	<< "PEA starting... (" << GINAN_BRANCH_NAME << " " GINAN_COMMIT_VERSION " from " << GINAN_COMMIT_DATE << ")" << std::endl;

	auto peaStartTime = boost::posix_time::from_time_t(system_clock::to_time_t(system_clock::now()));

	bool pass = configure(argc, argv);
	if (pass == false)
	{
		BOOST_LOG_TRIVIAL(error) 	<< "Incorrect configuration";
		BOOST_LOG_TRIVIAL(info) 	<< "PEA finished";
		NtripSocket::io_service.stop();
		return EXIT_FAILURE;
	}
	
	if (acsConfig.output_log)
	{
		addFileLog();
	}
	
	exitOnErrors();

	TestStack::openData();

	BOOST_LOG_TRIVIAL(info)
	<< "Threading with " << Eigen::nbThreads()
	<< " threads" << std::endl;

	// Ensure the output directories exist
	for (auto& directory : {
								acsConfig.log_directory,
								acsConfig.rtcm_directory,
								acsConfig.ionex_directory,
								acsConfig.sinex_directory,
								acsConfig.trace_directory,
								acsConfig.orbits_directory,
								acsConfig.clocks_directory,
								acsConfig.ionstec_directory,
								acsConfig.ppp_sol_directory,
								acsConfig.summary_directory,
								acsConfig.testOpts.directory,
								acsConfig.rinex_obs_directory,
								acsConfig.rinex_nav_directory,
								acsConfig.trop_sinex_directory,
								acsConfig.bias_sinex_directory,
								acsConfig.persistance_directory,
								acsConfig.pppOpts.rts_directory,
								acsConfig.netwOpts.rts_directory,
								acsConfig.ssrOpts.rtcm_directory,
								acsConfig.ionFilterOpts.rts_directory
							})
	{
		if (directory == "./")	continue;
		if (directory.empty())	continue;
		
		boost::filesystem::create_directories(directory);
	}

	BOOST_LOG_TRIVIAL(info)
	<< "Logging with trace level:" << acsConfig.trace_level << std::endl << std::endl;

	tracelevel(acsConfig.trace_level);

	if (acsConfig.caster_test)
	{
		casterTest();
	}

	
	for (auto& [id, s] : ntripRtcmMultimap)
	{
		NtripRtcmStream& downStream = *s;
		downStream.ntripTrace.level_trace		= acsConfig.trace_level;
		downStream.print_stream_statistics		= acsConfig.print_stream_statistics;
	}

	for (auto& [id, s] : outStreamManager.ntripUploadStreams)
	{
		NtripBroadcaster::NtripUploadClient& uploadStream = *s;
		uploadStream.ntripTrace.level_trace		= acsConfig.trace_level;
		uploadStream.print_stream_statistics	= acsConfig.print_stream_statistics;
	}



	//read orography file for VMF3
	{
		int orog_sucess = false;
		if 	( acsConfig.tropOpts.orography	.empty() == false
			&&acsConfig.tropOpts.vmf3dir	.empty() == false)
		{
			orog_sucess = readorog(acsConfig.tropOpts.orography, nav.orography);
		}

		if (orog_sucess == false)
		{
			// read gpt2 grid file
			readgrid(acsConfig.tropOpts.gpt2grid, &nav.gptg);
		}
	}

	if (acsConfig.input_persistance)
	{
		BOOST_LOG_TRIVIAL(info)
		<< "Loading persistant navigation object to continue processing...";

		inputPersistanceNav();
	}

	if (acsConfig.process_ionosphere)
	{
		bool pass = config_ionosph_model();
		if	(pass == false)
		{
			BOOST_LOG_TRIVIAL(error)
			<< "Error in Ionosphere Model configuration";

			return EXIT_FAILURE;
		}
	}

	map<string, Station> stationMap;
	
	//prepare the satNavMap so that it at least has entries for everything
	{
		for (int prn = 1; prn <= NSATGPS; prn++)	{	SatSys s(E_Sys::GPS, prn);	nav.satNavMap[s];	}
		for (int prn = 1; prn <= NSATGLO; prn++)	{	SatSys s(E_Sys::GLO, prn);	nav.satNavMap[s];	}
		for (int prn = 1; prn <= NSATGAL; prn++)	{	SatSys s(E_Sys::GAL, prn);	nav.satNavMap[s];	}
		for (int prn = 1; prn <= NSATCMP; prn++)	{	SatSys s(E_Sys::BDS, prn);	nav.satNavMap[s];	}	
	}

	BOOST_LOG_TRIVIAL(debug)
	<< "\tCreating trace files ";

	Network net;
	{
		net.kfState.max_filter_iter		= acsConfig.netwOpts.max_filter_iter;
		net.kfState.max_prefit_remv		= acsConfig.netwOpts.max_prefit_remv;
		net.kfState.inverter			= acsConfig.netwOpts.inverter;
		net.kfState.output_residuals	= acsConfig.output_residuals;
		net.kfState.rejectCallbacks.push_back(deweightMeas);
		net.kfState.rejectCallbacks.push_back(incrementPhaseSignalError);
	}

	if (acsConfig.process_rts)
	{
		if (acsConfig.netwOpts.rts_lag)
		{
			net.kfState.rts_lag					= acsConfig.netwOpts.rts_lag;
		}

		if (acsConfig.ionFilterOpts.rts_lag)
		{
			iono_KFState.rts_lag					= acsConfig.ionFilterOpts.rts_lag;	
		}
	}

	if (acsConfig.input_persistance)
	{
		BOOST_LOG_TRIVIAL(info)
		<< "Loading persistant states to continue processing...";

		inputPersistanceStates(stationMap, net.kfState);
	}

	acsConfig.parse();
	
	reloadInputFiles();
	
	doDebugs();

	if (acsConfig.start_epoch.is_not_a_date_time() == false)
	{
		tsync.time = boost::posix_time::to_time_t(acsConfig.start_epoch);
	}

	BOOST_LOG_TRIVIAL(info)
	<< std::endl;
	BOOST_LOG_TRIVIAL(info)
	<< "Starting to process epochs...";

	TestStack ts(acsConfig.config_description);

	NtripSocket::startClients();

	//============================================================================
	// MAIN PROCESSING LOOP														//
	//============================================================================

	// Read the observations for each station and do stuff
	bool	complete					= false;							///< When all input files are empty the processing is deemed complete - run until then, or until something else breaks the loop
	int		loopEpochs					= 0;								///< A count of how many loops of epoch_interval this loop used up (usually one, but may be more if skipping epochs)
	auto	nextNominalLoopStartTime	= system_clock::now() + 10s;		///< The time the next loop is expected to start - if it doesnt start until after this, it may be skipped
	while (complete == false)
	{
		if (tsync != GTime::noTime())
		{
			tsync.time				+= loopEpochs * acsConfig.epoch_interval;
		}

		epoch						+= loopEpochs;
		nextNominalLoopStartTime	+= loopEpochs * std::chrono::milliseconds((int)(acsConfig.wait_next_epoch * 1000));

		///Calculate the time at which we will stop waiting for data to come in for this epoch
		auto breakTime	= nextNominalLoopStartTime
						+ std::chrono::milliseconds((int)(acsConfig.wait_all_stations	* 1000));

		BOOST_LOG_TRIVIAL(info) << std::endl
		<< "Starting epoch #" << epoch;

		TestStack ts("Epoch " + std::to_string(epoch));
		
		for (auto& [id, rec] : stationMap)
		{
			rec.ready = false;
		}

		//get observations from streams (allow some delay between stations, and retry, to ensure all messages for the epoch have arrived)
		bool 	foundFirst		= false;
		bool	repeat			= true;
		int		stationCount	= 0;
		while	( repeat
				&&system_clock::now() < breakTime)
		{
			repeat = false;

			for (auto& [id, s] : navStreamMultimap)
			{
				NavStream& navStream = *s;

				navStream.getNav();
			}

			//remove any dead streams
			for (auto iter = obsStreamMultimap.begin(); iter != obsStreamMultimap.end(); )
			{
				auto& [dummy, obsStream_ptr]	= *iter;
				auto& obsStream					= *obsStream_ptr;
				
				if (obsStream.isDead())
				{
					BOOST_LOG_TRIVIAL(info)
					<< "No more data available on " << obsStream.sourceString << std::endl;

					//record as dead and erase
					streamDOAMap[obsStream.sourceString] = true;

					iter = obsStreamMultimap.erase(iter);
				}
				else
				{
					iter++;
				}
			}

			if	(  obsStreamMultimap		.empty()
				&& pseudoObsStreamMultimap	.empty())
			{
				BOOST_LOG_TRIVIAL(info)
				<< std::endl;
				BOOST_LOG_TRIVIAL(info)
				<< "Inputs finished at epoch #" << epoch;

				complete = true;

				break;
			}

			for (auto& [id, s] : obsStreamMultimap)
			{
				ObsStream&	obsStream	= *s;

				auto& recOpts = acsConfig.getRecOpts(id);

				if (recOpts.exclude)
				{
					continue;
				}

				auto& rec = stationMap[id];

				if	( (rec.obsList.empty()		== false)
					&&(rec.obsList.front().time	== tsync))
				{
					//already have observations for this epoch.
					continue;
				}

				//try to get some data
				rec.obsList = obsStream.getObs(tsync);

				if (rec.obsList.empty())
				{
					//failed to get observations
					if (obsStream.obsWaitCode == +E_ObsWaitCode::NO_DATA_WAIT)
					{
						// try again later
						repeat = true;
						sleep_for(1ms);
					}
					
					continue;
				}
				
				if (tsync == GTime::noTime())
				{
					tsync.time				= ((int) (rec.obsList.front().time / acsConfig.epoch_interval)) * acsConfig.epoch_interval;
					acsConfig.start_epoch	= boost::posix_time::from_time_t(tsync);
					
					if (tsync + 0.5 < rec.obsList.front().time)
					{
						repeat = true;
						continue;	
					}
				}

				stationCount++;
				
				if (foundFirst == false)
				{
					foundFirst = true;

					//first observation found for this epoch, give any other stations some time to get their observations too
					//only shorten waiting periods, never extend
					auto now = system_clock::now();

					auto alternateBreakTime = now + std::chrono::milliseconds((int)(acsConfig.wait_all_stations	* 1000));
					auto alternateStartTime = now + std::chrono::milliseconds((int)(acsConfig.wait_next_epoch	* 1000));

					if (alternateBreakTime < breakTime)						{	breakTime					= alternateBreakTime;	}
					if (alternateStartTime < nextNominalLoopStartTime)		{	nextNominalLoopStartTime	= alternateStartTime;	}
				}


				//initialise the station if required
				if (rec.id.empty())
				{
					initialiseStation(id, rec);
				}
				
				rec.ready = true;
			}
			
			for (auto& [id, s] : pseudoObsStreamMultimap)
			{
				PseudoObsStream&	pseudoObsStream	= *s;
	
				auto& pseudoRec = stationMap[id];
				
				if	( (pseudoRec.pseudoObsList.empty()		== false)
					&&(pseudoRec.pseudoObsList.front().time	== tsync))
				{
					//already have observations for this epoch.
					continue;
				}
				
				//try to get some data
				pseudoRec.pseudoObsList = pseudoObsStream.getObs(tsync);

				if (pseudoRec.pseudoObsList.empty())
				{
					continue;
				}
				
				if (tsync == GTime::noTime())
				{
					tsync.time				= ((int)(pseudoRec.pseudoObsList.front().time / acsConfig.epoch_interval)) * acsConfig.epoch_interval;
					acsConfig.start_epoch	= boost::posix_time::from_time_t(tsync);
					
					if (tsync + 0.5 < pseudoRec.pseudoObsList.front().time)
					{
						repeat = true;
						continue;	
					}
				}
			}
		}

		if	(complete)
		{
			break;
		}

		if	(tsync == GTime::noTime())
		{
			continue;
		}
		
		BOOST_LOG_TRIVIAL(info)
		<< "Synced " << stationCount << " stations...";
		
		
		
		auto epochStartTime	= boost::posix_time::from_time_t(system_clock::to_time_t(system_clock::now()));
		{
			mainOncePerEpoch(net, stationMap, tsync);
		}
		auto epochStopTime	= boost::posix_time::from_time_t(system_clock::to_time_t(system_clock::now()));

		
		
		int week;
		double sec = time2gpst(tsync, &week);
		auto boostTime = boost::posix_time::from_time_t(tsync.time);

		BOOST_LOG_TRIVIAL(info)
		<< "Processed epoch #" << epoch
		<< " - " << "GPS time: " << week << " " << std::setw(6) << sec << " - " << boostTime 
		<< " (took " << (epochStopTime-epochStartTime) << ")";

		// Check end epoch
		if	(  acsConfig.end_epoch.is_not_a_date_time() == false
			&& boostTime >= acsConfig.end_epoch)
		{
			BOOST_LOG_TRIVIAL(info)
			<< "Exiting at epoch " << epoch << " (" << boostTime
			<< ") as end epoch " << acsConfig.end_epoch
			<< " has been reached";

			break;
		}

		// Check number of epochs
		if	(  acsConfig.max_epochs	> 0
			&& epoch					>= acsConfig.max_epochs)
		{
			BOOST_LOG_TRIVIAL(info)
			<< std::endl
			<< "Exiting at epoch " << epoch << " (" << boostTime
			<< ") as epoch count " << acsConfig.max_epochs
			<< " has been reached";

			break;
		}

		///Calculate how many loops need to be skipped based on when the next loop was supposed to begin
		auto loopStopTime		= system_clock::now();
		auto loopExcessDuration = loopStopTime - (nextNominalLoopStartTime + std::chrono::milliseconds((int)(acsConfig.wait_all_stations * 1000)));
		int excessLoops			= loopExcessDuration / std::chrono::milliseconds((int)(acsConfig.wait_next_epoch * 1000));

		if (excessLoops < 0)		{	excessLoops = 0;	}
		if (excessLoops > 0)	
		{
			BOOST_LOG_TRIVIAL(warning) << std::endl 
			<< "Excessive time elapsed, skipping " << excessLoops 
			<< " epochs. Configuration 'wait_next_epoch' is " << acsConfig.wait_next_epoch;	
		}

		loopEpochs = 1 + excessLoops;
	}

	// Disconnect the downloading clients and stop the io_service for clean shutdown.
	for (auto& [id, s] : ntripRtcmMultimap)
	{
		NtripStream& downStream = *s;
		downStream.disconnect();
	}
	outStreamManager.stopBroadcast();
	NtripSocket::io_service.stop();

	auto peaInterTime = boost::posix_time::from_time_t(system_clock::to_time_t(system_clock::now()));
	BOOST_LOG_TRIVIAL(info)
	<< std::endl
	<< "PEA started  processing at : " << peaStartTime << std::endl
	<< "and finished processing at : " << peaInterTime << std::endl
	<< "Total processing duration  : " << (peaInterTime - peaStartTime) << std::endl << std::endl;

	BOOST_LOG_TRIVIAL(info)
	<< std::endl
	<< "Finalising streams and post processing...";

	mainPostProcessing(net, stationMap);

	auto peaStopTime = boost::posix_time::from_time_t(system_clock::to_time_t(system_clock::now()));

	BOOST_LOG_TRIVIAL(info)
	<< std::endl
	<< "PEA started  processing at : " << peaStartTime	<< std::endl
	<< "and finished processing at : " << peaStopTime	<< std::endl
	<< "Total processing duration  : " << (peaStopTime - peaStartTime) << std::endl << std::endl;

	BOOST_LOG_TRIVIAL(info)
	<< "PEA finished";

	return EXIT_SUCCESS;
}
