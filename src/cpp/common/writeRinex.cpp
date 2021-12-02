
#include "streamTrace.hpp"
#include "writeRinex.hpp"
#include "acsConfig.hpp"

#include <boost/algorithm/string/replace.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/log/trivial.hpp>

#include <algorithm>
#include <fstream>
#include <math.h>

void recordRinexObservations(
	RinexOutput&	rinexOutput,
	ObsList&		obsList)
{
	bool foundNewObs = false;
	for (auto& obs : obsList)
	{
		E_Sys sys = obs.Sat.sys;

		auto& obsCodeDesc = rinexOutput.codesPerSys[sys];

		for (auto& [ftype,sigsList] : obs.SigsLists)
		for (auto& sig : sigsList)
		{
			if (sig.code == +E_ObsCode::NONE)
				continue;

			if (std::find(obsCodeDesc.begin(), obsCodeDesc.end(), pair<E_ObsCode,E_ObsDesc>(sig.code,E_ObsDesc::C)) == obsCodeDesc.end()
				&& sig.P != 0
				&& acsConfig.rinex_obs_print_C_code)
			{
				foundNewObs = true;
				obsCodeDesc.push_back({sig.code, E_ObsDesc::C});
			}

			if (std::find(obsCodeDesc.begin(), obsCodeDesc.end(), pair<E_ObsCode,E_ObsDesc>(sig.code,E_ObsDesc::L)) == obsCodeDesc.end() 
				&& sig.L != 0 
				&& acsConfig.rinex_obs_print_L_code)
			{
				foundNewObs = true;
				obsCodeDesc.push_back({sig.code, E_ObsDesc::L});
			}

			if (std::find(obsCodeDesc.begin(), obsCodeDesc.end(), pair<E_ObsCode,E_ObsDesc>(sig.code,E_ObsDesc::D)) == obsCodeDesc.end() 
				&& sig.D != 0
				&& acsConfig.rinex_obs_print_D_code)
			{
				foundNewObs = true;
				obsCodeDesc.push_back({sig.code, E_ObsDesc::D});
			}

			double sn_raw = (sig.snr-0.5) / 4;	//todo aaron, check this
			
			if (std::find(obsCodeDesc.begin(), obsCodeDesc.end(), pair<E_ObsCode,E_ObsDesc>(sig.code,E_ObsDesc::S)) == obsCodeDesc.end() 
				&& sn_raw != 0
				&& acsConfig.rinex_obs_print_S_code)
			{
				foundNewObs = true;
				obsCodeDesc.push_back({sig.code, E_ObsDesc::S});
			}
		}
		
		if (obsCodeDesc.size() == 0)
			rinexOutput.codesPerSys.erase(rinexOutput.codesPerSys.find(sys));
	}

	//first create if non existing
	{
		std::fstream maker	(rinexOutput.fileName, std::ios::app);
	}
	std::fstream rinexStream(rinexOutput.fileName);

	rinexStream.seekp(0, std::ios::end);
	
	long endFilePos = rinexStream.tellp();
	
	if (endFilePos == 0)
	{
		if (obsList.empty())
		{
			BOOST_LOG_TRIVIAL(error) << "Writing RINEX file header with no observations.";
			return;
		}
		
		rinexOutput.headerTimePos = 0;

		// Write the RINEX header.
		GTime now = obsList[0].time;
		
		char tempStr[20];
		time2str(now, tempStr, 0);
		
		string timeDate(tempStr);
		boost::replace_all(timeDate, "/", "");
		boost::replace_all(timeDate, ":", "");
		timeDate += " UTC";

		double rnxver = 3.05;
		const char sys[]	= "M: Mixed";
		const char prog[]	= "PEA v1";
		const char runby[]	= "Geoscience Australia";

		tracepdeex(0,rinexStream,"%9.2f%-11s%-20s%-20s%-20s\n",
			rnxver,
			"",
			"OBSERVATION DATA",
			sys,
			"RINEX VERSION / TYPE");

		tracepdeex(0,rinexStream,"%-20.20s%-20.20s%-20.20s%-20s\n",
			prog,
			runby,
			timeDate.c_str(),
			"PGM / RUN BY / DATE");

		tracepdeex(0,rinexStream,"%-60.60s%-20s\n",
			rinexOutput.snx.sitecode,
			"MARKER NAME");

		tracepdeex(0,rinexStream,"%-20.20s%-40.40s%-20s\n",
			rinexOutput.snx.monuid,
			"",
			"MARKER NUMBER");

		//TODO Add marker type as RINEX version is greater than 2.99
		//tracepdeex(0,rinexStream,"%-20.20s%-40.40s%-20s\n",rinexOutput.snx.,"","MARKER TYPE");

		tracepdeex(0,rinexStream,"%-20.20s%-40.40s%-20s\n",
			"",
			acsConfig.analysis_center,
			"OBSERVER / AGENCY");

		tracepdeex(0, rinexStream, "%-20.20s%-20.20s%-20.20s%-20s\n",
			rinexOutput.snx.recsn,
			rinexOutput.snx.rectype,
			rinexOutput.snx.recfirm,
			"REC # / TYPE / VERS");

		tracepdeex(0, rinexStream, "%-20.20s%-20.20s%-20.20s%-20s\n",
			rinexOutput.snx.antsn,
			rinexOutput.snx.anttype,
			"",
			"ANT # / TYPE");

		tracepdeex(0, rinexStream, "%14.4f%14.4f%14.4f%-18s%-20s\n",
			rinexOutput.snx.pos.x(),
			rinexOutput.snx.pos.y(),
			rinexOutput.snx.pos.z(),
			"",
			"APPROX POSITION XYZ");

		tracepdeex(0, rinexStream, "%14.4f%14.4f%14.4f%-18s%-20s\n",
			rinexOutput.snx.ecc[2],
			rinexOutput.snx.ecc[1],
			rinexOutput.snx.ecc[0],
			"",
			"ANTENNA: DELTA H/E/N");

		rinexOutput.headerObsPos = rinexStream.tellp();
	}


	if	(  endFilePos == 0 
		|| foundNewObs)
	{
		rinexStream.seekp(rinexOutput.headerObsPos);
		const char label[] = "SYS / # / OBS TYPES";

		int numSysLines = 0;
		for (auto &[sys, obsCodeDesc] : rinexOutput.codesPerSys)
		{
			auto dummySat = SatSys(sys, 0);
			char sys_c = dummySat.sysChar();
	
			if (sys_c == '-')
			{
				BOOST_LOG_TRIVIAL(error) << "Writing RINEX file undefined system.";
				return;
			}

			tracepdeex(0, rinexStream, "%c  %3d", sys_c, obsCodeDesc.size());
			
			int obsCodeCnt = 0;
			for (auto& [obsCode, obsDesc] : obsCodeDesc)
			{
				obsCodeCnt++;
				auto obsDescStr = obsDesc._to_string();
				auto obsCodeStr = obsCode._to_string();
				char obsStr[4];
				obsStr[0] = obsDescStr[0];
				obsStr[1] = obsCodeStr[1];
				obsStr[2] = obsCodeStr[2];
				obsStr[3] = 0;

				if	( obsCodeCnt % 13	== 1
					&&obsCodeCnt		!= 1)
				{
					tracepdeex(0, rinexStream, "      ");
				}
				
				tracepdeex(0, rinexStream, " %3s", obsStr);

				if (obsCodeCnt % 13 == 0)
				{
					// After 13 observations make a new line.
					tracepdeex(0, rinexStream, "  %-20s\n", label);
					numSysLines++;
				}
			}

			if (obsCodeCnt % 13 != 0)
			{
				// less than 13 entries and a new line is required.
				while (obsCodeCnt % 13 != 0)
				{
					obsCodeCnt++;
					tracepdeex(0, rinexStream, " %3s", "");
				}
				
				tracepdeex(0, rinexStream, "  %-20s\n", label);
				numSysLines++;
			}
		}
		
		while (numSysLines < 2 * E_Sys::NUM_SYS)
		{
			//add some lines to be filled in later to allow for the maximum number expected
			tracepdeex(0, rinexStream, "%-60.60s%-20s\n", "", "COMMENT");
			numSysLines++;
		}
	}
	
	char	tsys[] = "GPS"; // PEA internal time is GPS.
	double	ep[6];
	time2epoch(obsList[0].time, ep);	
	
	if (rinexOutput.headerTimePos == 0)
	{
		tracepdeex(0, rinexStream, "%10.3f%50s%-20s\n",
			acsConfig.epoch_interval,
			"",
			"INTERVAL");
		
		tracepdeex(0,rinexStream,"  %04.0f%6.0f%6.0f%6.0f%6.0f%13.7f     %-12s%-20s\n",
			ep[0],
			ep[1],
			ep[2],
			ep[3],
			ep[4],
			ep[5],
			tsys,
			"TIME OF FIRST OBS");
		
		rinexOutput.headerTimePos = rinexStream.tellp();
	}

	rinexStream.seekp(rinexOutput.headerTimePos);
	
    tracepdeex(0, rinexStream, "  %04.0f%6.0f%6.0f%6.0f%6.0f%13.7f     %-12s%-20s\n",
		ep[0],
		ep[1],
		ep[2],
		ep[3],
		ep[4],
		ep[5],
		tsys,
		"TIME OF LAST OBS");	

	if (endFilePos == 0)
	{
		tracepdeex(0, rinexStream, "%-60.60s%-20s\n",
			"",
			"END OF HEADER");
	}

	// Write the RINEX body.
	rinexStream.seekp(0, std::ios::end);
	
	// flag epoch flag (0:ok,1:power failure,>1:event flag)
	int		flag = 0;
	tracepdeex(0,rinexStream,"> %04.0f %02.0f %02.0f %02.0f %02.0f%11.7f  %d%3d%21s\n",
		ep[0],
		ep[1],
		ep[2],
		ep[3],
		ep[4],
		ep[5],
		flag,
		obsList.size(),
		"");

	for (auto& obs : obsList) 
	{
		tracepdeex(0, rinexStream, "%s", obs.Sat.id().c_str());

		auto& obsCodeDesc = rinexOutput.codesPerSys[obs.Sat.sys];

		for (auto& [obsCode, obsDesc] : obsCodeDesc)
		{
			if (obsCode == +E_ObsCode::NONE)
				continue;
			
			bool foundObsPair = false;

			for (auto& [ftype, sigList]	: obs.SigsLists)
			for (auto& sig				: sigList)
			{
				if (sig.code != obsCode)
					continue;

				// if it locates the E_ObsCode then it will always locate E_ObsDesc.
				if (foundObsPair)
				{
					BOOST_LOG_TRIVIAL(error) << "Writing RINEX file duplicated observation.";
					break;
				}
				else
				{
					foundObsPair = true;
				}

				double sn_raw = (sig.snr - 0.5) / 4;
				int sn_rnx = std::min(std::max((int) std::round(sn_raw / 6.0), 1), 9);
				
				switch (obsDesc)
				{
					case E_ObsDesc::C:
						//tracepdeex(0,rinexStream,"%14.3f %d",sig.P,sSI);
						if (sig.P == 0)		tracepdeex(0, rinexStream, "%14.3s  ", "");
						else				tracepdeex(0, rinexStream, "%14.3f  ", sig.P);
						break;

					case E_ObsDesc::L:
						if (sig.L == 0)		tracepdeex(0, rinexStream, "%14.3s  ",  "");
						else				tracepdeex(0, rinexStream, "%14.3f%d%d", sig.L, (uint)sig.LLI, sn_rnx);
						break;

					case E_ObsDesc::D:
						if (sig.D == 0)		tracepdeex(0, rinexStream, "%14.3s  ", "");
						else				tracepdeex(0, rinexStream, "%14.3f  ", sig.D);
						break;

					case E_ObsDesc::S:
						if (sn_raw == 0)	tracepdeex(0, rinexStream, "%14.3s  ", "");
						else				tracepdeex(0, rinexStream, "%14.3f  ", sn_raw);
						break;

					default :
						BOOST_LOG_TRIVIAL(error) << "Writing RINEX unknown/unused observation code.";
						break;
				}
			}
			
			if (foundObsPair == false)
			{
				// Observation code and description not in observation.
				tracepdeex(0, rinexStream, "%14.3s  ", "");
			}
		}
		rinexStream << "\n";
	}
}
