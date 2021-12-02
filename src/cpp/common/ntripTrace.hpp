#ifndef ACS_RTCM_TRACE
#define ACS_RTCM_TRACE

#include "enums.h"
#include "navigation.hpp"
#include "satSys.hpp"

#include <iostream>
#include <string>
#include <vector>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/asio/buffer.hpp>
#include <boost/asio.hpp>


struct NetworkData
{
	std::string streamName;
	
	GTime startTime;
	GTime endTime;
    long int	numPreambleFound	= 0;
    long int	numFramesFailedCRC	= 0;
    long int	numFramesPassCRC	= 0;
    long int	numFramesDecoded	= 0;
    long int	numNonMessBytes		= 0;
	long int	numMessagesLatency	= 0;	
	double		totalLatency		= 0;

    int disconnectionCount = 0;
    boost::posix_time::time_duration connectedDuration		= boost::posix_time::hours(0);
    boost::posix_time::time_duration disconnectedDuration	= boost::posix_time::hours(0);
	
    int numberErroredChunks = 0; 
	int numberChunks = 0;
	
	void clearStatistics(GTime tStart, GTime tEnd);
	void accumulateStatisticsFrom(NetworkData& dataToAdd);
	
	std::string previousJSON;
	std::string getJsonNetworkStatistics(GTime now,std::string label);
	void printTraceNetworkStatistics(Trace& trace);
};

struct NtripTrace 
{
	int level_trace = 0;
	std::string mountPoint;
	
	boost::asio::streambuf ssrPhBBuf;
	boost::asio::streambuf ssrCoBBuf;
	boost::asio::streambuf ssrClkBuf;
	boost::asio::streambuf ssrEphBuf;
	boost::asio::streambuf broEphBuf;
	
	boost::asio::streambuf netConnBuf;
	boost::asio::streambuf messErrChunkBuf;
	boost::asio::streambuf messErrRtcmBuf;
	boost::asio::streambuf messErrRtcmByteBuf;
	
	void traceSsrEph(SatSys Sat,SSREph ssrEph);
	void traceSsrClk(SatSys Sat,SSRClk ssrClk);
	void traceSsrCodeB(SatSys Sat,E_ObsCode mode, SSRBias ssrBias);
	void traceSsrPhasB(SatSys Sat,E_ObsCode mode, SSRBias ssrBias);
	void traceBroEph(Eph eph,E_Sys sys);
	
	
	void networkLog(std::string message)
	{
		if( level_trace < 3 )
			return;
		
		//BOOST_LOG_TRIVIAL(debug) << "NtripTrace::networkLog : " << message << std::endl;
		std::ostream outStream(&netConnBuf);
		outStream << boost::posix_time::from_time_t(std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()));
		outStream << " " << message << std::endl;
	}
	
	void messageChunkLog(std::string message)
	{
		if( level_trace < 3 )
			return;
		
		std::ostream outStream(&messErrChunkBuf);
		outStream << boost::posix_time::from_time_t(std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()));
		outStream << " " << message << std::endl;
	}
	
	void messageRtcmLog(std::string message)
	{
		if( level_trace < 3 )
			return;
		
		std::ostream outStream(&messErrRtcmBuf);
		outStream << boost::posix_time::from_time_t(std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()));
		outStream << " " << message << std::endl;
	}

	void messageRtcmByteLog(std::string message)
	{
		if( level_trace < 3 )
			return;
		
		std::ostream outStream(&messErrRtcmByteBuf);
		outStream << boost::posix_time::from_time_t(std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()));
		outStream << " " << message << std::endl;
	}   

	void traceWriteEpoch(Trace& trace);
};
#endif
