
#ifndef ACS_STREAM_H
#define ACS_STREAM_H

#include <iostream>
#include <sstream>
#include <utility>
#include <vector>
#include <string>
#include <thread>
#include <list>
#include <map>


using std::multimap;
using std::string;
using std::tuple;
using std::list;
using std::pair;
using std::map;


#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/iostreams/device/array.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/iostreams/stream.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/trivial.hpp>
#include <boost/filesystem.hpp>
#include <boost/date_time.hpp>
#include <boost/format.hpp>
#include <boost/regex.hpp>
#include <boost/asio.hpp>

namespace B_io		= boost::iostreams;
namespace B_asio	= boost::asio;

using boost::asio::ip::tcp;

#ifndef WIN32
#	include <dirent.h>
#	include <time.h>
#	include <sys/time.h>
#	include <sys/stat.h>
#	include <sys/types.h>
#endif

// #include "observations.hpp"
// #include "navigation.hpp"
// #include "ntripTrace.hpp"
// #include "station.hpp"
// #include "common.hpp"
// #include "gTime.hpp"
// #include "rinex.hpp"
// #include "enum.h"


#include "acsObsStream.hpp"
#include "acsNavStream.hpp"
#include "acsSp3Stream.hpp"
#include "acsFileStream.hpp"
#include "acsRtcmStream.hpp"
#include "acsRinexStream.hpp"
#include "acsNtripStream.hpp"




/** Object that streams RTCM data from NTRIP castors.
* Overrides interface functions
*/
struct NtripRtcmStream : NtripStream, RtcmStream
{
    /** NtripTrace is an object in this class as it is also used for the uploading
     * stream and so it cannot inherit NtripStream and RtcmStream. The overide function
     * provide for other classes to call functions for NtripTrace without a pointer or
     * address both of which are problematic for code resuse and understanding.
     */
    NtripTrace ntripTrace;
    bool print_stream_statistics = false;
	
	NetworkDataDownload epochData;
	NetworkDataDownload hourlyData;
	NetworkDataDownload runData;
	
	NtripRtcmStream(const string& url_str) : NtripStream(url_str)
	{
		ntripTrace.mountPoint = url.path;
		//url = URL::parse(url_str);		// (moved to ntripSocket.hpp)
		sourceString = url_str; //# new
		open();
	}

	void setUrl(const string& url_str)
	{		
		sourceString = url_str;
		NtripStream::setUrl(url_str);
	}

	void open()
	{
// 		connect();
	}

	ObsList getObs() override
	{
		//get all data available from the ntrip stream and convert it to an iostream for processing
		getData();

		B_io::basic_array_source<char>					input_source(receivedData.data(), receivedData.size());
		B_io::stream<B_io::basic_array_source<char>>	inputStream(input_source);

		parseRTCM(inputStream);

		int pos = inputStream.tellg();
		if (pos < 0)
		{
			receivedData.clear();
		}
		else
		{
			receivedData.erase(receivedData.begin(), receivedData.begin() + pos);
		}

		//call the base function once it has been prepared
		ObsList obsList = ObsStream::getObs();
		return obsList;
	}
	
	void traceBroEph(Eph eph,E_Sys sys) override
	{
		ntripTrace.traceBroEph(eph,sys);
	}
	
	void traceSsrEph(SatSys Sat,SSREph ssrEph) override
	{
		ntripTrace.traceSsrEph(Sat,ssrEph);
	}
	
	void traceSsrClk(SatSys Sat,SSRClk ssrClk) override
	{
		ntripTrace.traceSsrClk(Sat,ssrClk);
	}
	
	void traceSsrCodeB(SatSys Sat,E_ObsCode mode, SSRBias ssrBias) override
	{
		ntripTrace.traceSsrCodeB(Sat,mode,ssrBias);
	}
	
	void traceSsrPhasB(SatSys Sat,E_ObsCode mode, SSRBias ssrBias) override
	{
		ntripTrace.traceSsrPhasB(Sat,mode,ssrBias);
	}
	
	void messageChunkLog(std::string message) override 
	{
		ntripTrace.messageChunkLog(message);
	}
	
	void messageRtcmLog(std::string message) override
	{
		ntripTrace.messageRtcmLog(message);
	}
	
	void messageRtcmByteLog(std::string message) override
	{
		ntripTrace.messageRtcmByteLog(message);
	}    
	
	void networkLog(std::string message) override
	{
		ntripTrace.networkLog(message);
	}

	void connectionError(const boost::system::error_code& err, std::string operation) override;
	void serverResponse(unsigned int status_code, std::string http_version) override;
 
	void traceMakeNetworkOverview(Trace& trace, NetworkDataDownload& netData);
	
	void traceWriteEpoch(Trace& trace)
	{
		traceMakeNetworkOverview(trace,runData);
		ntripTrace.traceWriteEpoch(trace);
	}

	void getJsonNetworkStatistics(GTime now);

	void getNav() override
	{
		//get all data available from the ntrip stream and convert it to an iostream for processing
		getData();

		B_io::basic_array_source<char>					input_source(receivedData.data(), receivedData.size());
		B_io::stream<B_io::basic_array_source<char>>	inputStream(input_source);

		parseRTCM(inputStream);

		int pos = inputStream.tellg();
		if (pos < 0)
		{
			receivedData.clear();
		}
		else
		{
			receivedData.erase(receivedData.begin(), receivedData.begin() + pos);
		}
	}
};

/** Object that streams RTCM data from a file.
* Overrides interface functions
*/
struct FileRtcmStream : ACSFileStream, RtcmStream
{
	FileRtcmStream(const string& path)
	{		
		sourceString = path;
		setPath(path);
		open();
	}

	void open()
	{
		openFile();
	}

	int lastObsListSize = -1;

	ObsList getObs() override
	{
// 		getData();	not needed for files
		FileState fileState = openFile();

		if (obsListList.size() < 3)
		{
			parseRTCM(fileState.inputStream);
		}

		//call the base function once it has been prepared
		ObsList obsList = ObsStream::getObs();

		lastObsListSize = obsList.size();
		return obsList;
	}
    
	void getNav() override
	{
// 		getData();	not needed for files
		FileState fileState = openFile();
		
		parseRTCM(fileState.inputStream);
	}

	bool isDead() override
	{
		if (filePos < 0)
		{
			return true;
		}
		
		FileState fileState = openFile();
		
		if	( (lastObsListSize != 0)
			||(fileState.inputStream))
		{
			return false;
		}
		else
		{
			return true;
		}
	}
};

/** Object that streams RINEX data from a file.
* Overrides interface functions
*/
struct FileRinexStream : ACSFileStream, RinexStream
{
	FileRinexStream()
	{

	}

	FileRinexStream(const string& path)
	{		
		sourceString = path;
		setPath(path);
		open();
	}

	void open()
	{
		FileState fileState = openFile();
		
		readRinexHeader(fileState.inputStream);
	}

	int lastObsListSize = -1;

	ObsList getObs() override
	{
		FileState fileState = openFile();
		
// 		getData();	not needed for files

		if (obsListList.size() < 2)
		{
			parseRINEX(fileState.inputStream);
		}

		//call the base function once it has been prepared
		ObsList obsList = ObsStream::getObs();

		lastObsListSize = obsList.size();
		return obsList;
	}
	
	bool parse()
	{
		FileState fileState = openFile();
		
		parseRINEX(fileState.inputStream);
		
		return true;
	}

	bool isDead() override
	{
		if (filePos < 0)
		{
			return true;
		}
		
		FileState fileState = openFile();
		
		if	( lastObsListSize != 0
			||fileState.inputStream)
		{
			return false;
		}
		else
		{
			return true;
		}
	}
};

/** Object that streams RINEX data from a file.
* Overrides interface functions
*/
struct FileSp3Stream : ACSFileStream, SP3Stream
{
	FileSp3Stream()
	{

	}

	FileSp3Stream(const string& path)
	{		
		sourceString = path;
		setPath(path);
		open();
	}

	void open()
	{
		FileState fileState = openFile();
		
		readSP3Header(fileState.inputStream);
	}

	int lastObsListSize = -1;

	PseudoObsList getObs() override
	{
		FileState fileState = openFile();
		
// 		getData();	not needed for files

		if (obsListList.size() < 2)
		{
			parseSP3(fileState.inputStream);
		}

		//call the base function once it has been prepared
		PseudoObsList obsList = PseudoObsStream::getObs();

		lastObsListSize = obsList.size();
		return obsList;
	}
	
	bool parse()
	{
		FileState fileState = openFile();
		
		parseSP3(fileState.inputStream);
		
		return true;
	}

	bool isDead() override
	{
		if (filePos < 0)
		{
			return true;
		}
		
		FileState fileState = openFile();
		
		if	( lastObsListSize != 0
			||fileState.inputStream)
		{
			return false;
		}
		else
		{
			return true;
		}
	}
};


typedef std::shared_ptr<ObsStream>			ACSObsStreamPtr;
typedef std::shared_ptr<NavStream>			ACSNavStreamPtr;
typedef std::shared_ptr<PseudoObsStream>	ACSPseudoObsStreamPtr;

extern	multimap<string, std::shared_ptr<NtripRtcmStream>>	ntripRtcmMultimap;
extern	multimap<string, ACSObsStreamPtr>					obsStreamMultimap;
extern	multimap<string, ACSNavStreamPtr>					navStreamMultimap;
extern	multimap<string, ACSPseudoObsStreamPtr>				pseudoObsStreamMultimap;
extern	map		<string, bool>								streamDOAMap;



void writeNetworkTraces(
	StationMap&		stationMap);
	
#endif
