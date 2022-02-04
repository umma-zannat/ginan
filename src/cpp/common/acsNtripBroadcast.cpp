#define BOOST_BIND_GLOBAL_PLACEHOLDERS

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

using boost::property_tree::ptree;
using boost::property_tree::write_json;

#include "acsNtripBroadcast.hpp"
#include "ntripCasterService.hpp"
#include "acsStream.hpp"

NtripBroadcaster outStreamManager;

void NtripBroadcaster::stopBroadcast()
{
	for (auto [label, outStream] : ntripUploadStreams)
	{
		outStream->disconnect();
	}
	
	ntripUploadStreams.clear();
}

void NtripBroadcaster::NtripUploadClient::connectionError(
	const boost::system::error_code& 	err,
	string 								operation)
{
	if (acsConfig.output_log == false)
		return;

	std::ofstream logStream(acsConfig.log_filename, std::ofstream::app);

	if (!logStream)
	{
		BOOST_LOG_TRIVIAL(warning) << "Error opening log file.\n";
		return;
	}
	
	auto time = std::chrono::system_clock::to_time_t(system_clock::now());

	ptree root;
	root.put("label", 			"connectionError");
	root.put("Stream", 			url.path.substr(1,url.path.length()));
	root.put("Time", 			std::put_time(std::localtime( &time ), "%F %X"));
	root.put("BoostSysErrCode", err.value());
	root.put("BoostSysErrMess", err.message());
	root.put("SocketOperation", operation);
	
	write_json(logStream, root, false);
}

void NtripBroadcaster::NtripUploadClient::serverResponse(
	unsigned int	status_code,
	string 			http_version)
{
	if (acsConfig.output_log == false)
		return;

	std::ofstream logStream(FileLog::path_log, std::ofstream::app);
	
	if (!logStream)
	{
		BOOST_LOG_TRIVIAL(warning) << "Error opening log file.\n";
		return;
	}

	auto time = std::chrono::system_clock::to_time_t(system_clock::now());

	ptree root;
	root.put("label", 			"serverResponse");
	root.put("Stream", 			url.path.substr(1,url.path.length()));
	root.put("Time", 			std::put_time(std::localtime( &time ), "%F %X"));
	root.put("ServerStatus", 	status_code);
	root.put("VersionHTTP",		http_version);
	write_json(logStream, root, false);
}


void NtripBroadcaster::NtripUploadClient::getJsonNetworkStatistics(
	GTime now)
{
	if (acsConfig.output_log == false)
		return;

	std::ofstream logStream(FileLog::path_log, std::ofstream::app);

	if (!logStream)
	{
		BOOST_LOG_TRIVIAL(warning) << "Error opening log file.\n";
		return;
	}

	// Copy statistics into total run based statics.
	if (runData.startTime.time == 0)
	{
		GTime start;
		start.time = boost::posix_time::to_time_t(startTime);
		start.sec = (startTime.time_of_day().total_milliseconds() - startTime.time_of_day().total_milliseconds()) / 1000.0;
		runData		.startTime = start;
		hourlyData	.startTime = start;
		epochData	.startTime = start;

		runData.endTime.time	= 0;
		runData.endTime.sec		= 0;

		runData		.streamName = url.path.substr(1, url.path.length());
		hourlyData	.streamName = runData.streamName;
		epochData	.streamName = runData.streamName;

		// Set the first hours data to zero.
		GTime tEnd = now;
		tEnd.time = now.time + 1 * 60 * 60;
		hourlyData.clearStatistics(now, tEnd);
		hourlyData.getJsonNetworkStatistics(now, "hourlyData");
	}
	else
	{
		epochData.startTime = epochData.endTime;
	}

	epochData.endTime = now;
	epochData.disconnectionCount = disconnectionCount;	disconnectionCount = 0;

	// When the connection has cycled within outside an epoch.
	epochData.connectedDuration = connectedDuration - epochConnectedDuration;
	epochConnectedDuration = connectedDuration;

	epochData.disconnectedDuration = disconnectedDuration - epochDisconnectedDuration;
	epochDisconnectedDuration = disconnectedDuration;

	// When the connection cycle is within the epoch transition.
	if (isConnected)
	{
		boost::posix_time::ptime pNow	= boost::posix_time::from_time_t(now.time)
										+ boost::posix_time::milliseconds((int)now.sec * 1000);

		boost::posix_time::ptime pLast	= boost::posix_time::from_time_t(epochData.startTime.time)
										+ boost::posix_time::milliseconds((int)epochData.startTime.sec * 1000);

		boost::posix_time::time_duration epochDur	= pNow - pLast;
		boost::posix_time::time_duration conDur		= pNow - connectedTime;

		if (conDur < epochDur)
		{
			epochData.connectedDuration 	+= conDur;
			epochConnectedDuration 			+= conDur;
			epochData.disconnectedDuration 	+= epochDur - conDur;
			epochDisconnectedDuration 		+= epochDur - conDur;
		}
	}
	else
	{
		boost::posix_time::ptime pNow	= boost::posix_time::from_time_t(now.time)
										+ boost::posix_time::milliseconds((int)now.sec * 1000);

		boost::posix_time::ptime pLast	= boost::posix_time::from_time_t(epochData.startTime.time)
										+ boost::posix_time::milliseconds((int)epochData.startTime.sec * 1000);

		boost::posix_time::time_duration epochDur	= pNow - pLast;
		boost::posix_time::time_duration disConDur	= pNow - disconnectedTime;

		if (disConDur < epochDur)
		{
			epochData.disconnectedDuration	+= disConDur;
			epochDisconnectedDuration		+= disConDur;
			epochData.connectedDuration		+= epochDur - disConDur;
			epochConnectedDuration			+= epochDur - disConDur;
		}
	}

	epochData.numberChunks = numberValidChunks + numberErroredChunks;
	numberValidChunks = 0;
	epochData.numberErroredChunks = numberErroredChunks;
	numberErroredChunks = 0;

	if (hourlyData.endTime < now)
	{
		hourlyData.getJsonNetworkStatistics(now, "hourlyData");
		GTime tEnd = now;
		tEnd.time = now.time + 1 * 60 * 60;
		hourlyData.clearStatistics(now, tEnd);
	}

	hourlyData	.accumulateStatisticsFrom(epochData);
	runData		.accumulateStatisticsFrom(epochData);

	string strJson;
	strJson = epochData.getJsonNetworkStatistics(now, "epochData");		logStream << strJson;
	strJson = hourlyData.previousJSON;									logStream << strJson;
	strJson = runData.getJsonNetworkStatistics(now, "runData");			logStream << strJson;
}

void NtripBroadcaster::NtripUploadClient::sendMessages(
	bool useSsrOut)
{
	for (auto RtcmMess : rtcmMessagesTypes)
	{
		switch (RtcmMess)
		{
			case +RtcmMessageType::GPS_SSR_COMB_CORR:		encodeSsrComb	(E_Sys::GPS, useSsrOut);		break;	
			case +RtcmMessageType::GAL_SSR_COMB_CORR:		encodeSsrComb	(E_Sys::GAL, useSsrOut);		break;		
			case +RtcmMessageType::GPS_SSR_PHASE_BIAS:		encodeSsrPhase	(E_Sys::GPS, useSsrOut);		break;		
			case +RtcmMessageType::GAL_SSR_PHASE_BIAS:		encodeSsrPhase	(E_Sys::GAL, useSsrOut);		break;		
			case +RtcmMessageType::GPS_SSR_CODE_BIAS:		encodeSsrCode	(E_Sys::GPS, useSsrOut);		break;		
			case +RtcmMessageType::GAL_SSR_CODE_BIAS:		encodeSsrCode	(E_Sys::GAL, useSsrOut);		break; 
			default:
				BOOST_LOG_TRIVIAL(error) << "Error, attempting to upload incorrect message type.\n";
		}
	}
	std::stringstream messStream;
	encodeWriteMessages(messStream);

	messStream.seekg(0, messStream.end);
	int length = messStream.tellg();
			
	BOOST_LOG_TRIVIAL(debug) << "Called, NtripBroadcaster::NtripUploadClient::sendMessages(), MessageLength : " << length << std::endl;
	if (length != 0)
	{    
		vector<char> data;
		data.resize(length);
		
		outMessagesMtx.lock();
		std::ostream chunkedStream(&outMessages);
		chunkedStream << std::uppercase << std::hex << length << "\r\n";
		
		messStream.seekg(0, messStream.beg);
		
		
		messStream.read(&data[0], length);
		chunkedStream.write(&data[0], length);
		chunkedStream << "\r\n";        
	
		if (url.protocol == "https")
		{
			boost::asio::async_write(*_sslsocket, outMessages,
				boost::bind(&NtripBroadcaster::NtripUploadClient::writeResponse, this,
				boost::asio::placeholders::error));          
		}
		else
		{
			boost::asio::async_write(*_socket, outMessages,
				boost::bind(&NtripBroadcaster::NtripUploadClient::writeResponse, this,
				boost::asio::placeholders::error));
		}
		numberValidChunks++;
	}
}

void NtripBroadcaster::NtripUploadClient::connected()
{
	// Although there should be no downloading attempting to download monitors the socket connection.
	start_read_stream();
}


void NtripBroadcaster::NtripUploadClient::writeResponse(
	const boost::system::error_code& err)
{
	if (err)
	{
		outMessages.consume(outMessages.size());
		outMessagesMtx.unlock();
		BOOST_LOG_TRIVIAL(error) << "Error " << url.sanitised() << " NtripUploadClient::writeResponse : " << err.message() << "\n";
		delayed_reconnect();
		return;
	}
	else
	{
		numberChunksSent++;
		outMessagesMtx.unlock();
	}
}


void NtripBroadcaster::sendMessages(
	bool useSsrOut)
{
	for (auto [label, outStream] : ntripUploadStreams)
		outStream->sendMessages(useSsrOut);    
}


void NtripBroadcaster::NtripUploadClient::traceMakeNetworkOverview(
	Trace&			   trace,
	NetworkDataUpload& netData)
{
	netData.printTraceNetworkStatistics(trace);
}

void casterTest()
{
	// This stops normal operation of the PEA and puts it into
	// caster testing mode and sets up the stream performance web service.


	// Existing streams from the configuration are not required. To remove them
	// the networking is started and shutdown as work for the network thread gets
	// queued when the stream is created.
	NtripSocket::startClients();
	for (auto& [id, s] : ntripRtcmMultimap)
	{
		NtripStream& downStream = *s;
		downStream.disconnect();
	}
	outStreamManager.stopBroadcast();

	ntripRtcmMultimap.clear();
	obsStreamMultimap.clear();
	outStreamManager.ntripUploadStreams.clear();

	NtripCasterService casterService;
	casterService.caster_stream_root = acsConfig.caster_stream_root;
	casterService.startPerformanceMonitoring();

	NtripSocket::io_service.stop();
	exit(0);
}
