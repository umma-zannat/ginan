
#ifndef NTRIPSSRBROADCASTER_H
#define NTRIPSSRBROADCASTER_H



#include "acsNtripStream.hpp"
#include "rtcmEncoder.hpp"
#include "ntripSocket.hpp"
#include "ntripTrace.hpp"
#include "navigation.hpp"
#include "acsConfig.hpp"
#include "enums.h"



struct NtripBroadcaster
{
	struct NtripUploadClient : NtripSocket, RtcmEncoder::SSREncoder
	{
	private:

		// This mutex ensures that the main thread and the io_service thread do
		// not alter the outMessages buffer at the same time.
		std::mutex outMessagesMtx;
		boost::asio::streambuf outMessages;
		
		int numberChunksSent = 0;
	public:
		bool print_stream_statistics = false;
		
		string	ntripStr = "";
		
		// Message Type and Broadcast Interval in milliseconds.
		set<RtcmMessageType> rtcmMessagesTypes;
		
		NetworkDataUpload epochData;
		NetworkDataUpload hourlyData;
		NetworkDataUpload runData;

		NtripTrace	ntripTrace;
		string traceFilename;
		string		id = "NtripUploadClient";
		
		NtripUploadClient(
			const string& url_str) 
		: NtripSocket(url_str)
		{
			std::stringstream request_stream;
										request_stream	<< "POST " 		<< url.path << " HTTP/1.1\r\n";
										request_stream	<< "Host: " 	<< url.host << "\r\n";
										request_stream	<< "Ntrip-Version: Ntrip/2.0\r\n";
			if (!url.username.empty())
			{
										request_stream	<< "Authorization: Basic "
														<< Base64::encode(string(url.username + ":" + url.password))
														<< "\r\n";
			}
										request_stream	<< "User-Agent: NTRIP ACS/1.0\r\n";
			if (!ntripStr.empty())	    request_stream	<< "Ntrip-STR: " << ntripStr << "\r\n";
										request_stream	<< "Connection: close\r\n";
										request_stream  << "Transfer-Encoding: chunked\r\n";
										request_stream	<< "\r\n";
			request_string = request_stream.str();

			connect();
		};

		void connected() 
		override;

		void sendMessages(
			bool useSsrOut);
		
		void writeResponse(
			const boost::system::error_code& err);

		void connectionError(
			const boost::system::error_code& err,
			string 							 operation) 
		override;

		void serverResponse(
			unsigned int status_code,
			string		 http_version)
		override;

		void getJsonNetworkStatistics(
			GTime now);

		void traceSsrEph(
			SatSys Sat, 
			SSREph ssrEph)
		override
		{
			ntripTrace.traceSsrEph(Sat, ssrEph);
		}

		void traceSsrClk(
			SatSys Sat, 
			SSRClk ssrClk) 
		override
		{
			ntripTrace.traceSsrClk(Sat, ssrClk);
		}

		void traceSsrCodeB(
			SatSys 		Sat,
			E_ObsCode 	code, 
			SSRBias 	ssrBias)
		override
		{
			ntripTrace.traceSsrCodeB(Sat, code, ssrBias);
		}

		void traceSsrPhasB(
			SatSys 	 	Sat,
			E_ObsCode	mode, 
			SSRBias  	ssrBias)
		override
		{
			ntripTrace.traceSsrPhasB(Sat, mode, ssrBias);
		}

		void messageChunkLog(
			string message) 
		override 
		{
			ntripTrace.messageChunkLog(message);
		}

		void networkLog(
			string message)
		override
		{
			ntripTrace.networkLog(message);
		}

		void traceMakeNetworkOverview(
			Trace&				trace,
			NetworkDataUpload&	netData);

		void traceWriteEpoch(
			Trace& trace)
		{
			traceMakeNetworkOverview(trace, runData);
			ntripTrace.traceWriteEpoch(trace);
		}
	};

	void sendMessages(
		bool useSsrOut);

	void stopBroadcast();

	map<string, std::shared_ptr<NtripUploadClient>> ntripUploadStreams;
};

extern	NtripBroadcaster outStreamManager;

#endif
