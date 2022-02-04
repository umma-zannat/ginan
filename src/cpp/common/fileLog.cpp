
//#pragma GCC optimize ("O0")

#include "fileLog.hpp"

#include <chrono>
#include <ctime>

#define BOOST_BIND_GLOBAL_PLACEHOLDERS

#include <boost/log/sinks/sync_frontend.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/log/trivial.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

using boost::property_tree::ptree;
using boost::property_tree::write_json;
namespace sinks = boost::log::sinks;



string FileLog::path_log;

void FileLog::consume(
	boost::log::record_view																	const&	rec,
	sinks::basic_formatted_sink_backend<char, sinks::synchronized_feeding>::string_type		const&	log_string)
{
	string mess = log_string.c_str();
	boost::erase_all(mess, "\n");
	if (mess == "")
		return;

	int logLevel = 2;
	auto attrs = rec.attribute_values();
	auto sev = attrs[boost::log::trivial::severity].get();
	switch (sev)
	{
		case boost::log::trivial::trace:			logLevel = 5;			break;
		case boost::log::trivial::debug:			logLevel = 4;			break;
		case boost::log::trivial::info:				logLevel = 3;			break;
		case boost::log::trivial::warning:			logLevel = 2;			break;
		case boost::log::trivial::error:			logLevel = 1;			break;
		case boost::log::trivial::fatal:			logLevel = 0;			break;
	}

	std::ofstream logStream(FileLog::path_log, std::ofstream::app);

	if (!logStream)
		return;
	
	std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
	ptree root;
	root.put("label", 		"message");
	root.put("timestamp", 	std::put_time(std::localtime( &now ),"%F %X"));
	root.put("level", 		logLevel);
	root.put("str", 		mess);
	write_json(logStream, root, false);
}


void addFileLog()
{
	// Construct the sink
	using LogSink = sinks::synchronous_sink<FileLog>;

	boost::shared_ptr<LogSink> logSink = boost::make_shared<LogSink>();

	// Register the sink in the logging core
	boost::log::core::get()->add_sink(logSink);
}
