
//#pragma GCC optimize ("O0")

#ifdef ENABLE_MONGODB


#include "observations.hpp"
#include "acsConfig.hpp"
#include "satStat.hpp"
#include "common.hpp"
#include "mongo.hpp"

#include <boost/log/trivial.hpp>
#include <boost/log/sinks/sync_frontend.hpp>


namespace sinks = boost::log::sinks;

Mongo*	mongo_ptr = nullptr;

void mongoooo()
{
	if	( acsConfig.output_mongo_measurements	== false
		&&acsConfig.output_mongo_states			== false
		&&acsConfig.output_mongo_metadata		== false
		&&acsConfig.output_mongo_logs			== false)
	{
		return;
	}
		
	if (mongo_ptr)
		return;

	try
	{
		mongo_ptr = new Mongo(acsConfig.mongo_uri);
	}
	catch (...) {} // just eat any exception

	if (mongo_ptr == nullptr)
	{
		return;
	}

	Mongo& mongo = *mongo_ptr;

	auto c = mongo.pool.acquire();
	mongocxx::client&		client	= *c;
	mongocxx::database		db		= client[acsConfig.mongo_database];

	if (acsConfig.delete_mongo_history)
	{
		db["Measurements"]	.drop();
		db["States"]		.drop();
		db["Console"]		.drop();
	}

	db["Measurements"]	.create_index(
							document{}
								<< "Epoch"	<< 1
								<< "Site"	<< 1
								<< "Sat"	<< 1
								<< finalize,
							{});

	db["States"]		.create_index(
							document{}
								<< "Epoch"	<< 1
								<< "Site"	<< 1
								<< "Sat"	<< 1
								<< finalize,
							{});

	if (acsConfig.output_mongo_logs)
	{
		// Construct the sink
		using MongoLogSink = sinks::synchronous_sink<MongoLogSinkBackend>;

		boost::shared_ptr<MongoLogSink> mongoLogSink = boost::make_shared<MongoLogSink>();

		// Register the sink in the logging core
		boost::log::core::get()->add_sink(mongoLogSink);
	}
}

void MongoLogSinkBackend::consume(
	boost::log::record_view																	const&	rec,
	sinks::basic_formatted_sink_backend<char, sinks::synchronized_feeding>::string_type		const&	log_string)
{
	if (mongo_ptr == nullptr)
	{
		return;
	}

	Mongo& mongo = *mongo_ptr;

	auto 						c		= mongo.pool.acquire();
	mongocxx::client&			client	= *c;
	mongocxx::database			db		= client[acsConfig.mongo_database];
	mongocxx::collection		coll	= db	["Console"];

	//add a azimuth to a chunk
	coll.insert_one(
		document{}
			<< "Epoch"			<< bsoncxx::types::b_date {std::chrono::system_clock::from_time_t(tsync.time)}
			<< "Log"			<< log_string.c_str()
			<< finalize
		);
}

void mongoMeasSatStat_all(
	StationMap& stationMap)
{	
	if (mongo_ptr == nullptr)
	{
		return;
	}

	Mongo& mongo = *mongo_ptr;

	auto 						c		= mongo.pool.acquire();
	mongocxx::client&			client	= *c;
	mongocxx::database			db		= client[acsConfig.mongo_database];
	mongocxx::collection		coll	= db	["Measurements"];

	mongocxx::options::update	options;
	options.upsert(true);

	mongocxx::options::bulk_write bulk_opts;
	bulk_opts.ordered(false);

	auto bulk = coll.create_bulk_write(bulk_opts);

	bool update = false;

	for (auto& [id, rec] : stationMap)
	for (auto& obs : rec.obsList)
	{
		if (obs.exclude)
			continue;

		if (obs.satStat_ptr == nullptr)
			continue;

		SatStat& satStat = *obs.satStat_ptr;

		mongocxx::model::update_one  mongo_req{
			document{}
				<< "Epoch"			<< bsoncxx::types::b_date {std::chrono::system_clock::from_time_t(tsync.time)}
				<< "Site"			<< obs.mount
				<< "Sat"			<< obs.Sat.id()
				<< finalize,

			document()
				<< "$set"
				<< open_document
					<< "Azimuth"	<< satStat.az * R2D
					<< "Elevation"	<< satStat.el * R2D
				<< close_document
				<< finalize
			};

		mongo_req.upsert(true);
		bulk.append(mongo_req);
		update = true;
	}

	if (update)
		bulk.execute();
}


void mongoMeasSatStat(
	ObsList&			obsList)
{	
	if (mongo_ptr == nullptr)
	{
		return;
	}

	Mongo& mongo = *mongo_ptr;

	auto 						c		= mongo.pool.acquire();
	mongocxx::client&			client	= *c;
	mongocxx::database			db		= client[acsConfig.mongo_database];
	mongocxx::collection		coll	= db	["Measurements"];

	mongocxx::options::update	options;
	options.upsert(true);

	mongocxx::options::bulk_write bulk_opts;
	bulk_opts.ordered(false);

	auto bulk = coll.create_bulk_write(bulk_opts);

	bool update = false;

	for (auto& obs : obsList)
	{
		if (obs.exclude)
		{
			continue;
		}
		
		if (obs.satStat_ptr == nullptr)
			continue;	

		SatStat& satStat = *obs.satStat_ptr;
		
		mongocxx::model::update_one  mongo_req{
			document{}
				<< "Epoch"			<< bsoncxx::types::b_date {std::chrono::system_clock::from_time_t(tsync.time)}
				<< "Site"			<< obs.mount
				<< "Sat"			<< obs.Sat.id()
				<< finalize,

			document{}
				<< "$set"
				<< open_document
					<< "Azimuth"	<< satStat.az * R2D
					<< "Elevation"	<< satStat.el * R2D
				<< close_document
				<< finalize
		};

		update = true;
		mongo_req.upsert(true);
		bulk.append(mongo_req);
	}
	if (update)
		bulk.execute();

}

void mongoMeasResiduals(
	vector<ObsKey>		obsKeys,
	VectorXd&			prefits,
	VectorXd&			postfits,
	MatrixXd&			variance,
	int					beg,
	int					num)
{
	if (mongo_ptr == nullptr)
	{
		return;
	}

	Mongo& mongo = *mongo_ptr;

	auto 						c		= mongo.pool.acquire();
	mongocxx::client&			client	= *c;
	mongocxx::database			db		= client[acsConfig.mongo_database];
	mongocxx::collection		coll	= db	["Measurements"];

	mongocxx::options::update	options;
	options.upsert(true);

	mongocxx::options::bulk_write bulk_opts;
	bulk_opts.ordered(false);

	auto bulk = coll.create_bulk_write(bulk_opts);

	bool update = false;

	if (num < 0)
	{
		num = prefits.rows();
	}
	
	options.upsert(true);
	for (int i = beg; i < beg + num; i++)
	{
		ObsKey& obsKey = obsKeys[i];

		string name = obsKey.type + std::to_string(obsKey.num);

		mongocxx::model::update_one  mongo_req(
			document{}
				<< "Epoch"		<< bsoncxx::types::b_date {std::chrono::system_clock::from_time_t(tsync.time)}
				<< "Site"		<< obsKey.str	+ acsConfig.mongo_suffix
				<< "Sat"		<< obsKey.Sat.id()
				<< finalize,

			document{}
				<< "$set"
				<< open_document
					<< name + "-Prefit"		<< prefits(i)
					<< name + "-Postfit"	<< postfits(i)
					<< name + "-Variance"	<< variance(i,i)
				<< close_document
				<< finalize
			);

		mongo_req.upsert(true);
		bulk.append(mongo_req);
		update = true;
	}

	if (update)
	{
		bulk.execute();
	}
}

void mongoStates(
	KFState&			kfState,
	string				suffix)
{
	if (mongo_ptr == nullptr)
	{
		return;
	}

	Mongo& mongo = *mongo_ptr;

	auto 						c		= mongo.pool.acquire();
	mongocxx::client&			client	= *c;
	mongocxx::database			db		= client[acsConfig.mongo_database];
	mongocxx::collection		coll	= db	["States"];

	mongocxx::options::update	options;
	options.upsert(true);

	mongocxx::options::bulk_write bulk_opts;
	bulk_opts.ordered(false);

	auto bulk = coll.create_bulk_write(bulk_opts);

	bool update = false;

	for (auto& [key, index] : kfState.kfIndexMap)
	{
		if (key.type == KF::ONE)
		{
			continue;
		}

		mongocxx::model::update_one  mongo_req(
			document{}
				<< "Epoch"		<< bsoncxx::types::b_date {std::chrono::system_clock::from_time_t(kfState.time.time)}
				<< "Site"		<< key.str		+ acsConfig.mongo_suffix + suffix
				<< "Sat"		<< key.Sat.id()	+ acsConfig.mongo_suffix + suffix
				<< "State"		<< KF::_from_integral_unchecked(key.type)._to_string()
				<< finalize,

			document{}
				<< "$set"
				<< open_document
					<< "x" + std::to_string(key.num)		<< kfState.x(index)
					<< "P" + std::to_string(key.num)		<< kfState.P(index,index)
				<< close_document
				<< finalize
			);

		update = true;

		mongo_req.upsert(true);
		bulk.append(mongo_req);
	}

	if (update)
	{
		bulk.execute();
	}
}

#endif
