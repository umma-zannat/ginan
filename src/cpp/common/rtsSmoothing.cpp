
//#pragma GCC optimize ("O0")

#include <map>

using std::map;

#include <boost/log/trivial.hpp>
#include "writeRinexNav.hpp"
#include "eigenIncluder.hpp"
#include "writeRinexObs.hpp"
#include "algebraTrace.hpp"
#include "rtsSmoothing.hpp"
#include "writeClock.hpp"
#include "acsConfig.hpp"
#include "constants.hpp"
#include "algebra.hpp"
#include "mongo.hpp"
#include "ppp.hpp"

bool isPositiveSemiDefinite(MatrixXd& mat)
{
	for (int i = 0; i < mat.rows(); i++)
	for (int j = 0; j < i; j++)
	{
		double a	= mat(i, i);
		double ab	= mat(i, j);
		double b	= mat(j, j);

		if (ab * ab > a * b)
		{
// 			std::cout << "large off diagonals " << std::endl;
// 			return false;
			if (ab > 0) ab = +sqrt(0.99 * a * b);
			else		ab = -sqrt(0.99 * a * b);
			mat(i, j) = ab;
			mat(j, i) = ab;
		}
	}
	return true;
}

void postRTSActions(
	bool		final,				///< This is a final answer, not intermediate - output to files
	KFState&	kfState,			///< State to get filter traces from
	string		clockFilename,		///< Filename for smoothed clock output
	string		tropFilename,		///< Filename for smoothed troposphere output
	StationMap*	stationMap_ptr)		///< Pointer to map of stations
{
	std::ofstream ofs(kfState.rts_filename, std::ofstream::out | std::ofstream::app);
	
	if	(   final
		&&  acsConfig.output_clocks
		&&( acsConfig.clocks_receiver_source	== +E_Ephemeris::KALMAN
		  ||acsConfig.clocks_satellite_source	== +E_Ephemeris::KALMAN))
	{
		tryPrepareFilterPointers(kfState, stationMap_ptr);

		auto filenameSysMap = getSysOutputFilenames(acsConfig.clocks_filename + SMOOTHED_SUFFIX, kfState.time);

		for (auto [filename, sysMap] : filenameSysMap)
		{
			outputClocks(filename, acsConfig.clocks_receiver_source, acsConfig.clocks_satellite_source, kfState.time, sysMap, kfState, stationMap_ptr);
		}
	}

	if	(   final
		&&  acsConfig.output_trop_sinex
		&&	acsConfig.trop_data_source == +E_Ephemeris::KALMAN)
	{
		outputTropSinex(tropFilename, kfState.time, *stationMap_ptr, kfState, "MIX", true);		//todo aaron, no site specific version here
	}

	if (final)
	{
		kfState.outputStates(ofs);
	}

#	ifdef ENABLE_MONGODB
	if	(   acsConfig.output_mongo_states
		&&( final
		  ||acsConfig.output_intermediate_rts))
	{
		mongoStates(kfState, acsConfig.mongo_rts_suffix);
	}
#	endif

// 	pppoutstat(ofs, archiveKF, true);
}

/** Output filter states from a reversed binary trace file
*/
void RTS_Output(
	KFState&	kfState,			///< State to get filter traces from
	StationMap*	stationMap_ptr,		///< Pointer to map of stations
	string		clockFilename,		///< Filename to output clocks to once smoothed
	string		tropFilename)		///< Filename for smoothed troposphere output
{
	string reversedStatesFilename = kfState.rts_filename + BACKWARD_SUFFIX;
	
	long int startPos = -1;
	while (1)
	{
		E_SerialObject type = getFilterTypeFromFile(startPos, reversedStatesFilename);

		switch (type)
		{
			default:
			{
				std::cout << "UNEXPECTED RTS OUTPUT TYPE";
				return;
			}

			case E_SerialObject::FILTER_PLUS:
			{
				KFState archiveKF;
				bool pass = getFilterObjectFromFile(type, archiveKF, startPos, reversedStatesFilename);
				
				
				if (pass == false)
				{
					std::cout << "BAD RTS OUTPUT READ";
					return;
				}
				
				archiveKF.rts_filename = kfState.rts_filename;
				postRTSActions(true, archiveKF, clockFilename, tropFilename, stationMap_ptr);
				
				break;
			}
		}

		if (startPos == 0)
		{
			return;
		}
	}
}

KFState RTS_Process(
	KFState&	kfState,
	bool		write,
	StationMap*	stationMap_ptr,
	string		clockFilename,
	string		tropFilename)
{
	if (kfState.rts_lag == 0)
	{
		return KFState();
	}
	
	MatrixXd transistionMatrix;

	KFState kalmanMinus;
	KFState smoothedKF;

	bool smoothedXready = false;
	bool smoothedPready = false;

	string inputFile	= kfState.rts_forward_filename;
	string outputFile	= kfState.rts_filename + BACKWARD_SUFFIX;

	if (write)
	{
		std::ofstream ofs(outputFile,	std::ofstream::out | std::ofstream::trunc);
	}

	long int startPos = -1;
	int lag = 0;
	while (lag != kfState.rts_lag)
	{
		E_SerialObject type = getFilterTypeFromFile(startPos, inputFile);

		if (type == +E_SerialObject::NONE)
		{
			break;
		}

		switch (type)
		{	
			default:
			{
				break;
			}
			case E_SerialObject::TRANSITION_MATRIX:
			{
				TransitionMatrixObject transistionMatrixObject;
				bool pass = getFilterObjectFromFile(type, transistionMatrixObject, startPos, inputFile);
				if (pass == false)
				{
					return KFState();
				}

//				std::cout << "Setting transition matrix " << transistionMatrixObject.rows << std::endl;

				transistionMatrix = MatrixXd::Zero(transistionMatrixObject.rows, transistionMatrixObject.cols);

				for (auto& [keyPair, value] : transistionMatrixObject.forwardTransitionMap)
				{
					transistionMatrix(keyPair.first, keyPair.second) = value;
				}

				break;
			}
			case E_SerialObject::FILTER_MINUS:
			{
				bool pass = getFilterObjectFromFile(type, kalmanMinus, startPos, inputFile);
				if (pass == false)
				{
					return KFState();
				}

				if (smoothedXready == false)
				{
					smoothedXready = true;
				}

				break;
			}
			case E_SerialObject::FILTER_PLUS:
			{
				lag++;

				if (write)
				{
					BOOST_LOG_TRIVIAL(info) 
					<< "Lag: " << lag;
				}

				KFState kalmanPlus;
				bool pass = getFilterObjectFromFile(type, kalmanPlus, startPos, inputFile);
				if (pass == false)
				{
					return KFState();
				}

				if (smoothedPready == false)
				{
					smoothedPready	= true;
					smoothedKF		= kalmanPlus;

					if (write)
					{
						spitFilterToFile(smoothedKF, E_SerialObject::FILTER_PLUS, outputFile);
					}

					break;
				}

				if (smoothedXready == false)
				{
					break;
				}

				smoothedKF.time = kalmanPlus.time;

				smoothedKF. P	= (smoothedKF.	P	+ smoothedKF.	P.transpose()).eval() / 2;
				kalmanMinus.P	= (kalmanMinus.	P	+ kalmanMinus.	P.transpose()).eval() / 2;
				kalmanPlus. P	= (kalmanPlus.	P	+ kalmanPlus.	P.transpose()).eval() / 2;

				//get process noise and dynamics
				auto& F = transistionMatrix;

				kalmanMinus.P(0,0) = 1;			//todo aaron, undo this later?

				MatrixXd Pinv = kalmanMinus.P.inverse();

				Pinv = (Pinv + Pinv.transpose()).eval()	/ 2;

				MatrixXd Ck = kalmanPlus.P * F.transpose() * Pinv;

				smoothedKF.x = ( kalmanPlus.x + Ck * (smoothedKF.x - kalmanMinus.x)						).eval();
				smoothedKF.P = ( kalmanPlus.P + Ck * (smoothedKF.P - kalmanMinus.P) * Ck.transpose()	).eval();

				smoothedKF.kfIndexMap = kalmanPlus.kfIndexMap;

				if (write)
				{
					spitFilterToFile(smoothedKF, E_SerialObject::FILTER_PLUS, outputFile);
				}
				else
				{
					bool final = false;
					if (lag == kfState.rts_lag)
					{
						final = true;
					}
					
					smoothedKF.rts_filename = kfState.rts_filename;
					postRTSActions(final, smoothedKF, clockFilename, tropFilename, stationMap_ptr);
				}
				
				break;
			}
		}

		if (startPos == 0)
		{
			break;
		}
	}
	
	if (write)
	{
		RTS_Output(kfState, stationMap_ptr, clockFilename, tropFilename);
	}

	if (lag == kfState.rts_lag)
	{
		//delete the beginning of the history file
		string tempFile	= kfState.rts_forward_filename + "_temp";
		{
			std::ofstream	tempStream(tempFile,	std::ifstream::binary | std::ofstream::out | std::ofstream::trunc);
			std::fstream	inputStream(inputFile,	std::ifstream::binary | std::ifstream::in);

			inputStream.seekg(0,	inputStream.end);
			long int lengthPos = inputStream.tellg();

			vector<char>	fileContents(lengthPos - startPos);

			inputStream.seekg(startPos,	inputStream.beg);

			inputStream.read(&fileContents[0], lengthPos - startPos);
			tempStream.write(&fileContents[0], lengthPos - startPos);
		}

		std::remove(inputFile.c_str());
		std::rename(tempFile.c_str(), inputFile.c_str());
	}
	
	if (kfState.rts_lag < 0)
	{
		BOOST_LOG_TRIVIAL(info) 
		<< "Removing RTS file: " << inputFile;
		
		std::remove(inputFile.c_str());
		
		string reversedStatesFilename = kfState.rts_filename + BACKWARD_SUFFIX;
		
		BOOST_LOG_TRIVIAL(info) 
		<< "Removing RTS file: " << reversedStatesFilename;
		
		std::remove(reversedStatesFilename.c_str());
	}

	if (lag == kfState.rts_lag)
	{
		return smoothedKF;
	}
	else
	{
		return KFState();
	}
}
