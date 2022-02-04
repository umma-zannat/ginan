

#include <functional>
#include <utility>
#include <list>

using std::list;
using std::pair;


#include "eigenIncluder.hpp"
#include "algebraTrace.hpp"
#include "streamTrace.hpp"
#include "instrument.hpp"
#include "acsConfig.hpp"
#include "algebra.hpp"
#include "common.hpp"
#include "mongo.hpp"

// #pragma GCC optimize ("O0")

/** \file
* This software uses specialised kalman filter classes to perform filtering.
* Using a classes such as KFState, KFMeas, etc, prevents duplication of code, and ensures that many edge cases are taken care of
* without the need for the developer to consider them explicitly.
*
* The basic workflow for using the filter is:
* create filter object,
* initialise the state transition matrix at the beginning of each epoch
* create a list of measurements (only adding entries for required states, using KFKeys to reference the state element)
* combining measurements that were in a list into a single matrix corresponding to the new state,
* and filtering - filtering internally saves the states for RTS code, using a single sequential file that has some headers added so that it can be traversed backwards
*
* The filter has some pre/post fit checks that remove measurements that are out of expected ranges,
* and there are functions provided for setting, resetting, and getting values, noises, and covariance values.
*
* KFKeys are used to identify states. They may have a type, SatStat value, string, and number associated with them, and can be set and read from filter objects as required.
*
* KFMeasEntrys are used for an individual measurement, before being combined into KFMeas objects that contain all of the measurements for a filter iteration.
*
* Internally, the data is stored in maps and Eigen matrices/vectors, but the accessors should be used rather than the vectors themselves to ensure that states have been initialised and are in the expected order.
*/


bool KFKey::operator ==(const KFKey& b) const
{
	if (str.compare(b.str)	!= 0)		return false;
	if (Sat					!= b.Sat)	return false;
	if (type				!= b.type)	return false;
	if (num					!= b.num)	return false;
	else								return true;
}

bool KFKey::operator <(const KFKey& b) const
{
	int strCompare = str.compare(b.str);

	if (strCompare < 0)		return true;
	if (strCompare > 0)		return false;
	
	if (type < b.type)		return true;
	if (type > b.type)		return false;

	if (Sat < b.Sat)		return true;
	if (Sat > b.Sat)		return false;

	if (num < b.num)		return true;
	else					return false;
}


/** Clears and initialises the state transition matrix to identity at the beginning of an epoch.
* Also clears any noise that was being added for the initialisation of a new state.
*/
void KFState::initFilterEpoch()
{
	ZAdditionMap.		clear();
	initNoiseMap.		clear();

	KFKey oneKey;
	oneKey.type = KF::ONE;
	
	for (auto& [key1, mapp]	: stateTransitionMap)
	{
		if (key1 == oneKey)
		{
			continue;
		}
		
		//remove initialisation elements for subsequent epochs
		mapp.erase(oneKey);
	}
	
	stateTransitionMap	[oneKey][oneKey]	= {1, 0};
}

/** Finds the position in the KF state vector of particular states.
*/
int KFState::getKFIndex(
	KFKey		key)		///< [in]	Key to search for in state
{
	auto index = kfIndexMap.find(key);
	if (index == kfIndexMap.end())
	{
		return -1;
	}
	return index->second;
}

/** Returns the value and variance of a state within the kalman filter object
*/
bool KFState::getKFValue(
	KFKey		key,			///< [in]	Key to search for in state
	double&		value,			///< [out]	Output value
	double*		variance)		///< [out]	Output variance
{
	auto a = kfIndexMap.find(key);
	if (a == kfIndexMap.end())
	{
//		std::cout << std::endl << "Warning: State not found in filter: " << key << std::endl;
		return false;
	}
	int index = a->second;
	if (index >= x.size())
	{
		return false;
	}
	value = x(index);

	if (variance)
	{
		*variance = P(index,index);
	}

	return true;
}

/** Returns the standard deviation of a state within the kalman filter object
*/
bool KFState::getKFSigma(
	KFKey		key,		///< [in]	Key to search for in state
	double&		sigma)		///< [out]	Output value
{
	auto a = kfIndexMap.find(key);
	if (a == kfIndexMap.end())
	{
		return false;
	}
	int index = a->second;
	if (index >= x.size())
	{
		return false;
	}

	sigma = sqrt(P(index,index));
	return true;
}

/** Sets the value of a state within the kalman filter object
*/
bool KFState::setKFValue(
	KFKey		key,		///< [in]	Key to search for in state
	double		value)		///< [in]	Input value
{
	auto a = kfIndexMap.find(key);
	if (a == kfIndexMap.end())
	{
		return false;
	}

	int index = a->second;
	if (index >= x.size())
	{
		return false;
	}

	x(index) = value;

	return true;
}

/** Sets the process noise of a state within the kalman filter object
*/
bool KFState::setKFNoise(
	KFKey		kfKey,		///< [in]	Key to search for in state
	double		value)		///< [in]	Input value
{
	procNoiseMap[kfKey] = value;

	return true;
}


/** Adds dynamics to a filter state by inserting off-diagonal, time dependent elements to transition matrix
*/
void KFState::setKFTransRate(
	KFKey			dest,			///< [in]	Key to search for in state to change in transition
	KFKey			source,			///< [in]	Key to search for in state as source
	double			value,			///< [in]	Input value
	InitialState	initialState)	///< [in]	Initial state for rate state.
{
	addKFState(source, initialState);

	stateTransitionMap[dest][source] = {value, 1};
}

/** Adds dynamics to a filter state by inserting off-diagonal, non-time dependent elements to transition matrix
*/
void KFState::setKFTrans(
	KFKey			dest,			///< [in]	Key to search for in state to change in transition
	KFKey			source,			///< [in]	Key to search for in state as source
	double			value,			///< [in]	Input value
	InitialState	initialState)	///< [in]	Initial state for rate state.
{
	addKFState(dest, initialState);

	auto& [oldValue, rate] = stateTransitionMap[dest][source];
	
	if (rate != 0)
	{
		std::cout << "ERROR: BAD TRANS SUM" << std::endl;
		return;
	}
	
	value += oldValue;
	
	stateTransitionMap[dest][source] = {value, 0};
}

/** Remove a state from a kalman filter object.
*/
void KFState::removeState(
	KFKey			kfKey)				///< [in]	Key to search for in state
{
	ZTransitionMap.			erase(kfKey);
	stateTransitionMap.		erase(kfKey);
	procNoiseMap.			erase(kfKey);
	gaussMarkovTauMap.		erase(kfKey);
	gaussMarkovMuMap.		erase(kfKey);
}

/** Tries to add a state to the filter object.
*  If it does not exist, it adds it to a list of states to be added.
*  Call consolidateKFState() to apply the list to the filter object
*/
void KFState::addKFState(
	KFKey			kfKey,			///< [in]	The key to add to the state
	InitialState	initialState)	///< [in]	The initial conditions to add to the state
{
	auto iter = stateTransitionMap.find(kfKey);
	if (iter != stateTransitionMap.end())
	{
		//is an existing state, just update values
		if (initialState.Q		!= 0)		{	procNoiseMap		[kfKey]	= initialState.Q;		}	
		if (initialState.mu		!= 0)		{	gaussMarkovMuMap	[kfKey]	= initialState.mu;		}	
		if (initialState.tau	!= 0)		{	gaussMarkovTauMap	[kfKey] = initialState.tau;		}

		return;
	}

	//this is a new state, add to the state transition matrix to create a new state.

	KFKey ONE = {KF::ONE};
	ZAdditionMap		[kfKey]			=  1;
	ZTransitionMap		[kfKey][kfKey]	=  1;
	stateTransitionMap	[kfKey][kfKey]	= {1,				0};
	stateTransitionMap	[kfKey][ONE]	= {initialState.x,	0};
	initNoiseMap		[kfKey]			=  initialState.P;		//todo aaron, check if these can be neglected for zero entries
	procNoiseMap		[kfKey]			=  initialState.Q;
	gaussMarkovTauMap	[kfKey]			=  initialState.tau;
	gaussMarkovMuMap	[kfKey]			=  initialState.mu;

	if (initialState.P == 0)
	{
		//will be an uninitialised variable, do a least squares solution
		lsqRequired = true;
	}
}


/** Tries to add a noise element.
*  If it does not exist, it adds it to a list of states to be added.
*  Call consolidateKFState() to apply the list to the filter object
*/
void KFState::addNoiseElement(
	ObsKey			obsKey,
	double			variance)
{
	noiseElementMap[obsKey]	= variance;
}

/** Add process noise and dynamics to filter object according to time gap.
 * This will also sort states according to their kfKey as a result of the way the state transition matrix is generated.
 */
void KFState::stateTransition(
	Trace&		trace,		///< [out]	Trace file for output
	GTime		newTime)	///< [in]	Time of update for process noise and dynamics (s)
{
	KFState& kfState = *this;
	
	double tgap = 0;
	if	( newTime	!= GTime::noTime()
		&&time		!= GTime::noTime())
	{
		tgap = newTime.time - time.time;
	}
	
	if	( newTime	!= GTime::noTime())
	{
		time = newTime;
	}
	
//	TestStack ts(__FUNCTION__);

	int newStateCount = stateTransitionMap.size();
	if (newStateCount == 0)
	{
		std::cout << "THIS IS WEIRD" << std::endl;
		return;
	}

	//if we're part way through a filter iteration, make sure we finish the old one before starting the new one, or it will crash rts.
	if	(rtsFilterInProgress)
	{
		//add some states to prvent bad things happening.
		spitFilterToFile(kfState, E_SerialObject::FILTER_MINUS,	kfState.rts_forward_filename);
		spitFilterToFile(kfState, E_SerialObject::FILTER_PLUS,	kfState.rts_forward_filename);
		rtsFilterInProgress = false;
	}

	//Initialise and populate a state transition and Z transition matrix
	SparseMatrix<double>	F_z		= SparseMatrix<double>	(newStateCount, x.rows());
	SparseMatrix<double>	F		= SparseMatrix<double>	(newStateCount, x.rows());
// 	MatrixXd			F		= MatrixXd::Zero(newStateCount, x.rows());
	VectorXd			Z_plus	= VectorXd::Zero(newStateCount);
	
	//add transitions for any states (usually close to identity)
	int row = 0;
	map<KFKey, short int> newKFIndexMap;
	for (auto& [newStateKey, newStateMap] : stateTransitionMap)
	{
		newKFIndexMap[newStateKey] = row;

		for (auto& [sourceStateKey, values] : newStateMap)
		{
			int sourceIndex	= getKFIndex(sourceStateKey);

			if	( (sourceIndex < 0)
				||(sourceIndex >= F.cols()))
			{
				continue;
			}
			auto& [value, tExp] = values;

			double tau = -1;
			
			auto gmIter = gaussMarkovTauMap.find(sourceStateKey);
			if (gmIter != gaussMarkovTauMap.end())
			{
				auto& [dummy, sourceTau] = *gmIter;
			
				tau = sourceTau;
			}

			double scalar = 1;			
			
			if (tau < 0)
			{
				//Random Walk model (special case for First Order Gauss Markov model when tau == inf)
			
				for (int i = 0; i < tExp; i++)
				{
					scalar *= tgap / (i+1);
				}
				
// 				F(row, sourceIndex) = value * scalar;
				F.insert(row, sourceIndex) = value * scalar;
				
				continue;
			}
			
			//First Order Gauss Markov model, Ref: Carpenter and Lee (2008) - A Stable Clock Error Model Using Coupled First- and Second-Order Gauss-Markov Processes - https://ntrs.nasa.gov/api/citations/20080044877/downloads/20080044877.pdf
		
			double tempTerm = 1;
			scalar = exp(-tgap/tau);
			
			for (int i = 0; i < tExp; i++)
			{
				scalar = tau * (tempTerm - scalar);	//recursive formula derived according to Ref: Carpenter and Lee (2008)
				tempTerm *= tgap / (i+1);
			}
			
			double transition = value * scalar;
			
// 			F(row, sourceIndex) = transition;
			F.insert(row, sourceIndex) = transition;
			
			
			//Add state transitions to ONE element, to allow for tiedown to average value mu
			//derived from integrating and distributing terms for v = (v0 - mu) * exp(-t/tau) + mu;
			//tempTerm calculated above appears to be same as required for these terms too, (at least for tExp = 0,1)
			
			auto muIter = gaussMarkovMuMap.find(sourceStateKey);
			if (muIter != gaussMarkovMuMap.end())
			{
				auto& [dummy2, mu] = *muIter;
				
// 				F(row, 0) = mu * (tempTerm - transition);
				F.insert(row, 0) = mu * (tempTerm - transition);
			}
		}
		
		row++;
	}
	
	for (auto& [kfKey1, map] : ZTransitionMap)
	for (auto& [kfKey2, value] : map)
	{
		int index2	= getKFIndex(kfKey2);

		if	( (index2 < 0)
			||(index2 >= F.cols()))
		{
			continue;
		}
		
		auto it = newKFIndexMap.find(kfKey1);	//todo aaron, this shold be ensured true elsewhere
		if (it == newKFIndexMap.end())
		{
			continue;
		}
		
		auto& [dummy, row] = *it;
		
// 		F_z(row, index2) = value;
		F_z.insert(row, index2) = value;
	}
	
	
	for (auto& [kfKey, value] : ZAdditionMap)
	{
		int row = newKFIndexMap[kfKey];
		
		Z_plus(row) = value;
	}

	//scale and add process noise
	MatrixXd Q0 = MatrixXd::Zero(newStateCount, newStateCount);
	tgap = fabs(tgap);

	//add noise as 'process noise' as the method of initialising a state's variance
	for (auto& [kfKey, value]	: initNoiseMap)
	{
		auto iter = newKFIndexMap.find(kfKey);
		if (iter == newKFIndexMap.end())
		{
			std::cout << kfKey << " broke" << std::endl;
			continue;
		}
		int index	= iter->second;

		if	( (index < 0)
			||(index >= Q0.rows()))
		{
			continue;
		}

		Q0(index, index) = value;
	}

	//add time dependent process noise
	if (tgap)
	for (auto& [dest,	map]	: stateTransitionMap)
	for (auto& [source,	vals]	: map)
	{
		auto& [val, tExp] = vals;
	
		auto initIter = initNoiseMap.find(dest);
		if (initIter != initNoiseMap.end())
		{
			//this was initialised this epoch
			double init = initIter->second;

			if (init == 0)
			{
				//this was initialised with no noise, do lsq, dont add process noise
				continue;
			}
		}

		auto destIter = newKFIndexMap.find(dest);
		if (destIter == newKFIndexMap.end())
		{
			std::cout << dest << " broke" << std::endl;
			continue;
		}
		
		int destIndex	= destIter->second;

		if	( (destIndex < 0)
			||(destIndex >= Q0.rows()))
		{
			continue;
		}

		auto sourceIter = newKFIndexMap.find(source);
		if (sourceIter == newKFIndexMap.end())
		{
			std::cout << dest << " broKe" << std::endl;
			continue;
		}
		
		int sourceIndex	= sourceIter->second;

		if	( (sourceIndex < 0)
			||(sourceIndex >= Q0.rows()))
		{
			continue;
		}

		auto iter2 = procNoiseMap.find(source);
		if (iter2 == procNoiseMap.end())
		{
// 			std::cout << dest << " brOke" << std::endl;
			continue;
		}
		
		auto [dummy, sourceProcessNoise] = *iter2;
		
		auto gmIter = gaussMarkovTauMap.find(source);
		if (gmIter != gaussMarkovTauMap.end())
		{
			auto& [dummy, tau] = *gmIter;

			if (tau < 0)
			{
				//Random Walk model (special case for First Order Gauss Markov model when tau == inf)
				
				if		(tExp == 0)	{	Q0(destIndex,	destIndex) += sourceProcessNoise / 1	* tgap;}
				else if	(tExp == 1)	{	Q0(destIndex,	destIndex) += sourceProcessNoise / 3	* tgap * tgap * tgap;	
		// 								Q0(sourceIndex, destIndex) += sourceProcessNoise / 2	* tgap * tgap;	
		// 								Q0(destIndex, sourceIndex) += sourceProcessNoise / 2	* tgap * tgap; 
									}
				else if (tExp == 2)	{	Q0(destIndex,	destIndex) += sourceProcessNoise / 20	* tgap * tgap * tgap * tgap * tgap;}
			}
			else
			{
				//First Order Gauss Markov model, Ref: Carpenter and Lee (2008) - A Stable Clock Error Model Using Coupled First- and Second-Order Gauss-Markov Processes - https://ntrs.nasa.gov/api/citations/20080044877/downloads/20080044877.pdf
			
				if		(tExp == 0)	{	Q0(destIndex,	destIndex) += sourceProcessNoise / 2	* tau * (1 - exp(-2*tgap/tau));		}
				else if	(tExp == 1)	{	Q0(destIndex,	destIndex) += sourceProcessNoise / 2	* tau * tau * (	+ 2 * tgap 								//one tau from front tau3 distributed to prevent divide by zero
																												- 4 * tau * (1 - exp(-1*tgap/tau)) 
																												+ 1 * tau * (1 - exp(-2*tgap/tau)));	//correct formula re-derived according to Ref: Carpenter and Lee (2008)
		// 								Q0(sourceIndex, destIndex) += sourceProcessNoise / 2	* tau * tau * (1-exp(-tgap/tau)) * (1-exp(-tgap/tau));
		// 								Q0(destIndex, sourceIndex) += sourceProcessNoise / 2	* tau * tau * (1-exp(-tgap/tau)) * (1-exp(-tgap/tau));
									}
				else if (tExp == 2)	{	std::cout << "FOGM model is not applied to acceleration term at the moment" << std::endl;	}	//todo Eugene: add process noise for acceleration term
			}
		}
		else
		{
			std::cout << "Tau value not found in filter: " << source << std::endl;
			continue;
		}
	}

	//output the state transition matrix to a trace file (used by RTS smoother)
	if (rts_filename.empty() == false)
	{
		TransitionMatrixObject transitionMatrixObject;
		transitionMatrixObject.rows = F.rows();
		transitionMatrixObject.cols = F.cols();

		for (auto& [kfKey1, newIndex] : newKFIndexMap)
		for (auto& [kfKey2, oldIndex] : kfIndexMap)
		{
// 			double transition	= F(newIndex, oldIndex);
// 			double transition	= F(newIndex, oldIndex);
// 
// 			//only output non zero elements to save space
// 			if	(transition	!= 0)
// 			{
// 				transitionMatrixObject.forwardTransitionMap[{newIndex, oldIndex}] = transition;
// 			}
		}
		spitFilterToFile(transitionMatrixObject,	E_SerialObject::TRANSITION_MATRIX,	rts_forward_filename);
		rtsFilterInProgress = true;
	}
	
	//compute the updated states and permutation and covariance matrices
	if (F.rows() == F.cols())
	{
		Instrument	instrument("PPPalgebra0");
		dx = (F * x) - x;
	}
	
	{
		Instrument	instrument("PPPalgebra1");
		Z = (F_z	* Z	* F_z.transpose()			).eval();
	}
	{
		Instrument	instrument("PPPalgebra2");
		x = (F		* x								).eval();
	}
	{
		Instrument	instrument("PPPalgebra3");
		P = (F		* P * F.transpose()		+ Q0	).eval();
	}
	{
		Instrument	instrument("PPPalgebra4");
		Z += Z_plus.asDiagonal();
	}
// 	std::cout << "F_z" << std::endl << F_z << std::endl;
// 	std::cout << "F" << std::endl << F << std::endl;
// 	std::cout << "Z" << std::endl << Z << std::endl;

	//replace the index map with the updated version that corresponds to the updated state
	kfIndexMap = std::move(newKFIndexMap);
	
	initFilterEpoch();
}

/** Compare variances of measurements and filtered states to detect unreasonable values
*/
int KFState::postFitSigmaCheck(
	Trace&		trace,      ///< Trace file to output to
	KFMeas&		kfMeas,		///< Measurements, noise, and design matrix
	VectorXd&	xp,         ///< The post-filter state vector to compare with measurements
	VectorXd&	dx,			///< The innovations from filtering to recalculate the deltas.
	int			iteration,	///< Number of iterations prior to this check
	int			begX,		///< Index of first state element to process
	int			numX,		///< Number of state elements to process
	int			begH,		///< Index of first measurement to process
	int			numH)		///< Number of measurements to process
{
	VectorXd	v = VectorXd::Zero(kfMeas.V.rows());
	
	auto		H = kfMeas.A.block(begH, begX, numH, numX);
	v.segment(begH, numH) = kfMeas.V.segment(begH, numH)	- H * dx.segment(begX, numX);

	//use 'array' for component-wise calculations
	auto		variations		= v.segment(begH, numH).array().square();	//delta squared
	auto		variances		= kfMeas.R.block(begH, begH, numH, numH).diagonal().array();
	auto		ratios			= variations / variances;
	auto		outsideExp		= ratios > SQR(4);
// 	double		meanVariations	= variations.mean();

	if (output_residuals)
	{
		trace << std::endl << "+ Residuals" << std::endl;
		tracepdeex(2, trace, "#\t%2s\t%19s\t%8s\t%3s\t%7s\t%13s\t%13s\t%16s\n", "It", "Time", "Str", "Sat", "Type", "Prefit Res", "Postfit Res", "Meas Variance");

		for (int i = begH; i < begH + numH; i++)
		{
			tracepdeex(2, trace, "%%\t%2d\t%19s\t%20s\t%13.4f\t%13.4f\t%16.9f\n", iteration, time.to_string(0).c_str(), ((string)kfMeas.obsKeys[i]).c_str(), kfMeas.V(i), v(i), kfMeas.R(i,i));
		}
		trace << "- Residuals" << std::endl;
	}

#	ifdef ENABLE_MONGODB
	if (acsConfig.output_mongo_measurements)
	{
		mongoMeasResiduals(kfMeas.obsKeys, kfMeas.V, v, kfMeas.R, begH, numH);
	}
#	endif

	trace << std::endl << "DOING SIGMACHECK: ";
// 	std::cout << std::endl << "meanVariations+: " << meanVariations << std::endl;

	//if any are outside the expected value, flag an error
	if (outsideExp.any())
	{
		Eigen::ArrayXd::Index index;

		trace << "LARGE ERROR OF " << ratios.maxCoeff(&index) << " AT " << index + begH << " : " << kfMeas.obsKeys[index + begH];

		return index + begH;
	}
	return -1;
}

/** Compare variances of measurements and pre-filtered states to detect unreasonable values
*/
int KFState::preFitSigmaCheck(
	Trace&		trace,		///< Trace to output to
	KFMeas&		kfMeas,		///< Measurements, noise, and design matrix
	int			begX,		///< Index of first state element to process
	int			numX,		///< Number of states elements to process
	int			begH,		///< Index of first measurement to process
	int			numH)		///< Number of measurements to process
{
	auto&	v = kfMeas.V;
	auto&	R = kfMeas.R;
	auto&	H = kfMeas.A;
	auto&	P = this->P;

	//use 'array' for component-wise calculations
	auto		variations	= v.segment(begH, numH).array().square();	//delta squared
	auto		variances	= (R.diagonal().segment(begH, numH) + (H.block(begH, begX, numH, numX) * P.block(begX, begX, numX, numX) * H.block(begH, begX, numH, numX).transpose()).diagonal()).array();

	auto		ratios		= variations / variances;
	auto		outsideExp	= ratios > SQR(4);

// 	double		meanRatios		= ratios.mean();
// 	double		meanVariations	= variations.mean();

	trace << std::endl << "DOING PRE SIGMA CHECK: ";
// 	std::cout << std::endl << "meanVariations-: " << meanVariations << std::endl;

	//if any are outside the expected value, flag an error
	if (outsideExp.any())
	{
		Eigen::ArrayXd::Index index;

		trace << "LARGE ERROR OF " << ratios.maxCoeff(&index) << " AT " << index + begH << " : " << kfMeas.obsKeys[index + begH];

		return index + begH;
	}

	return -1;
}

/** Kalman filter.
*/
int KFState::kFilter(
	Trace&			trace,		///< Trace to output to
	KFMeas&			kfMeas,		///< Measurements, noise, and design matrices
	VectorXd&		xp,   		///< Post-update state vector
	MatrixXd&		Pp,   		///< Post-update covariance of states
	VectorXd&		dx,			///< Post-update state innovation
	int				begX,		///< Index of first state element to process
	int				numX,		///< Number of state elements to process
	int				begH,		///< Index of first measurement to process
	int				numH)		///< Number of measurements to process
{
	auto& H = kfMeas.A;
	auto& R = kfMeas.R;
	auto& v = kfMeas.V;

	auto subH = H.block(begH, begX, numH, numX);
	
	MatrixXd HP	= subH	* P.block(begX, begX, numX, numX);
	MatrixXd Q	= HP	* subH.transpose();

	Q += R.block(begH, begH, numH, numH);
	
	MatrixXd K;

	bool repeat = true;
	while (repeat)
	{
		switch (inverter)
		{
			default:
			case E_Inverter::LDLT:
			{
				auto QQ = Q.triangularView<Eigen::Upper>().adjoint();
				LDLT<MatrixXd> solver;
				solver.compute(QQ);
				if (solver.info() != Eigen::ComputationInfo::Success)
				{
					xp = x;
					Pp = P;
					dx = VectorXd::Zero(xp.rows());

					return 1;
				}

				auto Kt = solver.solve(HP);
				if (solver.info() != Eigen::ComputationInfo::Success)
				{
					tracepdeex(1, trace, "Warning: kalman filter error2\n");
					xp = x;
					Pp = P;
					dx = VectorXd::Zero(xp.rows());

					return 1;
				}

				K = Kt.transpose();

				break;
			}
			case E_Inverter::LLT:
			{
				auto QQ = Q.triangularView<Eigen::Upper>().adjoint();
				LLT<MatrixXd> solver;
				solver.compute(QQ);
				if (solver.info() != Eigen::ComputationInfo::Success)
				{
					inverter = E_Inverter::LDLT;
					continue;
				}

				auto Kt = solver.solve(HP);
				if (solver.info() != Eigen::ComputationInfo::Success)
				{
					inverter = E_Inverter::LDLT;
					continue;
				}

				K = Kt.transpose();

				break;
			}
			case E_Inverter::INV:
			{
				MatrixXd Qinv = Q.inverse();
				K = P * H.transpose() * Qinv;

				break;
			}
		}
		repeat = false;
	}


	dx.segment(begX, numX)	= K * v.segment(begH, numH);
	xp.segment(begX, numX)	= x. segment(begX, numX)
							+ dx.segment(begX, numX);

// 	trace << std::endl << "h "	<< std::endl << subH;
// 	trace << std::endl << "hp "	<< std::endl << HP;
// 	trace << std::endl << "Q "	<< std::endl << Q;
// 	trace << std::endl << "K "	<< std::endl << K;
// 	trace << std::endl << "X "	<< std::endl << x. segment(begX, numX);
// 	trace << std::endl << "DX"	<< std::endl << dx.segment(begX, numX);
// 	trace << std::endl << "xp"	<< std::endl << xp.segment(begX, numX);
	
// 	if (acsConfig.joseph_stabilisation)
// 	{
// 		MatrixXd IKH = MatrixXd::Identity(P.rows(), P.cols()) - K * H;
// 		Pp = IKH * P * IKH.transpose() + K * R * K.transpose();
// 	}
// 	else
	{
		Pp.block(begX, begX, numX, numX) = P.block(begX, begX, numX, numX) - K * HP;
		
		Pp.block(begX, begX, numX, numX) = (	  Pp.block(begX, begX, numX, numX) 
												+ Pp.block(begX, begX, numX, numX).transpose()	).eval() / 2;
	}


	bool error = xp.segment(begX, numX).array().isNaN().any();
	if (error)
	{
		std::cout << std::endl << "xp:" << std::endl << xp << std::endl;
		std::cout << std::endl << "R :" << std::endl << R << std::endl;
		std::cout << std::endl << "v :" << std::endl << v << std::endl;
		std::cout << std::endl << "K :" << std::endl << K << std::endl;
		std::cout << std::endl << "P :" << std::endl << P << std::endl;
		std::cout << std::endl;
		std::cout << "NAN found. Exiting...";
		std::cout << std::endl;

		exit(0);
	}

	bool pass = true;
	return pass;
}

/** Perform chi squared quality control.
*/
bool KFState::chiQC(
	Trace&		trace,      ///< Trace to output to
	KFMeas&		kfMeas,		///< Measurements, noise, and design matrix
	VectorXd&	xp)         ///< Post filtered state vector
{
	auto& H = kfMeas.A;
	auto W = kfMeas.W(all, 0);

	VectorXd&	y		= kfMeas.Y;
	VectorXd	v		= y - H * xp;
	double		v_Wv	= v.transpose() * W.asDiagonal() * v;

	//trace << std::endl << "chiqcV" << v.rows() << std::endl;	tracematpde(5, trace, v, 15, 5);

	int obsNumber = v.rows() - x.rows() + 1;
	double val = v_Wv / (obsNumber);

	double thres;
	if (obsNumber <= 100) 	thres = chisqr_arr[obsNumber - 1] / (obsNumber);
	else					thres = 3;

	/* chi-square validation */
	if (val > thres)
	{
		tracepdeex(6, trace, " ChiSquare error detected");

// 		auto variations	= v.array().square();
// 		Eigen::MatrixXf::Index index;
// 		trace << " -> LARGEe ERROR OF " << sqrt(variations.maxCoeff(&index)) << " AT " << index;

		return false;
	}

	return true;
}

/** Combine a list of KFMeasEntrys into a single KFMeas object for used in the filter
*/
KFMeas KFState::combineKFMeasList(
	KFMeasEntryList&	kfEntryList,	///< List of input measurements as lists of entries
	GTime				measTime)		///< Time to use for measurements and hence state transitions
{
	int numMeas = kfEntryList.size();

	KFMeas kfMeas;

	kfMeas.time = measTime;
	
	kfMeas.V.resize(numMeas);
	kfMeas.Y.resize(numMeas);

	kfMeas.R = MatrixXd::Zero(numMeas, numMeas);
	kfMeas.A = MatrixXd::Zero(numMeas, x.rows());

	kfMeas.obsKeys		.resize(numMeas);
	kfMeas.metaDataMaps	.resize(numMeas);

	int meas = 0;
	for (auto& entry: kfEntryList)
	{
		kfMeas.R(meas, meas)	= entry.noise;
		kfMeas.Y(meas)			= entry.value;
		kfMeas.V(meas)			= entry.innov;

		for (auto& [kfKey, value] : entry.designEntryMap)
		{
			int index = getKFIndex(kfKey);
			if (index < 0)
			{
				std::cout << "hhuh?" << kfKey << std::endl;
				return KFMeas();
			}
			kfMeas.A(meas, index) = value;
		}

		kfMeas.obsKeys		[meas] = std::move(entry.obsKey);
		kfMeas.metaDataMaps	[meas] = std::move(entry.metaDataMap);
		meas++;
	}
	
	if (noiseElementMap.empty() == false)
	{
		VectorXd uncorrelatedNoise = VectorXd::Zero(noiseElementMap.size());
		
		map<ObsKey, int>	noiseIndexMap;
		int noises = 0;
		for (auto& [obsKey, variance] : noiseElementMap)
		{
			uncorrelatedNoise(noises) = variance;
			
			noiseIndexMap[obsKey] = noises;
			noises++;
		}
		
		SparseMatrix<double> R_A = SparseMatrix<double>(numMeas, noiseElementMap.size());
		
		meas = 0;
		for (auto& entry: kfEntryList)
		{
			for (auto& [obsKey, value] : entry.noiseEntryMap)
			{
				int noiseIndex = noiseIndexMap[obsKey];
				
				R_A.insert(meas, noiseIndex) = value;
			}
			
			meas++;
		}
		
		Instrument instrument("PPPnoisELment");
// 		std::cout << R_A << std::endl;
		kfMeas.R = R_A * uncorrelatedNoise.asDiagonal() * R_A.transpose();
		
// 		std::cout << std::setprecision(5);
// 		std::cout << "R" << std::endl << kfMeas.R << std::endl;
	}

	return kfMeas;
}

void KFState::doRejectCallbacks(
	Trace&		trace,				///< Trace file for output
	KFMeas&		kfMeas,				///< Measurements that were passed to the filter
	int			badIndex)			///< Index in measurement list that was unsatisfactory
{
	for (auto& callback : rejectCallbacks)
	{
		bool keepGoing = callback(trace, *this, kfMeas, badIndex);

		if (keepGoing == false)
		{
			return;
		}
	}
}

bool ObsKey::operator ==(const ObsKey& b) const
{
	if (str.compare(b.str)	!= 0)		return false;
	if (Sat					!= b.Sat)	return false;
	if (type.compare(b.type)!= 0)		return false;
	if (num					!= b.num)	return false;
	else								return true;
}

bool ObsKey::operator <(const ObsKey& b) const
{
	int typeCompare = type.compare(b.type);
	if (typeCompare < 0)		return true;
	if (typeCompare > 0)		return false;

	int strCompare = str.compare(b.str);
	if (strCompare < 0)		return true;
	if (strCompare > 0)		return false;

	if (Sat < b.Sat)		return true;
	if (Sat > b.Sat)		return false;

	if (num < b.num)		return true;
	if (num > b.num)		return false;

	else					return false;
}


/** Kalman filter operation
*/
int KFState::filterKalman(
	Trace&				trace,					///< Trace file for output
	KFMeas&				kfMeas,					///< Measurement object
	bool				innovReady,				///< Innovation already constructed
	list<FilterChunk>*	filterChunkList_ptr)	///< Optional ist of chunks for parallel processing of sub filters
{
	KFState& kfState = *this;

	if (kfMeas.time != GTime::noTime())
	{
		kfState.time = kfMeas.time;
	}

	if (kfState.rts_filename.empty() == false)
	{
		spitFilterToFile(kfState, E_SerialObject::FILTER_MINUS, kfState.rts_forward_filename);
	}

	if (kfMeas.A.rows() == 0)
	{
		//nothing to be done, clean up and return early

		if (kfState.rts_filename.empty() == false)
		{
			spitFilterToFile(kfState, E_SerialObject::FILTER_PLUS, kfState.rts_forward_filename);
			rtsFilterInProgress = false;
		}
		return 1;
	}

	/* kalman filter measurement update */
	if (innovReady == false)
	{
		kfMeas.V = kfMeas.Y - kfMeas.A * kfState.x;
	}

	list<FilterChunk> dummyFilterChunkList;
	if (filterChunkList_ptr == nullptr)
	{
		filterChunkList_ptr = &dummyFilterChunkList;
	}
	
	auto& filterChunkList = *filterChunkList_ptr;
	
	if (filterChunkList.empty())
	{
		FilterChunk filterChunk;
		filterChunk.trace_ptr = &trace;
		
		filterChunkList.push_back(filterChunk);
	}
	
	for (auto& filterChunk : filterChunkList)
	for (int i = 0; i < max_prefit_remv; i++)
	{
		auto& chunkTrace = *filterChunk.trace_ptr;
		
		if (filterChunk.numX < 0)	filterChunk.numX = x.rows();
		if (filterChunk.numH < 0)	filterChunk.numH = kfMeas.A.rows();
		
//		cout << "A" << endl;
//		cout << kfState.x.size() << endl;
//		cout << kfState.P.size() << endl;
//		cout << kfMeas.Y.size() << endl;
//		cout << kfMeas.V.size() << endl;
		int badIndex = kfState.preFitSigmaCheck(trace, kfMeas, filterChunk.begX, filterChunk.numX, filterChunk.begH, filterChunk.numH);
		if (badIndex < 0)	{	trace << std::endl << "PreSigma check passed" << std::endl;									break;		}
		else				{	trace << std::endl << "PreSigma check failed.";	doRejectCallbacks(trace, kfMeas, badIndex);	continue;	}
	}

	MatrixXd Pp = P;
	VectorXd xp = x;
			 dx = VectorXd::Zero(x.rows());
	
	for (auto& filterChunk : filterChunkList)
	for (int i = 0; i < max_filter_iter; i++)
	{
		auto& chunkTrace = *filterChunk.trace_ptr;
		
		if (filterChunk.numX < 0)	filterChunk.numX = x.rows();
		if (filterChunk.numH < 0)	filterChunk.numH = kfMeas.A.rows();
		
		bool pass = kfState.kFilter(chunkTrace, kfMeas, xp, Pp, dx, filterChunk.begX, filterChunk.numX, filterChunk.begH, filterChunk.numH);

		if (pass == false)
		{
			chunkTrace << "FILTER FAILED" << std::endl;
			return 0;
		}
		
		chunkTrace << "\nFrom " << filterChunk.begH << " for " << filterChunk.numH;
		chunkTrace << "\nStat " << filterChunk.begX << " for " << filterChunk.numX;
// 		outputStates(chunkTrace);

		int badIndex = kfState.postFitSigmaCheck(chunkTrace, kfMeas, xp, dx, i, filterChunk.begX, filterChunk.numX, filterChunk.begH, filterChunk.numH);	
		if (badIndex < 0)	{	chunkTrace << std::endl << "Sigma check passed" << std::endl;											break;		}
		else				{	chunkTrace << std::endl << "Sigma check failed.";	doRejectCallbacks(chunkTrace, kfMeas, badIndex);	continue;	}
	}

// 	if (pass)
	{
		kfState.x = std::move(xp);
		kfState.P = std::move(Pp);
	}

	if (kfState.rts_filename.empty() == false)
	{
		spitFilterToFile(kfState, E_SerialObject::FILTER_PLUS, kfState.rts_forward_filename);
		rtsFilterInProgress = false;
	}
	
	initFilterEpoch();
	return 1;
}

/** Perform least squares using the measurements in the KFMeas object
*/
int KFState::filterLeastSquares(
	Trace&			trace,		///< [in]		Trace to output to
	KFMeas&			kfMeas)		///< [in]		Measurements, noise, and design matrix
{
	KFState& kfState = *this;

	trace << std::endl << " -------STARTING LS --------" << std::endl;

	//invert measurement noise matrix to get a weight matrix
	kfMeas.W = (1 / kfMeas.R.diagonal().array()).matrix();		//todo, aaron straight inversion?

	if (kfMeas.R.rows() < kfState.x.rows())
	{
		trace << std::endl << "INSUFFICIENT MEASUREMENTS FOR LEAST SQUARES " << kfMeas.R.rows() <<  " " << kfState.x.rows();
		return 0;
	}

	auto& H = kfMeas.A;
	auto& W = kfMeas.W;
	auto& Y = kfMeas.Y;

	//calculate least squares solution
	MatrixXd H_W	= H.transpose() * W.asDiagonal();
	MatrixXd Q		= H_W * H;

	trace << std::endl << " -------DOING LS --------"<< std::endl;

	MatrixXd Qinv = Q.inverse();
	VectorXd x1 = Qinv * H_W * Y;

	bool error = x1.array().isNaN().any();
	if (error)
	{
		std::cout << "NAN found. Exiting...";
		std::cout	<< std::endl
		<< x1		<< std::endl
		<< Qinv		<< std::endl
		<< Q		<< std::endl;

		exit(0);
	}

// 	std::cout << std::endl << "postLSQ" << std::endl;

	chiQCPass = kfState.chiQC(trace, kfMeas, x1);
	if (chiQCPass == false)
	{
// 		return 0;	//todo aaron
	}

	//copy results to outputs
	for (int i = 0; i < kfState.x.rows(); i++)
	{
		if (kfState.P(i,i) == 0)
		{
			kfState.P(i,i)	= Qinv(i,i);
			kfState.x(i)	= x1(i);
		}
	}

	trace << std::endl << " -------STOPPING LS --------"<< std::endl;
	return 1;
}

//todo aaron, remove tgap, add time to filter

/** Least squares estimator for new kalman filter states.
* If new states have been added that do not contain variance values, the filter will assume that these states values and covariances should be
* estimated using least squares.
*
* This function will extract the minimum required states from the existing state vector,
* and the minimum required measurements in order to perform least squares for the uninitialised states.
*/
void KFState::leastSquareInitStates(
	Trace&			trace,				///< [in]		Trace file for output
	KFMeas&			kfMeas,				///< [in]		Measurement object
	bool			initCovars,			///< [in]		Option to also initialise off-diagonal covariance values
	VectorXd*		dx,					///< [out]		Optional output of state deltas
	bool			innovReady)			///< [in]		Perform conversion between V & Y		
{
	chiQCPass = false;

	if (innovReady)
	{
		kfMeas.Y = kfMeas.V;
	}
	
	vector<int> newStateIndicies;

	//find all the states that aren't initialised, they need least squaring.
	for (auto& [key, i] : kfIndexMap)
	{
		if	( (key.type != KF::ONE)
			&&(P(i,i) == 0))
		{
			//this is a new state and needs to be evaluated using least squares
			newStateIndicies.push_back(i);
		}
	}

	//get the subset of the measurement matrix that applies to the uninitialised states
	auto subsetA = kfMeas.A(all, newStateIndicies);

	//find the subset of measurements that are required for the initialisation
	auto usedMeas = subsetA.rowwise().any();

	map<int, bool> pseudoMeasStates;
	vector<int> leastSquareMeasIndicies;

	for (int meas = 0; meas < usedMeas.rows(); meas++)
	{
		//if not used, dont worry about it
		if (usedMeas(meas) == 0)
		{
			continue;
		}

		//this measurement is used to calculate a new state.
		//copy it to a new design matrix
		leastSquareMeasIndicies.push_back(meas);

		//remember make a pseudo measurement of anything it references that is already set
		for (int state = 0; state < kfMeas.A.cols(); state++)
		{
			if	( (kfMeas.A(meas, state)	!= 0)
				&&(P(state,state)			!= 0))
			{
				pseudoMeasStates[state] = true;
			}
		}
	}

	int newMeasCount = leastSquareMeasIndicies.size() + pseudoMeasStates.size();

	//Create new measurement objects with larger size, (using all states for now)
	KFMeas	leastSquareMeas;

	leastSquareMeas.Y = VectorXd::Zero(newMeasCount);
	leastSquareMeas.R = MatrixXd::Zero(newMeasCount, newMeasCount);
	leastSquareMeas.A = MatrixXd::Zero(newMeasCount, kfMeas.A.cols());

	int measCount = leastSquareMeasIndicies.size();

	//copy in the required measurements from the old set
	leastSquareMeas.Y.head		(measCount)		= kfMeas.Y(leastSquareMeasIndicies);
	leastSquareMeas.R.topLeftCorner	(measCount, measCount)	= kfMeas.R(leastSquareMeasIndicies, leastSquareMeasIndicies);
	leastSquareMeas.A.topRows	(measCount)		= kfMeas.A(leastSquareMeasIndicies, all);

	//append any new pseudo measurements to the end
	for (auto& [state, boool] : pseudoMeasStates)
	{
		leastSquareMeas.Y(measCount)				= x(state);
		leastSquareMeas.R(measCount, measCount)		= P(state, state);
		leastSquareMeas.A(measCount, state)			= 1;
		measCount++;
	}

	//find the subset of states required for these measurements
	vector<int> usedCols;
	auto usedStates = leastSquareMeas.A.colwise().any();
	for (int i = 0; i < usedStates.cols(); i++)
	{
		if (usedStates(i) != 0)
		{
			usedCols.push_back(i);
		}
	}

	//create a new meaurement object using only the required states.
	KFMeas	leastSquareMeasSubs;
	leastSquareMeasSubs.Y = leastSquareMeas.Y;
	leastSquareMeasSubs.R = leastSquareMeas.R;
	leastSquareMeasSubs.A = leastSquareMeas.A(all, usedCols);

	//invert measurement noise matrix to get a weight matrix
	leastSquareMeasSubs.W = (1 / leastSquareMeasSubs.R.diagonal().array()).matrix();//todo aaron straight inversion?

	VectorXd w = (1 / leastSquareMeasSubs.R.diagonal().array()).matrix().col(0);	//todo aaron straight inversion?

	for (int i = 0; i < w.rows(); i++)
	{
		if (std::isinf(w(i)))
		{
			w(i) = 0;
		}
	}

	if	( leastSquareMeasSubs.A.cols() == 0
		||leastSquareMeasSubs.A.rows() == 0)
	{
		trace << std::endl << "EMPTY DESIGN MATRIX DURING LEAST SQUARES";
		return;
	}

	if (leastSquareMeasSubs.R.rows() < leastSquareMeasSubs.A.cols())
	{
		trace << std::endl << "INSUFFICIENT MEASUREMENTS FOR LEAST SQUARES " << leastSquareMeasSubs.R.rows() <<  " " << x.rows();
		return;
	}
	auto& H = leastSquareMeasSubs.A;
	auto& Y = leastSquareMeasSubs.Y;

	//calculate least squares solution
	MatrixXd W		= w.asDiagonal();
	MatrixXd H_W	= H.transpose() * W;
	MatrixXd Q		= H_W * H;

	MatrixXd Qinv	= Q.inverse();
	VectorXd x1		= Qinv * H_W * Y;

// 	std::cout << "Q : " << std::endl << Q;
	bool error = x1.array().isNaN().any();
	if (error)
	{
		std::cout << std::endl << "x1:" << std::endl << x1 << std::endl;
		std::cout << std::endl << "w :" << std::endl << w << std::endl;
		std::cout << std::endl << "H :" << std::endl << H << std::endl;
		std::cout << std::endl << "P :" << std::endl << P << std::endl;
		std::cout << std::endl;
		std::cout << "NAN found. Exiting....";
		std::cout	<< std::endl;

		exit(-1);
	}

// 	std::cout << std::endl << "postLSQ" << std::endl;
	chiQCPass = chiQC(trace, leastSquareMeasSubs, x1);
	if (chiQCPass == false)
	{
// 		return 1;	//todo aaron
	}

	if (dx)
	{
		(*dx) = x1;
	}

	for (int i = 0; i < usedCols.size(); i++)
	{
		int stateRowIndex = usedCols[i];

		if (P(stateRowIndex, stateRowIndex) != 0)
		{
			continue;
		}

		double newStateVal = x1(i);
		double newStateCov = Qinv(i,i);

		if (dx)
		{
			x(stateRowIndex)				+=	newStateVal;
			P(stateRowIndex,stateRowIndex)	=	newStateCov;
		}
		else
		{
			x(stateRowIndex)				= newStateVal;
			P(stateRowIndex,stateRowIndex)	= newStateCov;
		}


		if (initCovars)
		{
			for (int j = 0; j < i; j++)
			{
				int stateColIndex = usedCols[j];

				newStateCov = Qinv(i,j);

				P(stateRowIndex,stateColIndex)	= newStateCov;
				P(stateColIndex,stateRowIndex)	= newStateCov;
			}
		}
	}
}
void KFState::leastSquareInitStatesA(
	Trace&			trace,				///< [in]		Trace file for output
	KFMeas&			kfMeas,				///< [in]		Measurement object
	bool			initCovars,			///< [in]		Option to also initialise off-diagonal covariance values
	VectorXd*		dx,					///< [out]		Optional output of state deltas
	bool			innovReady)			///< [in]		Perform conversion between V & Y		
{
	chiQCPass = false;

	if (innovReady)
	{
		kfMeas.Y = kfMeas.V;
	}
	
	vector<int> newStateIndicies;

	//find all the states that aren't initialised, they need least squaring.
	for (auto& [key, i] : kfIndexMap)
	{
		if	( (key.type != KF::ONE)
			&&(P(i,i) == 0))
		{
			//this is a new state and needs to be evaluated using least squares
			newStateIndicies.push_back(i);
		}
	}

	//get the subset of the measurement matrix that applies to the uninitialised states
	auto subsetA = kfMeas.A(all, newStateIndicies);

	//find the subset of measurements that are required for the initialisation
	auto usedMeas = subsetA.rowwise().any();

	map<int, bool> pseudoMeasStates;
	vector<int> leastSquareMeasIndicies;

	for (int meas = 0; meas < usedMeas.rows(); meas++)
	{
		//if not used, dont worry about it
		if (usedMeas(meas) == 0)
		{
			continue;
		}

		//this measurement is used to calculate a new state.
		//copy it to a new design matrix
		leastSquareMeasIndicies.push_back(meas);
	}

	//Create new measurement objects with larger size, (using all states for now)
	//copy in the required measurements from the old set
	KFMeas	leastSquareMeas;
	leastSquareMeas.Y	= kfMeas.Y(leastSquareMeasIndicies);
	leastSquareMeas.V	= kfMeas.V(leastSquareMeasIndicies);
	leastSquareMeas.R	= kfMeas.R(leastSquareMeasIndicies, leastSquareMeasIndicies);
	leastSquareMeas.A	= kfMeas.A(leastSquareMeasIndicies, all);

	//invert measurement noise matrix to get a weight matrix
	leastSquareMeas.W = (1 / leastSquareMeas.R.diagonal().array()).matrix();		//todo aaron straight inversion?

	VectorXd w = (1 / leastSquareMeas.R.diagonal().array()).matrix().col(0);	//todo aaron straight inversion?

	for (int i = 0; i < w.rows(); i++)
	{
		if (std::isinf(w(i)))
		{
			w(i) = 0;
		}
	}

	if (leastSquareMeas.R.rows() < leastSquareMeas.A.cols())
	{
		trace << std::endl << "INSUFFICIENT MEASUREMENTS FOR LEAST SQUARES " << leastSquareMeas.R.rows() <<  " " << x.rows();
		trace << std::endl << "Setting variances to large initial values ";;
		for (int i = 0; i < newStateIndicies.size(); i++)
		{
			int index = newStateIndicies[i];

			P(index, index) = SQR(10000);
		}
			
		return;
	}
	auto& H = leastSquareMeas.A;
	auto& Y = leastSquareMeas.Y;

	//calculate least squares solution
	MatrixXd W		= w.asDiagonal();
	MatrixXd H_W	= H.transpose() * W;
	MatrixXd Q		= H_W * H;

	MatrixXd Qinv	= Q.inverse();
	VectorXd x1		= Qinv * H_W * Y;

// 	std::cout << "Q : " << std::endl << Q;
	bool error = x1.array().isNaN().any();
	if (error)
	{
		std::cout << std::endl << "x1:" << std::endl << x1 << std::endl;
		std::cout << std::endl << "w :" << std::endl << w << std::endl;
		std::cout << std::endl << "H :" << std::endl << H << std::endl;
		std::cout << std::endl << "P :" << std::endl << P << std::endl;
		std::cout << std::endl;
		std::cout << "NAN found. Exiting....";
		std::cout	<< std::endl;

		exit(-1);
	}

// 	std::cout << std::endl << "postLSQ" << std::endl;
	chiQCPass = chiQC(trace, leastSquareMeas, x1);
	if (chiQCPass == false)
	{
// 		return 1;	//todo aaron
	}

	if (dx)
	{
		(*dx) = x1;
	}

	for (int i = 0; i < newStateIndicies.size(); i++)
	{
		int stateRowIndex = newStateIndicies[i];

		if (P(stateRowIndex, stateRowIndex) != 0)
		{
			continue;
		}

		double newStateVal = x1(i);
		double newStateCov = Qinv(i,i);

		if (dx)
		{
			x(stateRowIndex)				+=	newStateVal;
			P(stateRowIndex,stateRowIndex)	=	newStateCov;
		}
		else
		{
			x(stateRowIndex)				= newStateVal;
			P(stateRowIndex,stateRowIndex)	= newStateCov;
		}

		if (initCovars)
		{
			for (int j = 0; j < i; j++)
			{
				int stateColIndex = newStateIndicies[j];

				newStateCov = Qinv(i,j);

				P(stateRowIndex,stateColIndex)	= newStateCov;
				P(stateColIndex,stateRowIndex)	= newStateCov;
			}
		}
	}
}

/** Get a portion of the state vector by passing a list of keys
*/
VectorXd KFState::getSubState(
	map<KFKey, int>&	kfKeyMap,	///< List of keys to return within substate
	MatrixXd*			covarMat)	///< Optional pointer to a matrix for output of covariance submatrix
{
	vector<int> indices;
	indices.resize(kfKeyMap.size());

	for (auto& [kfKey, mapIndex] : kfKeyMap)
	{
		int stateIndex = getKFIndex(kfKey);
		if (stateIndex >= 0)
		{
			indices[mapIndex] = stateIndex;
		}
	}

	VectorXd subState = x(indices);
	if (covarMat)
	{
		*covarMat = P(indices, indices);
	}
	return subState;
}

/** Output keys and states in human readable format
*/
void KFState::outputStates(
		Trace&		trace,	///< Trace to output to
		int			begX,	///< Index of first state element to process
		int			numX)   ///< Number of state elements to process
{
	tracepdeex(2, trace, "\n\n");

	trace << std::endl << "+ States" << std::endl;
	
	tracepdeex(2, trace, "#\t%19s\t%20s\t%5s\t%3s\t%3s\t%13s\t%16s\t%10s\n", "Time", "Type", "Str", "Sat", "Num", "State", "Variance", "Adjust");

	int endX;
	if (numX < 0)	endX = x.rows();
	else			endX = begX + numX;
	
	for (auto& [key, index] : kfIndexMap)
	{
		if (index >= x.rows())
		{
			continue;
		}
		if	( index <  begX
			||index >= endX)
		{
			continue;
		}

		double _x	= x(index);
		double _dx = 0;
		if (index < dx.rows())
			_dx = dx(index);
		double _p	= P(index, index);
		string type	= KF::_from_integral(key.type)._to_string();

		tracepdeex(2, trace, "*\t%19s\t%20s\t%5s\t%3s\t%3d\t%13.4f\t%16.9f\t%10.3f\n", time.to_string(0).c_str(), type.c_str(), key.str.c_str(), key.Sat.id().c_str(), key.num, _x, _p, _dx);
	}
	trace << "- States" << std::endl;
}

void KFState::outputCorrelations(
			Trace&		trace)
{
	tracepdeex(2, trace, "\n\n");

	trace << std::endl << "+ Correlations" << std::endl;
	
	int skip = 0;
	for (auto& [key, index] : kfIndexMap)
	{
		if (key.type == KF::ONE)
		{
			continue;
		}
		
		tracepdeex(2, trace, "\n%28s", "");
		for (int i = 0; i < skip; i++)
		{
			tracepdeex(2, trace, "|    ");
		}
		
		trace << "> " << key;
		
		skip++;  
	}
	
	for (auto& [key, index] : kfIndexMap)
	{
		if (key.type == KF::ONE)
		{
			continue;
		}
		
		trace << std::endl << key << "  ";

		for (auto& [key2, index2] : kfIndexMap)
		{
			if (key2.type == KF::ONE)
			{
				continue;
			}
			
			double v1	= P(index,	index);
			double v2	= P(index2, index2);
			double v12	= P(index,	index2); 
			
			double correlation = v12 / sqrt(v1 * v2) * 100;
			
			if (index == index2)	tracepdeex(2, trace, "%4.0s ", "");
			else					tracepdeex(2, trace, "%4.0f ", correlation);
		
		}
	}
	trace << std::endl << "- Correlations" << std::endl;
}

InitialState initialStateFromConfig(
	KalmanModel&	kalmanModel,
	int				index)
{
	InitialState init = {};

	if (index < kalmanModel.apriori_val	.size())		init.x		= 		kalmanModel.apriori_val	[index];
	else												init.x		= 		kalmanModel.apriori_val	.back();
	if (index < kalmanModel.sigma		.size())		init.P		= SQR(	kalmanModel.sigma		[index]);
	else												init.P		= SQR(	kalmanModel.sigma		.back());
	if (index < kalmanModel.proc_noise	.size())		init.Q		= SQR(	kalmanModel.proc_noise	[index]);
	else												init.Q		= SQR(	kalmanModel.proc_noise	.back());
	if (index < kalmanModel.tau			.size())		init.tau	= 		kalmanModel.tau			[index];
	else												init.tau	= 		kalmanModel.tau			.back();
	if (index < kalmanModel.mu			.size())		init.mu		= 		kalmanModel.mu			[index];
	else												init.mu		= 		kalmanModel.mu			.back();

	return init;
}

KFState mergeFilters(
	list<KFState*>& kfStatePointerList,
	bool			includeTrop)
{
	map<KFKey, double>				stateValueMap;
	map<KFKey, map<KFKey, double>>	stateCovarMap;

	for (auto& statePointer : kfStatePointerList)
	{
		KFState& kfState = *statePointer;

		for (auto& [key1, index1] : kfState.kfIndexMap)
		{
			if	( key1.type != KF::REC_POS
				&&key1.type != KF::ONE
				&&key1.type != KF::TROP
				&&key1.type != KF::TROP_GM)
			{
				continue;
			}
			if	(  includeTrop == false
				&&(key1.type == KF::TROP
				|| key1.type == KF::TROP_GM))
			{
				continue;
			}

			stateValueMap[key1] = kfState.x(index1);

			for (auto& [key2, index2] : kfState.kfIndexMap)
			{
				if	( key2.type != KF::REC_POS
					&&key2.type != KF::TROP
					&&key2.type != KF::TROP_GM)
				{
					continue;
				}
				if	(  includeTrop == false
					&&(key2.type == KF::TROP
					|| key2.type == KF::TROP_GM))
				{
					continue;
				}

				if (kfState.P(index1, index2) != 0)
				{
					stateCovarMap[key1][key2] = kfState.P(index1, index2);
				}
			}
		}
	}

	KFState mergedKFState;

	mergedKFState.x		= VectorXd::Zero(stateValueMap.size());
	mergedKFState.dx	= VectorXd::Zero(stateValueMap.size());
	mergedKFState.P		= MatrixXd::Zero(stateValueMap.size(), stateValueMap.size());

	int i = 0;
	for (auto& [key, value] : stateValueMap)
	{
		mergedKFState.kfIndexMap[key]	= i;
		mergedKFState.x(i)				= value;

		i++;
	}

	for (auto& [key1, map2]		: stateCovarMap)
	for (auto& [key2, value]	: map2)
	{
		int index1 = mergedKFState.kfIndexMap[key1];
		int index2 = mergedKFState.kfIndexMap[key2];

		mergedKFState.P(index1, index2) = value;
	}

	return mergedKFState;
}

/** Calculates prefit residual, returning results as a KFMeasEntryList
* KFMeasEntryMap[ObsKey].value = prefit residual value
* KFMeasEntryMap[ObsKey].noise = prefit residual uncertainty
* KFMeasEntryMap[ObsKey].innov = kfMeas.V(i)
*/
KFMeasEntryList KFState::calcPrefitResids(
	Trace&			trace,				///< [out]	Trace file for output
	KFMeas&			kfMeas)				///< [in]	Measurement object
{
	KFState& kfState = *this;

	// Calculate prefit resid & uncertainty S
	Eigen::VectorXd PrefitResid = kfMeas.Y - kfMeas.A * kfState.x;
	Eigen::MatrixXd S = kfMeas.A * kfState.P * kfMeas.A.transpose() + Eigen::MatrixXd(kfMeas.R.asDiagonal());
	int numMeas = PrefitResid.size();
	//assert(kfMeas.R.size() == numMeas);
	assert(S.rows() == numMeas);
	assert(S.cols() == numMeas);
	assert(kfMeas.V.size() == numMeas);

	// Return result within a KFMeasEntryList
	KFMeasEntryList kfMeasEntryList;
	for (int meas=0; meas<numMeas; ++meas)
	{
		KFMeasEntry entry;
		entry.value = PrefitResid(meas);
		//entry.noise = kfMeas.R(meas);
		entry.noise = S(meas,meas);
		entry.innov = kfMeas.V(meas);
		entry.obsKey = kfMeas.obsKeys.at(meas);
		kfMeasEntryList.push_back(entry);
	}
	return kfMeasEntryList;
}
