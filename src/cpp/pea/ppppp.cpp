
#include <string>
#include <tuple>
#include <list>
#include <map>

#include <eigenIncluder.hpp>

using std::string;
using std::tuple;
using std::list;
using std::map;

#include "observations.hpp"
#include "streamTrace.hpp"
#include "corrections.hpp"
#include "forceModels.hpp"
#include "navigation.hpp"
#include "acsConfig.hpp"
#include "constants.hpp"
#include "antenna.hpp"
#include "station.hpp"
#include "preceph.hpp"
#include "algebra.hpp"
#include "common.hpp"
#include "orbits.hpp"
#include "tides.hpp"
#include "lambda.h"
#include "trop.h"

void makeLCs(
	KFMeas&		combinedMeas,
	KFState&	kfState)
{
	auto& recOpts = acsConfig.getRecOpts("");
	
	// Replace individual measurements with linear combinations
	if (recOpts.ion.estimate)
	{
		return;
	}
				KFMeas& newMeas = combinedMeas;
	vector<int> indices;
	
// 	std::cout << combinedMeas.R;
	if (1)
	for (int i = 1; i < combinedMeas.obsKeys.size(); i++)
	{
		int j = i - 1;
		auto& obsKeyi = combinedMeas.obsKeys[i];
		auto& obsKeyj = combinedMeas.obsKeys[j];
		if (obsKeyi.num != F2)
		{
			continue;
		}
		if (obsKeyj.num != F1)
		{
			continue;
		}
		if (obsKeyi.Sat != obsKeyj.Sat)
		{
			continue;
		}
		
		Obs& obs = *(Obs*)combinedMeas.metaDataMaps[i]["obs_ptr"];
		
		double lami = obs.satNav_ptr->lamMap[F2];
		double lamj = obs.satNav_ptr->lamMap[F1];
		
		double fi = CLIGHT / lami;
		double fj = CLIGHT / lamj;
		
		double coeffi = SQR(fi) / (SQR(fi) - SQR(fj));
		double coeffj = SQR(fj) / (SQR(fj) - SQR(fi));
		
// 		printf("\nAAAAA %f %f\n", coeffi, coeffj);
// 		printf("\nBBBBB %f %f\n", coeffi*coeffi, coeffj*coeffj);
		
		newMeas.A.row(j)	= combinedMeas.A.row(i) * coeffi
							+ combinedMeas.A.row(j) * coeffj;
		
		newMeas.V.row(j)	= combinedMeas.V.row(i) * coeffi
							+ combinedMeas.V.row(j) * coeffj;
		Vector2d A = Vector2d(coeffi, coeffj);
		Matrix2d R;
		R << combinedMeas.R(i,i), combinedMeas.R(i,j), combinedMeas.R(j, i), combinedMeas.R(j, j);
		
		R << 1, 1, 1, 1;
		
		double P = A.transpose() * R * A;
// 		printf("\nCCCCCC %f\n ", P);
		newMeas.R.row(j)	= coeffi * combinedMeas.R.row(i)
							+ coeffj * combinedMeas.R.row(j);
		newMeas.R.col(j)	= coeffi * combinedMeas.R.col(i)
							+ coeffj * combinedMeas.R.col(j);	
							
		obsKeyj.num		= 12;
// 		obsKeyj.type	= "ION";
		indices.push_back(j);
		newMeas.V.row(i).setZero();
		newMeas.A.row(i).setZero();
	}
	
	if (0)
	for (int i = 1; i < combinedMeas.obsKeys.size(); i++)
	{
		int j = i - 1;
		auto& obsKeyi = combinedMeas.obsKeys[i];
		auto& obsKeyj = combinedMeas.obsKeys[j];
		
		if (obsKeyi.num != F2)
		{
			continue;
		}
		if (obsKeyj.num != F1)
		{
			continue;
		}
		if (obsKeyi.Sat != obsKeyj.Sat)
		{
			continue;
		}
		
		for (auto& [key, index] : kfState.kfIndexMap)
		{
			if	( key.type != +KF::IONO_STEC
				||combinedMeas.A(i, index) == 0
				||combinedMeas.A(j, index) == 0)
			{
				continue;
			}
				
			double ai = combinedMeas.A(i, index);
			double aj = combinedMeas.A(j, index);
			
			double bi = aj / ai;
			double bj = -1;
			double scale = 1;//-2.5465;
			bi *= scale;
			bj *= scale;
			newMeas.A.row(i)	= combinedMeas.A.row(i) * bi
								+ combinedMeas.A.row(j) * bj;
			
			newMeas.V.row(i)	= combinedMeas.V.row(i) * bi
								+ combinedMeas.V.row(j) * bj;
			
			newMeas.R.row(i)	= combinedMeas.R.row(i) * (bi)
								+ combinedMeas.R.row(j) * (bj);
			
			newMeas.R.col(i)	= combinedMeas.R.col(i) * (bi)
								+ combinedMeas.R.col(j) * (bj);	
		}
		
		obsKeyj.num		= 12;
		obsKeyj.type	+= "2";
		indices.push_back(j);
	}
	
// 	std::cout << newMeas.A << std::endl;
// 	std::cout << newMeas.R << std::endl;
	
	vector<ObsKey> newObsKeys;
	for (auto& index : indices)
	{
		newObsKeys.push_back(combinedMeas.obsKeys[index]);
	}
	combinedMeas.obsKeys = newObsKeys;
	combinedMeas.Y = (combinedMeas.Y(indices)			).eval();
	combinedMeas.V = (combinedMeas.V(indices)			).eval();
	combinedMeas.A = (combinedMeas.A(indices, all)		).eval();
	combinedMeas.R = (combinedMeas.R(indices, indices)	).eval();
// 	std::cout << combinedMeas.R;
}
	


void lambdacalcs(
	KFState& kfState)
{
	printf("\n");
	const int numsols = 2;
// 	if (0)
	{
		vector<int> indices;
		for (auto& [key, index] : kfState.kfIndexMap)
		{
			if (key.type == KF::AMBIGUITY)
			{
				indices.push_back(index);
			}
		}

		VectorXd	parameters = kfState.x(indices);
		MatrixXd	covariance = kfState.P(indices, indices);
		MatrixXd	zTransform = kfState.Z(indices, indices);

		MatrixXd solutions(indices.size(), numsols);

		MatrixXd sumResiduals(numsols, 1);
		int index = -1;

		newLambda(std::cout,
			indices.size(),
			numsols,
			parameters,
			covariance,
			zTransform,
			solutions.data(),
			sumResiduals.data(),
			0.01,
			&index);

		
		for (int row = 0; row < zTransform.rows(); row++)
		for (int col = 0; col < zTransform.cols(); col++)
		{
			int r = indices[row];
			int c = indices[col];
			
			kfState.Z(r,c) = zTransform(row,col);
		}
		
// 		std::cout		<< std::endl << std::endl
// 		<< parameters	<< std::endl << std::endl 
// 		<< sumResiduals << std::endl << std::endl 
// 		<< index		<< std::endl << std::endl 
// 		<< solutions	<< std::endl << std::endl 
// 		<< (solutions.col(0) - solutions.col(1))	<< std::endl << std::endl;
		
		

		printf("\n");
	}
}








#define Rz(t,X) do {				\
	double sint = sin(t);			\
	(X)[8]=1;						\
	(X)[2]=(X)[5]=(X)[6]=(X)[7]=0;	\
	(X)[0]=(X)[4]=cos(t);			\
	(X)[3]=+sint;					\
	(X)[1]=-sint;					\
} while (0)


void eci2ecefA(
	const GTime		tutc,	///< time in utc
	const double*	erpv,	///< erp values {xp,yp,ut1_utc,lod} (rad,rad,s,s/d)
	Matrix3d&		U,		///< eci to ecef transformation matrix (3 x 3)
	double*			gmst)	///< greenwich mean sidereal time (rad)
{
	const double ep2000[] = {2000, 1, 1, 12, 0, 0};

//	trace(4,"%s: tutc=%s\n", __FUNCTION__, tutc.to_string(0).c_str());

	/* terrestrial time */
	GTime	tutc_	= tutc;
	GTime	tgps	= utc2gpst(tutc_);
	double	t		= (tgps - epoch2time(ep2000) + 19 + 32.184);	
	
	double angle = 360 * t / 86400 * D2R;
	
	Rz(angle,	U.data());
}










void stationPseudo(
			Trace&				netTrace,			///< Trace to output to
			Station&			rec,				///< Receiver to perform calculations for
/*	const*/	KFState&			kfState,			///< Kalman filter object containing the network state parameters
			KFMeasEntryList&	kfMeasEntryList)	///< Pointer to append kf measurements to
{	
	if (rec.pseudoObsList.empty())
	{
		return;
	}

	GTime time = rec.pseudoObsList.front().time;
	
	double erpv[5] = {};
	geterp(&nav.erp, time, erpv);
	
	Matrix3d i2tMatrix = Matrix3d::Identity();
	eci2ecef(time, erpv, i2tMatrix);
	
	
	for (auto& obs : rec.pseudoObsList)
	{
		auto& satOpts = acsConfig.getSatOpts(obs.Sat);		
		
		if (satOpts.pos.estimate)
		{
			Vector3d omegaVector	= Vector3d(0, 0, OMGE);
			Vector3d rSatInertial	= i2tMatrix.transpose() * obs.pos;
			Vector3d vSatInertial 	= i2tMatrix.transpose() * obs.vel 
									+ omegaVector.cross(rSatInertial);
									
			KFKey satPosKeys[3];
			KFKey satVelKeys[3];
			for (int i = 0; i < 3; i++)
			{
				satPosKeys[i].type	= KF::SAT_POS;
				satPosKeys[i].Sat	= obs.Sat;
				satPosKeys[i].num	= i;
				
				satVelKeys[i].type	= KF::SAT_POS_RATE;
				satVelKeys[i].Sat	= obs.Sat;
				satVelKeys[i].num	= i;
			}
			
			Vector3d statePosInertial = rSatInertial;
			Vector3d stateVelInertial = vSatInertial;
			
			for (int i = 0; i < 3; i++)
			{
				KFMeasEntry kfMeasEntry(&kfState);
				
				kfState.getKFValue(satPosKeys[i], statePosInertial[i]);
			
				if (satOpts.pos_rate.estimate)
				{				
					kfState.getKFValue(satVelKeys[i], stateVelInertial[i]);
				}
				
				InitialState init = initialStateFromConfig(satOpts.pos,			i);
				init.x = rSatInertial[i];
				
				kfMeasEntry.addDsgnEntry(satPosKeys[i], 1, init);
				
				if (satOpts.pos_rate.estimate)
				{
					InitialState init = initialStateFromConfig(satOpts.pos_rate,	i);
					
					init.x = vSatInertial[i];
					
					kfState.setKFTransRate(satPosKeys[i], satVelKeys[i],	1,	init);
				}
				
				double omc	= rSatInertial[i]
							- statePosInertial[i];
							
				kfMeasEntry.setInnov(omc);
				kfMeasEntry.setNoise(1);
					
				kfMeasEntry.obsKey.Sat	= obs.Sat;
				
				kfMeasEntryList.push_back(kfMeasEntry);
			}
		}
	}
}



void stationPPP(
			Trace&				netTrace,			///< Trace to output to
			Station&			rec,				///< Receiver to perform calculations for
/*	const*/	KFState&			kfState,			///< Kalman filter object containing the network state parameters
			gptgrid_t&			gptg,				///< [in/out]
// 			vmf3_t*				vmf3,				///< [in/out]
			double*				orography,			///< Pointer to orography maps
			KFMeasEntryList&	kfMeasEntryList)	///< List to append kf measurements to
{
	auto trace = getTraceFile(rec);
	
	const bool outputResidualChain = true;
	
	if (rec.obsList.empty())
	{
		return;
	}
	
	GTime time = rec.obsList.front().time;
	
	satposs(trace, time, rec.obsList, nav, E_Ephemeris::PRECISE, E_OffsetType::COM, false);
	
	double erpv[5] = {};
	geterp(&nav.erp, time, erpv);
	
	Matrix3d i2tMatrix = Matrix3d::Identity();
	eci2ecef(time, erpv, i2tMatrix);
	
	for (auto& obs			: rec.obsList)
	for (int measType		: {PHAS, CODE})
	for (auto& [ft, sig]	: obs.Sigs)
	{
		if (obs.exclude)
		{
			continue;
		}
		
		if (acsConfig.process_freq[ft] == false)
		{
			continue;
		}
		
		SatStat& satStat = *obs.satStat_ptr;
		SigStat& sigStat = satStat.sigStatMap[ft];
		
		auto& satOpts = acsConfig.getSatOpts(obs.Sat);
		auto& recOpts = acsConfig.getRecOpts(rec.id);
		
		list<tuple<string, double>> componentList;
		
		auto		Sat			= obs.Sat;
		auto		sys			= Sat.sys;
		int			biasGroup	= Sat.biasGroup();
		double		lambda		= obs.satNav_ptr->lamMap[ft];
		auto		code		= sig.code;
		
		PhaseCenterData* satPCD_ptr = findAntenna(Sat.id(), time, nav);
	
		double observed = 0;
		if		(measType == CODE)		observed = sig.P;
		else if	(measType == PHAS)		observed = sig.L * lambda;
		
		if (observed == 0)
		{
			continue;
		}
		
		KFMeasEntry measEntry(&kfState);
		
		measEntry.metaDataMap["obs_ptr"]	= &obs;
		
		if (measType == PHAS)
		{
			measEntry.metaDataMap["phaseRejectCount_ptr"] = &sigStat.netwPhaseRejectCount;
			measEntry.metaDataMap["phaseOutageCount_ptr"] = &sigStat.netwPhaseOutageCount;
		}
		
		
		measEntry.obsKey.Sat	= obs.Sat;
		measEntry.obsKey.str	= rec.id;
		measEntry.obsKey.num	= ft;
		if		(measType == CODE)		measEntry.obsKey.type = string("P ") + code._to_string();
		else if	(measType == PHAS)		measEntry.obsKey.type = string("L ") + code._to_string();
		
		//Start with the observed measurement
		
		componentList.push_back({"Observed", -observed});
		
		if (outputResidualChain)
		{
			tracepdeex(0, trace, "\n----------------------------------------------------");
			tracepdeex(0, trace, "\nMeasurement for %s", ((string) measEntry.obsKey).c_str());
		}
		
		//Calculate the basic range
		
		Vector3d vRec = Vector3d::Zero();
		{
			if (recOpts.pos_rate.estimate)
			for (int i = 0; i < 3; i++)
			{
				KFKey kfKey;
				kfKey.type	= KF::REC_POS_RATE;
				kfKey.str	= obs.mount;
				kfKey.num	= i;
				
				kfState.getKFValue(kfKey, vRec[i]);
				
				InitialState init = initialStateFromConfig(recOpts.pos_rate, i);
				
// 				measEntry.addDsgnEntry(kfKey, -satStat.e[i], init);
			}
		}
		
		Vector3d rRec = Vector3d::Zero();
		{
			bool sppInit = false;
			selectAprioriSource(rec, sppInit);

			rRec = rec.aprioriPos;
			
			if	(  recOpts.pos.estimate
				&& recOpts.keplers.estimate)
			{
				std::cout << "Incompatible estimation schemes" << std::endl;
			}
			
			if (recOpts.pos.estimate)
			for (int i = 0; i < 3; i++)
			{
				KFKey kfKey;
				kfKey.type	= KF::REC_POS;
				kfKey.str	= obs.mount;
				kfKey.num	= i;
				
				kfState.getKFValue(kfKey, rRec[i]);
				
				InitialState init = initialStateFromConfig(recOpts.pos, i);
				init.x = rRec[i];
				
				measEntry.addDsgnEntry(kfKey, -satStat.e[i], init);
			}
		}
		
		
			
		
		
		double pos[3];
		ecef2pos(rRec, pos);
		
		bool satNoise = true;
		Vector3d rSat = Vector3d::Zero();
		{
			//initialise using ephemeris values if available
			rSat					= obs.rSat;
			
			if (satOpts.pos.estimate)
			{
				Vector3d omegaVector	= Vector3d(0, 0, OMGE);
				Vector3d rSatInertial	= i2tMatrix.transpose() * rSat;
				Vector3d vSatInertial 	= i2tMatrix.transpose() * obs.satVel 
										+ omegaVector.cross(rSatInertial);
										
				Vector3d lookVectorInertial = i2tMatrix.transpose() * satStat.e;
				
				KFKey satPosKeys[3];
				KFKey satVelKeys[3];
				for (int i = 0; i < 3; i++)
				{
					satPosKeys[i].type	= KF::SAT_POS;
					satPosKeys[i].Sat	= Sat;
					satPosKeys[i].num	= i;
					
					satVelKeys[i].type	= KF::SAT_POS_RATE;
					satVelKeys[i].Sat	= Sat;
					satVelKeys[i].num	= i;
				}
				
				if (satOpts.pos.estimate)
				for (int i = 0; i < 3; i++)
				{
					bool found = kfState.getKFValue(satPosKeys[i], rSatInertial[i]);
					if (found) 
					{
						satNoise = false;
					}
				}
				
				
				if (satOpts.pos_rate.estimate)
				for (int i = 0; i < 3; i++)
				{				
					kfState.getKFValue(satVelKeys[i], vSatInertial[i]);
				}
				
				
				if (satOpts.pos.estimate)
				for (int i = 0; i < 3; i++)
				{
					InitialState init = initialStateFromConfig(satOpts.pos,			i);
					init.x = rSatInertial[i];
					
					measEntry.addDsgnEntry(satPosKeys[i], lookVectorInertial[i], init);
				}
				
				if (satOpts.pos_rate.estimate)
				for (int i = 0; i < 3; i++)
				{
					InitialState init = initialStateFromConfig(satOpts.pos_rate,	i);
					
					init.x = vSatInertial[i];
					
					kfState.setKFTransRate(satPosKeys[i], satVelKeys[i],	1,	init);
				}
				
				//refer actual time to estimated time
				//add offset from orbital motion				
				double approxRange	= sig.P;
				double flightTime	= approxRange / CLIGHT;
		
				rSatInertial -= flightTime * vSatInertial;
				
	// 			rSat = i2tMatrix * rSatInertial;
			}
		}
		
		//Range
// 		if (0)
		double rRecSat = (rSat - rRec).norm();
		{
			componentList.push_back({"Range", rRecSat});
		}
		
		//Add modelled adjustments and estimated parameter
		
		//Receiver Clock
		
		bool recClockNoise = true;
		double precDtRecVar	= 0;	//todo aaron, add this noise
// 		if (0)
		{
			double recClk_m = 0;
			
			double precDtRec	= 0;
			pephclk(obs.time, obs.mount, nav, precDtRec, &precDtRecVar);
			
			recClk_m		= precDtRec * CLIGHT;
			precDtRecVar	*= SQR(CLIGHT);
			
			if (recOpts.clk.estimate)
			{
				KFKey kfKey;
				kfKey.type			= KF::REC_CLOCK;
				kfKey.str			= obs.mount;
				kfKey.station_ptr	= &rec;
				// 			kfKey.num	= i;
				
				
				bool found = kfState.getKFValue(kfKey,	recClk_m);
				if (found) 
				{
					recClockNoise = false;
				}
				
				InitialState init		= initialStateFromConfig(recOpts.clk);
				init.x = recClk_m;
				init.P = 4;
				
				measEntry.addDsgnEntry(kfKey, 1, init);
			}
			
			componentList.push_back({"Rec Clock", recClk_m});
		}
		
		//Satellite Clock
// 		if (0)
		{
			double satClk_m = obs.dtSat[0] * CLIGHT;
			
			if (satOpts.clk.estimate)
			{
				KFKey kfKey;
				kfKey.type	= KF::SAT_CLOCK;
				kfKey.Sat	= Sat;
				kfState.getKFValue(kfKey,	satClk_m);
			
				InitialState init		= initialStateFromConfig(satOpts.clk);
				init.x = satClk_m;
				
				measEntry.addDsgnEntry(kfKey, -1, init);
			}
			componentList.push_back({"Sat Clock", -satClk_m});
		}
		
		//Receiver Antenna delta
// 		if (0)
		{
			Vector3d recAntVector;
			enu2ecef(pos, rec.rtk.opt.antdel.data(), recAntVector.data());
			
			{
				
			}
			double recAntDelta = -recAntVector.dot(satStat.e);
			componentList.push_back({"Rec Antenna Delta", recAntDelta});
		}
		
		//Satellite Antenna delta
 		if (0)
		{
			
		}
			
		//Receiver Phase Center Offset
// 		if (0)
		{
			Vector3d recPCOVector = Vector3d::Zero();
			
			/* receiver pco correction to the coordinates */
			if (rec.rtk.pcvrec)
			{
				Vector3d pco_enu;
				recpco(rec.rtk.pcvrec, ft, pco_enu);
				
				enu2ecef(pos, pco_enu.data(), recPCOVector.data());    /* convert enu to xyz */
			}
			
			double recPCODelta = -recPCOVector.dot(satStat.e);
			componentList.push_back({"Rec PCO", recPCODelta});
		}
		
		//Satellite Phase Center Offset
		// if (0)
		{
			PcoMapType* pcoMap_ptr = nullptr;
			{
				if (satPCD_ptr == nullptr)
				{
					tracepde(1, trace,	"Warning: no satellite (%s) pco information\n", Sat.id().c_str());
				}
				else
				{
					pcoMap_ptr = &satPCD_ptr->pcoMap;
				}
			}

			Vector3d satPCOVector = Vector3d::Zero();
			if (pcoMap_ptr != nullptr)
			{
				satantoff(trace, time, obs.rSat, ft, satPCOVector, pcoMap_ptr);
			}
			
			double satPCODelta = satPCOVector.dot(satStat.e);
			componentList.push_back({"Sat PCO", satPCODelta});
		}
		
		//Receiver Phase Center Variation
		if (acsConfig.rec_pcv)
		{
			double recPCVDelta = 0;
			
			// receiver pco correction to the coordinates 
			if (rec.rtk.pcvrec)
			{
				/* calculate pcv */
				double azDeg = satStat.az * R2D;
				double elDeg = satStat.el * R2D;
				recpcv(rec.rtk.pcvrec, ft, elDeg, azDeg, recPCVDelta);
			}
			
			componentList.push_back({"Rec PCV", recPCVDelta});
		}
		
		//Satellite Phase Center Variation
		if (acsConfig.sat_pcv)
		{
			//satellite and receiver antenna model
			map<int, double> dAntSat;
			{
				if (satPCD_ptr)
				{
					satantpcv(obs.rSat, rRec, *satPCD_ptr, dAntSat);
				}
				else
				{
					tracepde(1, trace,	"Warning: no satellite (%s) pcv information\n",	obs.Sat.id().c_str());
					continue;
				}
			}
			double satPCVDelta = dAntSat[ft];
			componentList.push_back({"Sat PCV", satPCVDelta});
		}
		
		//Receiver DCB transferral
		if (0)
		if (measType == CODE)
		{
			double recDCB[2] = {};
			componentList.push_back({"Rec DCB", recDCB[0]});
		}
		
		//Satellite DCB transferral
		if (0)
		if (measType == CODE)
		{
			double satDCB[2] = {};
			componentList.push_back({"Sat DCB", satDCB[0]});
		}
		
		//Tide delta
// 		if (0)
		{
			Vector3d tideVectorSum		= Vector3d::Zero();
			Vector3d tideVectorSolid	= Vector3d::Zero();
			Vector3d tideVectorOTL		= Vector3d::Zero();
			Vector3d tideVectorPole		= Vector3d::Zero();
			if	( acsConfig.tide_solid
				||acsConfig.tide_otl
				||acsConfig.tide_pole)
			{
				tidedisp(trace, gpst2utc(time), rRec, &nav.erp, &rec.rtk.opt.otlDisplacement[0][0], tideVectorSum, &tideVectorSolid, &tideVectorOTL, &tideVectorPole);
			}
			
			componentList.push_back({"Tides Solid",	-tideVectorSolid.dot(satStat.e)});
			componentList.push_back({"Tides OTL",	-tideVectorOTL	.dot(satStat.e)});
			componentList.push_back({"Tides Pole",	-tideVectorPole	.dot(satStat.e)});
		}
		
		//Relativity corrections
		//no rel in ppp code 1
		// if (0)
		{
			/* note that relativity effect to estimate sat clock */
			double dtRel1	= relativity1(rSat, obs.satVel);
			
			/* secondary relativity effect (Shapiro effect) */
			double ln		= log((rSat.norm() + rRec.norm() + rRecSat)
							/ (obs.rSat.norm() + rRec.norm() - rRecSat));
			double dtRel2 	= 2 * MU * ln / CLIGHT / CLIGHT / CLIGHT;
								
			componentList.push_back({"Relativity1", dtRel1	* CLIGHT});
			componentList.push_back({"Relativity2", dtRel2	* CLIGHT});
		}
		
		//Sagnac effect
// 		if (0)
		{
			double dSagnac = sagnac(rSat, rRec);
			
			componentList.push_back({"Sagnac", dSagnac});
		}
		
		//Ionospheric delay
// 		if (0)
		{
			double ionosphere_m = 0;
			//calculate the ionosphere values, variances, and gradients at the operating points
			if (recOpts.ion.estimate)
			{
				KFKey kfKey;
				kfKey.type	= KF::IONO_STEC;
				kfKey.str	= obs.mount;
				kfKey.Sat	= obs.Sat;
				if (acsConfig.ionoOpts.common_ionosphere == false)
				{
					//use separate ionospheres for code and phase
					kfKey.num	= measType;
				}
				
				double freq = CLIGHT / lambda;
				double alpha = 40.3e16 / SQR(freq);
				
				double ionosphere_stec  = 0;
				
				if (measType == CODE)		alpha *= +1;
				else						alpha *= -1;
				
				kfState.getKFValue(kfKey, ionosphere_stec);
				
				ionosphere_m = alpha * ionosphere_stec;
				
				InitialState init = initialStateFromConfig(recOpts.ion);
				init.Q = 100;
				
				measEntry.addDsgnEntry(kfKey, alpha, init);
			}
			
			componentList.push_back({"Ionosphere", ionosphere_m});
		}
		
		//Tropospheric delay
// 		if (0)
		double varTrop = 0;
		{
			double tropStates[3]	= {};
			double dTropDx[3]		= {};


			double troposphere_m = 0;
			//calculate the trop values, variances, and gradients at the operating points
			if (recOpts.trop.estimate)
			{
				KFKey kfKey;
				kfKey.type	= KF::TROP;
				kfKey.str	= obs.mount;
				
				//get the previous filter states for linearisation around this operating point
				for (short i = 0; i < 3; i++)
				{
					auto kfKeyX = kfKey;
					kfKeyX.num = i;
					
					bool pass = kfState.getKFValue(kfKeyX, tropStates[i]);
					if	( (i	== 0)
						&&(pass == false))
					{
						double ztd = sbstropcorr(time, rRec, PI/2);
						tropStates[i] = ztd;
					}
				}
				
				troposphere_m = trop_model_prec(obs.time, pos, satStat.azel, tropStates, dTropDx, varTrop);
				
				InitialState init = initialStateFromConfig(recOpts.trop);
				init.x = tropStates[0];
				init.P = varTrop;
				
				measEntry.addDsgnEntry(kfKey, dTropDx[0], init);
				
				if (recOpts.trop_grads.estimate)
				for (short i = 0; i < 2; i++)
				{
					auto kfKeyX = kfKey;
					kfKeyX.num = i+1;
					
					InitialState init = initialStateFromConfig(recOpts.trop_grads, i);
					
					measEntry.addDsgnEntry(kfKeyX, dTropDx[i+1], init);
				}
			}
			
			componentList.push_back({"Troposphere", troposphere_m});
			
			{
				// 		double tropGrads_m = 0;
				// 		componentList.push_back({"Troposphere Gradients", tropGrads_m});
			}
		}
		
		//Phase wind-up
		if (acsConfig.phase_windup)
		if (measType == PHAS)
		{
			bool pass = model_phw(time, obs, rRec, satStat.phw);
			if (pass == false)
			{
				continue;
			}
		
			double phaseWindup_m = satStat.phw * lambda;
			
			componentList.push_back({"Phase Wind-up", phaseWindup_m});
		}
		
		//Phase integer ambiguity
		if (measType == PHAS)
		{
			double ambiguity_m = 0;
			
			if (recOpts.amb.estimate)
			{
				KFKey kfKey;
				kfKey.type			= KF::AMBIGUITY;
				kfKey.str			= obs.mount;
				kfKey.Sat			= obs.Sat;
				kfKey.num			= ft;
				kfKey.station_ptr	= &rec;
				
				
				double ambiguity = 0;
				
				kfState.getKFValue(kfKey, ambiguity);
				
				ambiguity_m = ambiguity * lambda;
				
				InitialState init		= initialStateFromConfig(recOpts.amb);
				init.P /= SQR(lambda);
				
				measEntry.addDsgnEntry(kfKey, lambda, init);
			}
			
			componentList.push_back({"Phase Ambiguity", ambiguity_m});
		}
		
		//Receiver Code biases
		if (measType == CODE)
		{
			double recCodeBias = 0;
			
			if (ft != F1)
			if (recOpts.code_bias.estimate)
			{
				KFKey kfKey;
				kfKey.type	= KF::CODE_BIAS;
				kfKey.str	= obs.mount;
				kfKey.num	= ft;
				
				kfState.getKFValue(kfKey, recCodeBias);
				
				InitialState init		= initialStateFromConfig(recOpts.code_bias, ft);
				
				measEntry.addDsgnEntry(kfKey, 1, init);
			}
			
			componentList.push_back({"Rec Code Bias", recCodeBias});
		}
		
		//Satellite Code biases
		if (measType == CODE)
		{
			double satCodeBias = 0;
			
			if (ft != F1)
			if (satOpts.code_bias.estimate)
			{
				KFKey kfKey;
				kfKey.type	= KF::CODE_BIAS;
				kfKey.Sat	= Sat;
				kfKey.num	= ft;
				
				kfState.getKFValue(kfKey, satCodeBias);
				
				InitialState init		= initialStateFromConfig(satOpts.code_bias, ft);
				
				measEntry.addDsgnEntry(kfKey, 1, init);
			}
			
			componentList.push_back({"Sat Code Bias", satCodeBias});
		}
		
		//Receiver Phase biases
		if (measType == PHAS)
		{
			double recPhasBias = 0;
			
			if (recOpts.phas_bias.estimate)
			{
				KFKey kfKey;
				kfKey.type	= KF::PHASE_BIAS;
				kfKey.str	= obs.mount;
				kfKey.num	= ft;
				
				kfState.getKFValue(kfKey, recPhasBias);
				
				InitialState init		= initialStateFromConfig(recOpts.phas_bias, ft);
				
				measEntry.addDsgnEntry(kfKey, 1, init);
			}
			
			componentList.push_back({"Rec Phase Bias", recPhasBias});
		}
		
		//Satellite Phase biases
		if (measType == PHAS)
		{
			double satPhasBias = 0;
			
			if (satOpts.phas_bias.estimate)
			{
				KFKey kfKey;
				kfKey.type	= KF::PHASE_BIAS;
				kfKey.Sat	= Sat;
				kfKey.num	= ft;
				
				kfState.getKFValue(kfKey, satPhasBias);
				
				InitialState init		= initialStateFromConfig(satOpts.phas_bias, ft);
				
				measEntry.addDsgnEntry(kfKey, 1, init);
			}
			
			componentList.push_back({"Sat Phase Bias", satPhasBias});
		}
		
		
		//Calculate residuals and form up the measurement
		
		componentList.push_back({"Net Residual", 0});
		double residual = 0;
		for (auto& [componentName, componentVal] : componentList)
		{
			residual -= componentVal;
			
			if (outputResidualChain)
			{
				tracepdeex(0, trace, "\n%-20s %+14.4f -> %13.4f", componentName.c_str(), -componentVal, residual);
			}
		}
		measEntry.setInnov(residual);
		
	
		ObsKey obsKey;
		obsKey.str	= obs.mount;
		obsKey.Sat	= obs.Sat;
		
		//add ephemeris noise - common for all in obs
		if (satNoise)				{	obsKey.type = "satEph";		measEntry.addNoiseEntry(obsKey, 1, obs.ephVar);		}
		if (recClockNoise)			{	obsKey.type = "recClk";		measEntry.addNoiseEntry(obsKey, 1, precDtRecVar);	}
		
		//add signal noise - one per signal
		obsKey.type	= std::to_string(measType);
		obsKey.num	= ft;
// 		std::cout << obsKey << std::endl;
		if		(measType == CODE)	{								measEntry.addNoiseEntry(obsKey, 1, sig.codeVar);	}
		else if	(measType == PHAS)	{								measEntry.addNoiseEntry(obsKey, 1, sig.phasVar);	}
		
// 		tracepdeex(0, std::cout, "%14.6f %14.6f %14.6f %14.6f %14.6f\n", sig.codeVar, sig.phasVar, obs.ephVar, varTrop, 0);

		kfMeasEntryList.push_back(measEntry);
	}
}












/** Update the accelerations applied to orbital elements during the interval between epochs.
*/
void updateOrbits(
	Trace&			trace,
	KFState&		kfState)
{
	KFKey oneKey;
	oneKey.type = KF::ONE;
	
	for (auto& [kfKey, index] : kfState.kfIndexMap)
	{
		if	( kfKey.type	!= KF::SAT_POS_RATE
			||kfKey.num		!= 0)
		{
			continue;
		}
		
		Vector3d rSat = Vector3d::Zero();
		Vector3d vSat = Vector3d::Zero();
		
		KFKey posKey = kfKey;
		posKey.type = KF::SAT_POS;
		
	
		KFKey velKey = kfKey;
		
		for (int i = 0; i < 3; i++)
		{
			velKey.num = i;
			posKey.num = i;
			
			kfState.getKFValue(posKey, rSat[i]);
			kfState.getKFValue(velKey, vSat[i]);
		}
		
		double interval = tsync - kfState.time;
		
		Vector3d aSat = calculateAcceleration(std::cout, rSat, vSat, interval);
		
		//std::cout << std::endl << aSat.transpose() << std::endl;
		
		for (int i = 0; i < 3; i++)
		{
			velKey.num = i;
			posKey.num = i;
			
			kfState.setKFTransRate		(velKey, oneKey, aSat[i]);
			kfState.setKFTransRateRate	(posKey, oneKey, aSat[i]);
		}
	}
}


void updateRecClocks(
	Trace&			trace,			///< Trace to output to
	StationMap&		stations,		///< List of stations containing observations for this epoch
	KFState&		kfState)		///< Kalman filter object containing the network state parameters
{
	for (auto& [id, rec] : stations)
	{
		auto	trace	= getTraceFile(rec);
		auto&	recOpts	= acsConfig.getRecOpts(id);
		
		if (recOpts.clk.estimate == false)
		{
			continue;
		}
		
		KFKey clkKey;
		clkKey.type			= KF::REC_CLOCK;
		clkKey.str			= id;
		clkKey.station_ptr	= &rec;
		
		KFKey oneKey;
		oneKey.type	= KF::ONE;
		
				
		double C_dtRecAdj	= rec.rtk.sol.dtRec_m[0]
							- rec.rtk.sol.dtRec_m_pppp_old[0];
		
		rec.rtk.sol.dtRec_m_pppp_old[0] = rec.rtk.sol.dtRec_m[0];
		
		InitialState init		= initialStateFromConfig(recOpts.clk);
		
		kfState.setKFTrans(clkKey, oneKey, C_dtRecAdj, init);
	}
}

void propagateUncertainty(
	KFState&		kfState)
{
	MatrixXd F1;
	
	KFMeasEntryList kfMeasEntryList;
	
	KFKey pivotKey;
	
	if	(  acsConfig.pivot_station.empty()	== false
		&& acsConfig.pivot_station			!= "<AUTO>")
	{
		pivotKey.type	= KF::REC_CLOCK;
		pivotKey.str	= acsConfig.pivot_station;
	}
		
	if (0)
	for (auto& [key, index] : kfState.kfIndexMap)
	{
		if (key.type != KF::REC_CLOCK)
		{
			continue;
		}
		
		if (pivotKey.str.empty())
		{
			pivotKey = key;
		}
		
		KFMeasEntry kfMeasEntry;
		if (pivotKey.str != key.str)
		{
			kfMeasEntry.addDsgnEntry(key,		+1 / CLIGHT * 1000);
			kfMeasEntry.addDsgnEntry(pivotKey,	-1 / CLIGHT * 1000);
		}
		
		kfMeasEntry.obsKey.str = key.str + "-" + pivotKey.str;
		
		kfMeasEntryList.push_back(kfMeasEntry);
	}
	
	if (0)
	for (auto& [key, index] : kfState.kfIndexMap)
	{
		if (key.type != KF::SAT_CLOCK)
		{
			continue;
		}
		
		KFMeasEntry kfMeasEntry;
		kfMeasEntry.addDsgnEntry(key,		+1 / CLIGHT * 1000);
		kfMeasEntry.addDsgnEntry(pivotKey,	-1 / CLIGHT * 1000);
		
		kfMeasEntry.obsKey.str = " " + key.Sat.id() + "-" + pivotKey.str;
		
		kfMeasEntryList.push_back(kfMeasEntry);
	}
	
	for (int i = 1; i < kfState.kfIndexMap.size();	i++)
	for (int j = 0; j < i;							j++)
	{
		auto iti = kfState.kfIndexMap.begin();
		auto itj = kfState.kfIndexMap.begin();
		
		std::advance(iti, i);
		std::advance(itj, j);
		
		auto& [keyi, indexi] = *iti;
		auto& [keyj, indexj] = *itj;
		
		if (keyi.type != KF::AMBIGUITY)
		{
			continue;
		}
		
		if (keyj.type != KF::AMBIGUITY)
		{
			continue;
		}
		
		if (keyi.Sat != keyj.Sat)
		{
			continue;
		}
		
		if (keyi.str != keyj.str)
		{
			continue;
		}
		
		{
			KFMeasEntry kfMeasEntry;
			kfMeasEntry.addDsgnEntry(keyi,	-60.0/600);
			kfMeasEntry.addDsgnEntry(keyj,	+77.0/600);
			
			kfMeasEntry.obsKey.str = keyi.str + " " + keyi.Sat.id() + "C";
			
			kfMeasEntryList.push_back(kfMeasEntry);
		}
		{
			KFMeasEntry kfMeasEntry;
			kfMeasEntry.addDsgnEntry(keyi,	+60);
			kfMeasEntry.addDsgnEntry(keyj,	-77);
			
			kfMeasEntry.obsKey.str = keyi.str + " " + keyi.Sat.id() + "D";
			
			kfMeasEntryList.push_back(kfMeasEntry);
		}
	}
	
	KFMeas combinedMeas = kfState.combineKFMeasList(kfMeasEntryList);

	KFState propagatedState;
	
	propagatedState.P = combinedMeas.A * kfState.P * combinedMeas.A.transpose();
	propagatedState.x = combinedMeas.A * kfState.x;
	
	propagatedState.kfIndexMap.clear();
	
	int i = 0;
	for (auto& obsKey : combinedMeas.obsKeys)
	{
		KFKey kfKey;
		kfKey.type	= KF::CALC;
		kfKey.str	= obsKey.str;
		
		propagatedState.kfIndexMap[kfKey] = i;
		i++;
	}

	propagatedState.outputStates(std::cout);
}

void removeBadAmbiguities(
	Trace&				trace,			
	KFState&			kfState); 		


void PPP(
	Trace&			trace,			///< Trace to output to
	StationMap&		stations,		///< List of stations containing observations for this epoch
	KFState&		kfState,		///< Kalman filter object containing the network state parameters
	gptgrid_t&		gptg,			///< [in/out]
// 	vmf3_t*			vmf3,			///< [in/out]
	double*			orography)		///< Pointer to orography maps
{
	removeBadAmbiguities(trace, kfState);
	
	//prepare receiver clocks using spp values to minimise pre-fit residuals
	updateRecClocks(trace, stations, kfState);
	
	updateOrbits(trace, kfState);

	
	//add process noise and dynamics to existing states as a prediction of current state
	kfState.stateTransition(trace, tsync);
	
// 	trace << std::endl << "Predicted states";
	
// 	kfState.outputStates(trace);
	
	//prepare a map of lists of measurements for use below
	map<string, KFMeasEntryList> stationKFEntryListMap;
	for (auto& [id, rec] : stations)
	{
		stationKFEntryListMap[rec.id] = KFMeasEntryList();
	}
	
	//calculate the measurements for each station
// #	ifdef ENABLE_PARALLELISATION
// #	ifndef ENABLE_UNIT_TESTS
// 			Eigen::setNbThreads(1);
// #		pragma omp parallel for
// #	endif
// #	endif
	for (int i = 0; i < stations.size(); i++)
	{
		auto rec_iterator = stations.begin();
		std::advance(rec_iterator, i);
		
		auto& [id, rec] = *rec_iterator;
		
		stationPPP(std::cout, rec, kfState, nav.gptg, nav.orography, stationKFEntryListMap[rec.id]);
		stationPseudo(std::cout, rec, kfState, stationKFEntryListMap[rec.id]);
	}

	//combine all lists of measurements into a single list
	KFMeasEntryList kfMeasEntryList;
	for (auto& [rec, stationKFEntryList]	: stationKFEntryListMap)
	for (auto& kfMeasEntry					: stationKFEntryList)
	{
		kfMeasEntryList.push_back(std::move(kfMeasEntry));
	}
	
	//use state transition to initialise new state elements
	kfState.stateTransition(trace, tsync);
	
	KFMeas combinedMeas = kfState.combineKFMeasList(kfMeasEntryList, tsync);
	
	makeLCs(combinedMeas, kfState);
	
	if (kfState.lsqRequired)
	{
		kfState.lsqRequired = false;
		std::cout << std::endl << " -------INITIALISING PPPPP USING LEAST SQUARES--------" << std::endl;

		VectorXd dx;
 		kfState.leastSquareInitStatesA(trace, combinedMeas, false, &dx, true);
		
		trace << std::endl << "Least Squares Initialised States";
		kfState.outputStates(trace);
	}

	std::cout << std::endl << " -------DOING PPPPP KALMAN FILTER --------" << std::endl;
	
	kfState.filterKalman(trace, combinedMeas, true);
	
	postFilterChecks(combinedMeas);
	
	kfState.outputStates(trace);
	
// 	propagateUncertainty(kfState);
	
// 	lambdacalcs(kfState);
}





// check using same frame, check using ecef velocities



// 				inpt_hard_bias(trace, obs, sig.code, bias, bvar, {.OSB_biases = true, .REC_biases = true});

//todo aaron, this works, change ephemeris stuff to use this format





/* Lambda speed -
 * for N processors,
 * use log_2 (N) levels of lambda for one direction only (remove SGN)
 * report back to root process with worst best solution often.
 * 
 * 
 *
 * 
 * Do a test to see the difference between doing innovation ready or not on measurement noise coming out of OMC (using eg recpos state) vs more extended version
 * Just do it in demo.cpp
 */



// just make a function to copy every Nth variable then run N threads? to parallelise on that level?

