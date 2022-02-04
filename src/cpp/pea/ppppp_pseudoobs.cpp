
// #pragma GCC optimize ("O0")


#include "eigenIncluder.hpp"
#include "streamTrace.hpp"
#include "forceModels.hpp"
#include "acsConfig.hpp"
#include "constants.hpp"
#include "station.hpp"
#include "algebra.hpp"
#include "common.hpp"
#include "gTime.hpp"




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
// 			Vector3d rSatInertial	= i2tMatrix.transpose() * obs.pos;
// 			Vector3d vSatInertial 	= i2tMatrix.transpose() * obs.vel 
// 									+ omegaVector.cross(rSatInertial);
									
			Vector3d rSatInertial	= obs.pos;
			Vector3d vSatInertial 	= obs.vel;
			
			vSatInertial[0] = 1683;
			vSatInertial[1] = 2055;
			vSatInertial[2] = -941;
			
			Vector3d lSatInertial = rSatInertial.cross(vSatInertial);
			
			KFKey satPosKeys[3];
			KFKey satVelKeys[3];
			KFKey satMomKeys[3];
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
				kfMeasEntry.setNoise(0.0001);
					
				kfMeasEntry.obsKey.Sat	= obs.Sat;
				
				kfMeasEntryList.push_back(kfMeasEntry);
			}
		}
	}
}

