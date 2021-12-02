
#include "forceModels.hpp"
#include "constants.hpp"
#include "acsConfig.hpp"



/** Calculate the net acceleration on a satellite.
 * Accelerations are calculated in the inertial frame, and are applied to orbital elements as:
 * x_1 = x_0 + v_0 * interval + acc * interval^2;
 * v_1 = v_0 + acc * interval;
 * The acceleration here may not be an instantaneous acceleration, but adjusted such that it is correct, on average, over the span of the interval
*/
Vector3d calculateAcceleration(
	Trace&		trace,			///< Trace to output to (similar to cout)
	Vector3d&	rSat,			///< Inertial position of satellite at beginning of interval (m)
	Vector3d&	vSat,			///< Inertial velocity of satellite at beginning of interval (m/s)
	double		interval)		///< Interval that this acceleration will be applied for
{
	Vector3d acc = Vector3d::Zero();
	
	
	trace << std::endl << "rSat : " << rSat.transpose();
	trace << std::endl << "vSat : " << rSat.transpose();
	
	
	//calculate acceleration components
	Vector3d centralForceAcc = Vector3d::Zero();
	if (acsConfig.forceModels.central_force_gravity)
	{
		centralForceAcc = -MU_GPS * rSat.normalized() / rSat.squaredNorm();
		
		trace << std::endl << "Calculated central force gravity acceleration of " << centralForceAcc.transpose();
	}
	
	
	Vector3d directSolarRadiationAcc = Vector3d::Zero();
	if (acsConfig.forceModels.direct_solar_radiation)
	{
		double someValue = acsConfig.forceModels.some_configuration_parameter;
		
		directSolarRadiationAcc *= someValue;
		
		trace << std::endl << "Calculated solar radiation acceleration of " << directSolarRadiationAcc.transpose();
	}
	
	
	
	acc += centralForceAcc;
	acc += directSolarRadiationAcc;
	
	trace << std::endl;
	
	return acc;
}
