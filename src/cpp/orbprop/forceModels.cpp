
#include "forceModels.hpp"
#include "constants.hpp"
#include "acsConfig.hpp"

/** Calculate normalized Legendre polynomial values
 * 
 */
MatrixXd Legendre(
	int n,		///< Maximum degree
	int m,		///< Maximum order
	double phi) ///< Geocentric latitude in radian
{
	MatrixXd pnm = MatrixXd::Zero(n + 1, m + 1);
	double s = 0, h = 0;

	pnm(0, 0) = 1.0;
	pnm(1, 1) = sqrt(3.0) * cos(phi);

	// diagonal coefficients
	for (int i = 2; i <= n; i++)
	{
		s = i;
		pnm(i, i) = sqrt((2 * s + 1) / (2 * s)) * cos(phi) * pnm(i - 1, i - 1);
	}

	// horizontal first step coefficients
	for (int i = 1; i <= n; i++)
	{
		s = i;
		pnm(i, i - 1) = sqrt(2 * s + 1) * sin(phi) * pnm(i - 1, i - 1);
	}

	// horizontal second step coefficients
	int j = 0, k = 2;
	do
	{
		for (int i = k; i <= n; i++)
		{
			s = i;
			h = j;
			pnm(i, j) = sqrt((2 * s + 1) / ((s - h) * (s + h))) * (sqrt(2 * s - 1) * sin(phi) * pnm(i - 1, j) - sqrt(((s + h - 1) * (s - h - 1)) / (2 * s - 3)) * pnm(i - 2, j));
		}
		j++;
		k++;
	} while (j <= m);

	return pnm;
}
/** Calculate normalized Legendre polynomial first derivative values
 * 
 */
MatrixXd LegendreD(
	int n,		///< Maximum degree
	int m,		///< Maximum order
	double phi) ///< Geocentric latitude in radian
{
	MatrixXd pnm = MatrixXd::Zero(n + 1, m + 1);
	MatrixXd dpnm = MatrixXd::Zero(n + 1, m + 1);
	pnm(0, 0) = 1.0;
	dpnm(0, 0) = 0.0;
	pnm(1, 1) = sqrt(3) * cos(phi);
	dpnm(1, 1) = -sqrt(3) * sin(phi);

	// diagonal coefficients
	double s = 0, h = 0;
	for (int i = 2; i <= n; i++)
	{
		s = i;
		pnm(i, i) = sqrt((2 * s + 1) / (2 * s)) * cos(phi) * pnm(i - 1, i - 1);
		dpnm(i, i) = sqrt((2 * s + 1) / (2 * s)) * (cos(phi) * dpnm(i - 1, i - 1) - sin(phi) * pnm(i - 1, i - 1));
	}
	// horizontal first step coefficients
	for (int i = 1; i <= n; i++)
	{
		s = i;
		pnm(i, i - 1) = sqrt(2 * s + 1) * sin(phi) * pnm(i - 1, i - 1);
		dpnm(i, i - 1) = sqrt(2 * s + 1) * ((cos(phi) * pnm(i - 1, i - 1)) + (sin(phi) * dpnm(i - 1, i - 1)));
	}
	// horizontal second step coefficients
	int j = 0, k = 2;

	do
	{
		for (int i = k; i <= n; i++)
		{
			s = i;
			h = j;
			pnm(i, j) = sqrt((2 * s + 1) / ((s - h) * (s + h))) * (sqrt(2 * s - 1) * sin(phi) * pnm(i - 1, j) - sqrt(((s + h - 1) * (s - h - 1)) / (2 * s - 3)) * pnm(i - 2, j));
			dpnm(i, j) = sqrt((2 * s + 1) / ((s - h) * (s + h))) * ((sqrt(2 * s - 1) * sin(phi) * dpnm(i - 1, j)) + sqrt(2 * s - 1) * cos(phi) * pnm(i - 1, j) - sqrt(((s + h - 1) * (s - h - 1)) / (2 * s - 3)) * dpnm(i - 2, j));
		}
		j++;
		k++;
	} while (j <= m);
	return dpnm;
}

/** Computes the acceleration due to the harmonic gravity field of the Earth central body
 * 
 */
Vector3d earthCentralBodyGravityAcc(
	Trace&    trace,     ///< Trace to output to (similar to cout)
	Vector3d& rSat,		 ///< Inertial position of satellite at beginning of interval (m)
	Vector3d& vSat,		 ///< Inertial velocity of satellite at beginning of interval (m/s)
	Matrix3d& mECI2ECEF, ///< Transformation matrix from ECI coordinate to ECEF
	MatrixXd& cnm,		 ///< Earth model coefficients
	MatrixXd& snm,		 ///< Earth model coefficients
	double    n_max,	 ///< Maximum Earth gravity degree
	double    m_max)     ///< Maximum Earth gravity order
{
	double dUdr = 0;
	double dUdlatgc = 0;
	double dUdlongc = 0;
	double q1 = 0, q2 = 0, q3 = 0;
	double b1, b2, b3, r2xy;
	int nd;
	double ax, ay, az;
	Vector3d acc_bf = Vector3d::Zero();

	// Body-fixed position
	Vector3d rSat_bf = mECI2ECEF * rSat;

	// Geocentric latitude of satellite at beginning of interval (m)
	double rSat_latgc = asin(rSat_bf(2) / rSat_bf.squaredNorm());

	// Geocentric longitude of satellite at beginning of interval (m)
	double rSat_longc = atan2(rSat_bf(1), rSat_bf(0));

	// Legendre matrix given order/degree
	MatrixXd pnm = Legendre(n_max, m_max, rSat_latgc);

	// Normalised Legendre matrix given order/degree
	MatrixXd dpnm = LegendreD(n_max, m_max, rSat_latgc);

	for (int n = 0; n <= n_max; n++)
	{
		nd = n;
		b1 = (-MU_GPS / pow(rSat_bf.squaredNorm(), 2.0)) * pow((RE_WGS84 / rSat_bf.squaredNorm()), nd) * (n + 1);
		b2 = (MU_GPS / rSat_bf.squaredNorm()) * pow((RE_WGS84 / rSat_bf.squaredNorm()), nd);
		b3 = (MU_GPS / rSat_bf.squaredNorm()) * pow((RE_WGS84 / rSat_bf.squaredNorm()), nd);

		for (int m = 0; m <= m_max; m++)
		{
			q1 = q1 + pnm(n, m) * (cnm(n, m) * cos(m * rSat_longc) + snm(n, m) * sin(m * rSat_longc));
			q2 = q2 + dpnm(n, m) * (cnm(n, m) * cos(m * rSat_longc) + snm(n, m) * sin(m * rSat_longc));
			q3 = q3 + m * pnm(n, m) * (snm(n, m) * cos(m * rSat_longc) - cnm(n, m) * sin(m * rSat_longc));
		}

		dUdr = dUdr + q1 * b1;
		dUdlatgc = dUdlatgc + q2 * b2;
		dUdlongc = dUdlongc + q3 * b3;

		q3 = 0;
		q2 = q3;
		q1 = q2;
	}

	// Body-fixed acceleration
	r2xy = pow(rSat_bf(0), 2.0) + pow(rSat_bf(1), 2.0);

	ax = (1.0 / rSat_bf.squaredNorm() * dUdr - rSat_bf(2) / (pow(rSat_bf.squaredNorm(), 2.0) * sqrt(r2xy)) * dUdlatgc) * rSat_bf(0) - (1 / r2xy * dUdlongc) * rSat_bf(1);
	ay = (1.0 / rSat_bf.squaredNorm() * dUdr - rSat_bf(2) / (pow(rSat_bf.squaredNorm(), 2.0) * sqrt(r2xy)) * dUdlatgc) * rSat_bf(1) + (1 / r2xy * dUdlongc) * rSat_bf(0);
	az = 1.0 / rSat_bf.squaredNorm() * dUdr * rSat_bf(2) + sqrt(r2xy) / pow(rSat_bf.squaredNorm(), 2.0) * dUdlatgc;

	acc_bf(0) = ax;
	acc_bf(1) = ay;
	acc_bf(2) = az;

	// Inertial acceleration
	return mECI2ECEF.transpose() * acc_bf;
}

/** Calculate propagated values of orbital components
*/
void propagateOrbit(
	Trace&				trace,			///< Trace to output to (similar to cout)
	map<KFKey, int>&	keyMap,			///< Map of state keys to vector/matrix indices
	VectorXd&			states,			///< Orbital state elements at beginning of interval
	MatrixXd&			covariances,	///< Orbital covariances at beginning of interval
	Matrix3d&			mECI2ECEF,		///< Transformation matrix from ECI coordinate to ECEF
	double				interval)		///< Interval that this acceleration will be applied for
{
	MatrixXd			cnm;			///< Earth model coefficients
	MatrixXd			snm;			///< Earth model coefficients	
	double				n_max;			///< Maximum Earth gravity degree
	double				m_max;			///< Maximum Earth gravity order
	
	Vector3d rSat	= Vector3d::Zero();
	Vector3d vSat	= Vector3d::Zero();
	Vector3d aSat	= Vector3d::Zero();
	Vector3d lSat	= Vector3d::Zero();
	
	//retrieve named states from vector
	for (auto& [key, index] : keyMap)
	{
		if		(key.type == KF::SAT_POS)		{	rSat[key.num] = states[index];	}
		else if	(key.type == KF::SAT_POS_RATE)	{	vSat[key.num] = states[index];	}
		else if (key.type == KF::SRP)
		{
			//we dont have any of these yet. Will be empirical filtered parameters like SRP
		}
	}
	
	
	
	trace << std::endl << "rSat : " << rSat.transpose();
	trace << std::endl << "vSat : " << rSat.transpose();

	//calculate acceleration components
	Vector3d centralForceAcc = Vector3d::Zero();
	Vector3d rSat_bf = Vector3d::Zero(); ///< Body-fixed position of satellite at beginning of interval (m)
	
	Vector3d earthGrvAcc = Vector3d::Zero();
	if (acsConfig.forceModels.central_force_gravity)
	{
// 		earthGrvAcc = earthCentralBodyGravityAcc(trace, rSat, vSat, mECI2ECEF, cnm, snm, n_max, m_max);
		trace << std::endl
			  << "Calculated Earth central body gravity acceleration of " << earthGrvAcc.transpose();
	}

	Vector3d directSolarRadiationAcc = Vector3d::Zero();
	if (acsConfig.forceModels.direct_solar_radiation)
	{
		double someValue = acsConfig.forceModels.some_configuration_parameter;

		directSolarRadiationAcc *= someValue;

		trace << std::endl
			  << "Calculated solar radiation acceleration of " << directSolarRadiationAcc.transpose();
	}

	aSat += earthGrvAcc;
	aSat += directSolarRadiationAcc;
	
	//example only, calcluate the new rSat and vSat however is desired
	double n = 1000;
	for (int i = 0; i < n; i++)
	{
		aSat = - rSat.normalized() * MU_GPS / rSat.squaredNorm();
		
		rSat	= rSat
				+ vSat * interval/n
				+ aSat * interval/n*interval/n / 2;
				
		vSat	= vSat
				+ aSat * interval/n;
	}
	//output new states to vector
	for (auto& [key, index] : keyMap)
	{
		if		(key.type == KF::SAT_POS)			states[index] = rSat[key.num];
		else if	(key.type == KF::SAT_POS_RATE)		states[index] = vSat[key.num];
		else if (key.type == KF::SRP)	{}	//dont propagate other components
	}
	
	trace << std::endl;
}
