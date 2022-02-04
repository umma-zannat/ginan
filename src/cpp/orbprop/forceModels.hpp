
#ifndef __FORCE_MODELS_HPP__
#define __FORCE_MODELS_HPP__


#include <map>

using std::map;

#include "eigenIncluder.hpp"
#include "streamTrace.hpp"
#include "algebra.hpp"

// // Record for passing global data between Deriv and the calling program
// struct AuxParam {
//   double  Mjd0_TT;		    ///< First time epoch (TT) in modified Julian date
//   double  Area_drag;	    ///< Projection area for drag calculation
//   double  Area_solar;	    ///< Projection area for srp calculation
//   double  mass;			      ///< Mass of the satellite
//   double  CD;			        ///< Coefficient of drag
//   double  CR;			        ///< Coefficient of srp
//   int     n_a,m_a;		    ///< Order/degree of Earth central body gravity in calculating accelerations 
//   int     n_g,m_g;		    ///< Order/degree of Earth central body gravity in calculating partials 
//   bool    flagSun;		    ///< Flag of Sun third-body gravity 
//   bool    flagMoon;		    ///< Flag of Moon third-body gravity
//   bool    flagPlanets;	  ///< Flag of other planets third-body gravity
//   bool    flagSRad;		    ///< Flag of SRP acceleration
//   bool    flagDrag;		    ///< Flag of drag acceleration
//   bool    flagTides;	    ///< Flag of tides 
//   bool    flagRelativity; ///< Flag of relativity effect
//   };

MatrixXd Legendre(
	int n,		
	int m,		
	double phi);

void propagateOrbit(
	Trace&				trace,		
	map<KFKey, int>&	keyMap,		
	VectorXd&			states,		
	MatrixXd&			covariances,
	Matrix3d&			mECI2ECEF,	
	double				interval);	


Vector3d earthCentralBodyAcc(
	Trace &trace,		 ///< Trace to output to (similar to cout)
	Vector3d &rSat,		 ///< Inertial position of satellite at beginning of interval (m)
	Vector3d &vSat,		 ///< Inertial velocity of satellite at beginning of interval (m/s)
	Matrix3d &mECI2ECEF, ///< Transformation matrix from ECI coordinate to ECEF
	MatrixXd &cnm,		 ///< Earth model coefficients
	MatrixXd &snm,		 ///< Earth model coefficients
	double n_max,		 ///< Maximum Earth gravity degree
	double m_max);		 ///< Maximum Earth gravity order




#endif
