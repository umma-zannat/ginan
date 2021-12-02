#ifndef __NAVIGATION_HPP__
#define __NAVIGATION_HPP__

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/serialization/binary_object.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/unordered_map.hpp>

#include <unordered_map>
#include <unordered_set>
#include <string>
#include <list>
#include <map>
#include <set>

using std::unordered_set;
using std::unordered_map;
using std::string;
using std::list;
using std::map;
using std::set;


#include "observations.hpp"
#include "streamTrace.hpp"
#include "ephemeris.hpp"
#include "antenna.hpp"
#include "orbits.hpp"
#include "satSys.hpp"
#include "gTime.hpp"
#include "enums.h"
#include "ssr.hpp"
#include "trop.h"
#include "vmf3.h"

#define MAXDTE      900.0           /* max time difference to ephem time (s) */

typedef map<GTime, Peph>	PephList;
typedef list<Pclk>			PclkList;

struct TECPoint
{
	double data = 0;		///< TEC grid data (tecu)
	double rms	= 0;		///< RMS values (tecu)
};

struct tec_t
{
	/// TEC grid type
	GTime				time;			///< epoch time (GPST) 
	int					ndata[3];		///< TEC grid data size {nlat,nlon,nhgt} 
	double				rb;				///< earth radius (km) 
	double				lats[3];		///< latitude start/interval (deg) 
	double				lons[3];		///< longitude start/interval (deg) 
	double				hgts[3];		///< heights start/interval (km) 
	vector<TECPoint>	tecPointVector;
};

struct erpd_t
{        
	/// earth rotation parameter data type 
	double mjd;		///< mjd (days) 
	double xp,yp;	///< pole offset (rad) 
	double xpr,ypr;	///< pole offset rate (rad/day) 
	double ut1_utc;	///< ut1-utc (s) 
	double lod;		///< length of day (s/day) 
};

struct erp_t
{        
	/// earth rotation parameter type
	int n,nmax;			///< number and max number of data
	erpd_t *data;		///< earth rotation parameter data 
};

struct SatNav
{
	map<int, double>	lamMap;
	double				cBias_P1_P2;				///< Satellite DCB - P1-P2 (m)
	map<int, double>	cBiasMap;					///< Satellite DCB - Px-Cx (m)
	double				wlbias;						///< wide-lane bias (cycle) */
	ssr_t				ssr;  						///< SSR corrections */
	SSROut      		ssrOut;						///< SSR corrections to export */
	SatOrbit			satOrbit	 	= {};
	
	Eph*				eph_ptr			= nullptr;
	Geph*				geph_ptr		= nullptr;
	Seph*				seph_ptr		= nullptr;
	PephList*			pephList_ptr	= nullptr;
	MatrixXd			satPartialMat;				///< Partial derivative matrices for orbits
};

struct nav_t
{
	///< navigation data type

	map<string, map<GTime, PhaseCenterData,	std::greater<GTime>>>	pcMap;
	
	map<int,	map<GTime, Eph,				std::greater<GTime>>> 	ephMap;        /* GPS/QZS/GAL ephemeris */
	map<int,	map<GTime, Geph,			std::greater<GTime>>>	gephMap;       /* GLONASS ephemeris */
	map<int,	map<GTime, Seph,			std::greater<GTime>>> 	sephMap;       /* SBAS ephemeris */
	map<int, PephList> 		pephMap;       /* precise ephemeris */
	map<string, PclkList> 	pclkMap;       /* precise clock */
	map<int, SatNav>		satNavMap;
	map<GTime, tec_t,						std::greater<GTime>>	tecMap;         /* tec grid data */
// 	list<fcbd_t> 		fcbList;        /* satellite fcb data */
	erp_t  	erp;         /* earth rotation parameters */
	double utc_gps[4];  /* GPS delta-UTC parameters {A0,A1,T,W} */
	double utc_glo[4];  /* GLONASS UTC GPS time parameters */
	double utc_gal[4];  /* Galileo UTC GPS time parameters */
	double utc_qzs[4];  /* QZS UTC GPS time parameters */
	double utc_cmp[4];  /* BeiDou UTC parameters */
	double utc_sbs[4];  /* SBAS UTC parameters */
	double ion_gps[8];  /* GPS iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3} */
	double ion_gal[4];  /* Galileo iono model parameters {ai0,ai1,ai2,0} */
	double ion_qzs[8];  /* QZSS iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3} */
	double ion_cmp[8];  /* BeiDou iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3} */
	int leaps;          /* leap seconds (s) */
	char glo_fcn[MAXPRNGLO+1];  /* glonass frequency channel number + 8 */
	double glo_cpbias[4];       /* glonass code-phase bias {1C,1P,2C,2P} (m) */

	
	list<string>					podInfoList;

	double		orography[NGRID]	= {};	 		/* vmf3 orography information, config->orography */
	gptgrid_t	gptg				= {};			/* gpt grid information */

	template<class ARCHIVE>
	void serialize(ARCHIVE& ar, const unsigned int& version)
	{
		ar & ephMap;
		ar & satNavMap;
	}
};


void updatenav(
	Obs&	obs);

namespace boost::serialization
{
	template<class ARCHIVE>
	void serialize(ARCHIVE& ar, nav_t& nav)
	{
		ar & nav.ephMap;
	}
}




extern	nav_t	nav;


#endif
