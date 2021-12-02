// //------------------------------------------------------------------------------
// //
// // SAT_Force.cpp
// //
// // Purpose:
// //
// //    Force model for Earth orbiting satellites
// //
// // Last modified:
// //
// //   2000/03/04  OMO  Final version (1st edition)
// //   2005/04/14  OMO  Final version (2nd reprint)
// //
// // (c) 1999-2005  O. Montenbruck, E. Gill
// //
// //------------------------------------------------------------------------------
// #include "SAT_Const.h"
// #include "SAT_Force.h"
// #include "SAT_Time.h"
// #include "SAT_RefSys.h"
// #include "SAT_VecMat.h"
// #include "eopspw.h"
// 
// using namespace std;
// 
// extern Matrix cnm, snm;
// extern spwdata spwarr;
// extern eopdata eoparr;
// extern int dat;
// extern double dut1, xp, yp;
// extern double f107a, f107, f107bar, ap, avgap, kp;
// 
// // Local funtions
// namespace
// {
// // Fractional part of a number (y=x-[x])
// double Frac (double x) { return x - floor(x); };
// }
// 
// //------------------------------------------------------------------------------
// //
// // Sun
// //
// // Purpose:
// //
// //   Computes the Sun's geocentric position using a low precision
// //   analytical series
// //
// // Input/Output:
// //
// //   Mjd_TT    Terrestrial Time (Modified Julian Date)
// //   <return>  Solar position vector [m] with respect to the
// //             mean equator and equinox of J2000 (EME2000, ICRF)
// //
// //------------------------------------------------------------------------------
// Vector Sun (double Mjd_TT)
// {
// 	// Constants
// 	const double eps = 23.43929111 * Rad;           // Obliquity of J2000 ecliptic
// 	const double T   = (Mjd_TT - MJD_J2000) / 36525.0; // Julian cent. since J2000
// 
// 	// Variables
// 	double L, M, r;
// 	Vector r_Sun(3);
// 
// 	// Mean anomaly, ecliptic longitude and radius
// 	M = pi2 * Frac ( 0.9931267 + 99.9973583 * T);                  // [rad]
// 	L = pi2 * Frac ( 0.7859444 + M / pi2 +
// 					 (6892.0 * sin(M) + 72.0 * sin(2.0 * M)) / 1296.0e3); // [rad]
// 	r = 149.619e9 - 2.499e9 * cos(M) - 0.021e9 * cos(2 * M);       // [m]
// 
// 	// Equatorial position vector
// 	r_Sun = R_x(-eps) * Vector(r * cos(L), r * sin(L), 0.0);
// 
// 	return r_Sun;
// }
// 
// //------------------------------------------------------------------------------
// //
// // Moon
// //
// // Purpose:
// //
// //   Computes the Moon's geocentric position using a low precision
// //   analytical series
// //
// // Input/Output:
// //
// //   Mjd_TT    Terrestrial Time (Modified Julian Date)
// //   <return>  Lunar position vector [m] with respect to the
// //             mean equator and equinox of J2000 (EME2000, ICRF)
// //
// //------------------------------------------------------------------------------
// Vector Moon (double Mjd_TT)
// {
// 	// Constants
// 	const double eps = 23.43929111 * Rad;           // Obliquity of J2000 ecliptic
// 	const double T   = (Mjd_TT - MJD_J2000) / 36525.0; // Julian cent. since J2000
// 
// 	// Variables
// 	double  L_0, l, lp, F, D, dL, S, h, N;
// 	double  L, B, R, cosB;
// 	Vector  r_Moon(3);
// 
// 	// Mean elements of lunar orbit
// 	L_0 =     Frac ( 0.606433 + 1336.851344 * T );   // Mean longitude [rev]
// 	// w.r.t. J2000 equinox
// 	l   = pi2 * Frac ( 0.374897 + 1325.552410 * T ); // Moon's mean anomaly [rad]
// 	lp  = pi2 * Frac ( 0.993133 +   99.997361 * T ); // Sun's mean anomaly [rad]
// 	D   = pi2 * Frac ( 0.827361 + 1236.853086 * T ); // Diff. long. Moon-Sun [rad]
// 	F   = pi2 * Frac ( 0.259086 + 1342.227825 * T ); // Argument of latitude
// 
// 	// Ecliptic longitude (w.r.t. equinox of J2000)
// 	dL = +22640 * sin(l) - 4586 * sin(l - 2 * D) + 2370 * sin(2 * D) +  769 * sin(2 * l)
// 		 - 668 * sin(lp) - 412 * sin(2 * F) - 212 * sin(2 * l - 2 * D) - 206 * sin(l + lp - 2 * D)
// 		 + 192 * sin(l + 2 * D) - 165 * sin(lp - 2 * D) - 125 * sin(D) - 110 * sin(l + lp)
// 		 + 148 * sin(l - lp) - 55 * sin(2 * F - 2 * D);
// 
// 	L = pi2 * Frac( L_0 + dL / 1296.0e3 ); // [rad]
// 
// 	// Ecliptic latitude
// 	S  = F + (dL + 412 * sin(2 * F) + 541 * sin(lp)) / Arcs;
// 	h  = F - 2 * D;
// 	N  = -526 * sin(h) + 44 * sin(l + h) - 31 * sin(-l + h) - 23 * sin(lp + h)
// 		 + 11 * sin(-lp + h) - 25 * sin(-2 * l + F) + 21 * sin(-l + F);
// 
// 	B = ( 18520.0 * sin(S) + N ) / Arcs; // [rad]
// 
// 	cosB = cos(B);
// 
// 	// Distance [m]
// 	R = 385000e3 - 20905e3 * cos(l) - 3699e3 * cos(2 * D - l) - 2956e3 * cos(2 * D)
// 		- 570e3 * cos(2 * l) + 246e3 * cos(2 * l - 2 * D) - 205e3 * cos(lp - 2 * D)
// 		- 171e3 * cos(l + 2 * D) - 152e3 * cos(l + lp - 2 * D);
// 
// 	// Equatorial coordinates
// 	r_Moon = R_x(-eps) * Vector ( R * cos(L) * cosB, R * sin(L) * cosB, R * sin(B) );
// 
// 	return r_Moon;
// }
// 
// //------------------------------------------------------------------------------
// //
// // Illumination
// //
// // Purpose:
// //
// //   Computes the fractional illumination of a spacecraft in the
// //   vicinity of the Earth assuming a cylindrical shadow model
// //
// // Input/output:
// //
// //   r               Spacecraft position vector [m]
// //   r_Sun           Sun position vector [m]
// //   <return>        Illumination factor:
// //                     nu=0   Spacecraft in Earth shadow
// //                     nu=1   Spacecraft fully illuminated by the Sun
// //
// //------------------------------------------------------------------------------
// double Illumination ( const Vector& r, const Vector& r_Sun )
// {
// 
// 	Vector e_Sun = r_Sun / Norm(r_Sun);   // Sun direction unit vector
// 	double s     = Dot ( r, e_Sun );      // Projection of s/c position
// 
// 	return ( ( s > 0 || Norm(r - s * e_Sun) > R_Earth ) ?  1.0 : 0.0 );
// }
// 
// //--------------------------------------------------------------------------
// // Inputs:
// // n         maximum degree
// // m         maximum order
// // fi        angle [rad]
// //
// // Outputs:
// // pnm       normalized Legendre polynomial values
// //
// //--------------------------------------------------------------------------
// Matrix Legendre(int n, int m, double fi)
// {
// 	Matrix pnm(n + 1, m + 1);
// 
// 	pnm(0, 0) = 1.0;
// 	pnm(1, 1) = sqrt(3.0) * cos(fi);
// // diagonal coefficients
// 	double s, h;
// 	for (int i = 2; i <= n; i++)
// 	{
// 		s = i;
// 		pnm(i, i) = sqrt((2 * s + 1) / (2 * s)) * cos(fi) * pnm(i - 1, i - 1);
// 	}
// // horizontal first step coefficients
// 	for (int i = 1; i <= n; i++)
// 	{
// 		s = i;
// 		pnm(i, i - 1) = sqrt(2 * s + 1) * sin(fi) * pnm(i - 1, i - 1);
// 	}
// // horizontal second step coefficients
// 	int j = 0, k = 2;
// 	do
// 	{
// 		for (int i = k; i <= n; i++)
// 		{
// 			s = i;
// 			h = j;
// 			pnm(i, j) = sqrt((2 * s + 1) / ((s - h) * (s + h))) * (sqrt(2 * s - 1) * sin(fi) * pnm(i - 1, j)
// 						- sqrt(((s + h - 1) * (s - h - 1)) / (2 * s - 3)) * pnm(i - 2, j));
// 		}
// 		j = j + 1;
// 		k = k + 1;
// 	}
// 	while (j <= m);
// 
// 	return pnm;
// }
// 
// //--------------------------------------------------------------------------
// // Inputs:
// // n         maximum degree
// // m         maximum order
// // fi        angle [rad]
// //
// // Output:
// // dpnm      normalized Legendre polynomial first derivative values
// //
// //--------------------------------------------------------------------------
// Matrix LegendreP(int n, int m, double fi)
// {
// 	Matrix pnm(n + 1, m + 1), dpnm(n + 1, m + 1);
// 
// 	pnm(0, 0) = 1.0;
// 	dpnm(0, 0) = 0.0;
// 	pnm(1, 1) = sqrt(3.0) * cos(fi);
// 	dpnm(1, 1) = -sqrt(3.0) * sin(fi);
// // diagonal coefficients
// 	double s, h;
// 	for (int i = 2; i <= n; i++)
// 	{
// 		s = i;
// 		pnm(i, i) = sqrt((2 * s + 1) / (2 * s)) * cos(fi) * pnm(i - 1, i - 1);
// 		dpnm(i, i) = sqrt((2 * s + 1) / (2 * s)) * (cos(fi) * dpnm(i - 1, i - 1) - sin(fi) * pnm(i - 1, i - 1));
// 	}
// // horizontal first step coefficients
// 	for (int i = 1; i <= n; i++)
// 	{
// 		s = i;
// 		pnm(i, i - 1) = sqrt(2 * s + 1) * sin(fi) * pnm(i - 1, i - 1);
// 		dpnm(i, i - 1) = sqrt(2 * s + 1) * ((cos(fi) * pnm(i - 1, i - 1)) + (sin(fi) * dpnm(i - 1, i - 1)));
// 	}
// // horizontal second step coefficients
// 	int j = 0, k = 2;
// 
// 	do
// 	{
// 		for (int i = k; i <= n; i++)
// 		{
// 			s = i;
// 			h = j;
// 			pnm(i, j) = sqrt((2 * s + 1) / ((s - h) * (s + h))) * (sqrt(2 * s - 1) * sin(fi) * pnm(i - 1, j)
// 						- sqrt(((s + h - 1) * (s - h - 1)) / (2 * s - 3)) * pnm(i - 2, j));
// 			dpnm(i, j) = sqrt((2 * s + 1) / ((s - h) * (s + h))) * ((sqrt(2 * s - 1) * sin(fi) * dpnm(i - 1, j))
// 						 + sqrt(2 * s - 1) * cos(fi) * pnm(i - 1, j) - sqrt(((s + h - 1) * (s - h - 1)) / (2 * s - 3)) * dpnm(i - 2, j));
// 		}
// 		j = j + 1;
// 		k = k + 1;
// 	}
// 	while (j <= m);
// 	return dpnm;
// }
// 
// //------------------------------------------------------------------------------
// //
// // AccelHarmonic
// //
// // Purpose:
// //
// //   Computes the acceleration due to the harmonic gravity field of the
// //   central body
// //
// // Input/Output:
// //
// //   r           Satellite position vector in the inertial system
// //   E           Transformation matrix to body-fixed system
// //   GM          Gravitational coefficient
// //   R_ref       Reference radius
// //   CS          Spherical harmonics coefficients (un-normalized)
// //   n_max       Maximum degree
// //   m_max       Maximum order (m_max<=n_max; m_max=0 for zonals, only)
// //   <return>    Acceleration (a=d^2r/dt^2)
// //
// //------------------------------------------------------------------------------
// Vector AccelHarmonic (const Vector& r, const Matrix& E, double GM, double r_ref,
// 					  const Matrix& cnm, const Matrix& snm, int n_max, int m_max )
// {
// 	// Local variables
// 	int     n, m;                          // Loop counters
// 	double  d, rho, Fac;                   // Auxiliary quantities
// 	double  ax, ay, az;                    // Acceleration vector
// 	Vector  r_bf(3);                       // Body-fixed position
// 	Vector  a_bf(3);                       // Body-fixed acceleration
// 	Matrix pnm(n_max + 1, m_max + 1), dpnm(n_max + 1, m_max + 1); // Legendre polynomials
// 	double latgc, lon;                     // Geocentric latitude and longitude
// 	double dUdr, dUdlatgc, dUdlon;
// 	double q1, q2, q3, b1, b2, b3, r2xy;
// 	double nd;
// 
// 	// Body-fixed position
// 	r_bf = E * r;
// 
// 	// Auxiliary quantities
// 	d = Norm(r_bf);          // distance
// 	latgc = asin(r_bf(2) / d);
// 	lon = atan2(r_bf(1), r_bf(0));
// 	pnm = Legendre(n_max, m_max, latgc);
// 	dpnm = LegendreP(n_max, m_max, latgc);
// 
// 	dUdr = 0;
// 	dUdlatgc = 0;
// 	dUdlon = 0;
// 	q3 = 0; q2 = q3; q1 = q2;
// 	for (int n = 0; n <= n_max; n++)
// 	{
// 		nd = n;
// 		b1 = (-GM / pow(d, 2.0)) * pow((r_ref / d), nd) * (n + 1);
// 		b2 =  (GM / d) * pow((r_ref / d), nd);
// 		b3 =  (GM / d) * pow((r_ref / d), nd);
// 		for (int m = 0; m <= m_max; m++)
// 		{
// 			q1 = q1 + pnm(n, m) * (cnm(n, m) * cos(m * lon) + snm(n, m) * sin(m * lon));
// 			q2 = q2 + dpnm(n, m) * (cnm(n, m) * cos(m * lon) + snm(n, m) * sin(m * lon));
// 			q3 = q3 + m * pnm(n, m) * (snm(n, m) * cos(m * lon) - cnm(n, m) * sin(m * lon));
// 		}
// 		dUdr     = dUdr     + q1 * b1;
// 		dUdlatgc = dUdlatgc + q2 * b2;
// 		dUdlon   = dUdlon   + q3 * b3;
// 		q3 = 0; q2 = q3; q1 = q2;
// 	}
// 
// 	// Body-fixed acceleration
// 	r2xy = pow(r_bf(0), 2.0) + pow(r_bf(1), 2.0);
// 
// 	ax = (1.0 / d * dUdr - r_bf(2) / (pow(d, 2.0) * sqrt(r2xy)) * dUdlatgc) * r_bf(0) - (1 / r2xy * dUdlon) * r_bf(1);
// 	ay = (1.0 / d * dUdr - r_bf(2) / (pow(d, 2.0) * sqrt(r2xy)) * dUdlatgc) * r_bf(1) + (1 / r2xy * dUdlon) * r_bf(0);
// 	az =  1.0 / d * dUdr * r_bf(2) + sqrt(r2xy) / pow(d, 2.0) * dUdlatgc;
// 
// 	a_bf(0) = ax;
// 	a_bf(1) = ay;
// 	a_bf(2) = az;
// 
// 	// Inertial acceleration
// 	return  Transp(E) * a_bf;
// 
// }
// 
// //------------------------------------------------------------------------------
// //
// // AccelPointMass
// //
// // Purpose:
// //
// //   Computes the perturbational acceleration due to a point mass
// //
// // Input/Output:
// //
// //   r           Satellite position vector
// //   s           Point mass position vector
// //   GM          Gravitational coefficient of point mass
// //   <return>    Acceleration (a=d^2r/dt^2)
// //
// //------------------------------------------------------------------------------
// Vector AccelPointMass (const Vector& r, const Vector& s, double GM)
// {
// 	Vector d(3);
// 
// 	//  Relative position vector of satellite w.r.t. point mass
// 	d = r - s;
// 
// 	// Acceleration
// 	return  (-GM) * ( d / pow(Norm(d), 3) + s / pow(Norm(s), 3) );
// }
// 
// //------------------------------------------------------------------------------
// //
// // AccelSolrad
// //
// // Purpose:
// //
// //   Computes the acceleration due to solar radiation pressure assuming
// //   the spacecraft surface normal to the Sun direction
// //
// // Input/Output:
// //
// //   r           Spacecraft position vector
// //   r_Sun       Sun position vector
// //   Area        Cross-section
// //   mass        Spacecraft mass
// //   CR          Solar radiation pressure coefficient
// //   P0          Solar radiation pressure at 1 AU
// //   AU          Length of one Astronomical Unit
// //   <return>    Acceleration (a=d^2r/dt^2)
// //
// // Notes:
// //
// //   r, r_sun, Area, mass, P0 and AU must be given in consistent units,
// //   e.g. m, m^2, kg and N/m^2.
// //
// //------------------------------------------------------------------------------
// Vector AccelSolrad (const Vector& r, const Vector& r_Sun, double Area,
// 					double mass, double CR, double P0, double AU )
// {
// 	Vector d(3);
// 
// 	// Relative position vector of spacecraft w.r.t. Sun
// 	d = r - r_Sun;
// 
// 	// Acceleration
// 	return  CR * (Area / mass) * P0 * (AU * AU) * d / pow(Norm(d), 3);
// }
// 
// //------------------------------------------------------------------------------
// //
// // AccelDrag
// //
// // Purpose:
// //
// //   Computes the acceleration due to the atmospheric drag.
// //
// // Input/Output:
// //
// //   Mjd_TT      Terrestrial Time (Modified Julian Date)
// //   r           Satellite position vector in the inertial system [m]
// //   v           Satellite velocity vector in the inertial system [m/s]
// //   T           Transformation matrix to true-of-date inertial system
// //   Area        Cross-section [m^2]
// //   mass        Spacecraft mass [kg]
// //   CD          Drag coefficient
// //   <return>    Acceleration (a=d^2r/dt^2) [m/s^2]
// //
// //------------------------------------------------------------------------------
// Vector AccelDrag ( double Mjd_TT, const Vector& r, const Vector& v, const Matrix& T,
// 				   const Matrix& E, double Area, double mass, double CD )
// {
// 	// Constants
// 
// 	// Earth angular velocity vector [rad/s]
// 	const double Data_omega[3] = { 0.0, 0.0, omega_Earth };
// 	const Vector omega ( &Data_omega[0], 3);
// 
// 
// 	// Variables
// 	double v_abs, dens;
// 	Vector r_tod(3), v_tod(3);
// 	Vector v_rel(3), a_tod(3);
// 	Matrix T_trp(3, 3);
// 
// 	// Transformation matrix to ICRF/EME2000 system
// 	T_trp = Transp(T);
// 
// 	// Position and velocity in true-of-date system
// 	r_tod = T * r;
// 	v_tod = T * v;
// 
// 	// Velocity relative to the Earth's atmosphere
// 	v_rel = v_tod - Cross(omega, r_tod);
// 	v_abs = Norm(v_rel);
// 
// 	// Atmospheric density due to modified Harris-Priester model
// 	dens = Density_HP(Mjd_TT, r_tod);
// 
// 	// Acceleration
// 	a_tod = -0.5 * CD * (Area / mass) * dens * v_abs * v_rel;
// 
// 	return T_trp * a_tod;
// }
// 
// //------------------------------------------------------------------------------
// //
// // Density_HP
// //
// // Purpose:
// //
// //   Computes the atmospheric density for the modified Harris-Priester model.
// //
// // Input/Output:
// //
// //   Mjd_TT      Terrestrial Time (Modified Julian Date)
// //   r_tod       Satellite position vector in the inertial system [m]
// //   <return>    Density [kg/m^3]
// //
// //------------------------------------------------------------------------------
// double Density_HP(double Mjd_TT, const Vector& r_tod)
// {
// 	// Constants
// 	const double upper_limit =   1000.0;           // Upper height limit [km]
// 	const double lower_limit =    100.0;           // Lower height limit [km]
// 	const double ra_lag      = 0.523599;           // Right ascension lag [rad]
// 	const int    n_prm       =        3;           // Harris-Priester parameter
// 	// 2(6) low(high) inclination
// 
// 	// Harris-Priester atmospheric density model parameters
// 	// Height [km], minimum density, maximum density [gm/km^3]
// 	const int    N_Coef = 50;
// 	const double Data_h[N_Coef] =
// 	{
// 		100.0, 120.0, 130.0, 140.0, 150.0, 160.0, 170.0, 180.0, 190.0, 200.0,
// 		210.0, 220.0, 230.0, 240.0, 250.0, 260.0, 270.0, 280.0, 290.0, 300.0,
// 		320.0, 340.0, 360.0, 380.0, 400.0, 420.0, 440.0, 460.0, 480.0, 500.0,
// 		520.0, 540.0, 560.0, 580.0, 600.0, 620.0, 640.0, 660.0, 680.0, 700.0,
// 		720.0, 740.0, 760.0, 780.0, 800.0, 840.0, 880.0, 920.0, 960.0, 1000.0
// 	};
// 	const double Data_c_min[N_Coef] =
// 	{
// 		4.974e+05, 2.490e+04, 8.377e+03, 3.899e+03, 2.122e+03, 1.263e+03,
// 		8.008e+02, 5.283e+02, 3.617e+02, 2.557e+02, 1.839e+02, 1.341e+02,
// 		9.949e+01, 7.488e+01, 5.709e+01, 4.403e+01, 3.430e+01, 2.697e+01,
// 		2.139e+01, 1.708e+01, 1.099e+01, 7.214e+00, 4.824e+00, 3.274e+00,
// 		2.249e+00, 1.558e+00, 1.091e+00, 7.701e-01, 5.474e-01, 3.916e-01,
// 		2.819e-01, 2.042e-01, 1.488e-01, 1.092e-01, 8.070e-02, 6.012e-02,
// 		4.519e-02, 3.430e-02, 2.632e-02, 2.043e-02, 1.607e-02, 1.281e-02,
// 		1.036e-02, 8.496e-03, 7.069e-03, 4.680e-03, 3.200e-03, 2.210e-03,
// 		1.560e-03, 1.150e-03
// 	};
// 	const double Data_c_max[N_Coef] =
// 	{
// 		4.974e+05, 2.490e+04, 8.710e+03, 4.059e+03, 2.215e+03, 1.344e+03,
// 		8.758e+02, 6.010e+02, 4.297e+02, 3.162e+02, 2.396e+02, 1.853e+02,
// 		1.455e+02, 1.157e+02, 9.308e+01, 7.555e+01, 6.182e+01, 5.095e+01,
// 		4.226e+01, 3.526e+01, 2.511e+01, 1.819e+01, 1.337e+01, 9.955e+00,
// 		7.492e+00, 5.684e+00, 4.355e+00, 3.362e+00, 2.612e+00, 2.042e+00,
// 		1.605e+00, 1.267e+00, 1.005e+00, 7.997e-01, 6.390e-01, 5.123e-01,
// 		4.121e-01, 3.325e-01, 2.691e-01, 2.185e-01, 1.779e-01, 1.452e-01,
// 		1.190e-01, 9.776e-02, 8.059e-02, 5.741e-02, 4.210e-02, 3.130e-02,
// 		2.360e-02, 1.810e-02
// 	};
// 
// 	const Vector h ( &Data_h[0], N_Coef);
// 	const Vector c_min ( &Data_c_min[0], N_Coef);
// 	const Vector c_max ( &Data_c_max[0], N_Coef);
// 
// 	// Variables
// 	int    i, ih;                              // Height section variables
// 	double height;                             // Earth flattening
// 	double dec_Sun, ra_Sun, c_dec;             // Sun declination, right asc.
// 	double c_psi2;                             // Harris-Priester modification
// 	double density, h_min, h_max, d_min, d_max;// Height, density parameters
// 	Vector r_Sun(3);                           // Sun position
// 	Vector u(3);                               // Apex of diurnal bulge
// 
// 	// Satellite height
// 	height = Geodetic(r_tod).h / 1000.0;            //  [km]
// 
// 	// Exit with zero density outside height model limits
// 	if ( height >= upper_limit || height <= lower_limit )
// 	{
// 		return 0.0;
// 	}
// 
// 	// Sun right ascension, declination
// 	r_Sun = Sun ( Mjd_TT );
// 	ra_Sun  = atan2( r_Sun(1), r_Sun(0) );
// 	dec_Sun = atan2( r_Sun(2), sqrt( pow(r_Sun(0), 2) + pow(r_Sun(1), 2) ) );
// 
// 	// Unit vector u towards the apex of the diurnal bulge
// 	// in inertial geocentric coordinates
// 	c_dec = cos(dec_Sun);
// 	u(0) = c_dec * cos(ra_Sun + ra_lag);
// 	u(1) = c_dec * sin(ra_Sun + ra_lag);
// 	u(2) = sin(dec_Sun);
// 
// 	// Cosine of half angle between satellite position vector and
// 	// apex of diurnal bulge
// 	c_psi2 = 0.5 + 0.5 * Dot(r_tod, u) / Norm(r_tod);
// 
// 	// Height index search and exponential density interpolation
// 	ih = 0;                           // section index reset
// 	for ( i = 0; i < N_Coef - 1; i++) // loop over N_Coef height regimes
// 	{
// 		if ( height >= h(i) && height < h(i + 1) )
// 		{
// 			ih = i;                       // ih identifies height section
// 			break;
// 		}
// 	}
// 
// 	h_min = ( h(ih) - h(ih + 1) ) / log( c_min(ih + 1) / c_min(ih) );
// 	h_max = ( h(ih) - h(ih + 1) ) / log( c_max(ih + 1) / c_max(ih) );
// 
// 	d_min = c_min(ih) * exp( (h(ih) - height) / h_min );
// 	d_max = c_max(ih) * exp( (h(ih) - height) / h_max );
// 
// 	// Density computation
// 	density = d_min + (d_max - d_min) * pow(c_psi2, n_prm);
// 
// 	return density * 1.0e-12;       // [kg/m^3]
// }
