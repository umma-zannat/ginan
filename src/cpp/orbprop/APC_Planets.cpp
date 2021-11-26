// //------------------------------------------------------------------------------
// //
// // APC_Planets.cpp
// //
// // Purpose:
// //
// //    Computation of the planets' position
// //
// //------------------------------------------------------------------------------
// 
// #include <cmath>
// 
// #include "APC_Planets.h"
// #include "SAT_Const.h"
// 
// //------------------------------------------------------------------------------
// //
// // Frac: Gives the fractional part of a number
// //
// //------------------------------------------------------------------------------
// double Frac (double x)
// {
// 	return x - floor(x);
// }
// 
// //------------------------------------------------------------------------------
// //
// // Modulo: calculates x mod y
// //
// //------------------------------------------------------------------------------
// 
// double Modulo (double x, double y)
// {
// 	return y * Frac(x / y);
// }
// 
// //------------------------------------------------------------------------------
// //
// // AddThe: Calculates cos(alpha+beta) and sin(alpha+beta) using addition
// //         theorems
// //
// // Input:
// //
// //   c1        cos(alpha)
// //   s1        sin(alpha)
// //   c2        cos(beta)
// //   s2        sin(beta)
// //
// // Output:
// //
// //   c         cos(alpha+beta)
// //   s         sin(alpha+beta)
// //
// //------------------------------------------------------------------------------
// void AddThe ( double c1, double s1, double c2, double s2,
// 			  double& c, double& s )
// {
// 	c = c1 * c2 - s1 * s2;
// 	s = s1 * c2 + c1 * s2;
// }
// 
// namespace // Unnamed namespace
// {
// //
// // Constants
// //
// const int o   = 6;          // Index offset
// const int dim = 2 * o + 1;  // Work array dimension
// 
// //
// // Sine
// //
// double Sine (double x)
// {
// 	return sin(pi2 * Frac(x));
// }
// 
// //
// // Definition of ILE_Pert class for summing up Brown's lunar perturbation
// // series
// //
// class ILE_Pert
// {
// public:
// 
// 	// Initialization (mean arguments, long-periodic corrections, etc.)
// 	void Init (double T);
// 
// 	// Perturbation term
// 	void Term (int p, int q, int r, int s, double& x, double& y);
// 
// 	// Summation of solar perturbations
// 	void AddSol ( double coeffl, double coeffS, double coeffg,
// 				  double coeffP, int p, int q, int r, int s );
// 
// 	// Summation of perturbation in latitude
// 	void AddN (double coeffN, int p, int q, int r, int s);
// 
// 	// Planetary perturbations
// 	void Planetary (double T);
// 
// 	// Coordinates
// 	double lambda();  // ecliptic longitude in [rad]
// 	double beta();    // ecliptic latitude in [rad]
// 	double dist();    // geocentric distance in [km]
// 
// private:
// 	double Dgam;                       // Longperiodic perturbation
// 	double Dlam, DS, gam1C, sinPi, N;  // Periodic perturbations
// 	double L0, l, ls, F, D;            // Mean arguments of lunar orbit
// 	double Cos[dim][4], Sin[dim][4];   // Cosine and sine of mean arguments
// };
// 
// //
// // Initialization
// //
// void ILE_Pert::Init (double T)
// {
// 	//
// 	// Variables
// 	//
// 	double dL0, dl, dls, dF, dD;             // Longperiodic perturbations
// 	double T2, arg = 0.0, fac = 0.0;         // Auxiliary variables
// 	double S1, S2, S3, S4, S5, S6, S7;
// 	int    max = 0;
// 
// 	T2 = T * T; // Time
// 
// 	// Reset perturbations
// 	Dlam = 0.0; DS = 0.0; gam1C = 0.0; sinPi = 3422.7000; N = 0.0;
// 
// 	// Longperiodic perturbations
// 	S1 = Sine (0.19833 + 0.05611 * T);  S2 = Sine (0.27869 + 0.04508 * T);
// 	S3 = Sine (0.16827 - 0.36903 * T);  S4 = Sine (0.34734 - 5.37261 * T);
// 	S5 = Sine (0.10498 - 5.37899 * T);  S6 = Sine (0.42681 - 0.41855 * T);
// 	S7 = Sine (0.14943 - 5.37511 * T);
// 
// 	dL0 = 0.84 * S1 + 0.31 * S2 + 14.27 * S3 + 7.26 * S4 + 0.28 * S5 + 0.24 * S6;
// 	dl  = 2.94 * S1 + 0.31 * S2 + 14.27 * S3 + 9.34 * S4 + 1.12 * S5 + 0.83 * S6;
// 	dls = -6.40 * S1                                   - 1.89 * S6;
// 	dF  = 0.21 * S1 + 0.31 * S2 + 14.27 * S3 - 88.70 * S4 - 15.30 * S5 + 0.24 * S6 - 1.86 * S7;
// 	dD  = dL0 - dls;
// 
// 	Dgam   = -3332e-9 * Sine (0.59734 - 5.37261 * T)
// 			 - 539e-9 * Sine (0.35498 - 5.37899 * T)
// 			 - 64e-9 * Sine (0.39943 - 5.37511 * T);
// 
// 	// Mean arguments of the lunar orbit (incl. longperiodic corrections)
// 	// L0 mean longitude of the Moon
// 	// l  mean anomaly of the Moon     l' mean anomaly of the Sun
// 	// F  mean distance from the node  D  mean elongation from the Sun
// 	L0 = pi2 * Frac(0.60643382 + 1336.85522467 * T - 0.00000313 * T2) + dL0 / Arcs;
// 	l  = pi2 * Frac(0.37489701 + 1325.55240982 * T + 0.00002565 * T2) + dl / Arcs;
// 	ls = pi2 * Frac(0.99312619 +  99.99735956 * T - 0.00000044 * T2) + dls / Arcs;
// 	F  = pi2 * Frac(0.25909118 + 1342.22782980 * T - 0.00000892 * T2) + dF / Arcs;
// 	D  = pi2 * Frac(0.82736186 + 1236.85308708 * T - 0.00000397 * T2) + dD / Arcs;
// 
// 	// Cosine and sine of multiples of mean arguments
// 	// incl. secular correction
// 	for (int i = 0; i <= 3; i++)
// 	{
// 		switch (i)
// 		{
// 			case 0: arg = l;  max = 4; fac = 1.000002208;               break;
// 			case 1: arg = ls; max = 3; fac = 0.997504612 - 0.002495388 * T; break;
// 			case 2: arg = F;  max = 4; fac = 1.000002708 + 139.978 * Dgam;  break;
// 			case 3: arg = D;  max = 6; fac = 1.0;                       break;
// 		};
// 
// 		Cos[o][i] = 1.0;  Cos[o + 1][i] = cos(arg) * fac;  Cos[o - 1][i] = +Cos[o + 1][i];
// 		Sin[o][i] = 0.0;  Sin[o + 1][i] = sin(arg) * fac;  Sin[o - 1][i] = -Sin[o + 1][i];
// 
// 		for (int j = 2; j <= max; j++)
// 		{
// 			AddThe ( Cos[o + j - 1][i], Sin[o + j - 1][i], Cos[o + 1][i], Sin[o + 1][i],
// 					 Cos[o + j][i], Sin[o + j][i] );
// 			Cos[o - j][i] = +Cos[o + j][i];
// 			Sin[o - j][i] = -Sin[o + j][i];
// 		};
// 	};
// }
// 
// //
// // Term: calculates x=cos(p*l+q*ls+r*F+s*D) and y=sin(p*l+q*ls+r*F+s*D)
// //
// //
// void ILE_Pert::Term (int p, int q, int r, int s, double& x, double& y)
// {
// 	int i[4];
// 
// 	i[0] = p; i[1] = q; i[2] = r; i[3] = s;  x = 1.0; y = 0.0;
// 
// 	for (int k = 0; k <= 3; k++)
// 		if (i[k] != 0) AddThe(x, y, Cos[o + i[k]][k], Sin[o + i[k]][k], x, y);
// }
// 
// //
// // AddSol: Summation of solar perturbations
// //
// void ILE_Pert::AddSol (
// 	double coeffl, double coeffS, double coeffg, double coeffP,
// 	int p, int q, int r, int s )
// {
// 	//
// 	// Variables
// 	//
// 	double x, y;
// 
// 
// 	Term (p, q, r, s, x, y);
// 	Dlam  += coeffl * y; DS    += coeffS * y;
// 	gam1C += coeffg * x; sinPi += coeffP * x;
// }
// 
// //
// // AddN: Summation of perturbation in latitude
// //
// void ILE_Pert::AddN (double coeffN, int p, int q, int r, int s)
// {
// 	//
// 	// Variables
// 	//
// 	double x, y;
// 
// 
// 	Term(p, q, r, s, x, y);
// 	N += coeffN * y;
// }
// 
// //
// // Planetary: Perturbations of ecliptic latitude by Venus and Jupiter
// //
// //
// void ILE_Pert::Planetary (double T)
// {
// 	Dlam +=
// 		+0.82 * Sine(0.7736  - 62.5512 * T) + 0.31 * Sine(0.0466 - 125.1025 * T)
// 		+ 0.35 * Sine(0.5785  - 25.1042 * T) + 0.66 * Sine(0.4591 + 1335.8075 * T)
// 		+ 0.64 * Sine(0.3130  - 91.5680 * T) + 1.14 * Sine(0.1480 + 1331.2898 * T)
// 		+ 0.21 * Sine(0.5918 + 1056.5859 * T) + 0.44 * Sine(0.5784 + 1322.8595 * T)
// 		+ 0.24 * Sine(0.2275   - 5.7374 * T) + 0.28 * Sine(0.2965   + 2.6929 * T)
// 		+ 0.33 * Sine(0.3132   + 6.3368 * T);
// }
// 
// //
// // lambda, beta, dist
// //
// double ILE_Pert::lambda()
// {
// 	return Modulo(L0 + Dlam / Arcs, pi2);
// }
// 
// double ILE_Pert::beta()
// {
// 	//
// 	// Variables
// 	//
// 	double S   = F + DS / Arcs;
// 	double fac = 1.000002708 + 139.978 * Dgam;
// 
// 
// 	return (fac * (18518.511 + 1.189 + gam1C) * sin(S) - 6.24 * sin(3 * S) + N) / Arcs;
// }
// 
// double ILE_Pert::dist()
// {
// 	return R_Earth * Arcs / (sinPi * 0.999953253);
// }
// 
// //
// // Constants
// //
// const int o1   = 10;         // Index offset
// const int dim1 = 2 * o1 + 1;  // Work array dimension
// 
// //
// // Definition of Pert class for summing up trigonometric perturbation
// // series
// class Pert
// {
// public:
// 
// 	// Set time, mean anomalies and index range
// 	void Init ( double T,
// 				double M, int I_min, int I_max,
// 				double m, int i_min, int i_max );
// 
// 	// Sum-up perturbations in longitude, radius and latitude
// 	void Term ( int I, int i, int iT,
// 				double dlc, double dls,
// 				double drc, double drs,
// 				double dbc, double dbs );
// 
// 	// Retrieve perturbations in longitude, radius and latitude
// 	double dl();
// 	double dr();
// 	double db();
// 
// private:
// 	double m_T;
// 	double m_C[dim1], m_S[dim1], m_c[dim1], m_s[dim1];
// 	double m_dl, m_db, m_dr;
// 	double m_u, m_v;
// };
// 
// // Set time, mean anomalies and index range
// void Pert::Init ( double T,
// 				  double M, int I_min, int I_max,
// 				  double m, int i_min, int i_max )
// {
// 	//
// 	// Variables
// 	//
// 	int i;
// 
// 	m_dl = 0.0; m_dr = 0.0; m_db = 0.0; // reset perturbations
// 
// 	m_T = T; // set time
// 
// 	// cosine and sine of multiples of M
// 	m_C[o1] = 1.0; m_C[o1 + 1] = cos(M); m_C[o1 - 1] = +m_C[o1 + 1];
// 	m_S[o1] = 0.0; m_S[o1 + 1] = sin(M); m_S[o1 - 1] = -m_S[o1 + 1];
// 
// 	for (i = 1; i < I_max; i++)
// 		AddThe ( m_C[o1 + i], m_S[o1 + i], m_C[o1 + 1], m_S[o1 + 1], m_C[o1 + i + 1], m_S[o1 + i + 1] );
// 	for (i = -1; i > I_min; i--)
// 		AddThe ( m_C[o1 + i], m_S[o1 + i], m_C[o1 - 1], m_S[o1 - 1], m_C[o1 + i - 1], m_S[o1 + i - 1] );
// 
// 	// cosine and sine of multiples of m
// 	m_c[o1] = 1.0; m_c[o1 + 1] = cos(m); m_c[o1 - 1] = +m_c[o1 + 1];
// 	m_s[o1] = 0.0; m_s[o1 + 1] = sin(m); m_s[o1 - 1] = -m_s[o1 + 1];
// 
// 	for (i = 1; i < i_max; i++)
// 		AddThe ( m_c[o1 + i], m_s[o1 + i], m_c[o1 + 1], m_s[o1 + 1], m_c[o1 + i + 1], m_s[o1 + i + 1] );
// 	for (i = -1; i > i_min; i--)
// 		AddThe ( m_c[o1 + i], m_s[o1 + i], m_c[o1 - 1], m_s[o1 - 1], m_c[o1 + i - 1], m_s[o1 + i - 1] );
// }
// 
// // Sum-up perturbations in longitude, radius and latitude
// void Pert::Term ( int I, int i, int iT,
// 				  double dlc, double dls,
// 				  double drc, double drs,
// 				  double dbc, double dbs )
// {
// 	if ( iT == 0 )
// 		AddThe ( m_C[o1 + I], m_S[o1 + I], m_c[o1 + i], m_s[o1 + i], m_u, m_v );
// 	else
// 	{ m_u *= m_T; m_v *= m_T; };
// 
// 	m_dl += ( dlc * m_u + dls * m_v );
// 	m_dr += ( drc * m_u + drs * m_v );
// 	m_db += ( dbc * m_u + dbs * m_v );
// }
// 
// // Retrieve perturbations in longitude, radius and latitude
// double Pert::dl()
// { return m_dl; }
// 
// double Pert::dr()
// { return m_dr; }
// 
// double Pert::db()
// { return m_db; }
// 
// } // End of unnamed namespace
// 
// //------------------------------------------------------------------------------
// //
// // MoonPos: Computes the Moon's ecliptic position using Brown's theory
// //          (Improved Lunar Ephemeris)
// //
// // Input:
// //
// //   T         Time in Julian centuries since J2000
// //
// // <return>:   Geocentric position of the Moon (in [m]) referred to the
// //             ecliptic and equinox of date
// //
// // Notes: Light-time is already taken into account
// //
// //------------------------------------------------------------------------------
// Vector MoonPos (double T)
// {
// 	//
// 	// Variables
// 	//
// 	ILE_Pert Pert;
// 
// 
// 	Pert.Init(T);  // Initialization
// 
// 
// 	// Solar perturbations
// 	Pert.AddSol (   13.902,   14.06, -0.001,   0.2607, 0, 0, 0, 4);
// 	Pert.AddSol (    0.403,   -4.01, +0.394,   0.0023, 0, 0, 0, 3);
// 	Pert.AddSol ( 2369.912, 2373.36, +0.601,  28.2333, 0, 0, 0, 2);
// 	Pert.AddSol ( -125.154, -112.79, -0.725,  -0.9781, 0, 0, 0, 1);
// 	Pert.AddSol (    1.979,    6.98, -0.445,   0.0433, 1, 0, 0, 4);
// 	Pert.AddSol (  191.953,  192.72, +0.029,   3.0861, 1, 0, 0, 2);
// 	Pert.AddSol (   -8.466,  -13.51, +0.455,  -0.1093, 1, 0, 0, 1);
// 	Pert.AddSol (22639.500, 22609.07, +0.079, 186.5398, 1, 0, 0, 0);
// 	Pert.AddSol (   18.609,    3.59, -0.094,   0.0118, 1, 0, 0, -1);
// 	Pert.AddSol (-4586.465, -4578.13, -0.077,  34.3117, 1, 0, 0, -2);
// 	Pert.AddSol (   +3.215,    5.44, +0.192,  -0.0386, 1, 0, 0, -3);
// 	Pert.AddSol (  -38.428,  -38.64, +0.001,   0.6008, 1, 0, 0, -4);
// 	Pert.AddSol (   -0.393,   -1.43, -0.092,   0.0086, 1, 0, 0, -6);
// 	Pert.AddSol (   -0.289,   -1.59, +0.123,  -0.0053, 0, 1, 0, 4);
// 	Pert.AddSol (  -24.420,  -25.10, +0.040,  -0.3000, 0, 1, 0, 2);
// 	Pert.AddSol (   18.023,   17.93, +0.007,   0.1494, 0, 1, 0, 1);
// 	Pert.AddSol ( -668.146, -126.98, -1.302,  -0.3997, 0, 1, 0, 0);
// 	Pert.AddSol (    0.560,    0.32, -0.001,  -0.0037, 0, 1, 0, -1);
// 	Pert.AddSol ( -165.145, -165.06, +0.054,   1.9178, 0, 1, 0, -2);
// 	Pert.AddSol (   -1.877,   -6.46, -0.416,   0.0339, 0, 1, 0, -4);
// 	Pert.AddSol (    0.213,    1.02, -0.074,   0.0054, 2, 0, 0, 4);
// 	Pert.AddSol (   14.387,   14.78, -0.017,   0.2833, 2, 0, 0, 2);
// 	Pert.AddSol (   -0.586,   -1.20, +0.054,  -0.0100, 2, 0, 0, 1);
// 	Pert.AddSol (  769.016,  767.96, +0.107,  10.1657, 2, 0, 0, 0);
// 	Pert.AddSol (   +1.750,    2.01, -0.018,   0.0155, 2, 0, 0, -1);
// 	Pert.AddSol ( -211.656, -152.53, +5.679,  -0.3039, 2, 0, 0, -2);
// 	Pert.AddSol (   +1.225,    0.91, -0.030,  -0.0088, 2, 0, 0, -3);
// 	Pert.AddSol (  -30.773,  -34.07, -0.308,   0.3722, 2, 0, 0, -4);
// 	Pert.AddSol (   -0.570,   -1.40, -0.074,   0.0109, 2, 0, 0, -6);
// 	Pert.AddSol (   -2.921,  -11.75, +0.787,  -0.0484, 1, 1, 0, 2);
// 	Pert.AddSol (   +1.267,    1.52, -0.022,   0.0164, 1, 1, 0, 1);
// 	Pert.AddSol ( -109.673, -115.18, +0.461,  -0.9490, 1, 1, 0, 0);
// 	Pert.AddSol ( -205.962, -182.36, +2.056,  +1.4437, 1, 1, 0, -2);
// 	Pert.AddSol (    0.233,    0.36, 0.012,  -0.0025, 1, 1, 0, -3);
// 	Pert.AddSol (   -4.391,   -9.66, -0.471,   0.0673, 1, 1, 0, -4);
// 	Pert.AddSol (    0.283,    1.53, -0.111,  +0.0060, 1, -1, 0, +4);
// 	Pert.AddSol (   14.577,   31.70, -1.540,  +0.2302, 1, -1, 0, 2);
// 	Pert.AddSol (  147.687,  138.76, +0.679,  +1.1528, 1, -1, 0, 0);
// 	Pert.AddSol (   -1.089,    0.55, +0.021,   0.0, 1, -1, 0, -1);
// 	Pert.AddSol (   28.475,   23.59, -0.443,  -0.2257, 1, -1, 0, -2);
// 	Pert.AddSol (   -0.276,   -0.38, -0.006,  -0.0036, 1, -1, 0, -3);
// 	Pert.AddSol (    0.636,    2.27, +0.146,  -0.0102, 1, -1, 0, -4);
// 	Pert.AddSol (   -0.189,   -1.68, +0.131,  -0.0028, 0, 2, 0, 2);
// 	Pert.AddSol (   -7.486,   -0.66, -0.037,  -0.0086, 0, 2, 0, 0);
// 	Pert.AddSol (   -8.096,  -16.35, -0.740,   0.0918, 0, 2, 0, -2);
// 	Pert.AddSol (   -5.741,   -0.04, 0.0,  -0.0009, 0, 0, 2, 2);
// 	Pert.AddSol (    0.255,    0.0, 0.0,   0.0, 0, 0, 2, 1);
// 	Pert.AddSol ( -411.608,   -0.20, 0.0,  -0.0124, 0, 0, 2, 0);
// 	Pert.AddSol (    0.584,    0.84, 0.0,  +0.0071, 0, 0, 2, -1);
// 	Pert.AddSol (  -55.173,  -52.14, 0.0,  -0.1052, 0, 0, 2, -2);
// 	Pert.AddSol (    0.254,    0.25, 0.0,  -0.0017, 0, 0, 2, -3);
// 	Pert.AddSol (   +0.025,   -1.67, 0.0,  +0.0031, 0, 0, 2, -4);
// 	Pert.AddSol (    1.060,    2.96, -0.166,   0.0243, 3, 0, 0, +2);
// 	Pert.AddSol (   36.124,   50.64, -1.300,   0.6215, 3, 0, 0, 0);
// 	Pert.AddSol (  -13.193,  -16.40, +0.258,  -0.1187, 3, 0, 0, -2);
// 	Pert.AddSol (   -1.187,   -0.74, +0.042,   0.0074, 3, 0, 0, -4);
// 	Pert.AddSol (   -0.293,   -0.31, -0.002,   0.0046, 3, 0, 0, -6);
// 	Pert.AddSol (   -0.290,   -1.45, +0.116,  -0.0051, 2, 1, 0, 2);
// 	Pert.AddSol (   -7.649,  -10.56, +0.259,  -0.1038, 2, 1, 0, 0);
// 	Pert.AddSol (   -8.627,   -7.59, +0.078,  -0.0192, 2, 1, 0, -2);
// 	Pert.AddSol (   -2.740,   -2.54, +0.022,   0.0324, 2, 1, 0, -4);
// 	Pert.AddSol (    1.181,    3.32, -0.212,   0.0213, 2, -1, 0, +2);
// 	Pert.AddSol (    9.703,   11.67, -0.151,   0.1268, 2, -1, 0, 0);
// 	Pert.AddSol (   -0.352,   -0.37, +0.001,  -0.0028, 2, -1, 0, -1);
// 	Pert.AddSol (   -2.494,   -1.17, -0.003,  -0.0017, 2, -1, 0, -2);
// 	Pert.AddSol (    0.360,    0.20, -0.012,  -0.0043, 2, -1, 0, -4);
// 	Pert.AddSol (   -1.167,   -1.25, +0.008,  -0.0106, 1, 2, 0, 0);
// 	Pert.AddSol (   -7.412,   -6.12, +0.117,   0.0484, 1, 2, 0, -2);
// 	Pert.AddSol (   -0.311,   -0.65, -0.032,   0.0044, 1, 2, 0, -4);
// 	Pert.AddSol (   +0.757,    1.82, -0.105,   0.0112, 1, -2, 0, 2);
// 	Pert.AddSol (   +2.580,    2.32, +0.027,   0.0196, 1, -2, 0, 0);
// 	Pert.AddSol (   +2.533,    2.40, -0.014,  -0.0212, 1, -2, 0, -2);
// 	Pert.AddSol (   -0.344,   -0.57, -0.025,  +0.0036, 0, 3, 0, -2);
// 	Pert.AddSol (   -0.992,   -0.02, 0.0,   0.0, 1, 0, 2, 2);
// 	Pert.AddSol (  -45.099,   -0.02, 0.0,  -0.0010, 1, 0, 2, 0);
// 	Pert.AddSol (   -0.179,   -9.52, 0.0,  -0.0833, 1, 0, 2, -2);
// 	Pert.AddSol (   -0.301,   -0.33, 0.0,   0.0014, 1, 0, 2, -4);
// 	Pert.AddSol (   -6.382,   -3.37, 0.0,  -0.0481, 1, 0, -2, 2);
// 	Pert.AddSol (   39.528,   85.13, 0.0,  -0.7136, 1, 0, -2, 0);
// 	Pert.AddSol (    9.366,    0.71, 0.0,  -0.0112, 1, 0, -2, -2);
// 	Pert.AddSol (    0.202,    0.02, 0.0,   0.0, 1, 0, -2, -4);
// 	Pert.AddSol (    0.415,    0.10, 0.0,   0.0013, 0, 1, 2, 0);
// 	Pert.AddSol (   -2.152,   -2.26, 0.0,  -0.0066, 0, 1, 2, -2);
// 	Pert.AddSol (   -1.440,   -1.30, 0.0,  +0.0014, 0, 1, -2, 2);
// 	Pert.AddSol (    0.384,   -0.04, 0.0,   0.0, 0, 1, -2, -2);
// 	Pert.AddSol (   +1.938,   +3.60, -0.145,  +0.0401, 4, 0, 0, 0);
// 	Pert.AddSol (   -0.952,   -1.58, +0.052,  -0.0130, 4, 0, 0, -2);
// 	Pert.AddSol (   -0.551,   -0.94, +0.032,  -0.0097, 3, 1, 0, 0);
// 	Pert.AddSol (   -0.482,   -0.57, +0.005,  -0.0045, 3, 1, 0, -2);
// 	Pert.AddSol (    0.681,    0.96, -0.026,   0.0115, 3, -1, 0, 0);
// 	Pert.AddSol (   -0.297,   -0.27, 0.002,  -0.0009, 2, 2, 0, -2);
// 	Pert.AddSol (    0.254,   +0.21, -0.003,   0.0, 2, -2, 0, -2);
// 	Pert.AddSol (   -0.250,   -0.22, 0.004,   0.0014, 1, 3, 0, -2);
// 	Pert.AddSol (   -3.996,    0.0, 0.0,  +0.0004, 2, 0, 2, 0);
// 	Pert.AddSol (    0.557,   -0.75, 0.0,  -0.0090, 2, 0, 2, -2);
// 	Pert.AddSol (   -0.459,   -0.38, 0.0,  -0.0053, 2, 0, -2, 2);
// 	Pert.AddSol (   -1.298,    0.74, 0.0,  +0.0004, 2, 0, -2, 0);
// 	Pert.AddSol (    0.538,    1.14, 0.0,  -0.0141, 2, 0, -2, -2);
// 	Pert.AddSol (    0.263,    0.02, 0.0,   0.0, 1, 1, 2, 0);
// 	Pert.AddSol (    0.426,   +0.07, 0.0,  -0.0006, 1, 1, -2, -2);
// 	Pert.AddSol (   -0.304,   +0.03, 0.0,  +0.0003, 1, -1, 2, 0);
// 	Pert.AddSol (   -0.372,   -0.19, 0.0,  -0.0027, 1, -1, -2, 2);
// 	Pert.AddSol (   +0.418,    0.0, 0.0,   0.0, 0, 0, 4, 0);
// 	Pert.AddSol (   -0.330,   -0.04, 0.0,   0.0, 3, 0, 2, 0);
// 
// 
// 	// Solar perturbations in latitude
// 	Pert.AddN(-526.069, 0, 0, 1, -2);   Pert.AddN(  -3.352, 0, 0, 1, -4);
// 	Pert.AddN( +44.297, +1, 0, 1, -2);   Pert.AddN(  -6.000, +1, 0, 1, -4);
// 	Pert.AddN( +20.599, -1, 0, 1, 0);   Pert.AddN( -30.598, -1, 0, 1, -2);
// 	Pert.AddN( -24.649, -2, 0, 1, 0);   Pert.AddN(  -2.000, -2, 0, 1, -2);
// 	Pert.AddN( -22.571, 0, +1, 1, -2);   Pert.AddN( +10.985, 0, -1, 1, -2);
// 
// 
// 	// Planetary perturbations
// 	Pert.Planetary(T);
// 
// 	// Position vector
// 	return Vector(Pert.dist() * cos(Pert.lambda()) * cos(Pert.beta()), Pert.dist() * sin(Pert.lambda()) * cos(Pert.beta()), Pert.dist() * sin(Pert.beta()));
// 	// return VecPolar(Pert.lambda(),Pert.beta(),Pert.dist());
// }
// 
// //------------------------------------------------------------------------------
// //
// // SunPos: Computes the Sun's ecliptical position using analytical series
// //
// // Input:
// //
// //   T         Time in Julian centuries since J2000
// //
// // <return>:   Geocentric position of the Sun (in [AU]), referred to the
// //             ecliptic and equinox of date
// //
// //------------------------------------------------------------------------------
// Vector SunPos(double T)
// {
// 	//
// 	// Variables
// 	//
// 	double  M2, M3, M4, M5, M6;      // Mean anomalies
// 	double  D, A, U;                 // Mean arguments of lunar orbit
// 	Pert    Ven, Mar, Jup, Sat;      // Perturbations
// 	double  dl, dr, db;              // Corrections in longitude ["],
// 	// radius [AU] and latitude ["]
// 	double  l, b, r;                 // Ecliptic coordinates
// 
// 
// 	// Mean anomalies of planets and mean arguments of lunar orbit [rad]
// 	M2 = pi2 * Frac ( 0.1387306 + 162.5485917 * T );
// 	M3 = pi2 * Frac ( 0.9931266 +  99.9973604 * T );
// 	M4 = pi2 * Frac ( 0.0543250 +  53.1666028 * T );
// 	M5 = pi2 * Frac ( 0.0551750 +   8.4293972 * T );
// 	M6 = pi2 * Frac ( 0.8816500 +   3.3938722 * T );
// 
// 	D  = pi2 * Frac ( 0.8274 + 1236.8531 * T );
// 	A  = pi2 * Frac ( 0.3749 + 1325.5524 * T );
// 	U  = pi2 * Frac ( 0.2591 + 1342.2278 * T );
// 
// 
// 	// Keplerian terms and perturbations by Venus
// 	Ven.Init ( T, M3, 0, 7, M2, -6, 0 );
// 
// 	Ven.Term ( 1, 0, 0, -0.22, 6892.76, -16707.37, -0.54, 0.00, 0.00);
// 	Ven.Term ( 1, 0, 1, -0.06, -17.35,    42.04, -0.15, 0.00, 0.00);
// 	Ven.Term ( 1, 0, 2, -0.01,  -0.05,     0.13, -0.02, 0.00, 0.00);
// 	Ven.Term ( 2, 0, 0, 0.00,  71.98,  -139.57,  0.00, 0.00, 0.00);
// 	Ven.Term ( 2, 0, 1, 0.00,  -0.36,     0.70,  0.00, 0.00, 0.00);
// 	Ven.Term ( 3, 0, 0, 0.00,   1.04,    -1.75,  0.00, 0.00, 0.00);
// 	Ven.Term ( 0, -1, 0, 0.03,  -0.07,    -0.16, -0.07, 0.02, -0.02);
// 	Ven.Term ( 1, -1, 0, 2.35,  -4.23,    -4.75, -2.64, 0.00, 0.00);
// 	Ven.Term ( 1, -2, 0, -0.10,   0.06,     0.12,  0.20, 0.02, 0.00);
// 	Ven.Term ( 2, -1, 0, -0.06,  -0.03,     0.20, -0.01, 0.01, -0.09);
// 	Ven.Term ( 2, -2, 0, -4.70,   2.90,     8.28, 13.42, 0.01, -0.01);
// 	Ven.Term ( 3, -2, 0, 1.80,  -1.74,    -1.44, -1.57, 0.04, -0.06);
// 	Ven.Term ( 3, -3, 0, -0.67,   0.03,     0.11,  2.43, 0.01, 0.00);
// 	Ven.Term ( 4, -2, 0, 0.03,  -0.03,     0.10,  0.09, 0.01, -0.01);
// 	Ven.Term ( 4, -3, 0, 1.51,  -0.40,    -0.88, -3.36, 0.18, -0.10);
// 	Ven.Term ( 4, -4, 0, -0.19,  -0.09,    -0.38,  0.77, 0.00, 0.00);
// 	Ven.Term ( 5, -3, 0, 0.76,  -0.68,     0.30,  0.37, 0.01, 0.00);
// 	Ven.Term ( 5, -4, 0, -0.14,  -0.04,    -0.11,  0.43, -0.03, 0.00);
// 	Ven.Term ( 5, -5, 0, -0.05,  -0.07,    -0.31,  0.21, 0.00, 0.00);
// 	Ven.Term ( 6, -4, 0, 0.15,  -0.04,    -0.06, -0.21, 0.01, 0.00);
// 	Ven.Term ( 6, -5, 0, -0.03,  -0.03,    -0.09,  0.09, -0.01, 0.00);
// 	Ven.Term ( 6, -6, 0, 0.00,  -0.04,    -0.18,  0.02, 0.00, 0.00);
// 	Ven.Term ( 7, -5, 0, -0.12,  -0.03,    -0.08,  0.31, -0.02, -0.01);
// 
// 	dl = Ven.dl(); dr = Ven.dr();  db = Ven.db();
// 
// 
// 	// Perturbations by Mars
// 	Mar.Init ( T, M3, 1, 5, M4, -8, -1 );
// 
// 	Mar.Term ( 1, -1, 0, -0.22,   0.17,    -0.21, -0.27, 0.00, 0.00);
// 	Mar.Term ( 1, -2, 0, -1.66,   0.62,     0.16,  0.28, 0.00, 0.00);
// 	Mar.Term ( 2, -2, 0, 1.96,   0.57,    -1.32,  4.55, 0.00, 0.01);
// 	Mar.Term ( 2, -3, 0, 0.40,   0.15,    -0.17,  0.46, 0.00, 0.00);
// 	Mar.Term ( 2, -4, 0, 0.53,   0.26,     0.09, -0.22, 0.00, 0.00);
// 	Mar.Term ( 3, -3, 0, 0.05,   0.12,    -0.35,  0.15, 0.00, 0.00);
// 	Mar.Term ( 3, -4, 0, -0.13,  -0.48,     1.06, -0.29, 0.01, 0.00);
// 	Mar.Term ( 3, -5, 0, -0.04,  -0.20,     0.20, -0.04, 0.00, 0.00);
// 	Mar.Term ( 4, -4, 0, 0.00,  -0.03,     0.10,  0.04, 0.00, 0.00);
// 	Mar.Term ( 4, -5, 0, 0.05,  -0.07,     0.20,  0.14, 0.00, 0.00);
// 	Mar.Term ( 4, -6, 0, -0.10,   0.11,    -0.23, -0.22, 0.00, 0.00);
// 	Mar.Term ( 5, -7, 0, -0.05,   0.00,     0.01, -0.14, 0.00, 0.00);
// 	Mar.Term ( 5, -8, 0, 0.05,   0.01,    -0.02,  0.10, 0.00, 0.00);
// 
// 	dl += Mar.dl(); dr += Mar.dr();  db += Mar.db();
// 
// 
// 	// Perturbations by Jupiter
// 	Jup.Init ( T, M3, -1, 3, M5, -4, -1 );
// 
// 	Jup.Term (-1, -1, 0, 0.01,   0.07,     0.18, -0.02, 0.00, -0.02);
// 	Jup.Term ( 0, -1, 0, -0.31,   2.58,     0.52,  0.34, 0.02, 0.00);
// 	Jup.Term ( 1, -1, 0, -7.21,  -0.06,     0.13, -16.27, 0.00, -0.02);
// 	Jup.Term ( 1, -2, 0, -0.54,  -1.52,     3.09, -1.12, 0.01, -0.17);
// 	Jup.Term ( 1, -3, 0, -0.03,  -0.21,     0.38, -0.06, 0.00, -0.02);
// 	Jup.Term ( 2, -1, 0, -0.16,   0.05,    -0.18, -0.31, 0.01, 0.00);
// 	Jup.Term ( 2, -2, 0, 0.14,  -2.73,     9.23,  0.48, 0.00, 0.00);
// 	Jup.Term ( 2, -3, 0, 0.07,  -0.55,     1.83,  0.25, 0.01, 0.00);
// 	Jup.Term ( 2, -4, 0, 0.02,  -0.08,     0.25,  0.06, 0.00, 0.00);
// 	Jup.Term ( 3, -2, 0, 0.01,  -0.07,     0.16,  0.04, 0.00, 0.00);
// 	Jup.Term ( 3, -3, 0, -0.16,  -0.03,     0.08, -0.64, 0.00, 0.00);
// 	Jup.Term ( 3, -4, 0, -0.04,  -0.01,     0.03, -0.17, 0.00, 0.00);
// 
// 	dl += Jup.dl(); dr += Jup.dr();  db += Jup.db();
// 
// 
// 	// Perturbations by Saturn
// 	Sat.Init ( T, M3, 0, 2, M6, -2, -1 );
// 
// 	Sat.Term ( 0, -1, 0, 0.00,   0.32,     0.01,  0.00, 0.00, 0.00);
// 	Sat.Term ( 1, -1, 0, -0.08,  -0.41,     0.97, -0.18, 0.00, -0.01);
// 	Sat.Term ( 1, -2, 0, 0.04,   0.10,    -0.23,  0.10, 0.00, 0.00);
// 	Sat.Term ( 2, -2, 0, 0.04,   0.10,    -0.35,  0.13, 0.00, 0.00);
// 
// 	dl += Sat.dl(); dr += Sat.dr();  db += Sat.db();
// 
// 
// 	// Difference of Earth-Moon-barycentre and centre of the Earth
// 	dl += +  6.45 * sin(D) - 0.42 * sin(D - A) + 0.18 * sin(D + A)
// 		  +  0.17 * sin(D - M3) - 0.06 * sin(D + M3);
// 
// 	dr += + 30.76 * cos(D) - 3.06 * cos(D - A) + 0.85 * cos(D + A)
// 		  -  0.58 * cos(D + M3) + 0.57 * cos(D - M3);
// 
// 	db += + 0.576 * sin(U);
// 
// 
// 	// Long-periodic perturbations
// 	dl += + 6.40 * sin ( pi2 * (0.6983 + 0.0561 * T) )
// 		  + 1.87 * sin ( pi2 * (0.5764 + 0.4174 * T) )
// 		  + 0.27 * sin ( pi2 * (0.4189 + 0.3306 * T) )
// 		  + 0.20 * sin ( pi2 * (0.3581 + 2.4814 * T) );
// 
// 
// 	// Ecliptic coordinates ([rad],[AU])
// 	l = pi2 * Frac ( 0.7859453 + M3 / pi2 +
// 					 ( (6191.2 + 1.1 * T) * T + dl ) / 1296.0e3 );
// 	r = 1.0001398 - 0.0000007 * T + dr * 1.0e-6;
// 	b = db / Arcs;
// 
// 	return Vector(r * cos(l) * cos(b), r * sin(l) * cos(b), r * sin(b));
// 	// return Vector(VecPolar(l,b,r));  // Position vector
// }
