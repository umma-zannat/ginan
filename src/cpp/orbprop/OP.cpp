// //------------------------------------------------------------------------------
// //
// // High Precision Orbit Propagator
// //
// //
// // Last modified:
// //
// //   2000/03/04  OMO  Final version (1st edition)
// //   2005/04/14  OMO  Final version (2nd reprint)
// //
// // (c) 1999-2008  O. Montenbruck, E. Gill and Meysam Mahooti
// //
// //------------------------------------------------------------------------------
// 
// #include <iostream>
// #include <iomanip>
// #include <cmath>
// #include <fstream>
// #include <ctime>
// 
// #include "GNU_iomanip.h"
// 
// #include "SAT_Const.h"
// #include "SAT_DE.h"
// #include "SAT_Force.h"
// #include "SAT_RefSys.h"
// #include "SAT_Time.h"
// #include "SAT_VecMat.h"
// #include "APC_Planets.h"
// #include "eopspw.h"
// 
// using namespace std;
// 
// //------------------------------------------------------------------------------
// //
// // Global types and data
// //
// //------------------------------------------------------------------------------
// Matrix cnm(361, 361), snm(361, 361);
// const double R_ref = 6378.1363e3;   // Earth's radius [m]; GGM03S
// const double GM_ref = 398600.4415e9; // [m^3/s^2]; GGM03S
// eopdata eoparr[eopsize];
// int dat;
// char interp = 'l';
// double jd, jdeopstart, mfme, dut1, lod, xp, yp, ddpsi, ddeps, dx, dy, x, y, s,
// 	   deltapsi, deltaeps;
// double jdspwstart, f107a, f107, f107bar, ap, avgap, kp, sumkp, aparr[8], kparr[8];
// 
// // Record for passing global data between Deriv and the calling program
// struct AuxParam
// {
// 	double  Mjd0_TT;
// 	double  Area_drag, Area_solar, mass, CR, CD;
// 	int     n, m;
// 	bool    Sun, Moon, SRad, Drag;
// };
// 
// //------------------------------------------------------------------------------
// //
// // Accel
// //
// // Purpose:
// //
// //   Computes the acceleration of an Earth orbiting satellite due to
// //    - the Earth's harmonic gravity field,
// //    - the gravitational perturbations of the Sun and Moon
// //    - the solar radiation pressure and
// //    - the atmospheric drag
// //
// // Input/Output:
// //
// //   Mjd_TT      Terrestrial Time (Modified Julian Date)
// //   r           Satellite position vector in the ICRF/EME2000 system
// //   v           Satellite velocity vector in the ICRF/EME2000 system
// //   Area_drag   Cross-section
// //   Area_solar  Cross-section
// //   mass        Spacecraft mass
// //   CR          Radiation pressure coefficient
// //   CD          Drag coefficient
// //   <return>    Acceleration (a=d^2r/dt^2) in the ICRF/EME2000 system
// //
// //------------------------------------------------------------------------------
// Vector Accel(double Mjd_TT, const Vector& r, const Vector& v, double Area_drag,
// 			 double Area_solar, double mass, double CR, double CD, int n, int m,
// 			 bool FlagSun, bool FlagMoon, bool FlagSRad, bool FlagDrag)
// {
// 	double Mjd_UTC, Mjd_UT1;
// 	double T1;     // Julian cent. since J2000
// 	Vector a(3), r_Sun(3), r_Moon(3);
// 	Matrix P(3, 3), N(3, 3), T(3, 3), E(3, 3);
// 	char fluxtype, f81type, inputtype;
// 
// 	// Acceleration due to harmonic gravity field
// 	Mjd_UTC = Mjd_TT - IERS::TT_UTC(Mjd_UTC) / 86400.0;
// 	Mjd_UT1 = Mjd_UTC + IERS::UT1_UTC(Mjd_UTC) / 86400.0;
// 
// 	jd = Mjd_UTC + 2400000.5;
// 	mfme = 1440.0 * (Mjd_UTC - floor(Mjd_UTC));
// 
// 	findeopparam(jd, mfme, interp, eoparr, jdeopstart, dut1, dat, lod, xp, yp,
// 				 ddpsi, ddeps, dx, dy, x, y, s, deltapsi, deltaeps);
// 
// 	IERS::Set(dut1, -dat, xp, yp);
// 
// 	P = PrecMatrix(MJD_J2000, Mjd_TT);
// 	N = NutMatrix(Mjd_TT);
// 	T = N * P;
// 	E = GHAMatrix(Mjd_UT1) * T;
// 	// E = PoleMatrix(Mjd_UTC) * GHAMatrix(Mjd_UT1) * T;
// 
// 	a = AccelHarmonic(r, E, GM_ref, R_ref, cnm, snm, n, m);
// 
// 	// Luni-solar perturbations
// //  r_Sun  = Sun(Mjd_TT);
// //  r_Moon = Moon(Mjd_TT);
// 	T1   = (Mjd_TT - MJD_J2000) / 36525.0;
// 	r_Sun  = AU * Transp(EclMatrix(Mjd_TT) * P) * SunPos(T1);
// 	r_Moon = Transp(EclMatrix(Mjd_TT) * P) * MoonPos(T1);
// 
// 	if (FlagSun)  a += AccelPointMass( r, r_Sun,  GM_Sun  );
// 	if (FlagMoon) a += AccelPointMass( r, r_Moon, GM_Moon );
// 
// 	// Solar radiation pressure
// 	if (FlagSRad) a += AccelSolrad( r, r_Sun, Area_solar, mass, CR, P_Sol, AU );
// 
// 	// Atmospheric drag
// 	if (FlagDrag) a += AccelDrag( Mjd_TT, r, v, T, E, Area_drag, mass, CD );
// 
// 	// Acceleration
// 	return a;
// }
// 
// //------------------------------------------------------------------------------
// //
// // Deriv
// //
// // Purpose:
// //
// //   Computes the derivative of the state vector
// //
// // Note:
// //
// //   pAux is expected to point to a variable of type AuxDataRecord, which is
// //   used to communicate with the other program sections and to hold data
// //   between subsequent calls of this function
// //
// //------------------------------------------------------------------------------
// void Deriv(double t, const Vector& y, Vector& yp, void* pAux)
// {
// 	// Pointer to auxiliary data record
// 	AuxParam* p = static_cast<AuxParam*>(pAux);
// 
// 	// Time
// 	double  Mjd_TT = (*p).Mjd0_TT + t / 86400.0;
// 
// 	// State vector components
// 	Vector r = y.slice(0, 2);
// 	Vector v = y.slice(3, 5);
// 
// 	// Acceleration
// 	Vector a(3);
// 
// 	a = Accel(Mjd_TT, r, v, (*p).Area_drag, (*p).Area_solar, (*p).mass, (*p).CR, (*p).CD,
// 			  (*p).n, (*p).m, (*p).Sun, (*p).Moon, (*p).SRad, (*p).Drag);
// 
// 	// State vector derivative
// 	yp = Stack ( v, a );
// };
// 
// //------------------------------------------------------------------------------
// //
// // Ephemeris computation
// //
// //------------------------------------------------------------------------------
// void Ephemeris(const Vector& Y0, int N_Step, double Step, AuxParam p, Vector Eph[])
// {
// 	int       i;
// 	double    t, t_end;
// 	double    relerr, abserr;       // Accuracy requirements
// 	DE        Orb(Deriv, 6, &p);    // Object for integrating the eq. of motion
// 	Vector    Y(6);
// 
// 	relerr = 1.0e-13;
// 	abserr = 1.0e-6;
// 	t      = 0.0;
// 	Y      = Y0;
// 	Orb.Init(t, relerr, abserr);
// 	for (i = 0; i <= N_Step; i++)
// 	{
// 		t_end = Step * i;
// 		Orb.Integ ( t_end, Y);
// 		Eph[i] = Y;
// 	};
// }
// 
// //------------------------------------------------------------------------------
// //
// // Maximum computation
// //
// //------------------------------------------------------------------------------
// template<class T> const T& Max (const T& a, const T& b)
// {
// 	return (a < b) ? b : a;
// }
// 
// //------------------------------------------------------------------------------
// //
// // Main program
// //
// //------------------------------------------------------------------------------
// int main()
// {
// 	cout << "\n      High Precision Orbit Propagator     \n" << endl;
// 
// 	clock_t start;
// 	start = clock();
// 
// 	// Variables
// 	double    Mjd_UTC;
// 	Vector    Y0(6), Y(6);
// 	Vector    Kep(6);
// 	AuxParam  Aux;     // Auxiliary parameters
// 
// 	ifstream inp;
// 	inp.open("GGM03S.txt");// this code opens the Earth gravity models file to read initial vector
// 	int z = 0, n = 360;
// 	double temp;
// 	int Year, Month, Day, Hour, Min;
// 	double Sec;
// 
// 	do
// 	{
// 		for (int x = 0; x <= z; x++)
// 		{
// 			inp >> temp;
// 			inp >> temp;
// 			inp >> temp;
// 			cnm(z, x) = temp;
// 			inp >> temp;
// 			snm(z, x) = temp;
// 			inp >> temp;
// 			inp >> temp;
// 		}  z++;
// 	}
// 	while (z <= n);
// 	inp.close();
// 
// 	// Epoch state (Envisat)
// 	Mjd_UTC = Mjd(2008, 8, 1, 0, 0, 0.0);
// 	jd = Mjd_UTC + 2400000.5;
// 	mfme = 1440.0 * (Mjd_UTC - floor(Mjd_UTC));
// 
// 	initeop(eoparr, jdeopstart);
// 
// 	findeopparam(jd, mfme, interp, eoparr, jdeopstart, dut1, dat, lod, xp, yp,
// 				 ddpsi, ddeps, dx, dy, x, y, s, deltapsi, deltaeps);
// 
// 	// Initialize UT1-UTC and UTC-TAI time difference
// 	IERS::Set(dut1, -dat, xp, yp);
// 
// 	//Envisat
// 	inp.open("InitialState.txt");
// 	for (int j = 0; j < 6; j++)
// 	{
// 		inp >> temp;
// 		Y0(j) = 1e3 * temp;
// 	}
// 	inp.close();
// 
// 	// Model parameters
// 	Aux.Mjd0_TT    = Mjd_UTC + IERS::TT_UTC(Mjd_UTC) / 86400.0;
// 	Aux.Area_drag  = 55.64;  // [m^2]
// 	Aux.Area_solar = 88.4;   // [m^2]
// 	Aux.mass       = 8000.0; // [kg]
// 	Aux.CR         = 1.0;
// 	Aux.CD         = 2.7;
// 	Aux.n          = 20;
// 	Aux.m          = 20;
// 	Aux.Sun        = false;
// 	Aux.Moon       = false;
// 	Aux.SRad       = false;
// 	Aux.Drag       = false;
// 
// 	// Initial values
// 	double Step = 300; // [s]
// 	const int N_Step = 2016; // 7 days
// 	Vector Eph [N_Step + 1];
// 
// 	Ephemeris(Y0, N_Step, Step, Aux, Eph);
// 
// //c-standard codes as below
// 	FILE* f;
// 
// 	if ((f = fopen("SatelliteStates2.txt", "w+")) == nullptr)
// 	{
// 		fprintf(stdin, "Can't open \"words\" file.\n");
// 		exit(1);
// 	}
// 	for (int i = 0; i <= N_Step; i++)
// 	{
// 		Y = 1e-3 * Eph[i];
// 		//  CalDat((Mjd_UTC+(Step*i)/86400.0), Year, Month, Day, Hour, Min, Sec);
// 
// 		//  fprintf(f,"%4d/%02d/%02d-",Year,Month,Day);
// 		//  fprintf(f,"%02d:%02d:%6.3f",Hour,Min,Sec);
// 
// 		fprintf(f, "%10.6f", Mjd_UTC + (Step * i) / 86400.0); //modified julian date
// 
// 		for (int j = 0; j < 3; j++)
// 		{
// 			fprintf(f, "%15.6f", Y(j));
// 		}
// 		for (int j = 3; j < 6; j++)
// 		{
// 			fprintf(f, "%15.9f", Y(j));
// 		}
// 		fprintf(f, "\n");
// 	};
// 	fclose(f);
// 
// 	clock_t end;
// 	end = clock();
// 	printf("\n     CPU time used: %f seconds\n", double((end - start) / CLOCKS_PER_SEC));
// 
// 	return 0;
// }
