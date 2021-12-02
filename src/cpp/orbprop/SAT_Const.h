// //------------------------------------------------------------------------------
// //
// // SAT_Const.h
// // 
// // Purpose:
// //
// //   Definition of astronomical and mathematical constants (in MKS units)
// //
// // Last modified:
// //
// //   2000/03/04  OMO  Final version (1st edition)
// //   2005/04/14  OMO  Final version (2nd reprint)
// //
// // (c) 1999-2005  O. Montenbruck, E. Gill
// //
// //------------------------------------------------------------------------------
// 
// #ifndef INC_SAT_CONST_H
// #define INC_SAT_CONST_H
// 
// //
// // General
// //
// const double MJD_J2000 = 51544.5;             // Modif. Julian Date of J2000.0
// 
// //
// // Physical parameters of the Earth, Sun and Moon
// //
// 
// // Equatorial radius and flattening
// const double R_Earth     =   6378.137e3;      // Radius Earth [m]; WGS-84
// const double f_Earth     = 1.0/298.257223563; // Flattening; WGS-84
// const double R_Sun       = 696000.0e3;        // Sun's radius [m]; DE430
// const double R_Moon      =   1738.0e3;        // Moon's radius [m]; DE430
// 
// // Earth rotation (derivative of GMST at J2000; differs from inertial period by precession)
// const double omega_Earth = 15.04106717866910/3600*Rad; // [rad/s]; WGS-84
// 
// // Gravitational coefficient
// const double GM_Sun      = 132712440041.939400e9;      // [m^3/s^2]; DE430
// const double GM_Moon     = GM_Earth/81.30056907419062; // [m^3/s^2]; DE430
// 
// 
// // Solar radiation pressure at 1 AU
// const double P_Sol       = 1367/c_light; // [N/m^2] (1367 W/m^2); IERS 96
// 
// #endif  // include blocker
