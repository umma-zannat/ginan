// //------------------------------------------------------------------------------
// //
// // APC_Planets.h
// // 
// // Purpose:
// //
// //    Computation of the planets' position
// //
// //
// //------------------------------------------------------------------------------
// 
// #ifndef INC_APC_PLANETS_H
// #define INC_APC_PLANETS_H
// 
// #include "SAT_VecMat.h"
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
// // <return>:   Geocentric position of the Moon (in [km]) referred to the
// //             ecliptic and equinox of date
// //
// // Notes: Light-time is already taken into account
// //
// //------------------------------------------------------------------------------
// Vector MoonPos(double T);
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
// Vector SunPos(double T);
// 
// #endif   // include blocker
