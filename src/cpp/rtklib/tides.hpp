
#ifndef __TIDES_HPP__
#define __TIDES_HPP__

#include "eigenIncluder.hpp"
#include "streamTrace.hpp"
#include "gTime.hpp"


//forward declaration
struct erp_t;

/* earth tide models ---------------------------------------------------------*/

void sunmoonpos(
	GTime			tutc,
	const double*	erpv,
	Vector3d*		rsun = nullptr,
	Vector3d*		rmoon = nullptr,
	double*			gmst = nullptr);

void tidedisp(
	Trace&			trace,
	GTime			tutc,
	Vector3d&		recPos,
	const erp_t*	erp,
	const double*	otlDisplacement,
	Vector3d&		dr,
	Vector3d*		solid_ptr   = nullptr,
	Vector3d*		olt_ptr     = nullptr,
	Vector3d*		pole_ptr    = nullptr);



#endif
