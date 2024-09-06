#pragma once

#include "VectorMath.h"
#include "OrbMech.h"
#include <vector>

struct EnckeIntegratorInput
{
	EnckeIntegratorInput();

	//Position vector
	VECTOR3 R;
	//Velocity vector
	VECTOR3 V;
	//GMT of input
	double GMT;
	//Drag multiplier
	double KFactor;
	//Drag indicator (true = enabled)
	bool DragIndicator;
	//Vehicle weight
	double Weight;
	//Vehicle area
	double Area;
	//Propagation time
	double dt;
	//Ephemeris indicator (true = write ephemeris)
	bool EphemerisIndicator;
};

struct EnckeIntegratorState
{
	//Position vector
	VECTOR3 R;
	//Velocity vector
	VECTOR3 V;
	//GMT
	double GMT;
};

struct EnckeIntegratorOutput
{
	EnckeIntegratorOutput();

	//Position vector
	VECTOR3 R;
	//Velocity vector
	VECTOR3 V;
	//GMT of input
	double GMT;
	//Error indicator. 0 = no error, 1 = trajectory became reentrant, 2 = Kepler error
	int Error;
	//Ephemeris
	std::vector<EnckeIntegratorState> Ephemeris;
};

struct EnckeState
{
	EnckeState();

	double r_c0, v_c0, beta_c0, n_c0;
};

class EnckeIntegrator
{
public:
	EnckeIntegrator(OrbMech::GlobalConstants &cnst);
	void Propagate(const EnckeIntegratorInput& in, EnckeIntegratorOutput& out);
protected:

	void Init(const EnckeIntegratorInput& in);
	void IntegrationTerminationControlRoutine();
	void GaussJackson();
	void RungeKutta();
	void CalculateInitialEnckeState();
	void Rectification();

	//Acceleration
	void EffectiveForcesRoutine();
	void AccelerationRoutine();
	VECTOR3 GeopotentialRoutine() const;
	VECTOR3 DragRoutine(double &ALT) const;

	//Utility
	double fq(double q) const;

	//STATE
	// Time since rectification
	double T;
	// Position vector, dependent variable
	VECTOR3 Y;
	// Velocity vector, first derivative
	VECTOR3 YP;
	// Acceleration vector, second derivative
	VECTOR3 YPP;
	// Time of initial state vector
	double TSTART;
	// Current position and velocity vector
	VECTOR3 RSTATE, VSTATE;
	// State vector at last rectification
	VECTOR3 RBASE, VBASE;
	//Time of last rectification
	double TRECT;
	//Latest conic state vector propagated from R0, V0
	VECTOR3 RTB, RDTB;
	//Actual time associated with state
	double HRS;
	//Altitude
	double ALT;
	EnckeState enckestate;

	//INTERNAL
	//true = stop integration, false = continue
	bool IEND;
	//Step size
	double H, HRK;
	//Temporary for Runge-Kutta
	double HP, HD2, H2D2, H2D8, HD6;
	//Disturbing acceleration
	VECTOR3 A_D;
	//Temporary Runge-Kutta
	VECTOR3 F1, F2, F3, YS, YPS;
	//Error indicator
	int Err;
	//Maximum DT of integration
	double TMAX;
	//Direction indicator
	double HMULT;
	//Ephemeris
	std::vector<EnckeIntegratorState> Ephemeris;

	//CONSTANTS
	//Drag multiplier (-0.5*coefficient of drag*Area/Weight)
	double CSA;
	//Drag indicator (true = enabled)
	bool DragIndicator;
	//Polar axis
	VECTOR3 U_Z;
	//Integration step size constant
	static const double KK;
	//Rectification tolerance
	static const double EPSQR;
	static const double EPSQV;

	OrbMech::GlobalConstants& constants;

	OrbMech::ShepperdMethod shepperd;
};