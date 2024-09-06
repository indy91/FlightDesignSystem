/***************************************************************************
This file is part of the Apollo/Shuttle Flight Design System

Copyright (C) 2024 Niklas Beug

This program is free software: you can redistribute it and/or modify it under the terms of
the GNU General Public License as published by the Free Software Foundation, version 3.

This program is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this
program. If not, see <https://www.gnu.org/licenses/>.

**************************************************************************/

#pragma once

#include "VectorMath.h"

namespace OrbMech
{
	//Matrix converting J2000 to M50 (right-handed)
	const MATRIX3 M_J2000_to_M50 = _M(9.999257079515327e-01, 1.218927600080109e-02, 1.132264483832182e-05, -1.117893818888212e-02,
		9.174139277929019e-01, -3.977772195998275e-01, -4.859003868607689e-03, 3.977475413402031e-01, 9.174820343958939e-01);
	const double LAN_MJD = 51544.5;				//MJD of the LAN in the "beginning", J2000

	struct StateVector
	{
		StateVector();

		VECTOR3 R;
		VECTOR3 V;
		double GMT;
		double Weight;
		double Area;
		double KFactor;
	};

	struct OELEMENTS
	{
		double h = 0.0;
		double e = 0.0;
		double i = 0.0;
		double RA = 0.0;
		double w = 0.0;
		double TA = 0.0;
	};

	//Classical elements
	struct CELEMENTS
	{
		//Semi-major axis
		double a = 0.0;
		//Eccentricity
		double e = 0.0;
		//Inclination
		double i = 0.0;
		//Longitude of the ascending node
		double h = 0.0;
		//Argument of pericenter
		double g = 0.0;
		//Mean Anomaly
		double l = 0.0;
	};

	struct GlobalConstants
	{
		//Specific gravitational parameter and its square root
		double mu;
		double sqrt_mu;
		//Earth radius for gravity
		double R_E;
		//Earth equatorial radius
		double R_E_equ;
		//Rotational rate of the Earth
		double w_E;
		//J-coefficients
		double J2, J3, J4;
		//Square of equatorial and polar Earth radius, for Fischer ellipsoid
		double a_sq, b_sq;

		//Precession
		double T_p;		//Precession Period
		double L_0;		//LAN in the "beginning"
		double e_rel;	//Obliquity / axial tilt of the earth in radians
		double phi_0;	//Sidereal Rotational Offset
		double T_s;		//Sidereal Rotational Period
		MATRIX3 R_ref;
		MATRIX3 R_obl;
	};

	struct SessionConstants
	{
		//MJD at midnight before launch, days
		double GMTBASE;
		//Rotation matrix from TEG (true-equator and Greenwich meridian of date) to M50, right handed
		MATRIX3 M_TEG_TO_M50;
		//Rotation matrix from TEG (true-equator and Greenwich meridian of date) to J2000, right handed
		MATRIX3 M_TEG_TO_J2000;
		//GMT of liftoff on launch day, seconds
		double GMTLO;
		//Date
		int Year;
		int Month;
		int Day;
		int DayOfYear;
		int Hours;
		int Minutes;
		//Seconds of liftoff
		double launchdateSec;
	};

	//Gautschi's method for solving Gaussian continued fraction, used for Kepler's problem in time
	class GautschiMethod1
	{
	public:
		GautschiMethod1(double t = 1.0e-12);

		double Run(double q);
	protected:
		double A, B, G, GSAV;
		int n, l, d, k, iter;

		const double tol;
	};

	//Gautschi's method for solving Shepperd's w function, used for Kepler's problem in eccentric anomaly
	class GautschiMethod2
	{
	public:
		GautschiMethod2(double t = 1.0e-12);

		double Run(double alpha, double beta);
	protected:
		double x, p, u, s;
		int l, ltemp, d, iter;

		const double tol;
	};

	//Shepperd's method for solving Keplers problem
	class ShepperdMethod
	{
	public:
		ShepperdMethod(int imax = 100);

		//With input state vector
		int Run(VECTOR3 R0, VECTOR3 V0, double DT, double mu, VECTOR3& R1, VECTOR3& V1);
		//When the properties of the orbit are already known
		int Run2(double r0, double v0, double beta, double n0, double DT, double mu, double& f, double& g, double& F, double& G);
		
	protected:
		int InnerLoop();
		int KeplerIteration();

		double mu, DT;
		double q, U, U0, U1, U2, U3;
		double r0;
		double v0;
		double n0; //Angular momentum
		double rf; //Final position magnitude
		double beta;
		double u;
		double t;
		int iter;
		int err;
		double f, g, F, G;
		double DU;
		double umax;
		double umin;
		double du;
		double tsav;
		double terr;

		const double tol; //Time tolerance
		const int iter_max; //Maximum number of iterations

		GautschiMethod1 Gautschi;
	};

	void rv_from_r0v0(VECTOR3 R0, VECTOR3 V0, double t, VECTOR3& R1, VECTOR3& V1, double mu);
	int time_theta(VECTOR3 R1, VECTOR3 V1, double dtheta, double mu, double& dt);
	int time_theta(VECTOR3 R1, VECTOR3 V1, double dtheta, double mu, VECTOR3& R2, VECTOR3& V2, double& dt);

	VECTOR3 elegant_lambert(VECTOR3 R1, VECTOR3 V1, VECTOR3 R2, double dt, int N, bool prog, double mu);

	//1962 US standard atmosphere
	void GLFDEN(double Z, double& DEN, double& CS);

	//Analytical sun ephemeris
	VECTOR3 SUN(double MJD);
	VECTOR3 SUN(double GMTBASE, double GMT, const MATRIX3 &RM);

	//Convert left-handed to right-handed matrix
	MATRIX3 MatrixRH_LH(MATRIX3 A);
	//Transpose of matrix
	MATRIX3 tmat(MATRIX3 a);
	//Rotation matrix from inertial to LVLH
	MATRIX3 LVLH_Matrix(VECTOR3 R, VECTOR3 V);

	CELEMENTS CartesianToKeplerian(VECTOR3 R, VECTOR3 V, double mu);
	OELEMENTS coe_from_sv(VECTOR3 R, VECTOR3 V, double mu);
	void sv_from_coe(OELEMENTS el, double mu, VECTOR3& R, VECTOR3& V);	//computes the state vector(R, V) from the classical orbital elements
	double TrueToMeanAnomaly(double ta, double eccdp);
	double TrueToEccentricAnomaly(double ta, double ecc);
	double MeanToTrueAnomaly(double meanAnom, double eccdp, double error2 = 1e-12);
	double kepler_E(double e, double M, double error2 = 1.e-8);
	VECTOR3 PROJCT(VECTOR3 U1, VECTOR3 U2, VECTOR3 X);
	double PHSANG(VECTOR3 R, VECTOR3 V, VECTOR3 RD);
	double ArgLat(VECTOR3 R, VECTOR3 V);
	void KeplerianToCartesian(CELEMENTS coe, double mu, VECTOR3& R, VECTOR3& V);

	bool TLAT(VECTOR3 R, VECTOR3 V, double lat, int C, double& K_AD, double& dtheta);
	double TLON(VECTOR3 R, VECTOR3 V, double t, double lng, int C, double w_E);
	bool TALT(VECTOR3 R, VECTOR3 V, double rad, int C, double mu, double& dtheta);

	double GetSemiMajorAxis(VECTOR3 R, VECTOR3 V, double mu);
	double GetMeanMotion(VECTOR3 R, VECTOR3 V, double mu);
	double REVTIM(VECTOR3 R, VECTOR3 V, const GlobalConstants& cnst);
	void periapo(VECTOR3 R, VECTOR3 V, double mu, double& apo, double& peri);

	void REL_COMP(VECTOR3 R_T_INER, VECTOR3 V_T_INER, VECTOR3& R_S_INER, VECTOR3& R_REL);
	void REL_COMP(bool INER_TO_LVC, VECTOR3 R_T_INER, VECTOR3 V_T_INER, VECTOR3& R_S_INER, VECTOR3& V_S_INER, VECTOR3& R_REL, VECTOR3& V_REL);
	void RADUP(VECTOR3 R_W, VECTOR3 V_W, VECTOR3 R_C, double mu, VECTOR3& R_W1, VECTOR3& V_W1);

	void ITER(double& c, int& s, double e, double& p, double& x, double& eo, double& xo, double dx0 = 1.0);
	void PIFAAP(double a, double e, double i, double f, double u, double r, double R_E, double J2, double& r_A, double& r_P);
	void SecularRates(StateVector sv, CELEMENTS coe, const GlobalConstants& cnst, double& ldot, double& gdot);

	double normalize_angle(const double value, const double start, const double end);

	//Time conversions
	int Date2JD(int Y, int M, int D);
	double Date2MJD(int Y, int D, int H, int M, double S);
	int dayofyear(int year, int month, int day);
	bool isleapyear(int a);

	//Output formatting
	void SS2HHMMSS(double val, double& hh, double& mm, double& ss);
	void GMT2String(char* buf, double GMT, int Day);
	void GMT2String3(char* buf, double GMT, int Day);
	void GMT2String4(char* buf, double GMT);
	void MET2String(char* buf, double MET);

	MATRIX3 GetRotationMatrix(const GlobalConstants &cnst, double t);
	MATRIX3 GetObliquityMatrix(const GlobalConstants& cnst, double t);

	double acos2(double _X);
}
