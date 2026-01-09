/***************************************************************************
This file is part of the Apollo/Shuttle Flight Design System

Copyright (C) 2025 Niklas Beug

This program is free software: you can redistribute it and/or modify it under the terms of
the GNU General Public License as published by the Free Software Foundation, version 3.

This program is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this
program. If not, see <https://www.gnu.org/licenses/>.

**************************************************************************/

#pragma once

#include "OrbMech.h"

namespace ShuttleLWP
{
	struct OMSTargetSet
	{
		double DTIG;
		double C1;
		double C2;
		double HTGT;
		double THETA;
	};

	struct ShuttleLWPInputs
	{
		ShuttleLWPInputs();

		// Target state vector
		OrbMech::StateVector sv_T;
		// Chaser vehicle CD
		double CCD;
		// Chaser vehicle reference area
		double CAREA;
		// Chaser vehicle weight at insertion
		double CWHT;
		// Geocentric latitude of launch site
		double LATLS;
		// Geographic longitude of launch site
		double LONGLS;
		// Powered flight time
		double PFT;
		// Powered flight arc
		double PFA;
		// Yaw steering limit
		double YSMAX;
		// Radius of insertion
		double RINS;
		// Velocity magnitude of insertion
		double VINS;
		// Flightpath angle at insertion
		double GAMINS;
		// Delta time to be subtracted from analytical inplane launch time to obtain empirical inplane launch time
		double DTOPT;
		// Delta time for launch window opening
		double DTO;
		// Delta time for launch window closing
		double DTC;
		// Launch azimuth flag. 0 = northerly, 1 = southerly
		int NS;
		// Initial phase angle control flag (0 = 0 to 2*pi, 1 = -2*pi to 0, 2 = -pi to pi)
		int NEGTIV;
		// Flag to wrap initial phase angle. =N, added 2*N*PI to phase angle
		int WRAP;
		// Calculate phase angles at OMS-2
		bool OMS2Phase[2];
		// Desired phase angles at OMS-2
		double OMS2PhaseAngle[2];
		// Launch targeting GMT of liftoff
		double GMTLO;

		// DTIG between MECO and ET SEP
		double DTIG_ET_SEP;
		// ET SEP DV
		VECTOR3 DV_ET_SEP;
		// DTIG between MECO and MPS Dump
		double DTIG_MPS;
		// MPS Dump DV
		VECTOR3 DV_MPS;
		// false = Standard Insertion, true = Direct Insertion
		bool DirectInsertion;

		OMSTargetSet OMS1, OMS2;

		// External tank data
		double ET_Area;
		double ET_CD;
		double ET_Weight;
	};

	struct ShuttleLWPOutputs
	{
		double GMTLO_OPT;
		double PHASE_OPT;
		double GMTLO_OPEN;
		double PHASE_OPEN;
		double GMTLO_CLOSE;
		double PHASE_CLOSE;
		double GMTLO_PHASE[2];
	};

	struct ShuttleLTPOutputs
	{
		int Error = -1; //-1 = No data, 0 = no error, 1+ = errors

		double GMTLO;
		double MET_MECO;
		double VMECO;
		double RMECO;
		double GMECO;
		double IMECO;
		double PHASE_MECO;
		double HA_MECO;
		double HP_MECO;
		double LONG_MECO;
		double OMS1_TIG;
		double OMS1_DV;
		double OMS1_HA;
		double OMS1_HP;
		double OMS2_TIG;
		double OMS2_DV;
		double OMS2_HA;
		double OMS2_HP;
		double OMS2_NODE;
		double OMS2_PHASE;
		double OMS2_PERIOD;
		double TGT_HA;
		double TGT_HP;
		double TGT_LONG;
		double TGT_DELN;
		double TGT_PERIOD;

		// State vectors
		OrbMech::StateVector sv_C, sv_T, sv_C_OMS1;

		// Vectors
		VECTOR3 IY_MECO;
		VECTOR3 IY_OMS1;
		VECTOR3 IY_OMS2;

		// Reference data set
		OMSTargetSet OMS1, OMS2;

		// Additional data
		double NODE_SLOPE;
	};

	class PEG4A
	{
	public:
		PEG4A(double RE, double MU, double J2, double J3, double J4);
		int OMSBurnPrediction(VECTOR3 RGD, VECTOR3 VGD, double TGD, VECTOR3 RLS_TEG, double C1, double C2, double HTGT, double THETA, double FT, double VEX, double M);
		void GetOutputA(VECTOR3& RP, VECTOR3& VD, VECTOR3& VGO, double& TGO, double& MBO) const;
	private:
		// PEG SUBROUTINES
		VECTOR3 HTHETA_M50_TGT_TSK(VECTOR3 RLS_TEG) const;
		int VelocityToBeGainedSubtask();
		void TimeToGoSubtask();
		void ThrustIntegralSubtask();
		void ReferenceThrustVectorsSubtask();
		void BurnoutStateVectorPredictionSubtask();
		void ConvergenceCheckSubtask();

		// UTILITIES
		int LTVCON_TSK(VECTOR3 RD, VECTOR3 R1, VECTOR3 IY, double C1, double C2, VECTOR3& VD) const;
		void ASCENT_PRECISE_PREDICTOR(VECTOR3 R_INIT, VECTOR3 V_INIT, double T_INIT, double T_FINAL, double DT_MAX, int GMD_PRED, VECTOR3& R_FINAL, VECTOR3& V_FINAL) const;
		void CENTRAL(VECTOR3 R, VECTOR3& ACCEL, double& R_INV) const;
		VECTOR3 ACCEL_ENTRY(VECTOR3 R) const;

		// INPUT

		// Ignition state vector
		VECTOR3 RGD, VGD;
		double TGD;
		// PEG target
		double C1, C2, HTGT, THETA;
		double FT, VEX, M;

		// LOCAL
		// Target position vector
		VECTOR3 RT;
		// Cutoff state vector
		VECTOR3 RP, VP;
		double TP;
		double TGO;
		VECTOR3 VGO;
		double THETA_DOT, LAMDXZ;
		bool SCONV;
		VECTOR3 IY;
		VECTOR3 VMISS;
		double VGOMAG;
		double VRATIO;
		double ATR;
		double JOL, S, QPRIME;
		VECTOR3 LAM, LAMD;
		VECTOR3 RC1, VC1, RC2, VC2;
		double MBO;

		// OUTPUT
		VECTOR3 VD;

		// CONSTANTS

		// Earth radius
		double RADIUS_EARTH_EQUATOR;
		double EARTH_MU;
		double JArr[3];
		const double KMISS = 0.01;
		const double K_LAMDXZ = 0.0;
		// Mean equatorial radius
		const double RE = 6378166.0;
	};

	class ShuttleLaunchWindowProcessor
	{
	public:
		ShuttleLaunchWindowProcessor(OrbMech::GlobalConstants& cnst, OrbMech::SessionConstants& scnst);

		int CalculateLW(const ShuttleLWPInputs& in, ShuttleLWPOutputs& out);
		int CalculateLT(const ShuttleLWPInputs& in, ShuttleLTPOutputs& out);
	protected:

		// State vector propagation
		int Update(const OrbMech::StateVector& sv1, double dt, OrbMech::StateVector& sv2) const;
		// Update state vector to given argument of latitude
		int UpdateToArgumentOfLatitude(const OrbMech::StateVector& sv1, double u_desired, OrbMech::StateVector& sv2) const;
		// Simulate from MECO to OMS-2
		int OMS2();
		// Get inertial launch site vector
		VECTOR3 GetURLS(double GMTLO) const;
		// Phase angle
		double PHANG(const OrbMech::StateVector &sv_C, const OrbMech::StateVector &sv_T, bool wrapped) const;
		// Node
		double CalculateNode(const OrbMech::StateVector& sv_C) const;
		// Calculate node slope
		double NodeSlope(const OrbMech::StateVector& sv_C, const OrbMech::StateVector& sv_T) const;

		// INPUTS
		ShuttleLWPInputs inp;

		// OUTPUTS
		OrbMech::StateVector sv_MECO, sv_ET_Sep_pre, sv_ET_Sep_post, sv_OMS1_pre, sv_OMS1_post, sv_OMS2_pre, sv_OMS2_post;
		VECTOR3 VGO_OMS1, VGO_OMS2;
		double PA_OMS2;

		// CONSTANTS
		OrbMech::GlobalConstants& globconst;
		OrbMech::SessionConstants& sesconst;
	};

}