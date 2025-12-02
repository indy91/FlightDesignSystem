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

#include <vector>
#include <string>
#include "OrbMech.h"

namespace OMP
{
	const unsigned MAXSECONDARIES = 9;

	class OMPDefs
	{
	public:
		typedef enum { NOMAN, APSO, CIRC, DVPY, DVYP, EXDV, HA, HASH, LSDV, NOSH, PC, NC, NCC, NH, NHRD, NPC, NS, NSR, SOI, SOM, SOR, TPF, TPI, TPM } MANTYPE;
		typedef enum { NOTHR, THRES_APS, THRES_CAN, THRES_DLT, THRES_DT, THRES_DTL, THRES_M, THRES_REV, THRES_T, THRES_N, THRES_WT } THRESHOLD;
		typedef enum {
			NOSEC, A, ALT, APO, SEC_APS, ARG, ASC, CN, DEC, DSC, EL, LAT, LON, N, NA, NP, OPT, P, PER, RAS, TGTA, TGTP, U,
			LITI, LITM, LITO, NITI, NITM, NITO, CXYZ, DH, DNOD, DPC, DR, DV, DVLS, DVLV, HD, ITSR, MREV, SEC_NULL, PHA, PIT, VFIL, WEDG, YAW, ELA
		} SECONDARIES;
		typedef enum { NOTHRU, PX4, PX3, PX2, MXL, YL, MYL, ZH, ZL, MZH, MZL, M1, M2, OL, OR, OBP } THRUSTERS;
		typedef enum { NOGUID, M50, P7 } GUID;
	};

	struct SecData
	{
		OMPDefs::SECONDARIES type = OMPDefs::SECONDARIES::NOSEC;
		double value = 0.0;
	};

	struct ManeuverConstraintsInput
	{
		std::string Name;			//Maneuver name
		std::string Type;			//Maneuver type
		std::string Threshold;		//Threshold type
		std::string ThresholdValue; //Value associated with threshold
		std::string Secondary[9];
		std::string SecondaryValues[9];
	};

	struct ManeuverConstraints
	{
		std::string name;
		OMPDefs::MANTYPE type;
		OMPDefs::THRESHOLD threshold;
		double thresh_num;	//time, delta time or revs
		std::vector<SecData> secondaries;
	};

	struct ITERCONSTR
	{
		int type = 0;			//type of iterator (1 = NC, 2 = NH, 3 = NPC)
		unsigned man = 0;		//maneuver that is applying the DV
		unsigned constr = 0;	//maneuver for which the constraint is applied
		int constrtype = 0;		//For NC: 1 = DR, 2 = PHA
		double value = 0.0;		//Value of the constraint
	};

	struct ITERSTATE
	{
		int s_F = 0;
		double p_H = 0.0;
		double c_I = 0.0;
		double dv = 0.0;
		double dvo = 0.0;
		double err = 0.0;
		double erro = 0.0;
		bool converged = false;
	};

	struct MANEUVER
	{
		std::string name;
		double TIG_GMT;
		VECTOR3 dV_LVLH;
		OMPDefs::MANTYPE type;
	};

	struct MANEVALDATA
	{
		std::string type;
		std::string name;
		double DVMag;
		double GMTIG;
		double METIG;
		double DT;
		VECTOR3 DV;
		double HA;
		double HP;
		double DH;
		double RANGE;
		double PHASE;
		bool noon;
		double TTN;
		double Y;
		double Ydot;
		bool sunrise;
		double TTS;
	};

	struct MANEVALTABLE
	{
		//Header

		//Chaser state vector GMT
		double GMT_C;
		//Targer state vector GMT
		double GMT_T;
		//Total accumulated chaser DV
		VECTOR3 dv_C;
		//Total accumulated chaser DV
		VECTOR3 dv_T;

		std::vector<MANEVALDATA> Maneuvers;
	};

	struct OMPInputs
	{
		//Input state vectors
		OrbMech::StateVector CHASER;
		OrbMech::StateVector TARGET;
		//Table with maneuver constraints
		std::vector<ManeuverConstraints> ManeuverConstraintsTable;
		//Data for the evaluation table
		std::string ProjectFolder, OMPChaserFile, OMPTargetFile, OMPMCTFile;
		//Write output print
		bool PRINT = true;
		//Write output plot
		bool PLOT = false;
	};

	struct OMPOutputs
	{
		MANEVALTABLE ManeuverEvaluationTable;
		int Error;
		std::string ErrorMessage;
		std::vector<std::vector<std::string>> OutputPrint;
	};

	struct OMPVariablesTable
	{
		OMPVariablesTable();

		double ISP[5];
		double RangeTolerance;
	};

	class OrbitalManeuverProcessor
	{
	public:
		OrbitalManeuverProcessor(OrbMech::GlobalConstants& cnst, OrbMech::SessionConstants& scnst);

		//Parse input to internal format
		bool ParseManeuverConstraintsTable(const std::vector< ManeuverConstraintsInput>& tab_in, std::vector < ManeuverConstraints>& tab_out, std::string &errormessage);
		//Run OMP
		void Calculate(const OMPInputs& in, OMPOutputs &out);
	protected:
		int CalculateOMPPlan(const OMPInputs& in);
		void Init(const OMPInputs& in);
		int ProcessIterators();
		int ProcessManeuverConstraints();
		int ProcessTIGModifiers();
		int UpdateToThreshold();
		int UpdateToModfiedTIG();
		int RunIterators();
		int ApplyManeuver();
		bool IsOMPConverged() const;
		void CalculateManeuverEvalTable(OrbMech::StateVector sv_A0, OrbMech::StateVector sv_P0);
		void PrintManeuverEvaluationTable();
		void GetOMPError(int err, std::string& buf, unsigned int i = 0, unsigned int j = 0);

		//Trajectory propagation
		int coast_auto(OrbMech::StateVector sv0, double dt, OrbMech::StateVector& sv1) const;
		int DeltaOrbitsAuto(OrbMech::StateVector sv0, double M, OrbMech::StateVector& sv1) const;
		int GeneralTrajectoryPropagation(OrbMech::StateVector sv0, int opt, double param, double DN, OrbMech::StateVector& sv1) const;
		int timetoapo_auto(OrbMech::StateVector sv_A, double revs, OrbMech::StateVector& sv_out) const;
		int timetoperi_auto(OrbMech::StateVector sv_A, double revs, OrbMech::StateVector& sv_out) const;
		int FindCommonNode(OrbMech::StateVector sv_A, OrbMech::StateVector sv_P, VECTOR3& u_d, double& dt) const;
		int FindNthApsidalCrossingAuto(OrbMech::StateVector sv0, double N, OrbMech::StateVector& sv_out) const;
		int FindOptimumNodeShiftPoint(OrbMech::StateVector sv0, double dh, OrbMech::StateVector& sv_out) const;
		int SEARMT(OrbMech::StateVector sv0, int opt, double val, OrbMech::StateVector& sv1) const;
		int PositionMatch(OrbMech::StateVector sv_A, OrbMech::StateVector sv_P, OrbMech::StateVector& sv_P2) const;
		int QRDTPI(OrbMech::StateVector sv_P, double dh, double E_L, OrbMech::StateVector& sv_P2) const;

		//TIG modifiers
		int Sunrise(OrbMech::StateVector sv0, bool rise, bool midnight, OrbMech::StateVector& sv1) const;
		int FindOrbitalSunriseRelativeTime(OrbMech::StateVector sv0, bool sunrise, double dt1, OrbMech::StateVector& sv_out);
		int FindOrbitalMidnightRelativeTime(OrbMech::StateVector sv0, bool midnight, double dt1, OrbMech::StateVector& sv_out);

		//Maneuvers
		bool HeightManeuverAuto(OrbMech::StateVector sv_A, double r_D, bool horizontal, VECTOR3& DV, double dv_guess = 0.0);
		int Lambert(OrbMech::StateVector sv_A1, VECTOR3 RP2_off, double dt, VECTOR3& V_A1_apo);
		int SOIManeuver(OrbMech::StateVector sv_A1, OrbMech::StateVector sv_P, double dt, VECTOR3 off, VECTOR3& DV);
		int SORManeuver(OrbMech::StateVector sv_A1, OrbMech::StateVector sv_P, VECTOR3 off, VECTOR3& DV);
		VECTOR3 NPCManeuver(OrbMech::StateVector sv_A, VECTOR3 H_P) const;
		double CalculateYDot(VECTOR3 V_A, VECTOR3 R_P, VECTOR3 V_P) const;
		VECTOR3 NSRManeuver(OrbMech::StateVector sv_A, OrbMech::StateVector sv_P) const;
		VECTOR3 NodeShiftManeuver(OrbMech::StateVector sv0, double dh_D) const;
		VECTOR3 PlaneChangeManeuver(OrbMech::StateVector sv0, double dw_D) const;

		//Utilities
		double GMTfromGET(double get) const;
		double GETfromGMT(double gmt) const;
		OrbMech::StateVector ApplyLVLHManeuver(OrbMech::StateVector sv0, VECTOR3 DV_LVLH, int ThrustProfile, bool forwards = true) const;

		std::string GetOPMManeuverType( OMPDefs::MANTYPE type);
		OMPDefs::MANTYPE GetOPMManeuverType(std::string buf);

		OMP::OMPDefs::THRESHOLD GetOPMThresholdType(std::string buf) const;
		int GetOMPThresholdValue(std::string buf, OMP::OMPDefs::THRESHOLD type, double& val) const;
		OMP::OMPDefs::SECONDARIES GetSecondaryType(std::string buf) const;

		void ApsidesMagnitudeDetermination(OrbMech::StateVector sv0, double& r_A, double& r_P) const; //Apogee/Perigee Magnitude Determination

		// INPUTS
		OrbMech::StateVector sv_chaser;
		OrbMech::StateVector sv_target;
		std::vector<ManeuverConstraints> ManeuverConstraintsTable;
		std::string ProjectFolder, OMPChaserFile, OMPTargetFile, OMPMCTFile;

		// INTERNAL

		struct OMPManeuverArray
		{
			OMPManeuverArray()
			{
				ChaserMan = true;
				ThrustProfile = 0;
				dv_table = add_constraint = _V(0, 0, 0);
			}

			//Active vehicle for the maneuver
			bool ChaserMan;
			//Thrust profile (0-4) for the maneuver
			int ThrustProfile;
			//Chaser state vector at threshold time
			OrbMech::StateVector sv_A_threshold;
			//Target state vector at threshold time
			//Chaser state vector at TIG, before maneuver
			OrbMech::StateVector sv_A_bef_table;
			//Chaser state vector at TIG, after maneuver
			OrbMech::StateVector sv_A_aft_table;
			//Target state vector at TIG, before maneuver
			OrbMech::StateVector sv_P_bef_table;
			//Target state vector at TIG, after maneuver
			OrbMech::StateVector sv_P_aft_table;
			//LVLH Delta V of maneuver
			VECTOR3 dv_table;
			//The secondary constraint for TIG
			SecData tigmodifiers;
			//Additional constraints for the maneuver
			VECTOR3 add_constraint;
		};

		int Error;
		//Current maneuver counter
		unsigned int CurMan;
		size_t TAB;
		std::vector<ITERCONSTR> iterators;
		OrbMech::StateVector sv_cur, sv_P_cur;
		std::vector<OMPManeuverArray> ManeuverData;

		//Phantom state vector for the NPC maneuver
		OrbMech::StateVector sv_phantom;
		//Vector of iterations
		std::vector<ITERSTATE> iterstate;
		//Flag set by RunIterators if the iteration has to step back to an earlier maneuver
		bool recycle;
		//Table of maneuver definitions for generating the evaluation table
		std::vector<MANEUVER> ManeuverTable;
		std::vector<std::vector<std::string>> OutputPrint;

		// OUTPUTS
		MANEVALTABLE ManeuverEvaluationTable;

		// CONSTANTS
		OrbMech::GlobalConstants& constants;
		OrbMech::SessionConstants& sesconst;
		OMPVariablesTable ompvariables;
	};

}