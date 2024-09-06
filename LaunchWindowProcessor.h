#pragma once

#include <vector>
#include <string>
#include "OrbMech.h"

namespace LWP
{

	struct LWPConstants
	{
		LWPConstants();

		//false = no internal print, true = internal print
		bool PrintFlag;
		//Maximum iterations limit
		int CMAX;
		//First guess change in value of independent variable (used in subroutine ITERV)
		double DX;
		//Transfer angle tolerance, radians
		double tol_trans_ang;
		//Eccentricity tolerance
		double tol_e;
		//Travel angle convergence tolerance, radians
		double tol_trav_ang;
		//Radius iteration tolerance, meters
		double tol_rad;
		//Time iteration tolerance, seconds
		double tol_time;
		//Tolerance on plane change DV for calculating launch window inplane launch points, m/s
		double tol_dv;
	};

	struct LWPInputs
	{
		LWPInputs();

		//Input state vector for target vehicle
		OrbMech::StateVector TRGVEC;
		//Chaser vehicle K-Factor
		double CKFactor;
		//Chaser vehicle reference area
		double CAREA;
		//Chaser vehicle weight at insertion
		double CWHT;
		//Launch window/launch targeting options. 0 = LW only, 1 = LT only, 2 = both
		int LW;
		//Inplane launch window opening and closing times. 0 = inplane opening (north) only, 1 = inplane closing (south) only, 2 = both
		int NS;
		//Day on which launch window times are computed, relative to base date
		int DAY;
		//Launch window parameter table options. 0 = do not generate table, 1 = around launch window opening, 2 = around launch window closing, 3 = around both opening and closing, 4 = through entire window
		int LPT;
		//Delta time prior to inplane time to start parameter table
		double TSTART;
		//Delta time after inplane time to stop parameter table
		double TEND;
		//Time step in parameter table
		double TSTEP;
		//GMTLO* table flag. false = do not compute table, true = compute table
		bool STABLE;
		//Delta time prior to each inplane launch point to start GMTLO* search
		double STARS;
		//Delta time after each inplane launch to end GMTLO* search
		double STARE;
		//Geocentric latitude of launch site
		double LATLS;
		//Geographic longitude of launch site
		double LONGLS;
		//Powered flight time
		double PFT;
		//Powered flight arc
		double PFA;
		//Yaw steering limit
		double YSMAX;
		//Delta time to be subtracted from analytical inplane launch time to obtain empirical inplane launch time
		double DTOPT;
		//DT from lift-off, which defines the time of guidance reference release
		double DTGRR;
		//Launch azimuth coefficients
		double LAZCOE[4];
		//Radius of insertion
		double RINS;
		//Velocity magnitude of insertion
		double VINS;
		//Flightpath angle at insertion
		double GAMINS;
		//Lift-off time options for launch target.
		// 1 = lift-off on input time
		// 2 = compute lift-off time to achieve a desired phase angle (OFFSET) at insertion
		// 3 = lift-off on GMTLO* + BIAS, using GMTLOR as threshold time
		// 4 = lift-off on GMTLO* + BIAS, using TPLANE as threshold time
		// 5 = lift-off based on inplane launch time (GMTLO = TPLANE + TRANS)
		// 6 = iterate to lift-off on inplane launch time based on target orbit phase angle (final GMTLO = TYAW + TRANS)
		int LOT;
		//Recommended or threshold lift-off GMT
		double GMTLOR;
		//Phase angle desired at insertion
		double OFFSET;
		//Bias that is added to GMTLO* to produce lift-off time
		double BIAS;
		//GMT of in-plane lift-off (computed internally when LW = 0 or 2)
		double TPLANE;
		//Delta time added to inplane time to obtain lift-off time
		double TRANS;
		// Insertion cutoff conditions option flag.
		// 1 = Input VINS, GAMINS, RINS
		// 2 = Input GAMINS, RINS and height difference desired at an input angle from insertion
		// 3 = Input GAMINS, RINS and altitude desired at an input angle from insertion
		int INSCO;
		//Desired height difference between chaser and target, or alttiude of chaser, at input angle from insertion
		double DHW;
		//Angle from insertion to obtain a given altitude or delta altitude
		double DU;
		//Nominal semimajor axis at insertion
		double ANOM;
		//Flag for option to compute differential nodal regression from insertion to rendezvous. false = input DELNO, true = compute DELNO
		bool DELNOF;
		//Angle that is added to the target descending node to account for differential nodal regression
		double DELNO;
		//Initial phase angle control flag (0 = 0 to 2*pi, 1 = -2*pi to 0, 2 = -pi to pi)
		int NEGTIV;
		//Flag to wrap initial phase angle. =N, added 2*N*PI to phase angle
		int WRAP;
		//Print flag
		bool PRINT;

		std::string ProjectFolder;
	};

	struct LWPOrbitalElements
	{
		LWPOrbitalElements();

		//Semi-major axis
		double A;
		//Eccentricity
		double E;
		//Inclination
		double I;
		//Longitude of the ascending node
		double H;
		//Mean anomaly
		double L;
		//Mean semi-major axis
		double MA;
		//Mean inclination
		double MI;
		//Mean longitude of the ascending node
		double MH;
		//Rate of longitude of the ascending node
		double HDOT;
		//Rate
		double LDOT;
	};

	struct LWPCommon
	{
		LWPCommon();

		//Time of in-plane lift-off
		double TPLANE;
		//Time of minimum yaw steering lift-off
		double TYAW;
		double OPEN;
		double CLOSE;
		double PAO;
		double PAC;
		//Last GMTLO* time computed
		double GSTAR;
		//Number of GMTLO* times computed
		int K25;
		//GMTLO* time array
		double STAR[10];

		//LAUNCH TARGETING
		//Phase angle
		double PA;
		//Argument of latitude at insertion
		double UINS;
		//GMT of lift-off
		double GMTLO;
		//Azimuth
		double AZL;
		//Gemini parallel azimuth
		double GPAZ;
		//Gemini parallel launch wedge angle
		double YP;
		//Target inclination
		double IIGM;
		//Descending node of chaser
		double DN;
		//Angle measured from launch site meridian to chaser descending node, defined at TGRR
		double TIGM;
		//Rate of change of TIGM, defined at GRR
		double TDIGM;
		//Rate of change of DELNO, defined at insertion
		double DELNOD;
	};

	struct LWPSummaryTable
	{
		LWPSummaryTable();

		int Error; //-1 = No data, 0 = no error, 1+ = errors
		double GMTLO;
		double GETLO;
		double TINS;
		double AZL;
		double VIGM;
		double RIGM;
		double GIGM;
		double IIGM;
		double TIGM;
		double TDIGM;
		double DN;
		double DELNO;
		double PA;
		double TPLANE;
		double LATLS;
		double LONGLS;
		double DTGRR;
	};

	struct LWPOutputs
	{
		LWPOutputs();

		//LAUNCH TARGETING
		//Target vehicle state vector at initial time and insertion
		OrbMech::StateVector PVTABT[2];
		//Chaser vehicle state vector at insertion
		OrbMech::StateVector PVTABC;
		//Output summary table
		LWPSummaryTable SUMTAB;
		//GMT of liftoff
		double GMTLO;

		std::vector < std::vector<std::string>> LWPOutput;
	};

	class LaunchWindowProcessor
	{
	public:
		LaunchWindowProcessor(OrbMech::GlobalConstants& cnst, OrbMech::SessionConstants& scnst);
		void Calculate(const LWPInputs& in, LWPOutputs &out);
	protected:

		// SUBROUTINES
		void LWP(const LWPInputs& in, LWPOutputs &out);
		// Launch Window Processor data input routine
		void LWPIN();
		// Launch Window Time routine
		void LWT();
		// Inplane launch time routine
		void NPLAN(double& TIP);
		// Calculate analytical insertion vector
		void LENSR(double GMTLO, double &WEDGE);
		// Zero phase angle insertion
		void GMTLS(double TI, double TF);
		// Launch Window Display routine
		void LWDSP();
		// Launch Window Parameter Table routine
		void LWPT();
		// Recommended Lift-off Time routine
		void RLOT();
		// RLOT insertion parameters routine
		void NSERT(double GMTLO);
		// Targeting quantities routine
		void TARGT(double GMTLO);
		// Recommended Lift-off Time display routine
		void RLOTD();
		// Launch Window Processor Output Tables routine
		void LWPOT(LWPOutputs &out);
		// State vector display routine
		void SVDSP();

		// State vector update routine
		void UPDAT(bool chaser, double T);
		// Phase angle
		double PHANG(bool wrapped = false) const;
		// Travel angle
		double TTHET(double PH) const;
		// Travel angle to argument of latitude
		double TARGL(double UOC) const;
		// Gemini parallel launch azimuth
		double GeminiParallelLaunchAzimuth(double TLO, double i_c, double h_c, double DTheta_dot) const;

		// INPUTS
		LWPInputs inp;

		// INTERNAL
		// Working chaser state vector
		OrbMech::StateVector sv_C;
		LWPOrbitalElements coe_C;
		// Working target state vector
		OrbMech::StateVector sv_T;
		LWPOrbitalElements coe_T;
		//
		int Error;
		LWPCommon common;

		// OUTPUT
		std::vector < std::vector<std::string>> LWPOutput;

		// CONSTANTS
		OrbMech::GlobalConstants& globconst;
		OrbMech::SessionConstants& sesconst;
		LWPConstants lwpconst;
	};

}