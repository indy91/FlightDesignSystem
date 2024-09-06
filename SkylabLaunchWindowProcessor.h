#pragma once

#include "OrbMech.h"
#include "OrbitalManeuverProcessor.h"

namespace SkylabLWP
{

	struct SkylabLWPInputs
	{
		SkylabLWPInputs();

		//Target state vector
		OrbMech::StateVector sv_T;

		//Chaser vehicle K-Factor
		double CKFactor;
		//Chaser vehicle reference area
		double CAREA;
		//Chaser vehicle weight at insertion
		double CWHT;
		//Inplane launch window opening and closing times. 0 = inplane opening (north) only, 1 = inplane closing (south) only
		int NS;
		//Day on which launch window times are computed, relative to base date
		int DAY;
		//Geocentric latitude of launch site
		double LATLS;
		//Geographic longitude of launch site
		double LONGLS;
		//Time of powered flight
		double PFT;
		//Powered flight arc
		double PFA;
		//Yaw steering limit
		double YSMAX;
		//Delta time to be subtracted from analytical inplane launch time to obtain empirical inplane launch time
		double DTOP;
		//Radius of insertion
		double RINS;
		//Velocity magnitude of insertion
		double VINS;
		//Flightpath angle at insertion
		double GAMINS;
		//Initial phase angle control flag (0 = 0 to 2*pi, 1 = -2*pi to 0, 2 = -pi to pi)
		int NEGTIV;
		//Flag to wrap initial phase angle. =N, added 2*N*PI to phase angle
		int WRAP;
		//Delta time from optimum inplane launch time to opening of window (depends on fuel reserves allowed for steering)
		double DTOPEN;
		//Delta time from optimum inplane launch time to closing of window (depends on fuel reserves allowed for steering)
		double DTCLOSE;

		//Flag. false = compute launch window, true = compute DKI plan for input lift-off time TLO for display on launch window display
		bool IR;
		//Time of liftoff
		double TLO;
		//Initial M-line or rendezvous number
		int MI;
		//Final M-line or rendezvous number
		int MF;
		//Delta time from insertion to separation
		double DTSEP;
		//Delta V components of separation maneuver
		VECTOR3 DVSEP;
		//Delta velocity for phase window opening and closing
		double DVWO, DVWC;

		std::string ProjectFolder;
	};

	struct SkylabLWPOutputs
	{
		std::vector < std::vector<std::string>> OutputPrint;
	};

	class SkylabLaunchWindowProcessor
	{
	public:
		SkylabLaunchWindowProcessor(OrbMech::GlobalConstants& cnst, OrbMech::SessionConstants& scnst);

		int Calculate(const SkylabLWPInputs& in, SkylabLWPOutputs &out);
	protected:
		//Analytical launch time
		void LWT();
		//Launch targeting
		int RLOT(bool init);
		void BuildManeuverConstraintsTable();
		void SetMLine(int M);

		// INPUTS
		SkylabLWPInputs inp;

		// INTERNAL
		std::vector<OMP::ManeuverConstraints> ManeuverConstraintsTable;
		double GMTLO;
		OrbMech::StateVector sv_C;
		//Time of analytical inplane lift-off
		double TAIP;

		// OUTPUT
		std::vector < std::vector<std::string>> OutputPrint;

		// CONSTANTS
		OrbMech::GlobalConstants& constants;
		OrbMech::SessionConstants& sesconst;
	};

}