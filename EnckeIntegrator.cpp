#include "EnckeIntegrator.h"

EnckeIntegratorInput::EnckeIntegratorInput()
{
	R = V = _V(0, 0, 0);
	GMT = 0.0;
	KFactor = 1.0;
	DragIndicator = true;
	Weight = 1.0;
	Area = 1.0;
	dt = 0.0;
	EphemerisIndicator = false;
}

EnckeIntegratorOutput::EnckeIntegratorOutput()
{
	R = V = _V(0, 0, 0);
	GMT = 0.0;
	Error = 0;
}

EnckeState::EnckeState()
{
	r_c0 = v_c0 = beta_c0 = n_c0 = 0.0;
}

const double EnckeIntegrator::KK = 7.906012595544413e-06; //Equal to 0.0625*sqrt(6378165)/sqrt(mu_Earth) = 0.0625 beta units (Earth radii)
const double EnckeIntegrator::EPSQR = 0.0001;
const double EnckeIntegrator::EPSQV = 0.0001;

EnckeIntegrator::EnckeIntegrator(OrbMech::GlobalConstants& cnst) : constants(cnst)
{
	H = HRK = CSA = T = 0.0;
	HP = HD2 = H2D2 = H2D8 = HD6 = 0.0;
	HRS = ALT = 0.0;
	Y = YP = YPP = A_D = _V(0, 0, 0);
	F1 = F2 = F3 = YS = YPS = U_Z = _V(0, 0, 0);
	Err = 0;
	IEND = DragIndicator = false;
	HMULT = 0.0;
	RBASE = VBASE = RTB = RDTB = _V(0, 0, 0);
	RSTATE = VSTATE = _V(0, 0, 0);
	TMAX = TRECT = TSTART = 0.0;
}

void EnckeIntegrator::Init(const EnckeIntegratorInput& in)
{
	IEND = false;
	Y = YP = _V(0, 0, 0);
	TRECT = T = HRS = 0.0;

	RBASE = RSTATE = in.R;
	VBASE = VSTATE = in.V;
	TSTART = in.GMT;
	DragIndicator = in.DragIndicator;

	CSA = -0.5 * in.Area * 2.0 * in.KFactor / in.Weight; //Hardcoded CD of 2.0
	U_Z = _V(0, 0, 1);

	if (in.dt >= 0.0)
	{
		TMAX = in.dt;
		HMULT = 1.0;
	}
	else
	{
		TMAX = abs(in.dt);
		HMULT = -1.0;
	}

	CalculateInitialEnckeState();

	Ephemeris.clear();
}

void EnckeIntegrator::Propagate(const EnckeIntegratorInput& in, EnckeIntegratorOutput& out)
{
	Init(in);

	//Make initialization pass in forcing function and editor
	EffectiveForcesRoutine();
	IntegrationTerminationControlRoutine();
	while (IEND == false)
	{
		//Step
		GaussJackson();
		if (Err)
		{
			out.Error = Err;
			return;
		}
		//Edit
		IntegrationTerminationControlRoutine();
		if (Err)
		{
			out.Error = Err;
			return;
		}
	}

	out.R = RSTATE;
	out.V = VSTATE;
	out.GMT = HRS;
	out.Error = 0;
}

void EnckeIntegrator::IntegrationTerminationControlRoutine()
{
	// 1. Integration step size selection
	H = KK * length(RSTATE);
	// 2. Termination control
	if (DragIndicator)
	{
		if (ALT < 50.0 * 1852.0)
		{
			Err = 1;
			return;
		}
	}
	//Absolute time since start
	double TIME = abs(TRECT + T);
	//Reached the end?
	if (abs(TIME - TMAX) < 1e-7)
	{
		IEND = true;
		return;
	}
	//Would next step exceed TMAX?
	if (TIME + H > TMAX)
	{
		H = TMAX - TIME;
	}
	//Set right direction indicator
	H *= HMULT;
	// 3. Rectification
	if (dotp(Y, Y) / dotp(RSTATE, RSTATE) > EPSQR)
	{
		Rectification();
	}
	else if (dotp(YP, YP) / dotp(VSTATE, VSTATE) > EPSQV)
	{
		Rectification();
	}
}

void EnckeIntegrator::GaussJackson()
{
	//No actual GJ, just do 4 RK steps

	HRK = H / 4.0;

	RungeKutta();
	RungeKutta();
	RungeKutta();
	RungeKutta();
}

void EnckeIntegrator::RungeKutta()
{
	//Fourth-order Runge-Kutta method
	if (HRK - HP != 0.0)
	{
		HD2 = HRK / 2.0;
		H2D2 = HD2 * HRK;
		H2D8 = H2D2 / 4.0;
		HD6 = HRK / 6.0;
		HP = HRK;
	}

	//Save base and build state for 2nd derivative (2nd term)
	F1 = YPP;
	YS = Y;
	YPS = YP;
	Y = YS + YPS * HD2 + F1 * H2D8;
	YP = YPS + F1 * HD2;
	T = T + HD2;
	//Get 2nd derivative (F2)
	EffectiveForcesRoutine();
	//Save F2 and build state for 2nd deriv evaluation F3
	F2 = YPP;
	YP = YPS + F2 * HD2;
	EffectiveForcesRoutine();
	F3 = YPP;
	Y = YS + YPS * HRK + F3 * H2D2;
	YP = YPS + F3 * HRK;
	T = T + HD2;
	//Get 2nd deriv F4
	EffectiveForcesRoutine();
	//Weighted sum for state at T + HRK
	Y = YS + (YPS + (F1 + F2 + F3) * HD6) * HRK;
	YP = YPS + (F1 + (F2 + F3) * 2.0 + YPP) * HD6;
	//Final acceleration
	EffectiveForcesRoutine();
}

void EnckeIntegrator::EffectiveForcesRoutine()
{
	double f, g, F, G, q;

	//Get two-body state
	Err = shepperd.Run2(enckestate.r_c0, enckestate.v_c0, enckestate.beta_c0, enckestate.n_c0, T, constants.mu, f, g, F, G);

	// Compute conic state vector
	RTB = RBASE * f + VBASE * g;
	RDTB = RBASE * F + VBASE * G;

	//Compute final state vector
	RSTATE = RTB + Y;
	VSTATE = RDTB + YP;
	HRS = TSTART + TRECT + T;

	//Calculates A_D
	AccelerationRoutine();

	//Calculate YPP
	q = dotp((Y - RSTATE * 2.0), Y) / pow(length(RSTATE), 2.0);
	YPP = -(RSTATE * fq(q) + Y) * constants.mu / pow(length(RTB), 3.0) + A_D;
}

void EnckeIntegrator::AccelerationRoutine()
{
	A_D = _V(0, 0, 0);

	//Gravity
	A_D += GeopotentialRoutine();
	//Drag
	if (DragIndicator)
	{
		A_D += DragRoutine(ALT);
	}
}

void EnckeIntegrator::CalculateInitialEnckeState()
{
	enckestate.r_c0 = length(RBASE);
	enckestate.v_c0 = length(VBASE);
	enckestate.beta_c0 = (2.0 * constants.mu / enckestate.r_c0) - enckestate.v_c0 * enckestate.v_c0;
	enckestate.n_c0 = dotp(RBASE, VBASE);
}

void EnckeIntegrator::Rectification()
{
	RBASE = RTB + Y;
	VBASE = RDTB + YP;
	Y = YP = _V(0, 0, 0);
	TRECT = TRECT + T;
	T = 0.0;
	CalculateInitialEnckeState();
	EffectiveForcesRoutine();
}

VECTOR3 EnckeIntegrator::GeopotentialRoutine() const
{
	VECTOR3 U_R, a_dE;
	double r, cos_lat, P2, P3, P4, P5;

	r = length(RSTATE);
	U_R = RSTATE / r;
	cos_lat = dotp(U_R, U_Z);

	P2 = 3.0 * cos_lat;
	P3 = 0.5 * (15.0 * cos_lat * cos_lat - 3.0);
	P4 = 1.0 / 3.0 * (7.0 * cos_lat * P3 - 4.0 * P2);
	P5 = 0.25 * (9.0 * cos_lat * P4 - 5.0 * P3);

	a_dE = (U_R * P3 - U_Z * P2) * constants.J2 * pow(constants.R_E / r, 2);
	a_dE += (U_R * P4 - U_Z * P3) * constants.J3 * pow(constants.R_E / r, 3);
	a_dE += (U_R * P5 - U_Z * P4) * constants.J4 * pow(constants.R_E / r, 4);

	return a_dE * constants.mu / (r * r);
}

VECTOR3 EnckeIntegrator::DragRoutine(double& ALT) const
{
	VECTOR3 V_R;
	double d, r, DENS, CS, CDRAG, VRMAG;

	//Ellipsoid
	d = sqrt((RSTATE.x * RSTATE.x + RSTATE.y * RSTATE.y) / constants.a_sq + RSTATE.z * RSTATE.z / constants.b_sq);
	r = length(RSTATE);
	ALT = r - r / d;
	//Density
	OrbMech::GLFDEN(ALT, DENS, CS);
	//Drag
	CDRAG = DENS * CSA;
	V_R = VSTATE - crossp(U_Z * constants.w_E, RSTATE);
	VRMAG = length(V_R);
	return V_R * VRMAG * CDRAG;
}

double EnckeIntegrator::fq(double q) const
{
	//Encke-Term
	return q * (3.0 + 3.0 * q + q * q) / (1.0 + pow(1.0 + q, 1.5));
}