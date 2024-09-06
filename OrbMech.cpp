#include "OrbMech.h"
#include "Constants.h"
#include "cmath"
#include <stdio.h>

namespace OrbMech
{
	StateVector::StateVector()
	{
		R = V = _V(0, 0, 0);
		GMT = Area = KFactor = Weight = 0.0;
	}

	GautschiMethod1::GautschiMethod1(double t) : tol(t)
	{
		n = l = d = k = iter = 0;
		A = B = G = GSAV = 0.0;
	}

	double GautschiMethod1::Run(double q)
	{
		// Continued fraction for G(5,0,5/2,q)
		// q: intermediate parameter with: -inf < q <= 1/2 (up to 1 converges)

		iter = 0;
		k = -9;
		l = 3;
		n = 0;
		d = 15;
		A = 1.0;
		B = 1.0;
		G = 1.0;

		while (1)
		{
			iter++;
			GSAV = G;
			k *= -1;
			l += 2;
			d += 4 * l;
			n += (1 + k) * l;
			A = d / (d - n * A * q);
			B *= (A - 1.0);
			G += B;

			if (fabs(G - GSAV) < tol || iter > 100)
				break;
		}

		return G;
	}

	GautschiMethod2::GautschiMethod2(double t) : tol(t)
	{
		x = p = u = s = 0.0;
		l = ltemp = d = iter = 0;
	}

	double GautschiMethod2::Run(double alpha, double beta)
	{
		x = alpha * pow(0.25 * beta, 2);

		l = ltemp = 1;
		d = 1;
		p = 1.0;
		u = 1.0;
		s = 1.0;

		iter = 0;

		while (1)
		{
			iter++;
			l = l + 2;
			d = l * ltemp;
			ltemp = l;

			p = d / (d - p * x);
			u = u * (p - 1.0);
			s = s + u;

			if (fabs(u) < tol || iter > 100)
				break;
		}

		return beta * 0.25 * s;
	}

	ShepperdMethod::ShepperdMethod(int imax) : iter_max(imax), tol(1.0e-12)
	{
		q = U = U0 = U1 = U2 = U3 = rf = t = f = g = F = G = DU = 0.0;
		DT = beta = mu = n0 = r0 = v0 = u = umax = umin = du = 0.0;
		terr = tsav = 0.0;
		iter = 0;
		err = 0;
	}

	int ShepperdMethod::Run(VECTOR3 R0, VECTOR3 V0, double DT, double mu, VECTOR3& R1, VECTOR3& V1)
	{
		this->DT = DT;
		this->mu = mu;

		// Properties of the orbit
		r0 = length(R0);
		v0 = length(V0);
		beta = (2.0 * mu / r0) - v0 * v0;
		n0 = dotp(R0, V0);

		err = InnerLoop();
		if (err)
		{
			R1 = R0;
			V1 = V0;
			return err;
		}

		// Compute final state vector
		R1 = R0 * f + V0 * g;
		V1 = R0 * F + V0 * G;

		return 0;
	}

	int ShepperdMethod::Run2(double r0, double v0, double beta, double n0, double DT, double mu, double &f, double &g, double &F, double &G)
	{
		this->r0 = r0;
		this->v0 = v0;
		this->beta = beta;
		this->n0 = n0;
		this->DT = DT;
		this->mu = mu;

		err = InnerLoop();
		if (err) return err;

		f = this->f;
		g = this->g;
		F = this->F;
		G = this->G;

		return 0;
	}

	int ShepperdMethod::InnerLoop()
	{
		// Initial guess
		u = 0;

		// Limits
		umax = beta == 0 ? 1.0e+24 : 1.0 / sqrt(fabs(beta));
		umin = -umax;

		// Multiple orbits
		if (beta <= 0)
		{
			DU = 0;
		}
		else
		{
			double p = PI2 * mu / sqrt(beta * beta * beta);
			double n = floor((1.0 / p) * (DT + 0.5 * p - 2.0 * n0 / beta));
			DU = PI2 * n / sqrt(beta * beta * beta * beta * beta);
		}

		//Solve Kepler's problem
		err = KeplerIteration();
		if (err) return err;

		// Compute Lagrange coefficients
		f = 1.0 - (mu / r0) * U2;
		g = r0 * U1 + n0 * U2;
		F = -mu * U1 / (rf * r0);
		G = 1.0 - (mu / rf) * U2;

		return 0;
	}

	int ShepperdMethod::KeplerIteration()
	{
		// Kepler iteration loop
		iter = 0;
		tsav = 1.0e99;

		while (1)
		{
			// Intermediate parameter (limits: -inf < q <= 0.5)
			q = beta * u * u;
			q /= (1.0 + q);

			// Stumpff functions for w/2
			U0 = 1.0 - 2.0 * q;
			U1 = 2.0 * u * (1.0 - q);

			// Stumpff functions for w
			U = (16.0 / 15.0) * U1 * U1 * U1 * U1 * U1 * Gautschi.Run(q) + DU;

			U2 = 2.0 * U1 * U1;
			U1 = 2.0 * U0 * U1;
			U0 = 2.0 * U0 * U0 - 1.0;
			U3 = beta * U + U1 * U2 / 3.0;

			// Final radius and time
			rf = r0 * U0 + n0 * U1 + mu * U2;
			t = r0 * U1 + n0 * U2 + mu * U3;

			// Check for time convergence
			if (fabs(t - tsav) < tol)
				break;

			// Error in time estimate
			terr = DT - t;

			// Save old dependent variables
			tsav = t;

			// More time convergence
			if (fabs(terr) <= fabs(DT) * tol)
				break;

			// Slope of u
			du = terr / (4.0 * (1.0 - q) * rf);

			// Update u, but prevent violating the min/max boundaries
			if (du < 0)
			{
				umax = u;
				u = u + du;
				if (u < umin)
					u = 0.5 * (umin + umax);
			}
			else
			{
				umin = u;
				u = u + du;
				if (u > umax)
					u = 0.5 * (umin + umax);
			}

			// Iteration limit
			iter++;
			if (iter > iter_max) return 1;
		}

		return 0;
	}

	void rv_from_r0v0(VECTOR3 R0, VECTOR3 V0, double t, VECTOR3& R1, VECTOR3& V1, double mu)
	{
		ShepperdMethod shepperd;

		shepperd.Run(R0, V0, t, mu, R1, V1);
	}

	int time_theta(VECTOR3 R1, VECTOR3 V1, double dtheta, double mu, double& dt)
	{
		//Wrapper function in case the output state vector isn't needed
		VECTOR3 R2, V2;

		return time_theta(R1, V1, dtheta, mu, R2, V2, dt);
	}

	int MarscherEquationInversion(double sin_theta, double cos_theta, double cot_gamma, double r1, double alpha_N, double p_N, double& x, double& xi, double& c1, double& c2)
	{
		// Marscher equation inversion
		// INPUTS:
		// sin_theta: sine of true anomaly difference
		// cos_theta: cosine of true anomaly difference
		// cot_gamma: cotangent of flight path angle (measured from local vertical)
		// r1: Radius magnitude at initial position
		// alpha_N: Ratio of magnitude of initial position vector to semi-major axis
		// p_N: Ratio of semi-latus rectum to initial position vector magnitude
		// OUTPUTS:
		// x: Universal anomaly
		// xi: Product of alpha_N and x squared
		// c1 and c2: Intermediate variables

		double W[4], a;
		int n;
		bool W1MAX; //W[0] is near infinite

		const double C_A_MAX = 79.0;
		const double C_TOL = 1e-7;

		//Divisor safety
		W[0] = 1.0 - cos_theta;

		W1MAX = false;
		if (abs(W[0]) < 1e-10)
		{
			W1MAX = true;
			if (sin_theta >= 0.0)
			{
				W[0] = 1e10;
			}
			else
			{
				W[0] = -1e10;
			}
		}
		else
		{
			W[0] = sqrt(p_N) * (sin_theta / W[0] - cot_gamma);
		}

		// Error checking for physically impossible solution
		if (alpha_N < 0.0)
		{
			if (W[0] <= 0.0)
			{
				return 1;
			}
			if (!W1MAX && W[0] * W[0] + alpha_N <= 0.0)
			{
				return 2;
			}
		}

		if (abs(W[0]) <= 1.0)
		{
			for (n = 0; n < 3; n++)
			{
				W[n + 1] = sqrt(W[n] * W[n] + alpha_N) + abs(W[n]);
			}
			a = 1.0 / W[3];
		}
		else
		{
			double V[4], W1inv;

			W1inv = abs(sin_theta / (sqrt(p_N) * (1.0 + cos_theta - sin_theta * cot_gamma)));
			V[0] = 1.0;

			for (n = 0; n < 3; n++)
			{
				V[n + 1] = sqrt(V[n] * V[n] + alpha_N * W1inv * W1inv) + V[n];
			}
			a = W1inv / V[3];
		}

		double C_A, C_B, C_C, C_D, X_N;

		C_A = C_B = X_N = 1.0;
		C_C = alpha_N * a * a;
		do
		{
			C_A += 2.0;
			C_B = -C_B * C_C;
			C_D = C_B / C_A;
			X_N = X_N + C_D;
			if (C_A >= C_A_MAX)
			{
				//Series nonconvergent
				return 3;
			}
		} while (abs(C_D) >= C_TOL);

		X_N = 16.0 * a * X_N;
		if (W[0] <= 0.0)
		{
			X_N = PI2 / sqrt(alpha_N) - X_N;
		}
		xi = alpha_N * X_N * X_N;
		x = sqrt(r1) * X_N;
		c1 = sqrt(r1 * p_N) * cot_gamma;
		c2 = 1.0 - alpha_N;

		return 0;
	}

	double stumpS(double z)
	{
		double s;

		s = 0;
		if (z > 0) {
			s = (sqrt(z) - sin(sqrt(z))) / pow(sqrt(z), 3);
		}
		else if (z < 0) {
			s = (sinh(sqrt(-z)) - sqrt(-z)) / pow(sqrt(-z), 3);
		}
		else {
			s = 1.0 / 6.0;
		}
		return s;
	}

	double stumpC(double z)
	{
		double c;
		c = 0;
		if (z > 0) {
			c = (1.0 - cos(sqrt(z))) / z;
		}
		else if (z < 0) {
			c = (cosh(sqrt(-z)) - 1.0) / (-z);
		}
		else {
			c = 0.5;
		}
		return c;
	}

	int time_theta(VECTOR3 R1, VECTOR3 V1, double dtheta, double mu, VECTOR3& R2, VECTOR3& V2, double& dt)
	{
		// INPUTS:
		// R1: Input position vector
		// V1: Input velocity vector
		// dtheta: Change in true anomaly
		// mu: specific gravitation parameter
		// OUTPUTS:
		// return: 0 = no errors, 1 = multiple orbits requested for hyperbolic orbit, 2 = orbit too nearly rectilinear, 3 and 4 = no physically realizable solution (hyperbolic)
		// R2: Output position vector
		// V2: Output velocity vector
		// dt: time equivalent to input angle dtheta

		// Handle zero case first
		if (dtheta == 0.0)
		{
			dt = 0.0;
			R2 = R1;
			V2 = V1;
			return 0;
		}

		// Internally this function will only handle positive dtheta, so set a flag if the angle is negative
		bool negative;

		// Handle negative angle
		if (dtheta < 0)
		{
			dtheta = -dtheta;
			V1 = -V1;
			negative = true;
		}
		else
		{
			negative = false;
		}

		int n;

		// Number of orbits
		n = (int)(dtheta / PI2);
		dtheta = fmod(dtheta, PI2);

		double r1, v1, alpha;

		// Initial position magnitude
		r1 = length(R1);
		// Initial velocity magnitude
		v1 = length(V1);
		// Reciprocal of semi-major axis
		alpha = 2.0 / r1 - v1 * v1 / mu;

		// Error check: Cannot request multiple orbits for hyperbolic orbit
		if (n > 0 && alpha <= 0.0)
		{
			return 1;
		}

		//Geometric parameters
		VECTOR3 I_R_1, I_V_1;
		double sin_gamma, cos_gamma, cot_gamma, C3, alpha_N, p_N, x, xi, c1, c2, S, C, r2;
		int err;

		//Unit position vector
		I_R_1 = R1 / r1;
		//Unit velocity vector
		I_V_1 = V1 / v1;

		//Sine of flight path angle (measured from local vertical)
		sin_gamma = length(crossp(I_R_1, I_V_1));

		//Rectilinear orbit check
		if (abs(sin_gamma) < 1e-12)
		{
			return 2;
		}

		//Cosine of flight path angle (measured from local vertical)
		cos_gamma = dotp(I_R_1, I_V_1);
		//Cotangent of flight path angle (measured from local vertical)
		cot_gamma = cos_gamma / sin_gamma;
		//Energy
		C3 = r1 * v1 * v1 / mu;
		//Ratio of magnitude of initial position vector to semi-major axis
		alpha_N = 2.0 - C3;
		//Ratio of semi-latus rectum to initial position vector magnitude
		p_N = C3 * pow(sin_gamma, 2);

		//Marscher equation inversion
		err = MarscherEquationInversion(sin(dtheta), cos(dtheta), cot_gamma, r1, alpha_N, p_N, x, xi, c1, c2);
		if (err)
		{
			//Error return. Plus 2 so that time_theta returns a unique error code for each error type
			return err + 2;
		}

		//Stumpff functions. TBD: These Stumpff functions can run into numerical trouble very close to parabolic orbits
		S = stumpS(xi);
		C = stumpC(xi);
		dt = (c1 * x * x * C + x * (c2 * x * x * S + r1)) / sqrt(mu);

		//State vector
		R2 = R1 * (1.0 - x * x / r1 * C) + V1 * (dt - x * x * x / sqrt(mu) * S);
		r2 = length(R2);
		V2 = R1 * (sqrt(mu) / (r1 * r2) * x * (xi * S - 1.0)) + V1 * (1.0 - x * x / r2 * C);

		//Multiple orbits
		if (n > 0)
		{
			double P;

			//Calculate orbital period
			P = PI2 / (pow(alpha, 1.5) * sqrt(mu));
			dt = dt + P * (double)n;
		}

		//If input angle was negative, reverse the output
		if (negative)
		{
			dt = -dt;
			V2 = -V2;
		}

		return 0;
	}

	double fraction_an(int n)
	{
		if (n == 0)
		{
			return 4.0;
		}
		else
		{
			return fraction_an(n - 1) + (2.0 * n + 1.0);
		}
	}

	double fraction_ad(int n)
	{
		if (n == 0)
		{
			return 15.0;
		}
		else
		{
			return fraction_ad(n - 1) + 4.0 * (2.0 * n + 1.0);
		}
	}

	double fraction_a(int n, double x)
	{
		if (n == 0)
		{
			return 1.0 / 5.0;
		}
		else
		{
			return -fraction_an(n) / fraction_ad(n) * x;
		}
	}

	double fraction_b(int n, double x)
	{
		return 1.0;
	}

	double fraction_delta(int n, double x)
	{
		if (n == 0)
		{
			return 1.0;
		}
		else
		{
			return 1.0 / (1.0 - fraction_a(n, x) * fraction_delta(n - 1, x));
		}
	}

	double fraction_u(int n, double x)
	{
		if (n == 0)
		{
			return 1.0 / 5.0;
		}
		else
		{
			return fraction_u(n - 1, x) * (fraction_delta(n, x) - 1.0);
		}
	}

	double fraction_pq(double x)
	{
		int n = 0;
		double u = 100.0;
		double pq = 0.0;

		while (abs(u) > 1e-9)
		{
			u = fraction_u(n, x);
			pq += u;
			n++;
		}
		return pq;
	}

	double fraction_xi(double x)
	{
		double eta, xi_eta, xi_x;

		eta = (sqrt(1.0 + x) - 1.0) / (sqrt(1.0 + x) + 1.0);
		xi_eta = fraction_pq(eta);
		xi_x = 1.0 / (8.0 * (sqrt(1.0 + x) + 1.0)) * (3.0 + xi_eta / (1.0 + eta * xi_eta));
		return xi_x;
	}

	VECTOR3 elegant_lambert(VECTOR3 R1, VECTOR3 V1, VECTOR3 R2, double dt, int N, bool prog, double mu)
	{
		double tol, ratio, r1, r2, c, s, theta, lambda, T, l, m, x, h1, h2, B, y, z, x_new, A, root;
		int nMax, n;
		VECTOR3 c12, Vt1, Vt2;

		tol = 1e-8;
		nMax = 1000;
		n = 0;
		ratio = 1;

		r1 = length(R1);
		r2 = length(R2);
		c = length(R2 - R1);
		s = (r1 + r2 + c) / 2;

		c12 = crossp(unit(R1), unit(R2));

		theta = acos(dotp(R1, R2) / r1 / r2);
		if ((prog == true && c12.z < 0) || (prog == false && c12.z >= 0))
		{
			theta = PI2 - theta;
		}

		lambda = sqrt(r1 * r2) / s * cos(theta / 2.0);
		T = sqrt(8.0 * mu / pow(s, 3.0)) * dt;
		l = pow((1.0 - lambda) / (1.0 + lambda), 2.0);
		m = pow(T, 2.0) / (pow(1.0 + lambda, 6.0));

		//T_m = pi*((2 * N + 1) - lambda ^ 5);

		//Low Energy
		x = 1.0 + 4.0 * l;

		while (abs(ratio) > tol && nMax >= n)
		{
			n = n + 1;
			if (N == 0)
			{
				double xi = fraction_xi(x);
				h1 = pow(l + x, 2.0) * (1.0 + xi * (1.0 + 3.0 * x)) / ((1.0 + 2.0 * x + l) * (3.0 + x * (1.0 + 4.0 * xi)));
				h2 = m * (1.0 + (x - l) * xi) / ((1.0 + 2.0 * x + l) * (3.0 + x * (1.0 + 4.0 * xi)));
			}
			else
			{
				h1 = pow(l + x, 2.0) / (4.0 * pow(x, 2.0) * (1.0 + 2.0 * x + l)) * (3.0 * pow(1.0 + x, 2.0) * (N * PI / 2.0 + atan(sqrt(x))) / sqrt(x) - (3.0 + 5.0 * x));
				h2 = m / (4.0 * pow(x, 2.0) * (1.0 + 2.0 * x + l)) * ((pow(x, 2.0) - (1.0 + l) * x - 3.0 * l) * (N * PI / 2.0 + atan(sqrt(x))) / sqrt(x) + (3.0 * l + x));
			}
			B = 27 * h2 / (4 * pow(1 + h1, 3));
			if (B >= 0)
			{
				z = 2.0 * cosh(1.0 / 3.0 * acosh(sqrt(B + 1.0)));
			}
			else
			{
				z = 2.0 * cos(1.0 / 3.0 * acos(sqrt(B + 1.0)));
			}
			y = 2.0 / 3.0 * (1.0 + h1) * (sqrt(B + 1.0) / z + 1.0);
			x_new = sqrt(pow((1.0 - l) / 2.0, 2) + m / y / y) - (1.0 + l) / 2.0;
			ratio = x - x_new;
			x = x_new;
		}

		Vt1 = ((R2 - R1) + R1 * s * pow(1.0 + lambda, 2.0) * (l + x) / (r1 * (1.0 + x))) * 1.0 / (lambda * (1 + lambda)) * sqrt(mu * (1.0 + x) / (2.0 * pow(s, 3.0) * (l + x)));
		if (N == 0)
		{
			return Vt1;
		}

		//High Energy
		root = pow((pow((2.0 * m) / (pow(N, 2.0) * pow(PI, 2.0)), 1.0 / 3.0) - (1.0 + l) / 2.0), 2.0) - 4.0 * l;
		if (root >= 0)
		{
			x = (pow(2.0 * m / (pow(N, 2.0) * pow(PI, 2.0)), 1.0 / 3.0) - (1.0 + l) / 2.0) - sqrt(root);
		}
		else
		{
			x = 0.0001;
		}

		ratio = 1;
		n = 0;
		while (abs(ratio) > tol && nMax >= n)
		{
			n = n + 1;
			h1 = (l + x) * (1.0 + 2.0 * x + l) / (2.0 * (l - pow(x, 2.0)));
			h2 = m * sqrt(x) / (2.0 * (l - pow(x, 2))) * ((l - pow(x, 2.0)) * (N * PI / 2.0 + atan(sqrt(x))) / sqrt(x) - (l + x));
			B = 27.0 * h2 / (4.0 * pow(sqrt(x) * (1 + h1), 3));
			if (B >= 0)
			{
				z = 2.0 * cosh(1.0 / 3.0 * acosh(sqrt(B + 1.0)));
			}
			else
			{
				z = 2.0 * cos(1.0 / 3.0 * acos(sqrt(B + 1.0)));
			}
			A = pow((sqrt(B) + sqrt(B + 1.0)), 1.0 / 3.0);
			y = 2.0 / 3.0 * sqrt(x) * (1.0 + h1) * (sqrt(B + 1.0) / (A + 1.0 / A) + 1.0);
			//y = 2 / 3 * (1 + h1)*(sqrt(B + 1) / z + 1);
			x_new = 0.5 * ((m / y / y - (1.0 + l)) - sqrt(pow(m / y / y - (1.0 + l), 2) - 4.0 * l));

			ratio = x - x_new;
			x = x_new;
		}

		Vt2 = ((R2 - R1) + R1 * s * pow(1.0 + lambda, 2.0) * (l + x) / (r1 * (1.0 + x))) * 1.0 / (lambda * (1.0 + lambda)) * sqrt(mu * (1.0 + x) / (2.0 * pow(s, 3.0) * (l + x)));

		if (length(V1 - Vt1) > length(V1 - Vt2))
		{
			return Vt2;
		}
		else
		{
			return Vt1;
		}
	}

	void GLFDEN(double Z, double& DEN, double& CS)
	{
		//Z is the geometric altitude in meters, DEN is the output density in kg/m^2 and CS is the output speed of sound in m/s

		static const double HB[10] = { -5000.0, 0.0, 11000.0, 20000.0, 32000.0, 47000.0, 52000.0, 61000.0, 79000.0, 88743.0 };
		static const double ZB[14] = { 90000.0, 100000.0, 110000.0, 120000.0, 150000.0, 160000.0, 170000.0, 190000.0, 230000.0, 300000.0, 400000.0, 500000.0, 600000.0, 700000.0 };
		static const double TMB[24] = { 320.65, 288.15, 216.65, 216.65, 228.65, 270.65, 270.65, 252.65, 180.65, 180.65, 180.65, 210.65,
										260.65, 360.65, 960.65, 1110.65, 1210.65, 1350.65, 1550.65, 1830.65, 2160.65, 2420.65, 2590.65, 2700.65 };
		static const double ALP[24] = { 1.2087778e1, 1.1526088e1, 1.002712e1, 8.6079235, 6.7662077, 4.7086738, 4.0775458, 2.9019653, 3.7006732e-2, -1.8055744, -1.8055744,
			-3.504061, -4.9124564, -5.9828218, -7.5886378, -7.9035491, -8.183367, -8.6884559, -9.5726883, -1.0879634e1, -1.2421644e1, -1.3724116e1, -1.4879663e1, -1.594263e1 };
		static const double GB[14] = { 9.535, 9.505, 9.476, 9.447, 9.36, 9.331, 9.302, 9.246, 9.134, 8.942, 8.679, 8.428, 8.187, 7.956 };

		//Test for lower limit of atmosphere
		if (Z < -5000.0)
		{
			//Below -5km density and speed of sound set to 0
			CS = DEN = 0.0;
			return;
		}

		//Test for upper limit of atmosphere
		if (Z >= 700000.0)
		{
			//Above 700km density is set to zero and speed of sound is set to the 90km value
			DEN = 0.0;
			CS = 269.44;
			return;
		}

		double ALPP, TM, P, AL;
		int N;

		//Test for atmosphere layers for Z less than or equal to 90 km
		if (Z <= 90000.0)
		{
			double H;

			//Below 90km
			//Compute geopotential altitude H
			H = ((((((-9.5013649e-35 * Z) + 6.0621354e-28) * Z - 3.8667054e-21) * Z + 2.4656553e-14) * Z - 1.5731262e-7) * Z + 1.0) * Z;

			for (N = 0; N < 8; N++)
			{
				if (H <= HB[N + 1]) break;
			}

			//Compute AL, the temperature gradient
			AL = (TMB[N + 1] - TMB[N]) / (HB[N + 1] - HB[N]);
			//Compute TM, the molecular scale temperature and CS, the speed of sound
			TM = TMB[N] + AL * (H - HB[N]);
			CS = 20.04680273 * sqrt(TM);
			if (AL != 0.0)
			{
				//When AL is not zero, the natural log of pressure is computed as a function of AL
				ALPP = ALP[N] - 3.41631947e-2 * log((AL * (H - HB[N]) + TMB[N]) / TMB[N]) / AL;
			}
			else
			{
				//When ALP is zero, the natural log of pressure is computed without using AL
				ALPP = ALP[N] - 3.41631947e-2 * (H - HB[N]) / TMB[N];
			}
		}
		else
		{
			double G, GG;
			int M;

			//Above 90km
			//Set speed of sound equalo to the 90km value
			CS = 269.44;
			//Test Z against base layer altitudes
			for (M = 0; M < 12; M++)
			{
				if (Z <= ZB[M + 1]) break;
			}
			//Set layer number N to correct base layer value
			N = M + 10;
			//Compute AL, temperature gradient
			AL = (TMB[N + 1] - TMB[N]) / (ZB[M + 1] - ZB[M]);
			//G, acceleration of gravity at altitude Z
			G = (((((-5.5905936e-33) * Z + 2.972462e-26) * Z - 1.5167771e-19) * Z + 7.2539455e-13) * Z - 3.0854195e-6) * Z + 9.806665;
			//GG, approximation of G for extraction from within the integral
			GG = (G + GB[M]) / 2.0;
			//TM, molecular scale temperature
			TM = TMB[N] + AL * (Z - ZB[M]);
			if (AL != 0.0)
			{
				//When AL is not zero, the natural log of pressure is computed as a function of AL
				ALPP = ALP[N] - 3.48367635e-3 * GG * log((Z - ZB[M] + TMB[N] / AL) * AL / TMB[N]) / AL;
			}
			else
			{
				//When ALP is zero, the natural log of pressure is computed without using AL
				ALPP = ALP[N] - 3.48367635e-3 * GG * (Z - ZB[M]) / TMB[N];
			}
		}
		//Calculate pressure
		P = exp(ALPP);
		//Compute dewnsity in kg/meters-cubed
		DEN = 3.48367635e-3 * P / TM;
	}

	//Analytical sun ephemeris
	VECTOR3 SUN(double MJD)
	{
		//Calculate solar direction vector in ecliptic J2000 coordinates
		VECTOR3 R_Sun;
		double T_UT, lng_mean, T_TDB, M, lng_ecl, obl, r;

		T_UT = (MJD - 51544.5) / 36525.0;
		lng_mean = 280.460 + 36000.77 * T_UT;
		lng_mean = normalize_angle(lng_mean, 0.0, 360.0);
		T_TDB = T_UT;
		M = 357.5277233 + 35999.05034 * T_TDB;
		M = normalize_angle(M, 0.0, 360.0) * RAD;
		lng_ecl = lng_mean + 1.914666471 * sin(M) + 0.019994643 * sin(2.0 * M);
		lng_ecl = lng_ecl - T_UT * 360.0 / 257.715; //Precession term
		lng_ecl = normalize_angle(lng_ecl, 0.0, 360.0) * RAD;
		obl = 0.0; // 23.439291 * RAD - 0.0130042 * T_TDB;
		r = 1.000140612 - 0.016708617 * cos(M) - 0.000139589 * cos(2.0 * M);
		R_Sun = _V(cos(lng_ecl), cos(obl) * sin(lng_ecl), sin(obl) * sin(lng_ecl)) * r * AU;
		return R_Sun;
	}

	VECTOR3 SUN(double GMTBASE, double GMT, const MATRIX3& RM)
	{
		//Return sun vector in TEG coordinates
		//RM: TEG to M50 rotation matrix

		VECTOR3 R_Sun;
		double MJD;

		MJD = GMTBASE + GMT / 24.0 / 3600.0;
		R_Sun = SUN(MJD);

		return tmul(RM, mul(M_J2000_to_M50, R_Sun));
	}

	MATRIX3 MatrixRH_LH(MATRIX3 A)
	{
		return _M(A.m11, A.m13, A.m12, A.m31, A.m33, A.m32, A.m21, A.m23, A.m22);
	}

	MATRIX3 tmat(MATRIX3 a)
	{
		MATRIX3 b;

		b = _M(a.m11, a.m21, a.m31, a.m12, a.m22, a.m32, a.m13, a.m23, a.m33);
		return b;
	}

	MATRIX3 LVLH_Matrix(VECTOR3 R, VECTOR3 V)
	{
		VECTOR3 i, j, k;
		j = unit(crossp(V, R));
		k = unit(-R);
		i = crossp(j, k);
		return _M(i.x, i.y, i.z, j.x, j.y, j.z, k.x, k.y, k.z); //rotation matrix to LVLH
	}

	CELEMENTS CartesianToKeplerian(VECTOR3 R, VECTOR3 V, double mu)
	{
		CELEMENTS out;
		OELEMENTS coe = coe_from_sv(R, V, mu);

		out.a = coe.h * coe.h / (mu * (1.0 - coe.e * coe.e));
		out.e = coe.e;
		out.i = coe.i;
		out.g = coe.w;
		out.h = coe.RA;
		out.l = TrueToMeanAnomaly(coe.TA, coe.e);

		return out;
	}

	OELEMENTS coe_from_sv(VECTOR3 R, VECTOR3 V, double mu)
	{
		/*% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
		%{
		% This function computes the classical orbital elements(coe)
		% from the state vector(R, V) using Algorithm 4.1.
		%
		mu - gravitational parameter(km ^ 3 / s ^ 2)
		R - position vector in the geocentric equatorial frame(km)
		V - velocity vector in the geocentric equatorial frame(km)
		r, v - the magnitudes of R and V
		vr - radial velocity component(km / s)
		H - the angular momentum vector(km ^ 2 / s)
		h - the magnitude of H(km ^ 2 / s)
		incl - inclination of the orbit(rad)
		N - the node line vector(km ^ 2 / s)
		n - the magnitude of N
		cp - cross product of N and R
		RA - right ascension of the ascending node(rad)
		E - eccentricity vector
		e - eccentricity(magnitude of E)
		eps - a small number below which the eccentricity is considered
		to be zero
		w - argument of perigee(rad)
		TA - true anomaly(rad)
		a - semimajor axis(km)
		pi - 3.1415926...
		coe - vector of orbital elements[h e RA incl w TA a]
		User M - functions required : None
		%}
		% -------------------------------------------- -*/
		double eps, r, v, vr, h, incl, n, RA, e, w, TA;
		VECTOR3 H, N, E, cp;
		OELEMENTS coe;

		eps = 1.e-10;
		r = length(R);
		v = length(V);
		vr = dotp(R, V) / r;
		H = crossp(R, V);
		h = length(H);
		incl = acos(H.z / h);
		N = crossp(_V(0, 0, 1), H);
		n = length(N);
		if (n != 0)
		{
			RA = acos(N.x / n);
			if (N.y < 0)
			{
				RA = PI2 - RA;
			}
		}
		else
		{
			RA = 0;
		}

		E = (R * (pow(v, 2) - mu / r) - V * r * vr) * 1.0 / mu;
		e = length(E);

		if (n != 0)
		{
			if (e > eps)
			{
				w = acos(dotp(N, E) / n / e);
				if (E.z < 0)
				{
					w = PI2 - w;
				}
			}
			else
			{
				w = 0;
			}
		}
		else
		{
			w = 0;
		}
		if (e > eps)
		{
			TA = acos2(dotp(unit(E), unit(R)));
			if (vr < 0.0)
			{
				TA = PI2 - TA;
			}
		}
		else
		{
			cp = crossp(N, R);
			if (cp.z >= 0)
			{
				TA = acos2(dotp(unit(N), unit(R)));
			}
			else
			{
				TA = PI2 - acos2(dotp(unit(N), unit(R)));
			}
		}
		//a = pow(h,2) / mu / (1 - pow(e,2));
		coe.e = e;
		coe.h = h;
		coe.i = incl;
		coe.RA = RA;
		coe.TA = TA;
		coe.w = w;
		return coe;
	}

	void sv_from_coe(OELEMENTS el, double mu, VECTOR3& R, VECTOR3& V)	//computes the state vector(R, V) from the classical orbital elements
	{
		double e, h, RA, incl, TA, w, rp_factor, vp_factor;
		VECTOR3 rp, vp;
		MATRIX3 R3_W, R1_i, R3_w, R2, Q_Xp;

		e = el.e;
		//a = el.a;
		//r_p = (1 - e)*a;
		h = el.h;// sqrt(r_p*mu*(1 + e));
		RA = el.RA;
		incl = el.i;
		TA = el.TA;
		w = el.w;// el.omegab - el.theta;
		rp_factor = h * h / (mu * (1.0 + e * cos(TA)));
		rp.x = rp_factor * cos(TA);
		rp.y = rp_factor * sin(TA);
		rp.z = 0;
		vp_factor = mu / h;
		vp.x = -vp_factor * sin(TA);
		vp.y = vp_factor * (e + cos(TA));
		vp.z = 0;
		R3_W = _M(cos(RA), sin(RA), 0.0, -sin(RA), cos(RA), 0.0, 0.0, 0.0, 1.0);
		R1_i = _M(1, 0, 0, 0, cos(incl), sin(incl), 0, -sin(incl), cos(incl));
		R3_w = _M(cos(w), sin(w), 0, -sin(w), cos(w), 0, 0, 0, 1);
		R2 = mul(R3_w, R1_i);
		Q_Xp = mul(R2, R3_W);
		R = tmul(Q_Xp, rp);
		V = tmul(Q_Xp, vp);
	}

	double TrueToMeanAnomaly(double ta, double eccdp)
	{
		double ma = 0.0;
		double E = TrueToEccentricAnomaly(ta, eccdp);
		ma = E - eccdp * sin(E);
		if (ma < 0.0)
			ma = ma + PI2;
		return ma;
	}

	double TrueToEccentricAnomaly(double ta, double ecc)
	{
		double ea;
		double cosTa = cos(ta);
		double eccCosTa = ecc * cosTa;
		double sinEa = (sqrt(1.0 - ecc * ecc) * sin(ta)) / (1.0 + eccCosTa);
		double cosEa = (ecc + cosTa) / (1.0 + eccCosTa);
		ea = atan2(sinEa, cosEa);

		if (ea < 0.0)
			ea = ea + PI2;

		return ea;
	}

	double MeanToTrueAnomaly(double meanAnom, double eccdp, double error2)
	{
		double ta;

		double E = kepler_E(eccdp, meanAnom, error2);
		if (E < 0.0)
			E = E + PI2;
		double c = abs(E - PI);
		if (c >= 1.0e-8)
		{
			ta = 2.0 * atan(sqrt((1.0 + eccdp) / (1.0 - eccdp)) * tan(E / 2.0));
		}
		else
		{
			ta = E;
		}

		if (ta < 0)
			ta += PI2;

		return ta;
	}

	double kepler_E(double e, double M, double error2)
	{
		double ratio, E;
		//{
		//	This function uses Newton's method to solve Kepler's
		//		equation E - e*sin(E) = M for the eccentric anomaly,
		//		given the eccentricity and the mean anomaly.
		//		E - eccentric anomaly(radians)
		//		e - eccentricity, passed from the calling program
		//		M - mean anomaly(radians), passed from the calling program
		//}
		// ----------------------------------------------

		ratio = 1.0;

		if (M < PI)
		{
			E = M + e / 2.0;
		}
		else
		{
			E = M - e / 2.0;
		}

		while (abs(ratio) > error2)
		{
			ratio = (E - e * sin(E) - M) / (1.0 - e * cos(E));
			E = E - ratio;
		}
		return E;
	} //kepler_E

	VECTOR3 PROJCT(VECTOR3 U1, VECTOR3 U2, VECTOR3 X)
	{
		VECTOR3 N, X1, X_PROJ;

		N = unit(crossp(U1, U2));
		X1 = X - N * dotp(X, N);
		X_PROJ = unit(X1) * length(X);
		return X_PROJ;
	}

	double PHSANG(VECTOR3 R, VECTOR3 V, VECTOR3 RD)
	{
		VECTOR3 H1, H2, R_PROJ;
		double theta, PHSANG;

		H1 = unit(crossp(R, V));
		H2 = unit(crossp(RD, R));

		R_PROJ = PROJCT(R, V, RD);
		theta = acos2(dotp(unit(R), unit(R_PROJ)));
		PHSANG = theta;
		if (dotp(H1, H2) < 0)
		{
			PHSANG = -PHSANG;
		}
		return PHSANG;
	}

	double ArgLat(VECTOR3 R, VECTOR3 V)
	{
		VECTOR3 K, h, n;
		double u;

		K = _V(0, 0, 1);
		h = unit(crossp(R, V));
		n = crossp(K, h);
		u = acos2(dotp(unit(n), unit(R)));
		if (R.z < 0)
		{
			u = PI2 - u;
		}
		return u;
	}

	void KeplerianToCartesian(CELEMENTS coe, double mu, VECTOR3& R, VECTOR3& V)
	{
		OELEMENTS el;

		el.e = coe.e;
		el.h = sqrt(coe.a * mu * (1.0 - coe.e * coe.e));
		el.i = coe.i;
		el.RA = coe.h;
		el.TA = MeanToTrueAnomaly(coe.l, coe.e);
		el.w = coe.g;

		sv_from_coe(el, mu, R, V);
	}

	bool TLAT(VECTOR3 R, VECTOR3 V, double lat, int C, double& K_AD, double& dtheta)
	{
		VECTOR3 H, N;
		double i, U, Ulat;

		H = unit(crossp(R, V));
		N = unit(crossp(_V(0, 0, 1), H));
		i = acos(H.z);
		if (abs(lat) > i) return true;
		U = OrbMech::PHSANG(R, V, N);
		if (U < 0)
		{
			U += PI2;
		}
		Ulat = asin(abs(sin(lat)) / sin(i));
		if (C == 0)
		{
			K_AD = -1.0;
		}
		if (lat < 0)
		{
			U = U - PI;
		}
		U = atan2(sin(U), cos(U));
		if (U < 0)
		{
			U += PI2;
		}
		if (K_AD > 0)
		{
			Ulat = PI - Ulat;
		}
		if (C == 0)
		{
			double Ulat_pi = PI - Ulat;

			if (U >= Ulat)
			{
				if (U < Ulat_pi)
				{
					Ulat = Ulat_pi;
					K_AD = 1.0;
				}
				else
				{
					Ulat = Ulat + PI2;
				}
			}
		}
		dtheta = Ulat - U;
		if (C != 0)
		{
			if (abs(dtheta) > PI)
			{
				dtheta = dtheta - PI2 * sign(dtheta);
			}
		}
		return false;
	}

	double TLON(VECTOR3 R, VECTOR3 V, double t, double lng, int C, double w_E)
	{
		VECTOR3 N, H;
		double alpha1, alpha2, I, NODE, U, pro, U_lng, dtheta;

		H = unit(crossp(R, V));
		N = unit(crossp(_V(0, 0, 1), H));
		I = acos(H.z);
		NODE = atan2(N.y, N.x);
		U = OrbMech::PHSANG(R, V, N);
		if (U < 0)
		{
			U += PI2;
		}
		pro = 1.0;
		if (PI05 - I < 0)
		{
			pro = -1.0;
		}

		alpha1 = pro * (lng - NODE);
		alpha2 = alpha1 + w_E * t;
		U_lng = atan2(sin(alpha2), pro * cos(alpha2) * cos(I));
		if (U_lng < 0)
		{
			U_lng += PI2;
		}
		if (C == 0)
		{
			if (U_lng < U)
			{
				U_lng = U_lng + PI2;
			}
			dtheta = U_lng - U;
		}
		else
		{
			dtheta = U_lng - U;
			if (abs(dtheta) >= PI)
			{
				dtheta = dtheta - sign(dtheta) * PI2;
			}
		}
		return dtheta;
	}

	bool TALT(VECTOR3 R, VECTOR3 V, double rad, int C, double mu, double& dtheta)
	{
		double r, v, r_dot, P, a, ecosE, esinE, e, r_MIN, r_MAX, theta_D, theta;

		r = length(R);
		v = length(V);
		r_dot = dotp(V, R) / r;
		P = pow(length(crossp(R, V)), 2) / mu;
		a = r / (2.0 - v * v * r / mu);
		ecosE = (a - r) / a;
		esinE = r * r_dot / sqrt(mu * a);
		e = sqrt(esinE * esinE + ecosE * ecosE);
		r_MIN = P / (1.0 + e);
		r_MAX = 2.0 * a - r_MIN;

		//Error condition
		if (rad < r_MIN || r > r_MAX) return true;

		theta_D = acos((P / rad - 1.0) / e);
		theta = acos((P / r - 1.0) / e);

		if (C != 0)
		{
			dtheta = theta_D - theta;
			if (r_dot < 0)
			{
				dtheta = -dtheta;
			}
		}
		else
		{
			dtheta = 0.0;

			if (r_dot > 0)
			{
				if (theta_D > theta)
				{
					dtheta = theta_D - theta;
				}
				else if (theta_D < theta)
				{
					dtheta = PI2 - (theta_D + theta);
				}
			}
			else
			{
				if (theta_D > theta)
				{
					dtheta = theta + theta_D;
				}
				else if (theta_D < theta)
				{
					dtheta = theta - theta_D;
				}
			}
		}
		return false;
	}

	double GetSemiMajorAxis(VECTOR3 R, VECTOR3 V, double mu)
	{
		double eps = length(V) * length(V) / 2.0 - mu / length(R);
		return -mu / (2.0 * eps);
	}

	double GetMeanMotion(VECTOR3 R, VECTOR3 V, double mu)
	{
		return sqrt(mu / pow(GetSemiMajorAxis(R, V, mu), 3));
	}

	double REVTIM(VECTOR3 R, VECTOR3 V, const GlobalConstants &cnst)
	{
		double AINV, r, eta, SINLAT;

		r = length(R);
		AINV = 2.0 / r - pow(length(V), 2) / cnst.mu;

		SINLAT = R.z / r;
		AINV = AINV + cnst.J2 * pow(cnst.R_E, 2) / pow(r, 3) * (1.0 - 3.0 * pow(SINLAT, 2)) +
			cnst.J3 * pow(cnst.R_E, 3) / pow(r, 4) * (3.0 * SINLAT - 5.0 * pow(SINLAT, 3)) -
			cnst.J4 * pow(cnst.R_E, 4) / pow(r, 5) * (3.0 - 30.0 * pow(SINLAT, 2) + 35.0 * pow(SINLAT, 4));

		eta = cnst.sqrt_mu*sqrt(pow(AINV, 3));
		return PI2 / eta;
	}

	void periapo(VECTOR3 R, VECTOR3 V, double mu, double& apo, double& peri)
	{
		double a, e, epsilon;
		VECTOR3 Ex, H;

		H = crossp(R, V);

		Ex = crossp(V, H) / mu - R / length(R);
		e = length(Ex);
		epsilon = 0.5 * pow(length(V), 2) - mu / length(R);
		a = -0.5 * mu / epsilon;
		peri = (-e + 1.0) * a;
		apo = (e + 1.0) * a;
		if (apo < 0)
		{
			apo = _DMAX;
		}
	}

	void REL_COMP(VECTOR3 R_T_INER, VECTOR3 V_T_INER, VECTOR3& R_S_INER, VECTOR3& R_REL)
	{
		VECTOR3 V_S_INER, V_REL;
		V_REL = _V(0, 0, 0);
		REL_COMP(false, R_T_INER, V_T_INER, R_S_INER, V_S_INER, R_REL, V_REL);
	}

	void REL_COMP(bool INER_TO_LVC, VECTOR3 R_T_INER, VECTOR3 V_T_INER, VECTOR3& R_S_INER, VECTOR3& V_S_INER, VECTOR3& R_REL, VECTOR3& V_REL)
	{
		MATRIX3 MAT_M50_LVIR;
		VECTOR3 V1, V2, V3, OMEGA_LV_PROX, RTS_M50, VTS_M50, RTS_LVIR, VTS_LVIR, RTS_LV, VTS_LV, VTAN;
		double RT_MAG, ZCON, THETA, THETA_DOT, OMEGA_PROX;

		RT_MAG = length(R_T_INER);
		V1 = unit(crossp(crossp(R_T_INER, V_T_INER), R_T_INER));
		V2 = -unit(crossp(R_T_INER, V_T_INER));
		V3 = -unit(R_T_INER);
		MAT_M50_LVIR = _M(V1.x, V1.y, V1.z, V2.x, V2.y, V2.z, V3.x, V3.y, V3.z);
		VTAN = V_T_INER - unit(R_T_INER) * dotp(R_T_INER, V_T_INER) / RT_MAG;
		OMEGA_PROX = length(VTAN) / RT_MAG;
		OMEGA_LV_PROX = _V(0, -1, 0) * OMEGA_PROX;
		if (INER_TO_LVC)
		{
			RTS_M50 = R_S_INER - R_T_INER;
			VTS_M50 = V_S_INER - V_T_INER;
			RTS_LVIR = mul(MAT_M50_LVIR, RTS_M50);
			VTS_LVIR = mul(MAT_M50_LVIR, VTS_M50);
			RTS_LV = RTS_LVIR;
			VTS_LV = VTS_LVIR - crossp(OMEGA_LV_PROX, RTS_LVIR);
			ZCON = RT_MAG - RTS_LV.z;
			THETA = atan2(RTS_LV.x, ZCON);
			R_REL = _V(RT_MAG * THETA, RTS_LV.y, RT_MAG - ZCON / cos(THETA));
			THETA_DOT = pow(cos(THETA), 2) * (VTS_LV.x * ZCON + RTS_LV.x * VTS_LV.z) / (pow(ZCON, 2));
			V_REL = _V(RT_MAG * THETA_DOT, VTS_LV.y, (VTS_LV.z - ZCON * THETA_DOT * tan(THETA)) / cos(THETA));
		}
		else
		{
			THETA = R_REL.x / RT_MAG;
			THETA_DOT = V_REL.x / RT_MAG;
			ZCON = RT_MAG - R_REL.z;
			RTS_LV = _V(ZCON * sin(THETA), R_REL.y, RT_MAG - ZCON * cos(THETA));
			VTS_LV = _V(ZCON * THETA_DOT * cos(THETA) - V_REL.z * sin(THETA), V_REL.y, RTS_LV.x * THETA_DOT + V_REL.z * cos(THETA));
			RTS_LVIR = RTS_LV;
			VTS_LVIR = VTS_LV + crossp(OMEGA_LV_PROX, RTS_LVIR);
			RTS_M50 = tmul(MAT_M50_LVIR, RTS_LVIR);
			VTS_M50 = tmul(MAT_M50_LVIR, VTS_LVIR);
			R_S_INER = R_T_INER + RTS_M50;
			V_S_INER = V_T_INER + VTS_M50;
		}
	}

	void RADUP(VECTOR3 R_W, VECTOR3 V_W, VECTOR3 R_C, double mu, VECTOR3& R_W1, VECTOR3& V_W1)
	{
		double theta, dt;

		theta = sign(dotp(crossp(R_W, R_C), crossp(R_W, V_W))) * acos2(dotp(R_W / length(R_W), R_C / length(R_C)));
		time_theta(R_W, V_W, theta, mu, R_W1, V_W1, dt);
	}

	void ITER(double& c, int& s, double e, double& p, double& x, double& eo, double& xo, double dx0)
	{
		double dx;

		if (c == 0)
		{
			dx = dx0;
			c = c + 1.0; eo = e; xo = x; x = x - dx;
		}
		else if (c == 0.5)
		{
			dx = e / p;
			c = c + 1.0; eo = e; xo = x; x = x - dx;
		}
		else
		{
			if (e - eo == 0)
			{
				dx = 3.0 * dx0;
				c = c + 1.0; eo = e; xo = x; x = x - dx;
			}
			else
			{
				p = (e - eo) / (x - xo);
				if (c > 30)
				{
					s = 1;
				}
				else
				{
					dx = e / p;
					c = c + 1.0; eo = e; xo = x; x = x - dx;
				}
			}
		}
	}

	void PIFAAP(double a, double e, double i, double f, double u, double r, double R_E, double J2, double& r_A, double& r_P)
	{
		double a_ref, e_ref, p_ref, p, K1, K2, df, r_1, r_2;

		a_ref = r + 1.5*J2 * R_E * (1.0 - 3.0 / 2.0 * pow(sin(i), 2) + 5.0 / 6.0 * pow(sin(i), 2) * cos(2.0 * u));
		e_ref = 1.0 - r / a_ref;
		p_ref = a_ref * (1.0 - e_ref * e_ref);
		p = a * (1.0 - e * e);
		K1 = e / sqrt(p);
		K2 = e_ref / sqrt(p_ref);
		df = atan2(K1 * sin(f), K2 - K1 * cos(f));
		r_1 = p / (1.0 + e * cos(f + df)) - p_ref / (1.0 + e_ref * cos(df)) + r;
		r_2 = p / (1.0 - e * cos(f + df)) - p_ref / (1.0 - e_ref * cos(df)) + r;
		if (r_1 >= r_2)
		{
			r_A = r_1;
			r_P = r_2;
		}
		else
		{
			r_A = r_2;
			r_P = r_1;
		}
	}

	void SecularRates(StateVector sv, CELEMENTS coe, const GlobalConstants& cnst, double &ldot, double &gdot)
	{
		//sv and coe are identical, just provided for convenience

		//Short period J2
		double r, f, u, p, ar3, da_sp, theta, theta2, theta4, eta, eta2, k2, gamma2, gamma2_apo;
		//Mean semi-major axis
		double MA;
		//Mean inclination
		double MI;
		double n0;

		r = length(sv.R);
		f = MeanToTrueAnomaly(coe.l, coe.e);
		u = f + coe.g;
		p = coe.a * (1.0 - coe.e * coe.e);

		ar3 = pow(coe.a / r, 3);
		da_sp = cnst.J2 * pow(cnst.R_E, 2) / coe.a * (ar3 - 1.0 / pow(1.0 - pow(coe.e, 2), 1.5) + (-ar3 + 1.0 / pow(1.0 - pow(coe.e, 2), 1.5) + ar3 * cos(2.0 * u)) * 3.0 * pow(sin(coe.i), 2) / 2.0);
		MA = coe.a - da_sp;

		//Constants
		n0 = sqrt(cnst.mu / pow(MA, 3));
		MI = coe.i - 1.5 * cnst.J2 * pow(cnst.R_E, 2) / (4.0 * p * p) * sin(2.0 * coe.i) * cos(2.0 * u);
		theta = cos(MI);
		theta2 = theta * theta;
		theta4 = theta2 * theta2;
		eta = sqrt(1.0 - pow(coe.e, 2));
		eta2 = eta * eta;
		k2 = cnst.J2 * pow(cnst.R_E, 2) / 2.0;
		gamma2 = k2 / pow(MA, 2);
		gamma2_apo = gamma2 / pow(eta, 4);

		//Mean anomaly rate
		ldot = PI2 / REVTIM(sv.R, sv.V, cnst);

		//Argument of perigee rate
		gdot = n0 * (gamma2_apo * (1.5 * (5.0 * theta2 - 1.0) + 3.0 / 32.0 * gamma2_apo * (25.0 * eta2 + 24.0 * eta - 35.0 + (90.0 - 192.0 * eta - 126.0 * eta2) * theta2 + (385.0 + 360.0 * eta + 45.0 * eta2) * theta4)));
	}

	double normalize_angle(const double value, const double start, const double end)
	{
		const double width = end - start;
		const double offsetValue = value - start;

		return (offsetValue - (floor(offsetValue / width) * width)) + start;
	}

	int Date2JD(int Y, int M, int D)
	{
		return (1461 * (Y + 4800 + (M - 14) / 12)) / 4 + (367 * (M - 2 - 12 * ((M - 14) / 12))) / 12 - (3 * ((Y + 4900 + (M - 14) / 12) / 100)) / 4 + D - 32075;
	}

	double Date2MJD(int Y, int D, int H, int M, double S)
	{
		double MJD, timeofday;
		int JD;

		JD = Date2JD(Y, 1, 1);
		JD = JD + D - 1;
		MJD = (double)JD - 2400000.5;
		timeofday = 3600.0 * (double)H + 60.0 * (double)M + S;
		timeofday /= 24.0 * 3600.0;
		MJD = MJD + timeofday - 0.5;
		return MJD;
	}

	int dayofyear(int year, int month, int day)
	{
		static const int days_in_prev_months[12] = { 0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334 };

		return days_in_prev_months[month - 1] + (isleapyear(year) && (month > 2)) + day;
	}

	bool isleapyear(int a)
	{
		return ((a % 4 == 0 && a % 100 != 0) || a % 400 == 0);
	}

	void SS2HHMMSS(double val, double& hh, double& mm, double& ss)
	{
		val = round(val);
		hh = floor(val / 3600.0);
		mm = floor(fmod(val, 3600.0) / 60.0);
		ss = fmod(val, 60.0);
	}

	void GMT2String(char* buf, double GMT, int Day)
	{
		//Format: DDD:HH:MM:SS.SSS
		GMT = round(GMT * 1000.0) / 1000.0;
		sprintf_s(buf, 100, "%03.0lf:%02.0lf:%02.0lf:%06.3lf", floor(GMT / 86400.0) + (double)Day, floor(fmod(GMT, 86400.0) / 3600.0), floor(fmod(GMT, 3600.0) / 60.0), fmod(GMT, 60.0));
	}

	void GMT2String3(char* buf, double GMT, int Day)
	{
		//Format: DD:HH:MM:SS
		GMT = round(GMT);
		sprintf_s(buf, 100, "%02.0lf:%02.0lf:%02.0lf:%02.0lf", floor(GMT / 86400.0) + (double)Day, floor(fmod(GMT, 86400.0) / 3600.0), floor(fmod(GMT, 3600.0) / 60.0), fmod(GMT, 60.0));
	}

	void GMT2String4(char* buf, double GMT)
	{
		//Format: HH:MM:SS.S
		GMT = round(GMT * 10.0) / 10.0;
		sprintf_s(buf, 100, "%02.0lf:%02.0lf:%04.1lf", floor(GMT / 3600.0), floor(fmod(GMT, 3600.0) / 60.0), fmod(GMT, 60.0));
	}

	void MET2String(char* buf, double MET)
	{
		bool neg = (MET < 0.0);

		MET = round(abs(MET) * 1000.0) / 1000.0;

		if (neg == false)
		{
			sprintf_s(buf, 100, "%03.0f:%02.0f:%02.0f:%06.3f", floor(MET / 86400.0), floor(fmod(MET, 86400.0) / 3600.0), floor(fmod(MET, 3600.0) / 60.0), fmod(MET, 60.0));
		}
		else
		{
			sprintf_s(buf, 100, "-%03.0f:%02.0f:%02.0f:%06.3f", floor(MET / 86400.0), floor(fmod(MET, 86400.0) / 3600.0), floor(fmod(MET, 3600.0) / 60.0), fmod(MET, 60.0));
		}
	}

	MATRIX3 GetRotationMatrix(const GlobalConstants& cnst, double t)
	{
		double L_rel, phi, CL, SL, CP, SP;
		MATRIX3 Rot3, R_rel, R_rot;

		L_rel = cnst.L_0 + PI2 * (t - LAN_MJD) / cnst.T_p;
		CL = cos(L_rel);
		SL = sin(L_rel);
		Rot3 = _M(CL, 0, -SL, 0, 1, 0, SL, 0, CL);
		R_rel = mul(Rot3, cnst.R_obl);
		phi = cnst.phi_0 + PI2 * (t - LAN_MJD) / cnst.T_s + (cnst.L_0 - L_rel) * cos(cnst.e_rel);
		CP = cos(phi);
		SP = sin(phi);
		R_rot = _M(CP, 0, -SP, 0, 1, 0, SP, 0, CP);
		return mul(cnst.R_ref, mul(R_rel, R_rot));
	}

	MATRIX3 GetObliquityMatrix(const GlobalConstants& cnst, double t)
	{
		MATRIX3 Rot, Rot5, Rot6;
		VECTOR3 s;
		double e_ecl, L_ecl;

		Rot = GetRotationMatrix(cnst, t);

		s = mul(Rot, _V(0.0, 1.0, 0.0));
		e_ecl = acos(s.y);
		L_ecl = atan(-s.x / s.z);
		Rot5 = _M(cos(L_ecl), 0.0, -sin(L_ecl), 0.0, 1.0, 0.0, sin(L_ecl), 0.0, cos(L_ecl));
		Rot6 = _M(1.0, 0.0, 0.0, 0.0, cos(e_ecl), -sin(e_ecl), 0.0, sin(e_ecl), cos(e_ecl));
		return mul(Rot5, Rot6);
	}

	double acos2(double _X)
	{
		return acos(fmin(1.0, fmax(-1.0, _X)));
	}
}