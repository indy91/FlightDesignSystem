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

#include "ShuttleLaunchWindowProcessor.h"
#include "LaunchWindowProcessor.h"
#include "EnckeIntegrator.h"
#include "Constants.h"

namespace ShuttleLWP
{
	ShuttleLWPInputs::ShuttleLWPInputs()
	{
		CCD = 2.0;
		CAREA = 2500.0 * pow(OrbMech::FT2M, 2);
		CWHT = 251679.0 * OrbMech::LBS;
		NS = 0;
		LATLS = 28.627 * OrbMech::RAD;
		LONGLS = 279.379 * OrbMech::RAD;
		PFT = 8.0 * 60.0 + 30.0;
		PFA = 14.4 * OrbMech::RAD;
		YSMAX = 14.0 * OrbMech::RAD;
		RINS = 6528178.0;
		VINS = 7835.13;
		GAMINS = 0.0;
		DTOPT = -(5.0 * 60.0 + 40.0);
		DTO = -5.0 * 60.0;
		DTC = 5.0 * 60.0;
		NEGTIV = 0;
		WRAP = 0;

		GMTLO = 0.0;

		DirectInsertion = true;
		OMS2Phase[0] = OMS2Phase[1] = false;
		OMS2PhaseAngle[0] = OMS2PhaseAngle[1] = 0.0;

		DTIG_ET_SEP = 12.0;
		DV_ET_SEP = _V(5.0, 0.0, -5.0) * OrbMech::FT2M;

		DTIG_MPS = 1.0 * 60.0 + 54.0;
		DV_MPS = _V(5.0, 0.0, -5.0) * OrbMech::FT2M;

		OMS1.DTIG = 2.0 * 60.0;
		OMS1.C1 = 0.0;
		OMS1.C2 = 0.0;
		OMS1.HTGT = 158.0 * OrbMech::NM;
		OMS1.THETA = 154.0 * OrbMech::RAD;

		OMS2.DTIG = 35.0 * 60.0 + 1.0;
		OMS2.C1 = 0.0;
		OMS2.C2 = 0.0;
		OMS2.HTGT = 110.8 * OrbMech::NM;
		OMS2.THETA = 338.0 * OrbMech::RAD;

		ET_Area = 598.0 * pow(OrbMech::FT2M, 2);
		ET_CD = 2.0;
		ET_Weight = 75000.0 * OrbMech::LBS;
	}

	PEG4A::PEG4A(double RE, double MU, double J2, double J3, double J4) :
		RADIUS_EARTH_EQUATOR(RE),
		EARTH_MU(MU)
	{
		JArr[0] = J2;
		JArr[1] = J3;
		JArr[2] = J4;

		RGD = VGD = RT = _V(0, 0, 0);
		TGD = 0.0;
	}

	int PEG4A::OMSBurnPrediction(VECTOR3 RGD, VECTOR3 VGD, double TGD, VECTOR3 RLS_TEG, double C1, double C2, double HTGT, double THETA, double FT, double VEX, double M)
	{
		this->RGD = RGD;
		this->VGD = VGD;
		this->TGD = TGD;
		this->C1 = C1;
		this->C2 = C2;
		this->HTGT = HTGT;
		this->THETA = THETA;
		this->FT = FT;
		this->VEX = VEX;
		this->M = M;

		RT = HTHETA_M50_TGT_TSK(RLS_TEG);

		RP = RGD;
		VP = VGD;
		TP = TGD;
		THETA_DOT = sqrt(EARTH_MU / (pow(length(RGD), 3)));
		LAMDXZ = THETA_DOT * K_LAMDXZ;
		TGO = 0;
		VGO = _V(0, 0, 0);

		SCONV = false;

		//Init
		if (VelocityToBeGainedSubtask()) return 1;

		ATR = FT / M;

		while (SCONV == false)
		{
			VGOMAG = length(VGO);
			TimeToGoSubtask();
			ThrustIntegralSubtask();
			ReferenceThrustVectorsSubtask();
			BurnoutStateVectorPredictionSubtask();
			if (VelocityToBeGainedSubtask()) return 1;
			ConvergenceCheckSubtask();
		}

		VECTOR3 RJ2, VJ2, RC3, VC3;
		ASCENT_PRECISE_PREDICTOR(RC1, VC1, TGD, TGD + TGO, 2.0, 2, RC3, VC3);
		VJ2 = VC3 - VC2;
		RJ2 = RC3 - RC2;
		RP = RP + RJ2;
		VD = VD + VJ2;
		MBO = M - FT / VEX * TGO;

		return 0;
	}

	void PEG4A::GetOutputA(VECTOR3& RP, VECTOR3& VD, VECTOR3& VGO, double& TGO, double& MBO) const
	{
		RP = this->RP;
		VD = this->VD;
		VGO = this->VGO;
		TGO = this->TGO;
		MBO = this->MBO;
	}

	VECTOR3 PEG4A::HTHETA_M50_TGT_TSK(VECTOR3 RLS_TEG) const
	{
		VECTOR3 IDR, IR, IRT, RT;
		double THETA_LS, DTHETA, RTMAG;

		IDR = unit(crossp(RGD, crossp(VGD, RGD)));
		IR = unit(RGD);
		THETA_LS = atan2(-dotp(RLS_TEG, IDR), dotp(RLS_TEG, IR));
		if (THETA_LS < 0)
		{
			THETA_LS = THETA_LS + OrbMech::PI2;
		}
		DTHETA = THETA - THETA_LS;
		IRT = IR * cos(DTHETA) + IDR * sin(DTHETA);
		RTMAG = HTGT + RE;
		RT = IRT * RTMAG;
		return RT;
	}

	int PEG4A::VelocityToBeGainedSubtask()
	{
		double DTCOAST, RHOMAG;

		IY = unit(crossp(VP, RP));
		//RT = RT - IY * dot(RT, IY);
		if (LTVCON_TSK(RP, RT, IY, C1, C2, VD)) return 1;
		DTCOAST = THETA / THETA_DOT;
		RHOMAG = 1.0 / (1.0 + 0.75 * TGO / DTCOAST);
		VMISS = VP - VD;
		VGO = VGO - VMISS * RHOMAG;
		return 0;
	}

	void PEG4A::TimeToGoSubtask()
	{
		VRATIO = VGOMAG / (6.0 * VEX);
		TGO = VGOMAG / (ATR * (1.0 + 3.0 * VRATIO + 3.0 * VRATIO * VRATIO));
		TP = TGD + TGO;
	}

	void PEG4A::ThrustIntegralSubtask()
	{
		JOL = 0.5 * TGO * (1.0 + VRATIO);
		S = 0.5 * VGOMAG * TGO * (1.0 - VRATIO);
		QPRIME = VGOMAG * TGO * TGO / 12.0;
	}

	void PEG4A::ReferenceThrustVectorsSubtask()
	{
		LAM = VGO / VGOMAG;
		LAMD = crossp(LAM, IY) * LAMDXZ;
	}

	void PEG4A::BurnoutStateVectorPredictionSubtask()
	{
		VECTOR3 RGO, RGRAV, VGRAV;

		RGO = LAMD * QPRIME + LAM * S;
		RC1 = RGD - RGO / 10.0 - VGO * TGO / 30.0;
		VC1 = VGD + RGO * 6.0 / 5.0 / TGO - VGO / 10.0;
		ASCENT_PRECISE_PREDICTOR(RC1, VC1, TGD, TP, 2.0, 0, RC2, VC2);
		VGRAV = VC2 - VC1;
		RGRAV = RC2 - RC1 - VC1 * TGO;
		RP = RGD + VGD * TGO + RGRAV + RGO;
		VP = VGD + VGRAV + VGO;
	}

	void PEG4A::ConvergenceCheckSubtask()
	{
		VGOMAG = length(VGO);
		if (length(VMISS) <= KMISS * VGOMAG)
		{
			SCONV = true;
		}
		else
		{
			SCONV = false;
		}
	}

	int PEG4A::LTVCON_TSK(VECTOR3 RD, VECTOR3 R1, VECTOR3 IY, double C1, double C2, VECTOR3& VD) const
	{
		double R0_MAG, R1_MAG, K, Z, W, A, B, C, THETA, SIN_THETA, D, VH1, VR1;

		R0_MAG = length(RD);
		R1_MAG = length(R1);
		K = (R1_MAG - R0_MAG) / R0_MAG;
		Z = R0_MAG * R1_MAG - dotp(RD, R1);

		W = -dotp(crossp(RD, R1), IY) / Z;
		A = K * (1.0 + W * W) + 2.0 * (1.0 - C2 * W);
		B = C1 * W;
		C = 2.0 * EARTH_MU / R1_MAG;
		THETA = OrbMech::PI + atan2(-2.0 * W, 1.0 - W * W);
		SIN_THETA = 2.0 * W / (1.0 * W * W);
		D = B * B + A * C;

		if (D < 0.0) return 1;

		VH1 = C / (-B + sqrt(D));
		VR1 = C1 + C2 * VH1;
		VD = (RD * (K * W * VH1 - VR1) + crossp(RD, IY) * (1.0 + K) * VH1) / R0_MAG;

		return 0;
	}

	void PEG4A::ASCENT_PRECISE_PREDICTOR(VECTOR3 R_INIT, VECTOR3 V_INIT, double T_INIT, double T_FINAL, double DT_MAX, int GMD_PRED, VECTOR3& R_FINAL, VECTOR3& V_FINAL) const
	{
		VECTOR3 G_PREVIOUS, G_FINAL;
		double STEP_SIZE, T, R_INV;
		int NUMBER_STEPS;

		R_FINAL = R_INIT;
		V_FINAL = V_INIT;

		NUMBER_STEPS = (int)std::max(round(abs(T_FINAL - T_INIT) / DT_MAX), 1.0);
		STEP_SIZE = (T_FINAL - T_INIT) / NUMBER_STEPS;
		T = T_INIT;

		if (GMD_PRED == 0)
		{
			CENTRAL(R_FINAL, G_PREVIOUS, R_INV);
		}
		else
		{
			G_PREVIOUS = ACCEL_ENTRY(R_FINAL);
		}

		for (int I = 1; I <= NUMBER_STEPS; I++)
		{
			T = T + STEP_SIZE;
			R_FINAL = R_FINAL + (V_FINAL + G_PREVIOUS * 0.5 * STEP_SIZE) * STEP_SIZE;

			if (GMD_PRED == 0)
			{
				CENTRAL(R_FINAL, G_FINAL, R_INV);
			}
			else
			{
				G_FINAL = ACCEL_ENTRY(R_FINAL);
			}

			V_FINAL = V_FINAL + (G_PREVIOUS + G_FINAL) * STEP_SIZE * 0.5;
			R_FINAL = R_FINAL + (G_FINAL - G_PREVIOUS) * STEP_SIZE * STEP_SIZE / 6.0;
			G_PREVIOUS = G_FINAL;
		}
	}

	void PEG4A::CENTRAL(VECTOR3 R, VECTOR3& ACCEL, double& R_INV) const
	{
		R_INV = 1.0 / length(R);
		ACCEL = -R * EARTH_MU * pow(R_INV, 3);
	}

	VECTOR3 PEG4A::ACCEL_ENTRY(VECTOR3 R) const
	{
		VECTOR3 ACCEL, U_R, U_Z, G_B;
		double R_INV, cos_lat, P[4];

		CENTRAL(R, ACCEL, R_INV);

		U_R = R * R_INV;
		U_Z = _V(0, 0, 1);
		cos_lat = dotp(U_R, U_Z);

		P[0] = 3.0 * cos_lat;
		P[1] = 0.5 * (15.0 * cos_lat * cos_lat - 3.0);
		P[2] = 1.0 / 3.0 * (7.0 * cos_lat * P[1] - 4.0 * P[0]);
		P[3] = 1.0 / 4.0 * (9.0 * cos_lat * P[2] - 5.0 * P[1]);

		G_B = _V(0, 0, 0);
		for (int i = 2; i <= 4; i++)
		{
			G_B = G_B + (U_R * P[i - 1] - U_Z * P[i - 2]) * pow(RADIUS_EARTH_EQUATOR * R_INV, i) * JArr[i - 2];
		}

		G_B = G_B * EARTH_MU * pow(R_INV, 2);
		ACCEL = ACCEL + G_B;
		return ACCEL;
	}

	ShuttleLaunchWindowProcessor::ShuttleLaunchWindowProcessor(OrbMech::GlobalConstants& cnst, OrbMech::SessionConstants& scnst) : globconst(cnst), sesconst(scnst)
	{
		VGO_OMS1 = VGO_OMS2 = _V(0, 0, 0);
		PA_OMS2 = 0.0;
	}

	int ShuttleLaunchWindowProcessor::CalculateLW(const ShuttleLWPInputs& in, ShuttleLWPOutputs& out)
	{
		inp = in;

		LWP::LWPInputs lwin;
		LWP::LWPOutputs lwout;

		lwin.PRINT = false;
		lwin.TRGVEC = in.sv_T;
		lwin.CKFactor = in.CCD / 2.0;
		lwin.CAREA = in.CAREA;
		lwin.CWHT = in.CWHT;
		lwin.RINS = in.RINS;
		lwin.VINS = in.VINS;
		lwin.GAMINS = in.GAMINS;
		lwin.PFA = in.PFA;
		lwin.PFT = in.PFT;
		lwin.LATLS = in.LATLS;
		lwin.LONGLS = in.LONGLS;
		lwin.NS = in.NS;
		lwin.DAY = 0;
		lwin.LPT = 0;
		lwin.STABLE = false;
		lwin.DTGRR = 0.0;
		lwin.INSCO = 1;
		lwin.DELNOF = true;
		lwin.NEGTIV = in.NEGTIV;
		lwin.WRAP = in.WRAP;
		lwin.DTOPT = -in.DTOPT;
		lwin.LOT = 6;

		LWP::LaunchWindowProcessor lwp(globconst, sesconst);

		lwp.Calculate(lwin, lwout);
		if (lwout.SUMTAB.Error) return lwout.SUMTAB.Error;

		//Store optimum launch time
		double GMTLO_OPT, PHASE_OPT, GMTLO_OPEN, PHASE_OPEN, GMTLO_CLOSE, PHASE_CLOSE;

		GMTLO_OPT = lwout.GMTLO;
		PHASE_OPT = lwout.SUMTAB.PA;

		//Rerun launch targeting only for planar open
		lwin.LW = 1;
		lwin.LOT = 1;
		lwin.GMTLOR = GMTLO_OPT + in.DTO;

		lwp.Calculate(lwin, lwout);
		if (lwout.SUMTAB.Error) return lwout.SUMTAB.Error;

		GMTLO_OPEN = lwout.GMTLO;
		PHASE_OPEN = lwout.SUMTAB.PA;

		//Rerun launch targeting only for planar closing
		lwin.LOT = 1;
		lwin.GMTLOR = GMTLO_OPT + in.DTC;

		lwp.Calculate(lwin, lwout);
		if (lwout.SUMTAB.Error) return lwout.SUMTAB.Error;

		GMTLO_CLOSE = lwout.GMTLO;
		PHASE_CLOSE = lwout.SUMTAB.PA;

		//Run LWP for phase angle at OMS-2
		for (int i = 0; i < 2; i++)
		{
			if (in.OMS2Phase[i])
			{
				lwin.LOT = 2;
				lwin.OFFSET = in.OMS2PhaseAngle[i] + 4.4 * OrbMech::RAD; // TBD
				lwin.GMTLOR = GMTLO_OPT;

				lwp.Calculate(lwin, lwout);
				if (lwout.SUMTAB.Error) return lwout.SUMTAB.Error;

				out.GMTLO_PHASE[i] = lwout.GMTLO;
			}
		}

		// Set output
		out.GMTLO_OPT = GMTLO_OPT;
		out.PHASE_OPT = PHASE_OPT;
		out.GMTLO_OPEN = GMTLO_OPEN;
		out.PHASE_OPEN = PHASE_OPEN;
		out.GMTLO_CLOSE = GMTLO_CLOSE;
		out.PHASE_CLOSE = PHASE_CLOSE;

		return 0;
	}

	int ShuttleLaunchWindowProcessor::CalculateLT(const ShuttleLWPInputs& in, ShuttleLTPOutputs& out)
	{
		inp = in;

		LWP::LWPInputs lwin;
		LWP::LWPOutputs lwout;

		lwin.PRINT = false;
		lwin.TRGVEC = in.sv_T;
		lwin.CKFactor = in.CCD / 2.0;
		lwin.CAREA = in.CAREA;
		lwin.CWHT = in.CWHT;
		lwin.NS = in.NS;
		lwin.RINS = in.RINS;
		lwin.VINS = in.VINS;
		lwin.GAMINS = in.GAMINS;
		lwin.PFA = in.PFA;
		lwin.PFT = in.PFT;
		lwin.LATLS = in.LATLS;
		lwin.LONGLS = in.LONGLS;
		lwin.DAY = 0;
		lwin.LPT = 0;
		lwin.STABLE = false;
		lwin.DTGRR = 0.0;
		lwin.INSCO = 1;
		lwin.DELNOF = true;
		lwin.NEGTIV = in.NEGTIV;
		lwin.WRAP = in.WRAP;
		lwin.DTOPT = -in.DTOPT;
		lwin.LOT = 1;
		lwin.LW = 1;
		lwin.GMTLOR = in.GMTLO;

		LWP::LaunchWindowProcessor lwp(globconst, sesconst);

		lwp.Calculate(lwin, lwout);
		if (lwout.SUMTAB.Error) return lwout.SUMTAB.Error;

		sv_MECO = lwout.PVTABC;

		// Simulate through OMS-2
		OMS2();
		
		// Outputs
		out.Error = 0;
		out.sv_C = sv_MECO;
		out.sv_C_OMS1 = sv_OMS1_post;
		out.sv_T = in.sv_T;
		out.OMS1 = in.OMS1;
		out.OMS2 = in.OMS2;
		out.GMTLO = lwout.GMTLO;
		out.MET_MECO = lwout.SUMTAB.TINS - lwout.GMTLO;
		out.VMECO = lwout.SUMTAB.VIGM;
		out.RMECO = lwout.SUMTAB.RIGM;
		out.GMECO = lwout.SUMTAB.GIGM;
		out.IMECO = lwout.SUMTAB.IIGM;
		out.PHASE_MECO = lwout.SUMTAB.PA;
		OrbMech::OPS3_ORB_ALT_TSK(sv_MECO.R, sv_MECO.V, globconst, out.HA_MECO, out.HP_MECO);
		out.LONG_MECO = atan2(sv_MECO.R.y, sv_MECO.R.x) - globconst.w_E * sv_MECO.GMT;
		out.LONG_MECO = OrbMech::normalize_angle(out.LONG_MECO, -OrbMech::PI, OrbMech::PI);
		out.OMS1_TIG = sv_OMS1_pre.GMT - lwout.GMTLO;
		out.OMS1_DV = length(VGO_OMS1);
		OrbMech::OPS3_ORB_ALT_TSK(sv_OMS1_post.R, sv_OMS1_post.V, globconst, out.OMS1_HA, out.OMS1_HP);
		out.OMS2_TIG = sv_OMS2_pre.GMT - lwout.GMTLO;
		out.OMS2_DV = length(VGO_OMS2);
		OrbMech::OPS3_ORB_ALT_TSK(sv_OMS2_post.R, sv_OMS2_post.V, globconst, out.OMS2_HA, out.OMS2_HP);
		out.OMS2_NODE = CalculateNode(sv_OMS2_post);
		out.OMS2_PHASE = PA_OMS2;
		out.OMS2_PERIOD = OrbMech::REVTIM(sv_OMS2_post.R, sv_OMS2_post.V, globconst);
		
		// Update target to ascending node
		OrbMech::StateVector sv_T_AN;

		if (UpdateToArgumentOfLatitude(lwout.PVTABT[1], 0.0, sv_T_AN)) return 1;

		OrbMech::OPS3_ORB_ALT_TSK(sv_T_AN.R, sv_T_AN.V, globconst, out.TGT_HA, out.TGT_HP);
		out.TGT_LONG = atan2(sv_T_AN.R.y, sv_T_AN.R.x) - globconst.w_E * sv_T_AN.GMT;
		out.TGT_LONG = OrbMech::normalize_angle(out.TGT_LONG, -OrbMech::PI, OrbMech::PI);
		out.TGT_DELN = lwout.SUMTAB.DELNO;
		out.TGT_PERIOD = OrbMech::REVTIM(sv_T_AN.R, sv_T_AN.V, globconst);

		out.IY_MECO = unit(crossp(sv_MECO.V, sv_MECO.R));
		out.IY_OMS1 = unit(crossp(sv_OMS1_post.V, sv_OMS1_post.R));
		out.IY_OMS2 = unit(crossp(sv_OMS2_post.V, sv_OMS2_post.R));

		out.NODE_SLOPE = NodeSlope(sv_MECO, lwout.PVTABT[1]);

		return 0;
	}

	int ShuttleLaunchWindowProcessor::Update(const OrbMech::StateVector& sv1, double dt, OrbMech::StateVector& sv2) const
	{
		EnckeIntegratorInput enckein;
		EnckeIntegratorOutput enckeout;

		enckein.R = sv1.R;
		enckein.V = sv1.V;
		enckein.GMT = sv1.GMT;
		enckein.KFactor = sv1.KFactor;
		enckein.Weight = sv1.Weight;
		enckein.Area = sv1.Area;
		enckein.dt = dt;

		EnckeIntegrator encke(globconst);

		encke.Propagate(enckein, enckeout);

		if (enckeout.Error) return 1;

		sv2.R = enckeout.R;
		sv2.V = enckeout.V;
		sv2.GMT = enckeout.GMT;
		sv2.Area = enckein.Area;
		sv2.KFactor = enckein.KFactor;
		sv2.Weight = enckein.Weight;

		return 0;
	}

	int ShuttleLaunchWindowProcessor::UpdateToArgumentOfLatitude(const OrbMech::StateVector& sv1, double u_desired, OrbMech::StateVector& sv2) const
	{
		OrbMech::StateVector sv_temp;
		double eta, U, DU, dt;
		int i, imax;

		i = 0;
		imax = 10;
		sv_temp = sv1;
		eta = OrbMech::PI2 / OrbMech::REVTIM(sv1.R, sv1.V, globconst);

		do
		{
			U = OrbMech::ArgLat(sv_temp.R, sv_temp.V);
			DU = u_desired - U;
			DU = OrbMech::normalize_angle(DU, -OrbMech::PI, OrbMech::PI);
			if (i == 0 && DU < 0.0)
			{
				DU += OrbMech::PI2;
			}
			dt = DU / eta;
			if (Update(sv_temp, dt, sv2)) return 1;
			sv_temp = sv2;
			i++;
		} while (abs(DU) > 0.01 * OrbMech::RAD && i < imax);
		sv2 = sv_temp;
		return 0;
	}

	double ShuttleLaunchWindowProcessor::CalculateNode(const OrbMech::StateVector& sv_C) const
	{
		VECTOR3 H, N;
		double Node;

		H = crossp(sv_C.R, sv_C.V);
		N = _V(-H.y, H.x, 0.0);
		Node = acos(N.x / length(N));
		if (N.y < 0)
		{
			Node = OrbMech::PI2 - Node;
		}
		if (Node > OrbMech::PI)
		{
			Node -= OrbMech::PI2;
		}
		return Node;
	}

	double ShuttleLaunchWindowProcessor::NodeSlope(const OrbMech::StateVector& sv_C, const OrbMech::StateVector& sv_T) const
	{
		OrbMech::CELEMENTS coe_osc_C, coe_mean_C, coe_osc_T, coe_mean_T;
		double l_dot_C, g_dot_C, h_dot_C, l_dot_T, g_dot_T, h_dot_T;

		// Calculate secular rates
		OrbMech::ConvertToAEGElements(sv_C, globconst, coe_osc_C, coe_mean_C, l_dot_C, g_dot_C, h_dot_C);
		OrbMech::ConvertToAEGElements(sv_T, globconst, coe_osc_T, coe_mean_T, l_dot_T, g_dot_T, h_dot_T);

		// Calculate launch azimuth
		double arg, beta;

		arg = cos(coe_mean_T.i) / cos(inp.LATLS);
		if (abs(arg) > 1.0)
		{
			if (arg > 1.0)
			{
				arg = 1.0;
			}
			else
			{
				arg = -1.0;
			}
		}
		beta = asin(arg);
		if (inp.NS == 1)
		{
			beta = OrbMech::PI - beta;
		}

		// Project of Earth rotational rate into orbit
		double w_E_orbit;

		w_E_orbit = globconst.w_E * sin(beta);

		// Phase angle change per second
		double PA_DOT;

		PA_DOT = l_dot_T - w_E_orbit;

		// Differential nodal regression
		double DELNOD, NODE_SLOPE;

		DELNOD = -((h_dot_T - h_dot_C) * PA_DOT) / (l_dot_T - l_dot_C);
		NODE_SLOPE = -(DELNOD + h_dot_T);

		return NODE_SLOPE;
	}

	int ShuttleLaunchWindowProcessor::OMS2()
	{
		// Use sv_MECO as input

		VECTOR3 URLS;
		double GMTLO, TGO, TF, FT, VEX;
		PEG4A peg4(globconst.R_E, globconst.mu, globconst.J2, globconst.J3, globconst.J4);

		FT = 2.0 * 26700.0;
		VEX = 316.0 * 9.80665;

		GMTLO = sv_MECO.GMT - inp.PFT;
		URLS = GetURLS(GMTLO);

		//Propagate to ET sep
		Update(sv_MECO, inp.DTIG_ET_SEP, sv_ET_Sep_pre);
		//Simulate ET sep
		sv_ET_Sep_post = sv_ET_Sep_pre;
		sv_ET_Sep_post.V += tmul(OrbMech::LVLH_Matrix(sv_ET_Sep_pre.R, sv_ET_Sep_pre.V), inp.DV_ET_SEP);

		if (inp.DirectInsertion)
		{
			//Propagate to MPS dump
			TF = sv_MECO.GMT + inp.DTIG_MPS;
			Update(sv_ET_Sep_post, TF - sv_ET_Sep_post.GMT, sv_OMS1_pre);

			//Simulate MPS dump
			sv_OMS1_post = sv_OMS1_pre;
			sv_OMS1_post.V += tmul(OrbMech::LVLH_Matrix(sv_OMS1_pre.R, sv_OMS1_pre.V), inp.DV_MPS);

			VGO_OMS1 = inp.DV_MPS;
		}
		else
		{
			//Propagate to OMS-1 TIG
			TF = sv_MECO.GMT + inp.OMS1.DTIG;
			Update(sv_ET_Sep_post, TF - sv_ET_Sep_post.GMT, sv_OMS1_pre);

			//Simulate OMS-1 burn
			if (peg4.OMSBurnPrediction(sv_OMS1_pre.R, sv_OMS1_pre.V, sv_OMS1_pre.GMT, URLS, inp.OMS1.C1, inp.OMS1.C2, inp.OMS1.HTGT, inp.OMS1.THETA, FT, VEX, sv_OMS1_pre.Weight))
			{
				return 1;
			}
			sv_OMS1_post = sv_OMS1_pre;
			peg4.GetOutputA(sv_OMS1_post.R, sv_OMS1_post.V, VGO_OMS1, TGO, sv_OMS1_post.Weight);
			sv_OMS1_post.GMT = sv_OMS1_pre.GMT + TGO;
		}

		//Propagate to OMS-2 TIG
		TF = sv_MECO.GMT + inp.OMS2.DTIG;
		Update(sv_OMS1_post, TF - sv_OMS1_post.GMT, sv_OMS2_pre);

		//Simulate OMS-2 burn
		if (peg4.OMSBurnPrediction(sv_OMS2_pre.R, sv_OMS2_pre.V, sv_OMS2_pre.GMT, URLS, inp.OMS2.C1, inp.OMS2.C2, inp.OMS2.HTGT, inp.OMS2.THETA, FT, VEX, sv_OMS2_pre.Weight))
		{
			return 1;
		}
		sv_OMS2_post = sv_OMS2_pre;
		peg4.GetOutputA(sv_OMS2_post.R, sv_OMS2_post.V, VGO_OMS2, TGO, sv_OMS2_post.Weight);
		sv_OMS2_post.GMT = sv_OMS2_pre.GMT + TGO;

		// Update target to OMS-2 burnout time
		OrbMech::StateVector sv_T_OMS2;
		Update(inp.sv_T, sv_OMS2_post.GMT - inp.sv_T.GMT, sv_T_OMS2);
		PA_OMS2 = PHANG(sv_OMS2_post, sv_T_OMS2, true);

		return 0;
	}

	VECTOR3 ShuttleLaunchWindowProcessor::GetURLS(double GMTLO) const
	{
		double LONG;

		LONG = inp.LONGLS + globconst.w_E * GMTLO;
		return _V(cos(inp.LATLS) * cos(LONG), cos(inp.LATLS) * sin(LONG), sin(inp.LATLS));
	}

	double ShuttleLaunchWindowProcessor::PHANG(const OrbMech::StateVector& sv_C, const OrbMech::StateVector& sv_T, bool wrapped) const
	{
		//wrapped: false = -180 to 180°, true = based on NEGTIV and WRAP
		double PA;
		PA = OrbMech::PHSANG(sv_T.R, sv_T.V, sv_C.R);
		if (wrapped == false) return PA;

		if (inp.NEGTIV == 0)
		{
			while (PA >= OrbMech::PI2) PA -= OrbMech::PI2;
			while (PA < 0) PA += OrbMech::PI2;
		}
		else if (inp.NEGTIV == 1)
		{
			while (PA >= 0) PA -= OrbMech::PI2;
			while (PA < -OrbMech::PI2) PA += OrbMech::PI2;
		}
		else
		{
			while (PA >= OrbMech::PI) PA -= OrbMech::PI2;
			while (PA < -OrbMech::PI) PA += OrbMech::PI2;
		}
		PA += OrbMech::PI2 * (double)inp.WRAP;

		return PA;
	}
}