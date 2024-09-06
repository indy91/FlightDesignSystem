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

#include "LaunchWindowProcessor.h"
#include "EnckeIntegrator.h"
#include "Constants.h"
#include <iostream>
#include <fstream>

namespace LWP
{
	LWPConstants::LWPConstants()
	{
		PrintFlag = false;
		CMAX = 20;
		DX = 25.0;
		tol_trans_ang = 2.e-4;
		tol_e = 6.e-4;
		tol_trav_ang = 2.e-4;
		tol_rad = 250.0 * 0.3048;
		tol_time = 1.0;
		tol_dv = 0.1 * 0.3048;
	}

	LWPInputs::LWPInputs()
	{
		LAZCOE[0] = 490.931 * OrbMech::RAD;
		LAZCOE[1] = -20.131;
		LAZCOE[2] = -1.2501;
		LAZCOE[3] = 0.0975 * OrbMech::DEG;

		CKFactor = 1.0;
		CAREA = 0.0;
		CWHT = 1.0;
		LW = 2;
		NS = 0;
		DAY = 0;
		LPT = 1;

		NEGTIV = 0;
		WRAP = 0;

		LATLS = 28.627 * OrbMech::RAD;
		LONGLS = 279.379 * OrbMech::RAD;
		PFT = 8.0 * 60.0 + 30.0;
		PFA = 14.4 * OrbMech::RAD;
		YSMAX = 14.0 * OrbMech::RAD;
		RINS = 6528178.0;
		VINS = 7835.13;
		GAMINS = 0.0;
		DTOPT = 6.0 * 60.0;
		DTGRR = 0.0;

		STABLE = false;

		TSTART = 5.0 * 60.0;
		TEND = 5.0 * 60.0;
		TSTEP = 5.0 * 60.0;

		INSCO = 1;
		LOT = 1;
		DELNOF = true;
		DELNO = 0.0;
		TRANS = 0.0;
		ANOM = 0.0;
		DHW = 0.0;
		DU = 0.0;
		GMTLOR = 0.0;
		BIAS = 0.0;
		OFFSET = 0.0;
		STARS = 0.0;
		STARE = 0.0;
		TPLANE = 0.0;

		PRINT = false;
	}

	LWPOrbitalElements::LWPOrbitalElements()
	{
		A = E = I = H = L = MA = MI = MH = HDOT = LDOT = 0.0;
	}

	LWPCommon::LWPCommon()
	{
		TPLANE = TYAW = 0.0;
		OPEN = 0.0;
		CLOSE = 0.0;
		PAO = 0.0;
		PAC = 0.0;
		GSTAR = PA = UINS = 0.0;
		K25 = 0;
		for (int i = 0; i < 10; i++)
		{
			STAR[i] = 0.0;
		}
		GMTLO = 0.0;
		AZL = GPAZ = IIGM = YP = TIGM = DN = TDIGM = DELNOD = 0.0;
	}

	LWPSummaryTable::LWPSummaryTable()
	{
		Error = -1;
		GMTLO = 0.0;
		GETLO = 0.0;
		TINS = 0.0;
		AZL = 0.0;
		VIGM = 0.0;
		RIGM = 0.0;
		GIGM = 0.0;
		IIGM = 0.0;
		TIGM = 0.0;
		TDIGM = 0.0;
		DN = 0.0;
		DELNO = 0.0;
		PA = 0.0;
		TPLANE = 0.0;
		LATLS = 0.0;
		LONGLS = 0.0;
		DTGRR = 0.0;
	}

	LWPOutputs::LWPOutputs()
	{
		GMTLO = 0.0;
	}

	LaunchWindowProcessor::LaunchWindowProcessor(OrbMech::GlobalConstants& cnst, OrbMech::SessionConstants& scnst) : globconst(cnst), sesconst(scnst)
	{
		Error = 0;
	}

	void LaunchWindowProcessor::Calculate(const LWPInputs& in, LWPOutputs& out)
	{
		LWP(in, out);

		if (in.PRINT == false) return;

		out.LWPOutput = LWPOutput;

		if (Error)
		{
			return;
		}

		//Print to file

		std::ofstream myfile;

		myfile.open(in.ProjectFolder + "Output/" + "LWPOutput.txt");
		if (myfile.is_open() == false) return;

		unsigned int i, j;

		for (i = 0; i < LWPOutput.size(); i++)
		{
			for (j = 0; j < LWPOutput[i].size(); j++)
			{
				myfile << LWPOutput[i][j] << std::endl;
			}
		}

		myfile.close();
	}

	void LaunchWindowProcessor::LWP(const LWPInputs& in, LWPOutputs& out)
	{
		inp = in;

		LWPIN();
		if (inp.LW != 1)
		{
			LWT();
			if (Error) return;
			LWDSP();
			if (Error) return;
			if (inp.LPT > 0)
			{
				LWPT();
				if (Error) return;
			}
		}
		if (in.LW != 0)
		{
			RLOT();
			if (Error) return;
			RLOTD();
			if (Error) return;
			LWPOT(out);
			if (Error) return;
			SVDSP();
		}
	}

	void LaunchWindowProcessor::LWPIN()
	{
		sv_T = inp.TRGVEC;
		Error = 0;

		common = LWPCommon();

		common.TPLANE = inp.TPLANE;

		LWPOutput.clear();
	}

	void LaunchWindowProcessor::LWT()
	{
		double TIP, DV, WEDGE, V, TINS;

	LWP_LWT_1_1:
		//Initial guess for inplane launch time
		TIP = 86400.0 * (double)(inp.DAY);

		//Find analytical inplane time
		NPLAN(TIP);
		if (Error) return;
		do
		{
			//Compute insertion vector
			LENSR(TIP, WEDGE);
			V = length(sv_C.V);
			DV = 2.0 * V * sin(WEDGE / 2.0);
			//TBD: Iterate
		} while (abs(DV) > lwpconst.tol_dv);

		if (inp.NS != 1)
		{
			common.OPEN = TIP - inp.DTOPT;
			TINS = common.OPEN + inp.PFT;
			common.TPLANE = common.OPEN;
			LENSR(common.OPEN, WEDGE);
			//Insertion phase angle
			common.PAO = PHANG(true);
		}
		if (inp.NS == 2)
		{
			if (coe_T.MI > inp.LATLS)
			{
				inp.NS = 1;
				goto LWP_LWT_1_1;
			}
		}
		if (inp.NS != 0)
		{
			common.CLOSE = TIP - inp.DTOPT;
			TINS = common.CLOSE + inp.PFT;
			LENSR(common.CLOSE, WEDGE);
			common.PAC = PHANG(true);
		}
		if (inp.STABLE)
		{
			double TI, TF;

			if (inp.NS != 1)
			{
				TI = common.OPEN - inp.STARS;
				TF = common.OPEN + inp.STARE;
				//Compute star times
				GMTLS(TI, TF);
			}
			if (inp.NS != 0)
			{
				TI = common.CLOSE - inp.STARS;
				TF = common.CLOSE + inp.STARE;
				//Compute star times
				GMTLS(TI, TF);
			}
		}

		//Save target vector
	}

	void LaunchWindowProcessor::NPLAN(double& TIP)
	{
		double DLON, UT, ETA, LAMBDA, LONG;
		int ITER;

		ITER = 0;

		do
		{
			//Update state vector to TIP
			UPDAT(false, TIP);
			if (Error) return;
			UT = sin(inp.LATLS) / sin(coe_T.MI);
			if (abs(UT) > 1.0)
			{
				//TBD: Add a closest approach flag?
				if (UT > 0.0)
				{
					UT = 1.0;
				}
				else
				{
					UT = -1.0;
				}
			}
			UT = asin(UT);
			if (inp.NS == 1)
			{
				UT = OrbMech::PI - UT;
			}
			ETA = acos(cos(UT) / cos(inp.LATLS));
			if (coe_T.MI > OrbMech::PI05)
			{
				ETA = -ETA;
			}
			LAMBDA = coe_T.H - globconst.w_E * TIP;
			while (LAMBDA < 0.0) LAMBDA += OrbMech::PI2;
			LONG = LAMBDA + ETA;
			if (LONG < 0.0)
			{
				LONG += OrbMech::PI2;
			}
			DLON = LONG - inp.LONGLS;
			if (DLON < 0.0 && ITER == 0)
			{
				DLON += OrbMech::PI2;
			}
			if (ITER > lwpconst.CMAX)
			{
				Error = -1;
				return;
			}
			ITER++;
			TIP = TIP + DLON / (globconst.w_E - coe_T.HDOT);

		} while (abs(DLON) > lwpconst.tol_trav_ang);
	}

	void LaunchWindowProcessor::LENSR(double GMTLO, double& WEDGE)
	{
		MATRIX3 MATR1, MATR2, MATR3;
		VECTOR3 URLS, H, K, J, RP, VP;
		double LONG, PSI, BGAMM, DELYAW, TP;

		//Update target to insertion
		UPDAT(false, GMTLO + inp.PFT);

		LONG = inp.LONGLS + globconst.w_E * GMTLO;
		URLS = _V(cos(inp.LATLS) * cos(LONG), cos(inp.LATLS) * sin(LONG), sin(inp.LATLS));

		H = unit(crossp(sv_T.R, sv_T.V));
		K = unit(crossp(H, URLS));
		J = crossp(K, H);

		PSI = asin(dotp(H, URLS));
		WEDGE = asin(sin(PSI) / cos(inp.PFA));
		BGAMM = asin(sin(inp.PFA) / cos(PSI));
		DELYAW = abs(WEDGE) - inp.YSMAX;
		if (inp.YSMAX > 0.)
		{
			if (DELYAW <= 0.)
			{
				BGAMM = acos(cos(inp.PFA) / cos(PSI));
				WEDGE = 0.0;
			}
			else
			{
				double cosx, x, y, sinG;

				cosx = cos(inp.PFA) / cos(inp.YSMAX);
				x = acos(cosx);
				y = asin(sin(PSI) / cosx);
				WEDGE = abs(y) - inp.YSMAX;
				sinG = sin(x) / cos(PSI);
				BGAMM = acos(sqrt(1.0 - sinG * sinG));
			}
		}
		MATR1 = _M(J.x, J.y, J.z, K.x, K.y, K.z, H.x, H.y, H.z);
		MATR2 = mul(_M(cos(BGAMM), sin(BGAMM), 0., -sin(BGAMM), cos(BGAMM), 0., 0., 0., 1.), MATR1);
		if (DELYAW > 0.)
		{
			double phi;
			phi = PSI / abs(PSI) * WEDGE;
			MATR2 = mul(_M(cos(phi), 0., sin(phi), 0., 1., 0., -sin(phi), 0., cos(phi)), MATR2);
		}
		MATR3 = mul(_M(cos(inp.GAMINS), -sin(inp.GAMINS), 0., sin(inp.GAMINS), cos(inp.GAMINS), 0., 0., 0., 1.), MATR2);
		RP = _V(MATR2.m11, MATR2.m12, MATR2.m13) * inp.RINS;
		VP = _V(MATR3.m21, MATR3.m22, MATR3.m23) * inp.VINS;
		TP = GMTLO + inp.PFT;

		//Assign to chaser
		sv_C.R = RP;
		sv_C.V = VP;
		sv_C.GMT = TP;
		sv_C.Weight = inp.CWHT;
		sv_C.Area = inp.CAREA;
		sv_C.KFactor = inp.CKFactor;
		UPDAT(true, sv_C.GMT);
	}

	void LaunchWindowProcessor::GMTLS(double TI, double TF)
	{
		double TLO, PH, DT, WEDGE;
		int K25, ITER;

		TLO = TI;
		K25 = 1;

		while (TLO <= TF)
		{
			K25++;
			if (K25 > 10)
			{
				Error = -1;
				return;
			}
			//Start search
			ITER = 0;
			do
			{
				//Compute insertion vector
				LENSR(TLO, WEDGE);
				//Compute insertion phase angle
				PH = PHANG();
				//Compute time to travel phase angle
				DT = TTHET(PH);
				if (ITER > lwpconst.CMAX)
				{
					Error = -1;
					return;
				}
				TLO = TLO - DT;
				ITER++;
			} while (abs(DT) > lwpconst.tol_time);
			common.STAR[K25 - 1] = TLO;
			TLO = TLO + OrbMech::PI2 / coe_T.LDOT;
			if (inp.STABLE == false)
			{
				TLO = TF + 60.0;
			}
		}
		common.GSTAR = common.STAR[K25 - 1];
	}

	void LaunchWindowProcessor::LWDSP()
	{
		if (inp.PRINT == false) return;

		std::vector<std::string> outarray;
		std::string temp;
		char Buffer[256], Buffer2[128], Buffer3[128], Buffer4[128];

		//LWT Display
		sprintf_s(Buffer, "%02d", inp.DAY);
		temp = "                        LAUNCH WINDOW TIMES FOR DAY ";
		temp.append(Buffer);
		outarray.push_back(temp);
		outarray.push_back("");
		outarray.push_back("");
		temp = "                            DATE ";
		sprintf_s(Buffer, "%02d %02d %04d", sesconst.Month, sesconst.Day, sesconst.Year);
		temp.append(Buffer);
		outarray.push_back(temp);
		outarray.push_back("");
		outarray.push_back("");
		outarray.push_back("INPLANE LAUNCH TIME");
		outarray.push_back("                         GMT           GMT          EST             PST");
		outarray.push_back("                    DAY HR MIN SEC   SECONDS   DAY HR MIN SEC  DAY HR MIN SEC");

		double pst, est;

		if (common.OPEN != 0.0)
		{
			pst = common.OPEN - 8.0 * 3600.0;
			if (pst < 0.0) pst += 86400.0;

			est = common.OPEN - 5.0 * 3600.0;
			if (est < 0.0) est += 86400.0;
		}
		else
		{
			pst = est = 0.0;
		}

		OrbMech::GMT2String3(Buffer2, common.OPEN, inp.DAY);
		OrbMech::GMT2String3(Buffer3, est, inp.DAY);
		OrbMech::GMT2String3(Buffer4, pst, inp.DAY);

		sprintf_s(Buffer, "OPENING OF WINDOW    %s  %08.1lf    %s   %s", Buffer2, common.OPEN, Buffer3, Buffer4);
		temp.assign(Buffer);
		outarray.push_back(temp);

		outarray.push_back("");

		if (common.CLOSE != 0.0)
		{
			pst = common.CLOSE - 8.0 * 3600.0;
			if (pst < 0.0) pst += 86400.0;

			est = common.CLOSE - 5.0 * 3600.0;
			if (est < 0.0) est += 86400.0;
		}
		else
		{
			pst = est = 0.0;
		}

		OrbMech::GMT2String3(Buffer2, common.CLOSE, inp.DAY);
		OrbMech::GMT2String3(Buffer3, est, inp.DAY);
		OrbMech::GMT2String3(Buffer4, pst, inp.DAY);

		sprintf_s(Buffer, "CLOSING OF WINDOW    %s  %08.1lf    %s   %s", Buffer2, common.CLOSE, Buffer3, Buffer4);
		temp.assign(Buffer);
		outarray.push_back(temp);

		outarray.push_back("");

		sprintf_s(Buffer, "  OPEN PHASE ANGLE  = %+07.2lf                         LATLS   =   %05.2lf DEG", common.PAO*OrbMech::DEG, inp.LATLS*OrbMech::DEG);
		temp.assign(Buffer);
		outarray.push_back(temp);

		sprintf_s(Buffer, "  CLOSE PHASE ANGLE = %+07.2lf                         LONGLS  =  %06.2lf DEG", common.PAC * OrbMech::DEG, inp.LONGLS * OrbMech::DEG);
		temp.assign(Buffer);
		outarray.push_back(temp);

		outarray.push_back("");
		outarray.push_back("");

		double apo, peri;
		OrbMech::periapo(sv_T.R, sv_T.V, globconst.mu, apo, peri);
		apo = (apo - globconst.R_E_equ) / 1852.0;
		peri = (peri - globconst.R_E_equ) / 1852.0;
		sprintf_s(Buffer, "  APOGEE  = %07.1lf", apo);
		temp.assign(Buffer);
		outarray.push_back(temp);

		sprintf_s(Buffer, "  PERIGEE = %07.1lf", peri);
		temp.assign(Buffer);
		outarray.push_back(temp);
		outarray.push_back("");

		LWPOutput.push_back(outarray);

		if (inp.STABLE == false) return;

		outarray.clear();

		//GMTLO* Display
		outarray.push_back("");
		outarray.push_back("                         GMTLO TABLE");
		outarray.push_back("");
		outarray.push_back("");
		outarray.push_back("  LATLS  =  XX.XX ");
		outarray.push_back("  LONGLS =  XXX.XX");
		outarray.push_back("");
		outarray.push_back("");
		outarray.push_back("               GMT                          STAR - PLANE");
		outarray.push_back("            D  H  M  S  GMT SECONDS     D  H  M  S      SECONDS");
		outarray.push_back("*STAR  1 = XX XX XX XX   XXXXXX.XX     XX XX XX XX      +XXXXX.XX");

		LWPOutput.push_back(outarray);
	}

	void LaunchWindowProcessor::LWPT()
	{
		if (inp.PRINT == false) return;

		double GMTLO_TAB[10], GPAZ_TAB[10], AZL_TAB[10], YP_TAB[10], WEDGE_TAB[10], DVPC_TAB[10], PHASE_TAB[10], TIGM_TAB[10];
		double TI, TF, GMTLO, WEDGE, V, DVPC, PHASE, AZL, DN, GPAZ, YP, TIGM, est;
		int counter;

		counter = 0;

		TI = common.OPEN - inp.TSTART;
		TF = common.OPEN + inp.TEND;
		GMTLO = TI;

		while (GMTLO <= TF)
		{
			LENSR(GMTLO, WEDGE);

			V = length(sv_C.V);
			DVPC = 2.0 * V * sin(WEDGE / 2.0);
			PHASE = PHANG(true);
			AZL = YP = TIGM = 0.0;

			DN = coe_C.H - OrbMech::PI;
			if (DN < 0.0) DN += OrbMech::PI2;
			GPAZ = GeminiParallelLaunchAzimuth(GMTLO, coe_C.I, coe_C.H, 0.0);

			TIGM = (DN - globconst.w_E * GMTLO) - inp.LONGLS + inp.DELNO + globconst.w_E * (inp.PFT + inp.DTGRR);
			if (TIGM < 0)
			{
				TIGM += OrbMech::PI2;
			}
			AZL = inp.LAZCOE[0] + inp.LAZCOE[1] * coe_C.I + inp.LAZCOE[2] * TIGM + inp.LAZCOE[3] * coe_C.I * TIGM;

			//Save
			GMTLO_TAB[counter] = GMTLO;
			GPAZ_TAB[counter] = GPAZ;
			AZL_TAB[counter] = AZL;
			YP_TAB[counter] = YP;
			WEDGE_TAB[counter] = WEDGE;
			DVPC_TAB[counter] = DVPC;
			PHASE_TAB[counter] = PHASE;
			TIGM_TAB[counter] = TIGM;

			counter++;
			if (counter >= 10) break;

			GMTLO += inp.TSTEP;
		}

		//Print
		std::vector<std::string> outarray;
		std::string temp;
		char Buffer[256], Buffer2[128], Buffer3[128];

		outarray.push_back("");
		outarray.push_back("              LAUNCH WINDOW PARAMETER TABLE");
		outarray.push_back("");

		sprintf_s(Buffer, "%02d %02d %04d", sesconst.Month, sesconst.Day, sesconst.Year);
		temp.assign(Buffer);

		outarray.push_back(" DATE " + temp);

		sprintf_s(Buffer, "%05.2lf", inp.LATLS * OrbMech::DEG);
		temp.assign(Buffer);
		outarray.push_back("                    LATLS  =    " + temp + " DEG");

		sprintf_s(Buffer, "%06.2lf", inp.LONGLS * OrbMech::DEG);
		temp.assign(Buffer);

		outarray.push_back("                    LONGLS =   " + temp + " DEG");
		outarray.push_back("");
		outarray.push_back("     TIMES BETWEEN WHICH LAUNCH PARAMETERS ARE COMPUTED");
		outarray.push_back("                       GMT                         EST");
		outarray.push_back("                         H  M  S              H  M  S");

		est = TI - 5.0 * 3600.0;
		if (est < 0.0)
		{
			est += 24.0 * 3600.0;
		}

		OrbMech::GMT2String4(Buffer2, TI);
		OrbMech::GMT2String4(Buffer3, est);
		sprintf_s(Buffer, "   TI =   %010.3lf    %s           %s", TI, Buffer2, Buffer3);
		temp.assign(Buffer);
		outarray.push_back(temp);

		est = TF - 5.0 * 3600.0;
		if (est < 0.0)
		{
			est += 24.0 * 3600.0;
		}
		OrbMech::GMT2String4(Buffer2, TF);
		OrbMech::GMT2String4(Buffer3, est);
		sprintf_s(Buffer, "   TF =   %010.3lf    %s           %s", TF, Buffer2, Buffer3);
		temp.assign(Buffer);
		outarray.push_back(temp);

		outarray.push_back("");
		outarray.push_back("  GMTLO          GPAZ    AZL     YP    WEDGE    DVPC    PHASE     TIGM");
		outarray.push_back(" H  M  S");
		outarray.push_back("");

		for (int i = 0; i < counter; i++)
		{
			OrbMech::GMT2String4(Buffer2, GMTLO_TAB[i]);
			sprintf_s(Buffer, "%s      %05.2lf   %05.2lf  %05.3lf   %05.3lf  %07.2lf  %07.3lf   %06.2lf", Buffer2, GPAZ_TAB[i]*OrbMech::DEG, AZL_TAB[i] * OrbMech::DEG,
				YP_TAB[i] * OrbMech::DEG, WEDGE_TAB[i] * OrbMech::DEG, DVPC_TAB[i] / OrbMech::FT2M, PHASE_TAB[i] * OrbMech::DEG, TIGM_TAB[i] * OrbMech::DEG);
			temp.assign(Buffer);
			outarray.push_back(temp);

			outarray.push_back("");
		}

		LWPOutput.push_back(outarray);
	}

	void LaunchWindowProcessor::RLOT()
	{
		double DALT, DTYAW, ANGLE;
		int ITINS, ITER, ITER2;
		int LAST;

		LAST = 1;
		DALT = 0.0;
		DTYAW = 0.0;

		if (inp.WRAP != 0)
		{
			inp.OFFSET = inp.OFFSET + OrbMech::PI2 * (double)inp.WRAP;
		}
		if (inp.INSCO != 1)
		{
			inp.VINS = sqrt(globconst.mu * (2.0 / inp.RINS - 1.0 / inp.ANOM)) + 10.0 * 0.3048;
		}

		if (inp.LOT <= 3)
		{
			common.GMTLO = inp.GMTLOR;
		}
		else if (inp.LOT == 4 || inp.LOT == 6)
		{
			common.GMTLO = common.TPLANE;
		}
		else
		{
			common.GMTLO = common.TPLANE + inp.TRANS;
		}

		LAST = 1;

		GMTLS(common.GMTLO, common.GMTLO + 1.0); //The 1.0 doesn't matter as STABLE is set to 2
		do
		{
			ITER2 = 0;
			do
			{
				ITINS = 0;
				do
				{
					switch (inp.LOT)
					{
					case 1:
					case 5:
					case 6:
						NSERT(common.GMTLO);
						break;
					case 2:
						ITER = 0;
						do
						{
							NSERT(common.GMTLO);
							ANGLE = common.PA - inp.OFFSET;
							common.GMTLO = common.GMTLO - ANGLE / coe_T.LDOT;
							if (ITER > lwpconst.CMAX)
							{
								Error = -1;
								return;
							}
						} while (abs(ANGLE) < lwpconst.tol_trans_ang);
						inp.LOT = 1;
						break;
					case 3:
					case 4:
						common.GMTLO = common.GSTAR + inp.BIAS;
						inp.LOT = 1;
						break;
					}
					if (inp.INSCO == 2 || inp.INSCO == 3)
					{
						double UOC = common.UINS + inp.DU;
						//Compute travel angle
						double DT = TARGL(UOC);
						//Advance chaser state vector
						UPDAT(true, sv_C.GMT + DT);
						if (inp.INSCO == 2)
						{
							//Advance target state vector
							double DHA = length(sv_T.R) - length(sv_C.R);
							DALT = DHA - inp.DHW;
						}
						else
						{
							double ALT = length(sv_C.R) - globconst.R_E_equ;
							DALT = inp.DHW - ALT;
						}
						inp.VINS = inp.VINS + DALT / 3420.0;
					}
					ITINS++;
					if (ITINS >= lwpconst.CMAX)
					{
						Error = -1;
						return;
					}
				} while (abs(DALT) >= lwpconst.tol_rad);

				inp.BIAS = common.GMTLO - common.GSTAR;
				TARGT(common.GMTLO);
				if (LAST == 1)
				{
					common.TYAW = common.TPLANE + inp.DELNO / globconst.w_E;
				}
				else
				{
					LAST = 1;
				}
				if (inp.LOT == 6)
				{
					DTYAW = common.TYAW - common.GMTLO;
					common.GMTLO = common.TYAW;
				}
				ITER2++;
				if (ITER2 >= lwpconst.CMAX)
				{
					Error = -1;
					return;
				}
			} while (abs(DTYAW) >= lwpconst.tol_time);

			if (inp.LOT == 6)
			{
				LAST = 0;
				inp.LOT = 1;
				common.GMTLO = common.TYAW + inp.TRANS;
			}

		} while (LAST != 1);
	}

	void LaunchWindowProcessor::NSERT(double GMTLO)
	{
		double PA, WEDGE, DH;

		LENSR(GMTLO, WEDGE);

		//Calculate insertion parameters
		PA = PHANG(true);

		common.PA = PA;
		common.UINS = OrbMech::ArgLat(sv_C.R, sv_C.V);
		DH = length(sv_T.R) - length(sv_C.R);
	}

	void LaunchWindowProcessor::TARGT(double GMTLO)
	{
		OrbMech::CELEMENTS coe_osc_C;
		double TGRR, TINS, HDOTC, UC, MHT;

		TGRR = GMTLO - inp.DTGRR;
		TINS = GMTLO + inp.PFT;

		//Update target to insertion
		UPDAT(false, TINS);
		//Update chaser to insertion
		UPDAT(true, TINS);

		coe_osc_C = OrbMech::CartesianToKeplerian(sv_C.R, sv_C.V, globconst.mu);

		//Redefine mean longitude of the ascending node to be relative to time of insertion
		MHT = coe_T.MH - globconst.w_E * TINS;
		MHT = OrbMech::normalize_angle(MHT, 0.0, OrbMech::PI2);
		//Calculate required chaser HDOT? If target orbit couldn't be reached due to yaw steering
		HDOTC = coe_C.HDOT;// *cos(coe_T.MI) / cos(coe_C.MI);
		//Compute descending node
		UC = OrbMech::ArgLat(sv_C.R, sv_C.V);
		common.DN = MHT - OrbMech::PI;
		if (common.DN < 0.0) common.DN += OrbMech::PI2;
		common.DN = common.DN + 1.5 * globconst.J2 * pow(globconst.R_E, 2) / (2.0 * pow(coe_C.A, 2) * pow(1.0 - pow(coe_C.E, 2), 2)) * cos(coe_T.MI) * sin(2.0 * UC);

		common.DELNOD = 0.0;
		if (inp.DELNOF)
		{
			inp.DELNO = -((coe_T.HDOT - HDOTC) * common.PA) / (coe_T.LDOT - coe_C.LDOT);
			if (abs(inp.BIAS) > 0.0)
			{
				common.DELNOD = inp.DELNO / inp.BIAS;
			}
		}

		//Compute IIGM
		common.IIGM = coe_T.MI + 1.5 * globconst.J2 * pow(globconst.R_E, 2) / (4.0 * pow(coe_C.A, 2) * pow(1.0 - pow(coe_C.E, 2), 2)) * sin(2.0 * coe_T.MI) * cos(2.0 * UC);

		//Compute TIGM
		common.TIGM = common.DN - inp.LONGLS + inp.DELNO + globconst.w_E * (inp.PFT + inp.DTGRR);
		if (common.TIGM < 0.0) common.TIGM += OrbMech::PI2;

		common.TDIGM = coe_T.HDOT - globconst.w_E + common.DELNOD;
		//Compute AZL
		common.AZL = inp.LAZCOE[0] + inp.LAZCOE[1] * coe_C.I + inp.LAZCOE[2] * common.TIGM + inp.LAZCOE[3] * coe_C.I * common.TIGM;
		
		coe_osc_C.i = common.IIGM;
		coe_osc_C.h = common.DN + OrbMech::PI + inp.DELNO + globconst.w_E * TINS;
		if (coe_osc_C.h >= OrbMech::PI2) coe_osc_C.h -= OrbMech::PI2;
		//Transform chaser vector to cartesian
		OrbMech::KeplerianToCartesian(coe_osc_C, globconst.mu, sv_C.R, sv_C.V);

		//Compute GPAZ
		common.GPAZ = GeminiParallelLaunchAzimuth(GMTLO, coe_osc_C.i, coe_osc_C.h, 0.0);
	}

	void LaunchWindowProcessor::RLOTD()
	{
		if (inp.PRINT == false) return;

		std::vector<std::string> outarray;
		std::string temp;
		char Buffer[256], Buffer2[128], Buffer3[128];

		outarray.push_back("                        SHUTTLE  PRELAUNCH  TARGETING");

		sprintf_s(Buffer, "%02d %02d %04d", sesconst.Month, sesconst.Day, sesconst.Year);
		temp.assign(Buffer);

		outarray.push_back("DATE      " + temp);
		outarray.push_back("");

		OrbMech::GMT2String4(Buffer2, common.GMTLO);
		temp.assign(Buffer2);

		outarray.push_back("GMTLO       " + temp + "       GETLO     00  0  00.0");

		OrbMech::GMT2String4(Buffer2, common.GMTLO + inp.PFT);
		OrbMech::GMT2String4(Buffer3, common.GMTLO - inp.DTGRR);

		sprintf_s(Buffer, "TINS        %s                                    TGRR     %s", Buffer2, Buffer3);
		temp.assign(Buffer);
		outarray.push_back(temp);

		sprintf_s(Buffer, "                             AZL            %07.3lf       VIGM       %08.2lf", common.AZL * OrbMech::DEG, inp.VINS / OrbMech::FT2M);
		temp.assign(Buffer);
		outarray.push_back(temp);

		OrbMech::GMT2String4(Buffer2, common.GSTAR);
		sprintf_s(Buffer, "GMTLO*      %s       LATLS           %06.3lf       RIGM    %011.2lf", Buffer2, inp.LATLS * OrbMech::DEG, inp.RINS / OrbMech::FT2M);
		temp.assign(Buffer);
		outarray.push_back(temp);

		sprintf_s(Buffer, "PFA             %06.3lf       LONGLS         %07.3lf       GIGM     %010.8lf", inp.PFA * OrbMech::DEG, inp.LONGLS * OrbMech::DEG, inp.GAMINS * OrbMech::DEG);
		temp.assign(Buffer);
		outarray.push_back(temp);

		sprintf_s(Buffer, "PFT            %07.3lf       GPAZ           %07.3lf       IIGM        %07.3lf", inp.PFT, common.GPAZ * OrbMech::DEG, common.IIGM * OrbMech::DEG);
		temp.assign(Buffer);
		outarray.push_back(temp);

		sprintf_s(Buffer, "DN              %06.3lf       YP              %06.4lf       TIGM        %07.3lf", common.DN * OrbMech::DEG, common.YP * OrbMech::DEG, common.TIGM * OrbMech::DEG);
		temp.assign(Buffer);
		outarray.push_back(temp);

		sprintf_s(Buffer, "                                                          TDIGM     %+09.6lf", common.TDIGM * OrbMech::DEG);
		temp.assign(Buffer);
		outarray.push_back(temp);

		outarray.push_back("");

		OrbMech::GMT2String4(Buffer2, common.TYAW);
		sprintf_s(Buffer, "TYAW      %s       DELNO      %.8lf", Buffer2, inp.DELNO*OrbMech::DEG);
		temp.assign(Buffer);
		outarray.push_back(temp);

		OrbMech::GMT2String4(Buffer2, common.TPLANE);
		sprintf_s(Buffer, "TPLANE    %s       DELNOD     %010.8lf", Buffer2, common.DELNOD * OrbMech::DEG);
		temp.assign(Buffer);
		outarray.push_back(temp);

		outarray.push_back("");
		outarray.push_back("      TARGET ORBIT                                         LAUNCH ORBIT");
		outarray.push_back("");

		double apo, peri, HA_C, HP_C, HA_T, HP_T;

		OrbMech::periapo(sv_C.R, sv_C.V, globconst.mu, apo, peri);
		HA_C = (apo - globconst.R_E_equ) / OrbMech::NM;
		HP_C = (peri - globconst.R_E_equ) / OrbMech::NM;

		OrbMech::periapo(sv_T.R, sv_T.V, globconst.mu, apo, peri);
		HA_T = (apo - globconst.R_E_equ) / OrbMech::NM;
		HP_T = (peri - globconst.R_E_equ) / OrbMech::NM;

		sprintf_s(Buffer, " APOGEE     =  %09.2lf    ABOVE REF. %012.2lf  APOGEE     =    %08.1lf", HA_T, globconst.R_E_equ / OrbMech::FT2M, HA_C);
		temp.assign(Buffer);
		outarray.push_back(temp);

		sprintf_s(Buffer, " PERIGEE    =  %09.2lf                             PERIGEE    =    %08.1lf", HP_T, HP_C);
		temp.assign(Buffer);
		outarray.push_back(temp);

		sprintf_s(Buffer, " INCLINATION=    %06.2lf                              T ANOMALY  =     %+07.2lf", coe_T.MI * OrbMech::DEG, OrbMech::MeanToTrueAnomaly(coe_C.L, coe_C.E) * OrbMech::DEG);
		temp.assign(Buffer);
		outarray.push_back(temp);

		sprintf_s(Buffer, " INS PHASE  =   %+08.3lf                             ALTITUDE   =    %08.1lf", common.PA * OrbMech::DEG, (length(sv_C.R) - globconst.R_E_equ) / OrbMech::NM);
		temp.assign(Buffer);
		outarray.push_back(temp);

		outarray.push_back(" DN TARGET  =   +XXX.XXX                             DH         =    XXXXXX.X");
		outarray.push_back(" BIAS       =  XXXXX.XXX                             TIME       =");

		LWPOutput.push_back(outarray);
	}

	void LaunchWindowProcessor::LWPOT(LWPOutputs& out)
	{
		out.PVTABC = sv_C;
		out.PVTABT[0] = inp.TRGVEC;
		out.PVTABT[1] = sv_T;

		out.GMTLO = common.GMTLO;

		out.SUMTAB.Error = 0;
		out.SUMTAB.GMTLO = common.GMTLO;
		out.SUMTAB.GETLO = 0.0;
		out.SUMTAB.TINS = common.GMTLO + inp.PFT;
		out.SUMTAB.AZL = common.AZL;
		out.SUMTAB.VIGM = inp.VINS;
		out.SUMTAB.RIGM = inp.RINS;
		out.SUMTAB.GIGM = inp.GAMINS;
		out.SUMTAB.IIGM = common.IIGM;
		out.SUMTAB.TIGM = common.TIGM;
		out.SUMTAB.TDIGM = common.TDIGM;
		out.SUMTAB.DN = common.DN;
		out.SUMTAB.DELNO = inp.DELNO;
		out.SUMTAB.PA = common.PA;
		out.SUMTAB.TPLANE = common.TPLANE;
		out.SUMTAB.LATLS = inp.LATLS;
		out.SUMTAB.LONGLS = inp.LONGLS;
		out.SUMTAB.DTGRR = inp.DTGRR;
	}

	void LaunchWindowProcessor::SVDSP()
	{

	}

	void LaunchWindowProcessor::UPDAT(bool chaser, double T)
	{
		EnckeIntegrator integ(globconst);

		EnckeIntegratorInput inp;
		EnckeIntegratorOutput outp;
		OrbMech::StateVector sv0, sv1;

		if (chaser)
		{
			sv0 = sv_C;
		}
		else
		{
			sv0 = sv_T;
		}
		sv1 = sv0;

		inp.R = sv0.R;
		inp.V = sv0.V;
		inp.GMT = sv0.GMT;
		inp.KFactor = sv0.KFactor;
		inp.DragIndicator = true;
		inp.Weight = sv0.Weight;
		inp.Area = sv0.Area;
		inp.dt = T - sv0.GMT;

		integ.Propagate(inp, outp);

		if (outp.Error)
		{
			Error = outp.Error + 1000;
			return;
		}

		sv1.R = outp.R;
		sv1.V = outp.V;
		sv1.GMT = outp.GMT;

		//Also calculate some orbital elements
		OrbMech::CELEMENTS coe = OrbMech::CartesianToKeplerian(sv1.R, sv1.V, globconst.mu);

		LWPOrbitalElements elem;
		double p, u;

		elem.A = coe.a;
		elem.E = coe.e;
		elem.I = coe.i;
		elem.H = coe.h;
		elem.L = coe.l;

		//Mean inclination and longitude of the ascending node
		p = coe.a * (1.0 - coe.e * coe.e);
		u = OrbMech::ArgLat(sv1.R, sv1.V);
		elem.MI = coe.i - 1.5 * globconst.J2 * pow(globconst.R_E, 2) / (4.0 * p * p) * sin(2.0 * coe.i) * cos(2.0 * u);
		elem.MH = coe.h - 1.5 * globconst.J2 * pow(globconst.R_E, 2) / (2.0 * p * p) * cos(elem.MI) * sin(2.0 * u);
	
		//Estimate the mean semi-major axis
		double ar3, da_sp;
		
		ar3 = pow(coe.a / length(sv1.R), 3);
		da_sp = globconst.J2 * pow(globconst.R_E, 2) / coe.a * (ar3 - 1.0 / pow(1.0 - pow(coe.e, 2), 1.5) + (-ar3 + 1.0 / pow(1.0 - pow(coe.e, 2), 1.5) + ar3 * cos(2.0 * u)) * 3.0 * pow(sin(coe.i), 2) / 2.0);
		elem.MA = coe.a - da_sp;

		//Secular rates
		elem.HDOT = -1.5 * (globconst.sqrt_mu * globconst.J2 * pow(globconst.R_E, 2) / (pow(1.0 - pow(coe.e, 2), 2) * pow(elem.MA, 3.5))) * cos(elem.MI);
		elem.LDOT = OrbMech::PI2 / OrbMech::REVTIM(sv1.R, sv1.V, globconst);

		if (chaser)
		{
			sv_C = sv1;
			coe_C = elem;
		}
		else
		{
			sv_T = sv1;
			coe_T = elem;
		}
	}

	double LaunchWindowProcessor::PHANG(bool wrapped) const
	{
		double PA;
		PA = OrbMech::PHSANG(sv_T.R, sv_T.V, sv_C.R);

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

	double LaunchWindowProcessor::TTHET(double PH) const
	{
		return PH / coe_T.LDOT;
	}

	double LaunchWindowProcessor::TARGL(double UOC) const
	{
		double U, DU;

		U = OrbMech::ArgLat(sv_C.R, sv_C.V);
		DU = UOC - U;
		if (DU < 0.0)
		{
			DU += OrbMech::PI2;
		}
		return DU / coe_C.LDOT;
	}

	double LaunchWindowProcessor::GeminiParallelLaunchAzimuth(double TLO, double i_c, double h_c, double DTheta_dot) const
	{
		double DN, TR_star, TS, phi_star;

		DN = h_c - OrbMech::PI - globconst.w_E * TLO;
		while (DN < 0.0)
		{
			DN += OrbMech::PI2;
		}
		TR_star = DN / (globconst.w_E + coe_T.HDOT) + TLO;
		TR_star = TR_star * (globconst.w_E + coe_T.HDOT - DTheta_dot * (TLO + inp.PFT)) / (globconst.w_E + coe_T.HDOT - DTheta_dot);
		TR_star = TR_star - trunc(TR_star / (24.0 * 3600.0)) * 24.0 * 3600.0;
		TLO = TLO - trunc(TLO / (24.0 * 3600.0)) * 24.0 * 3600.0;
		TS = TLO - TR_star;
		phi_star = OrbMech::PI2 - inp.LONGLS - globconst.w_E * TS - (coe_T.HDOT - DTheta_dot) * (TS + inp.PFT);

		return atan(-(cos(i_c) * cos(inp.LATLS) + sin(i_c) * sin(phi_star) * sin(inp.LATLS)) / (sin(i_c) * cos(phi_star)));
	}
}