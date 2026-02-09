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

#include "Core.h"
#include "EnckeIntegrator.h"
#include "Constants.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include "SGP4/SGP4.h"

Core::Core()
{
	Initialized = false;
}

void Core::Test(std::string project, std::string script)
{
	return;

	/*OrbMech::GlobalConstants constants;

	constants.mu = 0.3986032e15;
	constants.sqrt_mu = sqrt(constants.mu);
	constants.R_E = 6378165.0;
	constants.J2 = 1083.23e-6;// 1082.6269e-6;
	constants.J3 = 0.0; // -2.51e-6;
	constants.J4 = 0.0; // -1.60e-6;
	constants.w_E = 7.29211514667e-5;

	EnckeIntegratorInput inp;
	EnckeIntegratorOutput outp;

	inp.R = _V(3927254.911, 2232640.190, -4753819.073);
	inp.V = _V(-5785.6245, 4501.7155, -2664.4106);
	inp.GMT = 0.0;
	inp.dt = 24.0 * 3600.0;
	inp.KFactor = 1.0;
	inp.DragIndicator = false;
	inp.Weight = 15000.0;
	inp.Area = 129.4 * pow(0.3048, 2);

	EnckeIntegrator integ(constants);

	integ.Propagate(inp, outp);*/
}

int ParseGMT(std::string buf, double& val)
{
	int hh, mm;
	double sec;

	if (sscanf_s(buf.c_str(), "%d:%d:%lf", &hh, &mm, &sec) != 3)
	{
		return 1;
	}

	val = 3600.0 * (double)hh + 60.0 * (double)mm + sec;
	return 0;
}

int Core::RunLWP(std::string inputs[], std::vector<std::vector<std::string>>& data)
{
	LWP::LWPInputs in;
	LWP::LWPOutputs out;

	ProjectFolder = "Projects/" + inputs[0] + "/";

	if (LoadStateVector(ProjectFolder + "State Vectors/" + inputs[1], in.TRGVEC)) return 1;
	in.CKFactor = std::stod(inputs[2]);
	in.CAREA = std::stod(inputs[3]) * pow(OrbMech::FT2M, 2);
	in.CWHT = std::stod(inputs[4]) * OrbMech::LBS;
	if (inputs[5] == "LW")
	{
		in.LW = 0;
	}
	else if (inputs[5] == "LT")
	{
		in.LW = 1;
	}
	else if (inputs[5] == "WT")
	{
		in.LW = 2;
	}
	else return 1;

	if (inputs[6] == "OP")
	{
		in.NS = 0;
	}
	else if (inputs[6] == "CL")
	{
		in.NS = 1;
	}
	else if (inputs[6] == "OC")
	{
		in.NS = 2;
	}
	else return 1;

	in.DAY = std::stoi(inputs[7]);

	if (inputs[8] == "NO")
	{
		in.LPT = 0;
	}
	else if (inputs[8] == "OP")
	{
		in.LPT = 1;
	}
	else if (inputs[8] == "CL")
	{
		in.LPT = 2;
	}
	else if (inputs[8] == "OC")
	{
		in.LPT = 3;
	}
	else if (inputs[8] == "EN")
	{
		in.LPT = 4;
	}
	else return 1;

	in.TSTART = std::stod(inputs[9]);
	in.TEND = std::stod(inputs[10]);
	in.TSTEP = std::stod(inputs[11]);

	if (inputs[12] == "NO")
	{
		in.STABLE = false;
	}
	else if (inputs[12] == "YE")
	{
		in.STABLE = true;
	}
	else return 1;

	in.STARS = std::stod(inputs[13]);
	in.STARE = std::stod(inputs[14]);
	in.LATLS = std::stod(inputs[15]) * OrbMech::RAD;
	in.LONGLS = std::stod(inputs[16]) * OrbMech::RAD;
	in.PFT = std::stod(inputs[17]);
	in.PFA = std::stod(inputs[18]) * OrbMech::RAD;
	in.YSMAX = std::stod(inputs[19]) * OrbMech::RAD;
	in.DTOPT = std::stod(inputs[20]);
	in.DTGRR = std::stod(inputs[21]);
	in.RINS = std::stod(inputs[22]) * OrbMech::FT2M;
	in.VINS = std::stod(inputs[23]) * OrbMech::FT2M;
	in.GAMINS = std::stod(inputs[24]) * OrbMech::RAD;

	if (inputs[25] == "GMTLOR") in.LOT = 1;
	else if (inputs[25] == "OFFSET") in.LOT = 2;
	else if (inputs[25] == "RSTAR") in.LOT = 3;
	else if (inputs[25] == "PSTAR") in.LOT = 4;
	else if (inputs[25] == "TPLANE") in.LOT = 5;
	else if (inputs[25] == "TYAW") in.LOT = 6;
	else return 1;

	if (ParseGMT(inputs[26], in.GMTLOR)) return 1;

	in.OFFSET = std::stod(inputs[27]) * OrbMech::RAD;
	in.BIAS = std::stod(inputs[28]);

	if (ParseGMT(inputs[29], in.TPLANE)) return 1;

	in.TRANS = std::stod(inputs[30]);

	if (inputs[31] == "VGAMR")
	{
		in.INSCO = 1;
	}
	else if (inputs[31] == "DHGAMR")
	{
		in.INSCO = 2;
	}
	else if (inputs[31] == "HGAMR")
	{
		in.INSCO = 3;
	}
	else return 1;

	in.DHW = std::stod(inputs[32]) * OrbMech::NM;
	in.DU = std::stod(inputs[33]) * OrbMech::RAD;
	in.ANOM = std::stod(inputs[34]) * OrbMech::NM;

	if (inputs[35] == "IN")
	{
		in.DELNOF = false;
	}
	else if (inputs[35] == "CO")
	{
		in.DELNOF = true;
	}
	else return 2;

	in.DELNO = std::stod(inputs[36]) * OrbMech::RAD;

	if (inputs[37] == "PO")
	{
		in.NEGTIV = 0;
	}
	else if (inputs[37] == "NE")
	{
		in.NEGTIV = 1;
	}
	else if (inputs[37] == "LO")
	{
		in.NEGTIV = 2;
	}
	else return 1;

	in.WRAP = std::stoi(inputs[38]);

	in.PRINT = true;
	in.ProjectFolder = ProjectFolder;

	LWP::LaunchWindowProcessor lwp(globcnst, sescnst);

	lwp.Calculate(in, out);

	data = out.LWPOutput;

	//Save state vectors
	PVTABC = out.PVTABC;
	PVTABT = out.PVTABT[1];
	//Save summary table
	LWPSummaryTable = out.SUMTAB;

	return 0;
}
int Core::SaveLWPStateVector(std::string filename)
{
	if (PVTABC.GMT != 0.0)
	{
		SaveStateVector(ProjectFolder + "State Vectors/" + filename, PVTABC);
		return 0;
	}
	return 1;
}

int Core::LWPExportForLVDC()
{
	std::ofstream myfile;
	std::string file;

	if (!Initialized) return 1;
	if (LWPSummaryTable.Error != 0)
	{
		return 1;
	}

	file = ProjectFolder + "Outputs/LVDC.txt";

	myfile.open(file);
	if (myfile.is_open() == false) return 1;

	double G_T, Inclination, Lambda_0, lambda_dot, R_T, V_T, gamma_T, T_GRR0;
	char buf[1024];

	G_T = -globcnst.mu / (pow(LWPSummaryTable.RIGM, 2));
	Inclination = LWPSummaryTable.IIGM * OrbMech::DEG;
	Lambda_0 = LWPSummaryTable.TIGM * OrbMech::DEG;
	lambda_dot = LWPSummaryTable.TDIGM * OrbMech::DEG;
	R_T = LWPSummaryTable.RIGM;
	V_T = LWPSummaryTable.VIGM;
	gamma_T = LWPSummaryTable.GIGM; //TBD: Degree??
	T_GRR0 = LWPSummaryTable.GMTLO - LWPSummaryTable.DTGRR;

	//Get target state vector to T-4h
	
	double GMTSCEN, MJDSCEN;

	GMTSCEN = LWPSummaryTable.GMTLO - 4.0 * 3600.0;
	MJDSCEN = sescnst.GMTBASE + GMTSCEN / 24.0 / 3600.0;

	EnckeIntegratorInput in;
	EnckeIntegratorOutput out;

	in.R = PVTABT.R;
	in.V = PVTABT.V;
	in.GMT = PVTABT.GMT;
	in.KFactor = PVTABT.KFactor;
	in.DragIndicator = true;
	in.Weight = PVTABT.Weight;
	in.Area = PVTABT.Area;
	in.dt = GMTSCEN - PVTABT.GMT;

	EnckeIntegrator encke(globcnst);

	encke.Propagate(in, out);

	if (out.Error) return out.Error;

	//Convert to J2000 (left-handed)
	out.R = mul(sescnst.M_TEG_TO_J2000, out.R);
	out.V = mul(sescnst.M_TEG_TO_J2000, out.V);

	//Write to file
	sprintf_s(buf, "%04d-%02d-%02d", sescnst.Year, sescnst.Month, sescnst.Day);
	myfile << "Saturn IB LVDC targeting for " << buf << std::endl << std::endl;

	myfile << "MJD of T-4h:" << std::endl;
	sprintf_s(buf, "%.9lf", MJDSCEN);
	myfile << buf << std::endl << std::endl;

	myfile << "Target state vector for scenario:" << std::endl;

	sprintf_s(buf, "RPOS %.3lf %.3lf %.3lf", out.R.x, out.R.z, out.R.y);
	myfile << buf << std::endl;

	sprintf_s(buf, "RVEL %.4lf %.4lf %.4lf", out.V.x, out.V.z, out.V.y);
	myfile << buf << std::endl << std::endl;

	sprintf_s(buf, "%.1lf", R_T);
	myfile << "LVDC_R_T " << buf << std::endl;

	sprintf_s(buf, "%.2lf", V_T);
	myfile << "LVDC_V_T " << buf << std::endl;

	sprintf_s(buf, "%.3lf", gamma_T);
	myfile << "LVDC_gamma_T " << buf << std::endl;

	sprintf_s(buf, "%.2lf", G_T);
	myfile << "LVDC_G_T " << buf << std::endl;

	sprintf_s(buf, "%.5lf", Inclination);
	myfile << "LVDC_Inclination " << buf << std::endl;

	sprintf_s(buf, "%.5lf", Lambda_0);
	myfile << "LVDC_Lambda_0 " << buf << std::endl;

	sprintf_s(buf, "%.8lf", lambda_dot);
	myfile << "LVDC_lambda_dot " << buf << std::endl;

	sprintf_s(buf, "%.1lf", T_GRR0);
	myfile << "LVDC_T_GRR0 " << buf << std::endl;

	myfile.close();
	return 0;
}

int Core::RunShuttleLWP(bool IsLW, std::string strInputs[], double* dInputs, int* iInputs, std::vector<std::string>& data)
{
	ShuttleLWP::ShuttleLWPInputs in;
	bool UseDrag;

	ProjectFolder = "Projects/" + strInputs[0] + "/";

	if (LoadStateVector(ProjectFolder + "State Vectors/" + strInputs[1], in.sv_T)) return 1;

	in.CCD = dInputs[1];
	in.CAREA = dInputs[2] * pow(OrbMech::FT2M, 2);
	in.CWHT = dInputs[3] * OrbMech::LBS;
	in.LATD = dInputs[4] * OrbMech::RAD;
	in.LONGLS = dInputs[5] * OrbMech::RAD;
	in.PFT = dInputs[7];
	in.PFA = dInputs[6] * OrbMech::RAD;
	in.YSMAX = dInputs[0] * OrbMech::RAD;
	in.RINS = dInputs[8] * OrbMech::FT2M;
	in.VINS = dInputs[9] * OrbMech::FT2M;
	in.GAMINS = dInputs[10] * OrbMech::RAD;
	in.DTOPT = dInputs[11];
	in.DTO = dInputs[19];
	in.DTC = dInputs[20];
	in.OMS1.DTIG = dInputs[21];
	in.OMS1.C1 = dInputs[22] * OrbMech::FT2M;
	in.OMS1.C2 = dInputs[23];
	in.OMS1.HTGT = dInputs[24] * OrbMech::NM;
	in.OMS1.THETA = dInputs[25] * OrbMech::RAD;
	in.OMS2.DTIG = dInputs[26];
	in.OMS2.C1 = dInputs[27] * OrbMech::FT2M;
	in.OMS2.C2 = dInputs[28];
	in.OMS2.HTGT = dInputs[29] * OrbMech::NM;
	in.OMS2.THETA = dInputs[30] * OrbMech::RAD;

	in.DirectInsertion = (iInputs[0] == 1 ? true : false);
	in.NS = iInputs[1] == 1 ? 0 : 1;
	in.NEGTIV = iInputs[2];
	in.WRAP = iInputs[3];
	UseDrag = (iInputs[6] == 1 ? true : false);

	if (IsLW)
	{
		if (iInputs[4])
		{
			in.OMS2Phase[0] = true;
			in.OMS2PhaseAngle[0] = dInputs[31] * OrbMech::RAD;
		}
		else
		{
			in.OMS2Phase[0] = false;
		}
		if (iInputs[5])
		{
			in.OMS2Phase[1] = true;
			in.OMS2PhaseAngle[1] = dInputs[32] * OrbMech::RAD;
		}
		else
		{
			in.OMS2Phase[1] = false;
		}

	}
	else
	{
		in.GMTLO = dInputs[33];
	}

	// Other inputs
	in.ET_Area = dInputs[12] * pow(OrbMech::FT2M, 2);
	in.ET_CD = dInputs[13];
	in.ET_Weight = dInputs[14] * OrbMech::LBS;
	in.DTIG_ET_SEP = dInputs[15];
	in.DV_ET_SEP = _V(dInputs[16], dInputs[17], dInputs[18]) * OrbMech::FT2M;
	in.DTIG_MPS = dInputs[34];
	in.DV_MPS = _V(dInputs[35], dInputs[36], dInputs[37]) * OrbMech::FT2M;

	ShuttleLWP::ShuttleLaunchWindowProcessor lwp(globcnst, sescnst);

	if (UseDrag == false)
	{
		in.CCD = in.ET_CD = 0.0;
	}

	// Format output
	char Buffer[256];

	if (IsLW)
	{
		ShuttleLWP::ShuttleLWPOutputs out;

		int err = lwp.CalculateLW(in, out);
		if (err) return err;

		OrbMech::GMT2String4(Buffer, out.GMTLO_OPT);
		data.push_back(Buffer);
		sprintf_s(Buffer, "%.1lf", out.PHASE_OPT * OrbMech::DEG);
		data.push_back(Buffer);
		OrbMech::GMT2String4(Buffer, out.GMTLO_OPEN);
		data.push_back(Buffer);
		sprintf_s(Buffer, "%.1lf", out.PHASE_OPEN * OrbMech::DEG);
		data.push_back(Buffer);
		OrbMech::GMT2String4(Buffer, out.GMTLO_CLOSE);
		data.push_back(Buffer);
		sprintf_s(Buffer, "%.1lf", out.PHASE_CLOSE * OrbMech::DEG);
		data.push_back(Buffer);

		for (int i = 0; i < 2; i++)
		{
			if (in.OMS2Phase[i])
			{
				sprintf_s(Buffer, "%.1lf", in.OMS2PhaseAngle[i] * OrbMech::DEG);
				data.push_back(Buffer);
				OrbMech::GMT2String4(Buffer, out.GMTLO_PHASE[i]);
				data.push_back(Buffer);
			}
			else
			{
				data.push_back("");
				data.push_back("");
			}
		}
	}
	else
	{
		ShuttleLWP::ShuttleLTPOutputs out;

		int err = lwp.CalculateLT(in, out);
		if (err) return err;

		LTP_Outputs = out;

		OrbMech::GMT2String4(Buffer, out.GMTLO);
		data.push_back(Buffer);
		OrbMech::GMT2String4(Buffer, out.MET_MECO);
		data.push_back(Buffer);
		sprintf_s(Buffer, "%.1lf", out.VMECO / OrbMech::FT2M);
		data.push_back(Buffer);
		sprintf_s(Buffer, "%.4lf", out.RMECO / OrbMech::NM);
		data.push_back(Buffer);
		sprintf_s(Buffer, "%.4lf", out.GMECO * OrbMech::DEG);
		data.push_back(Buffer);
		sprintf_s(Buffer, "%.3lf", out.IMECO * OrbMech::DEG);
		data.push_back(Buffer);
		sprintf_s(Buffer, "%.1lf", out.PHASE_MECO * OrbMech::DEG);
		data.push_back(Buffer);
		sprintf_s(Buffer, "%.1lf", out.HA_MECO / OrbMech::NM);
		data.push_back(Buffer);
		sprintf_s(Buffer, "%.1lf", out.HP_MECO / OrbMech::NM);
		data.push_back(Buffer);
		sprintf_s(Buffer, "%.1lf", out.LONG_MECO * OrbMech::DEG);
		data.push_back(Buffer);
		OrbMech::GMT2String4(Buffer, out.OMS1_TIG);
		data.push_back(Buffer);
		sprintf_s(Buffer, "%.1lf", out.OMS1_DV / OrbMech::FT2M);
		data.push_back(Buffer);
		sprintf_s(Buffer, "%.1lf", out.OMS1_HA / OrbMech::NM);
		data.push_back(Buffer);
		sprintf_s(Buffer, "%.1lf", out.OMS1_HP / OrbMech::NM);
		data.push_back(Buffer);
		OrbMech::GMT2String4(Buffer, out.OMS2_TIG);
		data.push_back(Buffer);
		sprintf_s(Buffer, "%.1lf", out.OMS2_DV / OrbMech::FT2M);
		data.push_back(Buffer);
		sprintf_s(Buffer, "%.1lf", out.OMS2_HA / OrbMech::NM);
		data.push_back(Buffer);
		sprintf_s(Buffer, "%.1lf", out.OMS2_HP / OrbMech::NM);
		data.push_back(Buffer);
		sprintf_s(Buffer, "%.1lf", out.OMS2_NODE * OrbMech::DEG);
		data.push_back(Buffer);
		sprintf_s(Buffer, "%.1lf", out.OMS2_PHASE * OrbMech::DEG);
		data.push_back(Buffer);
		OrbMech::GMT2String4(Buffer, out.OMS2_PERIOD);
		data.push_back(Buffer);
		sprintf_s(Buffer, "%.1lf", out.TGT_HA / OrbMech::NM);
		data.push_back(Buffer);
		sprintf_s(Buffer, "%.1lf", out.TGT_HP / OrbMech::NM);
		data.push_back(Buffer);
		sprintf_s(Buffer, "%.1lf", out.TGT_LONG * OrbMech::DEG);
		data.push_back(Buffer);
		sprintf_s(Buffer, "%.6lf", out.TGT_DELN* OrbMech::DEG);
		data.push_back(Buffer);
		OrbMech::GMT2String4(Buffer, out.TGT_PERIOD);
		data.push_back(Buffer);
	}

	return 0;
}

int Core::ShuttleLTPExport()
{
	if (!Initialized) return 1;
	if (LTP_Outputs.Error != 0) return 1;

	std::ofstream myfile;
	std::string file;

	file = ProjectFolder + "Outputs/ShuttleTargeting.txt";

	myfile.open(file);
	if (myfile.is_open() == false) return 1;

	//Get target state vector to T-9 minutes and T-31 seconds
	OrbMech::StateVector sv_T_9min, sv_T_31sec;
	double GMT_9min, MJD_9min, GMT_31sec, MJD_31sec;
	char buf[1024];

	GMT_9min = LTP_Outputs.GMTLO - 540.1;
	MJD_9min = sescnst.GMTBASE + GMT_9min / 24.0 / 3600.0;

	EnckeIntegratorInput in;
	EnckeIntegratorOutput out;

	EnckeIntegrator encke(globcnst);

	in.R = LTP_Outputs.sv_T.R;
	in.V = LTP_Outputs.sv_T.V;
	in.GMT = LTP_Outputs.sv_T.GMT;
	in.KFactor = LTP_Outputs.sv_T.KFactor;
	in.DragIndicator = true;
	in.Weight = LTP_Outputs.sv_T.Weight;
	in.Area = LTP_Outputs.sv_T.Area;
	in.dt = GMT_9min - LTP_Outputs.sv_T.GMT;

	encke.Propagate(in, out);
	if (out.Error) return out.Error;

	//Convert to J2000 (right-handed)
	sv_T_9min.R = mul(sescnst.M_TEG_TO_J2000, out.R);
	sv_T_9min.V = mul(sescnst.M_TEG_TO_J2000, out.V);

	GMT_31sec = LTP_Outputs.GMTLO - 31.1;
	MJD_31sec = sescnst.GMTBASE + GMT_31sec / 24.0 / 3600.0;

	in.dt = GMT_31sec - LTP_Outputs.sv_T.GMT;

	encke.Propagate(in, out);
	if (out.Error) return out.Error;

	//Convert to J2000 (right-handed)
	sv_T_31sec.R = mul(sescnst.M_TEG_TO_J2000, out.R);
	sv_T_31sec.V = mul(sescnst.M_TEG_TO_J2000, out.V);

	// Calculate IY vectors
	VECTOR3 IY_MECO_M50, IY_OMS1_M50, IY_OMS2_M50;

	IY_MECO_M50 = mul(sescnst.M_TEG_TO_M50, LTP_Outputs.IY_MECO);
	IY_OMS1_M50 = mul(sescnst.M_TEG_TO_M50, LTP_Outputs.IY_OMS1);
	IY_OMS2_M50 = mul(sescnst.M_TEG_TO_M50, LTP_Outputs.IY_OMS2);

	double DT_ET_SET = 18.0;

	//Write to file
	sprintf_s(buf, "%04d-%02d-%02d", sescnst.Year, sescnst.Month, sescnst.Day);
	myfile << "Shuttle launch targeting for " << buf << std::endl;
	OrbMech::GMT2String4(buf, LTP_Outputs.GMTLO);
	myfile << "GMTLO = " << buf << std::endl << std::endl;

	myfile << "SCENARIO DATA" << std::endl << std::endl;

	myfile << "MJD of T-9min:" << std::endl;
	sprintf_s(buf, "%.9lf", MJD_9min);
	myfile << buf << std::endl << std::endl;

	myfile << "Target state vector for scenario (T-9min):" << std::endl;
	sprintf_s(buf, "RPOS %.3lf %.3lf %.3lf", sv_T_9min.R.x, sv_T_9min.R.z, sv_T_9min.R.y);
	myfile << buf << std::endl;
	sprintf_s(buf, "RVEL %.4lf %.4lf %.4lf", sv_T_9min.V.x, sv_T_9min.V.z, sv_T_9min.V.y);
	myfile << buf << std::endl << std::endl;

	myfile << "MJD of T-31sec:" << std::endl;
	sprintf_s(buf, "%.9lf", MJD_31sec);
	myfile << buf << std::endl << std::endl;

	myfile << "Target state vector for scenario (T-31sec):" << std::endl;
	sprintf_s(buf, "RPOS %.3lf %.3lf %.3lf", sv_T_31sec.R.x, sv_T_31sec.R.z, sv_T_31sec.R.y);
	myfile << buf << std::endl;
	sprintf_s(buf, "RVEL %.4lf %.4lf %.4lf", sv_T_31sec.V.x, sv_T_31sec.V.z, sv_T_31sec.V.y);
	myfile << buf << std::endl;

	myfile << std::endl << "LEGACY MECO" << std::endl << std::endl;

	sprintf_s(buf, "%.4lf", LTP_Outputs.IMECO * OrbMech::DEG);
	myfile << "Inclination (deg) " << buf << std::endl;
	sprintf_s(buf, "%.0lf", LTP_Outputs.RMECO - globcnst.R_E); // 6.37101e6 radius in default Orbiter
	myfile << "Altitude (m) " << buf << std::endl;
	sprintf_s(buf, "%.4lf", LTP_Outputs.VMECO);
	myfile << "Velocity (m/s) " << buf << std::endl;
	sprintf_s(buf, "%.4lf", LTP_Outputs.GMECO * OrbMech::DEG);
	myfile << "Flight Path Angle (deg) " << buf << std::endl;
	sprintf_s(buf, "GMTLO Ref %.3lf", LTP_Outputs.GMTLO + sescnst.DayOfYear * 24.0 * 3600.0);
	myfile << buf << std::endl;
	sprintf_s(buf, "%.8lf %.8lf %.8lf", IY_MECO_M50.x, IY_MECO_M50.y, IY_MECO_M50.z);
	myfile << "IY Vector (M50) " << buf << std::endl;
	sprintf_s(buf, "Node Slope %e", LTP_Outputs.NODE_SLOPE);
	myfile << buf << std::endl;

	myfile << std::endl << "I-LOADS" << std::endl << std::endl;

	sprintf_s(buf, "TLO %.3lf", LTP_Outputs.GMTLO + sescnst.DayOfYear * 24.0 * 3600.0);
	myfile << buf << std::endl;

	// TBD: Min IY

	sprintf_s(buf, "IYD_NOM = %+.8lf %+.8lf %+.8lf", IY_MECO_M50.x, IY_MECO_M50.y, IY_MECO_M50.z);
	myfile << buf << std::endl;
	sprintf_s(buf, "NODE_SLOPE = %e", LTP_Outputs.NODE_SLOPE);
	myfile << buf << std::endl;

	// TBD: D-psi, Delta Node Phase, GMT Phase

	sprintf_s(buf, "IYD_OMS1 = %+.8lf %+.8lf %+.8lf", IY_OMS1_M50.x, IY_OMS1_M50.y, IY_OMS1_M50.z);
	myfile << buf << std::endl;
	sprintf_s(buf, "IYD_OMS2 = %+.8lf %+.8lf %+.8lf", IY_OMS2_M50.x, IY_OMS2_M50.y, IY_OMS2_M50.z);
	myfile << buf << std::endl;
	sprintf_s(buf, "DTIG_OMS1 = %lf", LTP_Outputs.OMS1.DTIG - DT_ET_SET);
	myfile << buf << std::endl;
	sprintf_s(buf, "HTGT_OMS1 = %.1lf", LTP_Outputs.OMS1.HTGT / OrbMech::FT2M);
	myfile << buf << std::endl;
	sprintf_s(buf, "THETA_OMS1 = %lf", LTP_Outputs.OMS1.THETA);
	myfile << buf << std::endl;
	sprintf_s(buf, "C1_OMS1 = %lf", LTP_Outputs.OMS1.C1 / OrbMech::FT2M);
	myfile << buf << std::endl;
	sprintf_s(buf, "C2_OMS1 = %lf", LTP_Outputs.OMS1.C2);
	myfile << buf << std::endl;
	sprintf_s(buf, "DTIG_OMS2 = %lf", LTP_Outputs.OMS2.DTIG - DT_ET_SET);
	myfile << buf << std::endl;
	sprintf_s(buf, "HTGT_OMS2 = %.1lf", LTP_Outputs.OMS2.HTGT / OrbMech::FT2M);
	myfile << buf << std::endl;
	sprintf_s(buf, "THETA_OMS2 = %lf", LTP_Outputs.OMS2.THETA);
	myfile << buf << std::endl;
	sprintf_s(buf, "C1_OMS2 = %lf", LTP_Outputs.OMS2.C1 / OrbMech::FT2M);
	myfile << buf << std::endl;
	sprintf_s(buf, "C2_OMS2 = %lf", LTP_Outputs.OMS2.C2);
	myfile << buf << std::endl;

	myfile.close();
	return 0;
}

int Core::SaveShuttleLWPStateVector(std::string filename)
{
	if (LTP_Outputs.sv_C_OMS1.GMT != 0.0)
	{
		SaveStateVector(ProjectFolder + "State Vectors/" + filename, LTP_Outputs.sv_C_OMS1);
		return 0;
	}
	return 1;
}

bool Core::RunOMP(std::string inputs[], std::string& errormessage, std::vector<std::vector<std::string>>& data)
{
	OMP::OMPInputs in;
	OMP::OMPOutputs out;
	OMP::OrbitalManeuverProcessor omp(globcnst, sescnst);
	int hh, mm, err;
	double ss;

	ProjectFolder = "Projects/" + inputs[0] + "/";

	if (sscanf_s(inputs[1].c_str(), "%d:%d:%lf", &hh, &mm, &ss) != 3)
	{
		errormessage = "Launch time format invalid";
		return true;
	}
	SetLaunchTime(hh, mm, ss);

	if (LoadStateVector(ProjectFolder + "State Vectors/" + inputs[2], in.CHASER))
	{
		errormessage = "Chaser state vector file not found";
		return true;
	}
	if (LoadStateVector(ProjectFolder + "State Vectors/" + inputs[3], in.TARGET))
	{
		errormessage = "Target state vector file not found";
		return true;
	}

	err = ReadMCTFile(ProjectFolder + inputs[4], ManeuverConstraintsInput);

	if (err)
	{
		errormessage = "MCT file could not be read";
		return true;
	}

	if (omp.ParseManeuverConstraintsTable(ManeuverConstraintsInput, in.ManeuverConstraintsTable, errormessage))
	{
		return true;
	}

	in.ProjectFolder = ProjectFolder;
	in.OMPChaserFile = inputs[2];
	in.OMPTargetFile = inputs[3];
	in.OMPMCTFile = inputs[4];

	omp.Calculate(in, out);

	if (out.Error)
	{
		errormessage = out.ErrorMessage;
		return out.Error;
	}

	data = out.OutputPrint;

	return 0;
}

int Core::RunSkylabLWP(std::string inputs[], std::vector<std::vector<std::string>>& data)
{
	SkylabLWP::SkylabLWPInputs in;
	SkylabLWP::SkylabLWPOutputs out;
	SkylabLWP::SkylabLaunchWindowProcessor slwp(globcnst, sescnst);

	ProjectFolder = "Projects/" + inputs[0] + "/";

	if (LoadStateVector(ProjectFolder + "State Vectors/" + inputs[1], in.sv_T)) return 1;
	in.CKFactor = std::stod(inputs[2]);
	in.CAREA = std::stod(inputs[3]) * pow(OrbMech::FT2M, 2);
	in.CWHT = std::stod(inputs[4]) * OrbMech::LBS;
	
	if (inputs[5] == "NO")
	{
		in.NS = 0;
	}
	else if (inputs[5] == "SO")
	{
		in.NS = 1;
	}
	else return 1;

	in.DAY = std::stoi(inputs[6]);
	in.LATLS = std::stod(inputs[7]) * OrbMech::RAD;
	in.LONGLS = std::stod(inputs[8]) * OrbMech::RAD;
	in.PFT = std::stod(inputs[9]);
	in.PFA = std::stod(inputs[10]) * OrbMech::RAD;
	in.YSMAX = std::stod(inputs[11]) * OrbMech::RAD;
	in.DTOP = std::stod(inputs[12]);
	in.RINS = std::stod(inputs[13]) * OrbMech::FT2M;
	in.VINS = std::stod(inputs[14]) * OrbMech::FT2M;
	in.GAMINS = std::stod(inputs[15]) * OrbMech::RAD;

	if (inputs[16] == "PO")
	{
		in.NEGTIV = 0;
	}
	else if (inputs[16] == "NE")
	{
		in.NEGTIV = 1;
	}
	else if (inputs[16] == "LO")
	{
		in.NEGTIV = 2;
	}
	else return 1;

	in.WRAP = std::stoi(inputs[17]);
	in.DTOPEN = std::stod(inputs[18]) * 60.0;
	in.DTCLOSE = std::stod(inputs[19]) * 60.0;

	if (inputs[20] == "LW")
	{
		in.IR = false;
	}
	else if (inputs[20] == "LT")
	{
		in.IR = true;
	}
	else return 1;

	if (ParseGMT(inputs[21], in.TLO)) return 1;

	in.MI = std::stoi(inputs[22]);
	in.MF = std::stoi(inputs[23]);

	if (ParseGMT(inputs[24], in.DTSEP)) return 1;

	if (sscanf_s(inputs[25].c_str(), "%lf %lf %lf", &in.DVSEP.x, &in.DVSEP.y, &in.DVSEP.z) != 3) return 1;
	//in.DVSEP *= OrbMech::FT2M;

	in.DVWO = std::stod(inputs[26]) * OrbMech::FT2M;
	in.DVWC = std::stod(inputs[27]) * OrbMech::FT2M;

	int err = slwp.Calculate(in, out);
	if (err) return err;

	data = out.OutputPrint;

	return 0;
}

int Core::AgeOfStateVector(std::string inputs[], double& age) const
{
	if (sescnst.GMTBASE == 0.0) return 1;

	OrbMech::StateVector sv;
	std::string ProjectFolder, file;

	ProjectFolder = "Projects/" + inputs[0] + "/";
	file = inputs[1];

	if (LoadStateVector(ProjectFolder + "State Vectors/" + file, sv)) return 2;

	age = sv.GMT;
	return 0;
}

int Core::ReadMCTFile(std::string file, std::vector<OMP::ManeuverConstraintsInput>& MCT) const
{
	MCT.clear();

	std::ifstream myfile;

	myfile.open(file);
	if (myfile.is_open() == false) return 1;

	std::string line;
	int i;
	
	i = 1;

	while (std::getline(myfile, line))
	{
		if (ReadMCTLine(line, MCT))
		{
			std::cout << "Error in constraints for maneuver " << i << std::endl;
			return 1;
		}
		i++;
	}

	myfile.close();
	return 0;
}

int Core::ReadMCTLine(std::string line, std::vector<OMP::ManeuverConstraintsInput>& MCT) const
{
	std::string data[13];

	std::stringstream ss(line);
	std::string token;
	std::vector<std::string> tokens;
	char delimiter = ';';

	while (std::getline(ss, token, delimiter)) {
		tokens.push_back(token);
	}

	//Too small?
	if (tokens.size() < 4) return 1;

	OMP::ManeuverConstraintsInput temp;

	//Parse the individual elements
	temp.Name = tokens[0];
	temp.Type = tokens[1];
	temp.Threshold = tokens[2];
	temp.ThresholdValue = tokens[3];

	char Buff[5], Buff2[11];
	sprintf_s(Buff, "");
	sprintf_s(Buff2, "");

	for (unsigned i = 4; i < tokens.size(); i++)
	{
		//Secondaries
		if (sscanf_s(tokens[i].c_str(), "%[^'=']=%s", Buff, 5, Buff2, 11) != 2) return 2;

		temp.Secondary[i - 4].assign(Buff);
		temp.SecondaryValues[i - 4].assign(Buff2);
	}

	MCT.push_back(temp);
	return 0;
}

bool Core::IsInitialized() const
{
	return Initialized;
}

double Core::GetLWP_GMTLO() const
{
	return LWPSummaryTable.GMTLO;
}

int Core::GetDayOfYear() const
{
	return sescnst.DayOfYear;
}

int Core::GetEphemerisDT(double& EDT) const
{
	// Use session constant GMTBASE

	static const double MJD[] = { 41317.0, 41499.0, 41683.0, 42048.0, 42413.0, 42778.0, 43144.0, 43509.0, 43874.0, 44239.0, 44786.0, 45151.0, 45516.0, 46247.0,
		47161.0, 47892.0, 48257.0, 48804.0, 49169.0, 49534.0, 50083.0, 50630.0, 51179.0, 53736.0, 54832.0, 56109.0, 57204.0, 57754.0 };
	static const int NMAX = 28;

	// Out of bounds check
	if (sescnst.GMTBASE < MJD[0])
	{
		return 1;
	}

	double DAT;
	int i;

	i = 0;
	DAT = 9.0;

	while (i < NMAX && sescnst.GMTBASE > MJD[i])
	{
		DAT += 1.0;
		i++;
	}

	// TAI to TT
	EDT = DAT + 32.184;

	return 0;
}

int Core::SetConstants(int world, int Year, int Month, int Day, double EDT)
{
	Initialized = false;
	if (SetGlobalConstants(world)) return 1;
	if (SetSessionConstants(world, Year, Month, Day, EDT)) return 1;
	Initialized = true;
	return 0;
}

int Core::SetGlobalConstants(int world)
{
	double L_ref, e_ref, a, b;

	if (world == 0)
	{
		// NASSP
		globcnst.mu = 0.3986032e15;
		globcnst.R_E = globcnst.R_E_equ = 6.373338e6;
		globcnst.J2 = 1082.6269e-6;
		globcnst.J3 = -2.51e-6;
		globcnst.J4 = -1.60e-6;
		globcnst.w_E = 7.29211514667e-5;

		globcnst.T_p = -9413040.4;
		globcnst.L_0 = 0.00001553343;
		globcnst.e_rel = 0.4090928023;
		globcnst.phi_0 = 4.894942829;
		globcnst.T_s = 86164.098904 / 24.0 / 60.0 / 60.0;

		L_ref = 0.0;
		e_ref = 0.0;

		a = b = globcnst.R_E;
	}
	else if (world == 1)
	{
		// SSV
		globcnst.mu = 398600439968871.2;
		globcnst.R_E = 6.37101e6;
		globcnst.R_E_equ = 6378166.0;
		globcnst.J2 = 1082.6269e-6;
		globcnst.J3 = -2.51e-6;
		globcnst.J4 = -1.60e-6;
		globcnst.w_E = 7.292114942213369e-05;

		globcnst.T_p = -9413040.4;
		globcnst.L_0 = 0.0;
		globcnst.e_rel = 0.4090928023;
		globcnst.phi_0 = 4.88948754;
		globcnst.T_s = 86164.10132 / 24.0 / 60.0 / 60.0;

		L_ref = 0.0;
		e_ref = 0.0;

		a = b = globcnst.R_E;
	}
	else if (world == 2)
	{
		// "Real" world
		globcnst.mu = 398600640000000.0;
		globcnst.R_E = 6378140.0;
		globcnst.R_E_equ = 6378166.0;
		globcnst.J2 = 1.0826271e-3;
		globcnst.J3 = -2.5358868e-6;
		globcnst.J4 = -1.624618e-6;
		globcnst.w_E = 7.29211514646e-5;

		// TBD

		a = 6378166.0;
		b = 6356784.283603816;

		L_ref = 0.0;
		e_ref = 0.0;
	}
	else return 1;

	globcnst.R_ref = mul(_M(cos(L_ref), 0, -sin(L_ref), 0, 1, 0, sin(L_ref), 0, cos(L_ref)), _M(1, 0, 0, 0, cos(e_ref), -sin(e_ref), 0, sin(e_ref), cos(e_ref)));
	globcnst.R_obl = _M(1, 0, 0, 0, cos(globcnst.e_rel), -sin(globcnst.e_rel), 0, sin(globcnst.e_rel), cos(globcnst.e_rel));

	globcnst.sqrt_mu = sqrt(globcnst.mu);
	globcnst.a_sq = a * a;
	globcnst.b_sq = b * b;

	return 0;
}

int Core::SetSessionConstants(int world, int Year, int Month, int Day, double EDT)
{
	// EDT = Difference between Ephemeris Time and Universal Time

	if (Year < 1900 || Year > 2100) return 1;
	if (Month > 12) return 1;
	if (Day > 31) return 1;

	//Calculate base MJD
	int DayOfYear;
	DayOfYear = OrbMech::dayofyear(Year, Month, Day);
	sescnst.GMTBASE = OrbMech::Date2MJD(Year, DayOfYear, 0, 0, 0.0);
	sescnst.GMTLO = 0.0;

	if (world == 2)
	{
		// Real
		MATRIX3 RNP = OrbMech::B1950InertialToPseudoBodyFixed(sescnst.GMTBASE, EDT);
		MATRIX3 B1950_to_J2000_equ = _M(0.9999256794956877, -0.0111814832204662, -0.0048590038153592,
			0.0111814832391717, 0.9999374848933135, -0.0000271625947142,
			0.0048590037723143, -0.0000271702937440, 0.9999881946023742);

		double eps, ce, se;
		
		eps = 0.4090928023;// 23.44 * OrbMech::RAD;
		ce = cos(eps);
		se = sin(eps);

		MATRIX3 J2000_equ_to_J2000_ecl = _M(1.0, 0.0, 0.0, 0.0, ce, se, 0.0, -se, ce);

		sescnst.M_TEG_TO_M50 = OrbMech::tmat(RNP);
		sescnst.M_TEG_TO_J2000 = mul(mul(J2000_equ_to_J2000_ecl, B1950_to_J2000_equ), sescnst.M_TEG_TO_M50);

		sescnst.TimeSystem = 1;
		sescnst.EDT = EDT;
	}
	else
	{
		// Orbiter
		// TEG to J2000 ecliptic (left handed)
		MATRIX3 M_EFTOECL_AT_EPOCH = OrbMech::GetRotationMatrix(globcnst, sescnst.GMTBASE);
		// M50 to J2000 (left handed)
		MATRIX3 M_M50TOECL = OrbMech::GetObliquityMatrix(globcnst, 33281.923357);
		// TEG to M50 (right handed)
		sescnst.M_TEG_TO_M50 = OrbMech::MatrixRH_LH(mul(OrbMech::tmat(M_M50TOECL), M_EFTOECL_AT_EPOCH));
		// TEG to J2000 (right handed)
		sescnst.M_TEG_TO_J2000 = OrbMech::MatrixRH_LH(M_EFTOECL_AT_EPOCH);

		sescnst.TimeSystem = 2;
		sescnst.EDT = EDT;
	}

	sescnst.Year = Year;
	sescnst.Month = Month;
	sescnst.Day = Day;
	sescnst.Hours = sescnst.Minutes = 0;
	sescnst.launchdateSec = 0.0;
	sescnst.DayOfYear = DayOfYear;
	return 0;
}

void Core::SetLaunchTime(int H, int M, double S)
{
	//Launch time
	sescnst.GMTLO = (double)(H * 3600 + M * 60) + S;

	sescnst.Hours = H;
	sescnst.Minutes = M;
	sescnst.launchdateSec = S;
}

void Core::SaveStateVector(std::string file, const OrbMech::StateVector& sv)
{
	std::ofstream myfile;
	myfile.open(file);
	if (myfile.is_open() == false) return;

	std::vector<std::string> data;

	StateVectorToString(sv, data);

	for (unsigned i = 0; i < data.size(); i++)
	{
		myfile << data[i] << std::endl;
	}

	myfile.close();
}

int Core::LoadStateVector(std::string file, OrbMech::StateVector &sv) const
{
	//Return state vector in TEG coordinates

	std::ifstream myfile;

	myfile.open(file);
	if (myfile.is_open() == false) return 1;

	std::string line;
	int coord;

	//Line 1: Coordinate system
	std::getline(myfile, line);

	if (line == "TEG")
	{
		coord = 0;
	}
	else if (line == "J2000")
	{
		coord = 1;
	}
	else if (line == "M50")
	{
		coord = 2;
	}
	else if (line == "TLE")
	{
		coord = 10;
	}
	else return 2;

	if (coord < 10)
	{
		OrbMech::StateVector sv_temp;

		//Line 2: Position vector
		std::getline(myfile, line);
		if (sscanf_s(line.c_str(), "%lf %lf %lf", &sv_temp.R.x, &sv_temp.R.y, &sv_temp.R.z) != 3) return 2;

		//Line 3: Velocity vector
		std::getline(myfile, line);
		if (sscanf_s(line.c_str(), "%lf %lf %lf", &sv_temp.V.x, &sv_temp.V.y, &sv_temp.V.z) != 3) return 2;

		//Line 4: Time
		std::getline(myfile, line);
		if (sscanf_s(line.c_str(), "%lf", &sv_temp.GMT) != 1) return 2;

		//Line 5: Weight
		std::getline(myfile, line);
		if (sscanf_s(line.c_str(), "%lf", &sv_temp.Weight) != 1) return 2;
		sv_temp.Weight *= OrbMech::LBS;

		//Line 6: Area
		std::getline(myfile, line);
		if (sscanf_s(line.c_str(), "%lf", &sv_temp.Area) != 1) return 2;
		sv_temp.Area *= pow(0.3048, 2);

		//Line 7: K-Factor
		std::getline(myfile, line);
		if (sscanf_s(line.c_str(), "%lf", &sv_temp.KFactor) != 1) return 2;

		sv = sv_temp;

		// Coordinate conversion
		if (coord == 1 || coord == 2)
		{
			MATRIX3 Rot;

			if (coord == 1)
			{
				// J2000 to TEG
				Rot = mul(OrbMech::tmat(sescnst.M_TEG_TO_M50), OrbMech::M_J2000_to_M50);
				sv.R = mul(Rot, _V(sv_temp.R.x, sv_temp.R.z, sv_temp.R.y));
				sv.V = mul(Rot, _V(sv_temp.V.x, sv_temp.V.z, sv_temp.V.y));
			}
			else
			{
				// M50 to TEG
				Rot = OrbMech::tmat(sescnst.M_TEG_TO_M50);
				sv.R = mul(Rot, sv_temp.R);
				sv.V = mul(Rot, sv_temp.V);
			}

			sv.GMT = (sv_temp.GMT - sescnst.GMTBASE) * 24.0 * 3600.0; //MJD to GMT
		}
	}
	else
	{
		//TLE
		char Buffer1[130], Buffer2[130];

		//Line 2: TLE line 1
		std::getline(myfile, line);
		sprintf_s(Buffer1, "%s", line.c_str());

		//Line 3: TLE line 2
		std::getline(myfile, line);
		sprintf_s(Buffer2, "%s", line.c_str());

		double startmfe, stopmfe, deltamin, r[3], v[3], mjd_cur;
		char typerun, typeinput, opsmode;
		gravconsttype whichconst;
		elsetrec satrec;

		typerun = 'm';
		typeinput = 'm';
		opsmode = 'i';
		whichconst = gravconsttype::wgs84;

		// TLE to SGP4 array
		SGP4Funcs::twoline2rv(Buffer1, Buffer2, typerun, typeinput, opsmode, whichconst, startmfe, stopmfe, deltamin, satrec);
		if (satrec.error) return satrec.error;

		// Convert to cartesian
		SGP4Funcs::sgp4(satrec, 0.0, r, v);
		if (satrec.error) return satrec.error;

		// Calculate current MJD
		double JD_UTC = satrec.jdsatepoch + satrec.jdsatepochF;
		mjd_cur = OrbMech::JD2MJD(JD_UTC);

		// Convert from TEME to Earth fixed
		MATRIX3 M_TEME_PEF = OrbMech::TEMEToPseudoBodyFixed(JD_UTC);

		sv.R = _V(r[0], r[1], r[2]) * 1000.0;
		sv.V = _V(v[0], v[1], v[2]) * 1000.0;

		sv.R = mul(M_TEME_PEF, sv.R);
		sv.V = mul(M_TEME_PEF, sv.V);

		// Convert from Earth fixed to TEG
		sv.GMT = (OrbMech::JD2MJD(JD_UTC) - sescnst.GMTBASE) * 86400.0;

		if (sescnst.TimeSystem == 2)
		{
			// Convert from UTC to TDB
			sv.GMT += sescnst.EDT;
		}

		MATRIX3 M_EF_TEG = OrbMech::TEG_to_EF_Matrix(globcnst, sv.GMT);

		sv.R = tmul(M_EF_TEG, sv.R);
		sv.V = tmul(M_EF_TEG, sv.V);

		//Line 4: Weight
		std::getline(myfile, line);
		if (sscanf_s(line.c_str(), "%lf", &sv.Weight) != 1) return 2;
		sv.Weight *= OrbMech::LBS;

		//Line 5: Area
		std::getline(myfile, line);
		if (sscanf_s(line.c_str(), "%lf", &sv.Area) != 1) return 2;
		sv.Area *= pow(0.3048, 2);

		//Line 6: K-Factor
		std::getline(myfile, line);
		if (sscanf_s(line.c_str(), "%lf", &sv.KFactor) != 1) return 2;
	}

	myfile.close();
	return 0;
}

void Core::StateVectorToString(const OrbMech::StateVector& sv, std::vector<std::string>& data) const
{
	char Buffer[256];

	data.clear();

	//Line 1: Coordinate system
	data.push_back("TEG");
	//Line 2: Position vector
	sprintf_s(Buffer, "%lf %lf %lf", sv.R.x, sv.R.y, sv.R.z);
	data.push_back(Buffer);
	//Line 3: Velocity vector
	sprintf_s(Buffer, "%lf %lf %lf", sv.V.x, sv.V.y, sv.V.z);
	data.push_back(Buffer);
	//Line 4: Time
	sprintf_s(Buffer, "%.3lf", sv.GMT);
	data.push_back(Buffer);
	//Line 5: Weight
	sprintf_s(Buffer, "%.4lf", sv.Weight / OrbMech::LBS);
	data.push_back(Buffer);
	//Line 6: Area
	sprintf_s(Buffer, "%.2lf", sv.Area / pow(OrbMech::FT2M, 2));
	data.push_back(Buffer);
	//Line 7: K-Factor
	sprintf_s(Buffer, "%.3lf", sv.KFactor);
	data.push_back(Buffer);
}

int Core::StateVectorConverter(int* iInputs, double* dInputs, std::vector<std::string>& data)
{
	// Get conversion factors from input to internal units
	double ANG_CONV, DIST_CONV, VEL_CONV, MASS_CONV, LENGTH_CONV;

	// Angles
	switch (iInputs[2])
	{
	case 0: // Degrees
		ANG_CONV = OrbMech::RAD;
		break;
	case 1: // Radians
		ANG_CONV = 1.0;
		break;
	default:
		return 1;
	}

	// Distance
	switch (iInputs[3])
	{
	case 0: // Feet
		DIST_CONV = OrbMech::FT2M;
		break;
	case 1: // Meters
		DIST_CONV = 1.0;
		break;
	case 2: // Nautical miles
		DIST_CONV = OrbMech::NM;
		break;
	case 3: // Earth radii
		DIST_CONV = 6378165.0;
		break;
	case 4: // Kilometers
		DIST_CONV = 0.001;
		break;
	default:
		return 1;
	}

	// Velocity
	switch (iInputs[4])
	{
	case 0: // Feet/second
		VEL_CONV = OrbMech::FT2M;
		break;
	case 1: // Meters/second
		VEL_CONV = 1.0;
		break;
	case 2: // Nautical miles/hour
		VEL_CONV = OrbMech::NM / 3600.0;
		break;
	case 3: // Earth radii/hour
		VEL_CONV = 6378165.0 / 3600.0;
		break;
	case 4: // Kilometers/hour
		VEL_CONV = 1000.0 / 3600.0;
		break;
	default:
		return 1;
	}

	// Mass
	switch (iInputs[5])
	{
	case 0: // Pounds
		MASS_CONV = OrbMech::LBS;
		break;
	case 1: // Kilograms
		MASS_CONV = 1.0;
		break;
	case 2: // Slugs
		MASS_CONV = 14.59390;
		break;
	default:
		return 1;
	}

	// Length
	switch (iInputs[6])
	{
	case 0: // Feet
		LENGTH_CONV = OrbMech::FT2M;
		break;
	case 1: // Meters
		LENGTH_CONV = 1.0;
		break;
	case 2: // Inches
		LENGTH_CONV = 0.0254;
		break;
	default:
		return 1;
	}

	// Convert state vector
	OrbMech::StateVector sv_in, sv_out;

	sv_in.GMT = dInputs[0];

	if (iInputs[1] == 0)
	{
		// Cartesian
		sv_in.R.x = dInputs[1] * DIST_CONV;
		sv_in.R.y = dInputs[2] * DIST_CONV;
		sv_in.R.z = dInputs[4] * DIST_CONV;
		sv_in.V.x = dInputs[4] * VEL_CONV;
		sv_in.V.y = dInputs[5] * VEL_CONV;
		sv_in.V.z = dInputs[6] * VEL_CONV;
	}
	else if (iInputs[1] == 1)
	{
		// Orbital elements
		OrbMech::CELEMENTS coe;

		coe.a = dInputs[1] * DIST_CONV;
		coe.e = dInputs[2];
		coe.i = dInputs[3] * ANG_CONV;
		coe.g = dInputs[4] * ANG_CONV;
		coe.h = dInputs[5] * ANG_CONV;
		coe.l = dInputs[6] * ANG_CONV;

		if (coe.e >= 1.0) return 1;

		OrbMech::KeplerianToCartesian(coe, globcnst.mu, sv_in.R, sv_in.V);
	}
	else if (iInputs[1] == 2)
	{
		// Spherical
		double r, v, lat, lng, gamma, incl, azi;
		bool AscNodeFlag;

		r = dInputs[1] * DIST_CONV;
		v = dInputs[2] * VEL_CONV;
		gamma = dInputs[3] * ANG_CONV;
		lat = dInputs[4] * ANG_CONV;
		lng = dInputs[5] * ANG_CONV;
		incl = dInputs[6] * ANG_CONV;

		AscNodeFlag = true;
		if (incl < 0.0)
		{
			incl = abs(incl);
			AscNodeFlag = false;
		}

		if (abs(lat) > incl) return 1;

		// Calculate azimuth
		azi = asin(cos(incl) / cos(lat));
		if (AscNodeFlag == false)
		{
			azi = OrbMech::PI - azi;
		}

		OrbMech::PICSSC(false, sv_in.R, sv_in.V, r, v, lat, lng, gamma, azi);
	}
	else if (iInputs[1] == 3)
	{
		// Apsides
		double HA, HP, TA, lat, lng, incl, azi, RA, RP, a, e, p, r, v, gamma;
		bool AscNodeFlag;

		HA = dInputs[1] * DIST_CONV;
		HP = dInputs[2] * DIST_CONV;
		TA = dInputs[3] * ANG_CONV;
		lat = dInputs[4] * ANG_CONV;
		lng = dInputs[5] * ANG_CONV;
		incl = dInputs[6] * ANG_CONV;

		AscNodeFlag = true;
		if (incl < 0.0)
		{
			incl = abs(incl);
			AscNodeFlag = false;
		}

		if (abs(lat) > incl) return 1;

		// Calculate azimuth
		azi = asin(cos(incl) / cos(lat));
		if (AscNodeFlag == false)
		{
			azi = OrbMech::PI - azi;
		}

		RA = HA + globcnst.R_E_equ;
		RP = HP + globcnst.R_E_equ;
		a = (RA + RP) / 2.0;
		e = (RA - RP) / (RA + RP);

		if (e >= 1.0) return 1;

		p = a * (1.0 - e * e);
		r = p / (1.0 + e * cos(TA));
		v = sqrt(2.0 * (globcnst.mu / r - globcnst.mu / (2.0 * a)));
		gamma = atan(e * sin(TA) / (1.0 + e * cos(TA)));

		OrbMech::PICSSC(false, sv_in.R, sv_in.V, r, v, lat, lng, gamma, azi);
	}
	else return 1;

	// K-Factor
	sv_in.KFactor = dInputs[7];

	// Area
	sv_in.Area = dInputs[8] * pow(LENGTH_CONV, 2);

	// Weight
	sv_in.Weight = dInputs[9] * MASS_CONV;

	// Convert to output coordinate system
	sv_out = sv_in;
	if (iInputs[0] == 0)
	{
		// TEG
	}
	else if (iInputs[0] == 1)
	{
		// Earth-fixed
		MATRIX3 M_TEG_EF = OrbMech::TEG_to_EF_Matrix(globcnst, sv_in.GMT);

		sv_out.R = tmul(M_TEG_EF, sv_in.R);
		sv_out.V = tmul(M_TEG_EF, sv_in.V);
	}
	else return 1;

	StateVectorToString(sv_out, data);

	return 0;
}