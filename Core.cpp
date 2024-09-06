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

static inline bool papiReadScenario_double(const char* line, const char* item, double& d) {

	char buffer[256];
	double e;

	if (sscanf_s(line, "%s", buffer, 255) == 1) {
		if (!strcmp(buffer, item)) {
			if (sscanf_s(line, "%s %lf", buffer, 255, &e) == 2) {
				d = e;
				return true;
			}
		}
	}
	return false;
}

static inline bool papiReadScenario_int(const char* line, const char* item, int& i) {

	char buffer[256];
	int e;

	if (sscanf_s(line, "%s", buffer, 255) == 1) {
		if (!strcmp(buffer, item)) {
			if (sscanf_s(line, "%s %d", buffer, 255, &e) == 2) {
				i = e;
				return true;
			}
		}
	}
	return false;
}

static inline bool papiReadScenario_string(std::string line, std::string item, std::string& s) {

	std::string temp;

	temp = line.substr(0, line.find(' '));

	if (temp == item)
	{
		s = line.substr(item.size() + 1U);
		return true;
	}
	return false;
}

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
		SaveStateVector(ProjectFolder + "State Vectors/LWP.txt", PVTABC);
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

	err = ReadMCTFile(ProjectFolder + inputs[4]);

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

int Core::CalculateDayOfYear(int Year, int Month, int Day, int& DOY) const
{
	if (Year < 1900 || Year > 2100) return 1;
	if (Month < 1 || Month > 12) return 1;
	if (Day < 1 || Day > 31) return 1;

	DOY = OrbMech::dayofyear(Year, Month, Day);
	return 0;
}

int Core::ReadMCTFile(std::string file)
{
	ManeuverConstraintsInput.clear();

	std::ifstream myfile;

	myfile.open(file);
	if (myfile.is_open() == false) return 1;

	std::string line;
	int i;

	//Ignore first line
	std::getline(myfile, line);
	
	i = 1;

	while (std::getline(myfile, line))
	{
		if (ReadMCTLine(line))
		{
			std::cout << "Error in constraints for maneuver " << i << std::endl;
			return 1;
		}
		i++;
	}

	myfile.close();
	return 0;
}

int Core::ReadMCTLine(std::string line)
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

	ManeuverConstraintsInput.push_back(temp);
	return 0;
}

int Core::SetConstants(int world, int Year, int Month, int Day)
{
	if (SetGlobalConstants(world)) return 1;
	if (SetSessionConstants(Year, Month, Day)) return 1;
	Initialized = true;
	return 0;
}

int Core::SetGlobalConstants(int world)
{
	double L_ref, e_ref, a, b;

	if (world == 0)
	{
		//NASSP
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
		//SSV
		globcnst.mu = 0.3986032e15;
		globcnst.R_E = 6.37101e6;
		globcnst.R_E_equ = 6378165.0;
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
		//"Real" world

		a = 6389178.0;
		b = 6356797.0;

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

int Core::SetSessionConstants(int Year, int Month, int Day)
{
	if (Year < 1900 || Year > 2100) return 1;
	if (Month > 12) return 1;
	if (Day > 31) return 1;

	//Calculate base MJD
	int DayOfYear;
	DayOfYear = OrbMech::dayofyear(Year, Month, Day);
	sescnst.GMTBASE = OrbMech::Date2MJD(Year, DayOfYear, 0, 0, 0.0);
	sescnst.GMTLO = 0.0;

	// TEG to J2000 ecliptic (left handed)
	MATRIX3 M_EFTOECL_AT_EPOCH = OrbMech::GetRotationMatrix(globcnst, sescnst.GMTBASE);
	// M50 to J2000 (left handed)
	MATRIX3 M_M50TOECL = OrbMech::GetObliquityMatrix(globcnst, 33281.923357);
	// TEG to M50 (right handed)
	sescnst.M_TEG_TO_M50 = OrbMech::MatrixRH_LH(mul(OrbMech::tmat(M_M50TOECL), M_EFTOECL_AT_EPOCH));
	// TEG to J2000 (right handed)
	sescnst.M_TEG_TO_J2000 = OrbMech::MatrixRH_LH(M_EFTOECL_AT_EPOCH);

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

	char Buffer[256];

	//Line 1: Coordinate system
	myfile << "TEG" << std::endl;
	//Line 2: Position vector
	sprintf_s(Buffer, "%lf %lf %lf", sv.R.x, sv.R.y, sv.R.z);
	myfile << Buffer << std::endl;
	//Line 3: Velocity vector
	sprintf_s(Buffer, "%lf %lf %lf", sv.V.x, sv.V.y, sv.V.z);
	myfile << Buffer << std::endl;
	//Line 4: Time
	sprintf_s(Buffer, "%.3lf", sv.GMT);
	myfile << Buffer << std::endl;
	//Line 5: Weight
	sprintf_s(Buffer, "%.4lf", sv.Weight / OrbMech::LBS);
	myfile << Buffer << std::endl;
	//Line 6: Area
	sprintf_s(Buffer, "%.2lf", sv.Area / pow(OrbMech::FT2M, 2));
	myfile << Buffer << std::endl;
	//Line 7: K-Factor
	sprintf_s(Buffer, "%.3lf", sv.KFactor);
	myfile << Buffer << std::endl;

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

		//Coordinate conversion
		if (coord == 1)
		{
			//J2000 to TEG
			MATRIX3 Rot;

			Rot = mul(OrbMech::tmat(sescnst.M_TEG_TO_M50), OrbMech::M_J2000_to_M50);

			sv.R = mul(Rot, _V(sv_temp.R.x, sv_temp.R.z, sv_temp.R.y));
			sv.V = mul(Rot, _V(sv_temp.V.x, sv_temp.V.z, sv_temp.V.y));
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

		//TLE to SGP4 array
		SGP4Funcs::twoline2rv(Buffer1, Buffer2, typerun, typeinput, opsmode, whichconst, startmfe, stopmfe, deltamin, satrec);
		if (satrec.error) return satrec.error;

		//Calculate current MJD
		mjd_cur = satrec.jdsatepoch + satrec.jdsatepochF - 2400000.5;

		//Convert to cartesian
		SGP4Funcs::sgp4(satrec, 0.0, r, v);
		if (satrec.error) return satrec.error;

		//Convert from TEME to J2000
		MATRIX3 Rot = OrbMech::GetObliquityMatrix(globcnst, mjd_cur);
		Rot = OrbMech::MatrixRH_LH(Rot);

		sv.R = _V(r[0], r[1], r[2]) * 1000.0;
		sv.V = _V(v[0], v[1], v[2]) * 1000.0;

		sv.R = mul(Rot, sv.R);
		sv.V = mul(Rot, sv.V);

		//J2000 to TEG
		Rot = mul(OrbMech::tmat(sescnst.M_TEG_TO_M50), OrbMech::M_J2000_to_M50);

		sv.R = mul(Rot, sv.R);
		sv.V = mul(Rot, sv.V);
		sv.GMT = (mjd_cur - sescnst.GMTBASE) * 24.0 * 3600.0; //MJD to GMT


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