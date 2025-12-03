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

#include "VectorMath.h"
#include "OrbitalManeuverProcessor.h"
#include "LaunchWindowProcessor.h"
#include "SkylabLaunchWindowProcessor.h"

class Core
{
public:
	Core();
	void Test(std::string project, std::string script);

	int SetConstants(int world, int Year, int Month, int Day);

	int RunLWP(std::string inputs[], std::vector<std::vector<std::string>> &data);
	int SaveLWPStateVector(std::string filename);
	int LWPExportForLVDC();
	int LWPExportForSSV();

	bool RunOMP(std::string inputs[], std::string &errormessage, std::vector<std::vector<std::string>>& data);

	int RunSkylabLWP(std::string inputs[], std::vector<std::vector<std::string>>& data);

	int AgeOfStateVector(std::string inputs[], double& age) const;
	int CalculateDayOfYear(int Year, int Month, int Day, int& DOY) const;

protected:

	//General
	int SetGlobalConstants(int world);
	int SetSessionConstants(int Year, int Month, int Day);
	void SetLaunchTime(int H, int M, double S);

	//OMP
	int ReadMCTFile(std::string file);
	int ReadMCTLine(std::string line);

	//File parsing
	void SaveStateVector(std::string file, const OrbMech::StateVector& sv);
	int LoadStateVector(std::string file, OrbMech::StateVector& sv) const;

	//OMP
	std::vector<OMP::ManeuverConstraintsInput> ManeuverConstraintsInput;
	std::string OMPChaserFile, OMPTargetFile, OMPMCTFile;
	OrbMech::StateVector sv_C, sv_T;
	OMP::OMPOutputs OMPOutput;

	//LWP
	OrbMech::StateVector PVTABC, PVTABT;
	LWP::LWPSummaryTable LWPSummaryTable;

	OrbMech::GlobalConstants globcnst;
	OrbMech::SessionConstants sescnst;
	std::string ProjectFolder;
	bool Initialized;
};