#include "SkylabLaunchWindowProcessor.h"
#include "LaunchWindowProcessor.h"
#include "Constants.h"
#include <iostream>

namespace SkylabLWP
{
	SkylabLWPInputs::SkylabLWPInputs()
	{
		CKFactor = 1.0;
		CAREA = 129.4 * pow(OrbMech::FT2M, 2);
		CWHT = 15000.0;
		NS = 0;
		DAY = 0;
		LATLS = 28.627 * OrbMech::RAD;
		LONGLS = 279.379 * OrbMech::RAD;
		PFT = 10.0 * 60.0;
		PFA = 18.8 * OrbMech::RAD;
		YSMAX = 5.0 * OrbMech::RAD;
		RINS = 6528178.0;
		VINS = 7835.13;
		GAMINS = 0.0;
		DTOP = 6.0 * 60.0;
		DTOPEN = 7.75 * 60.0;
		DTCLOSE = 7.75 * 60.0;

		DVSEP = _V(3.0, 0, 0);
		DTSEP = 25.0 * 60.0 - PFT;
		MI = MF = 0;
		DVWO = 30.0 * OrbMech::FT2M;
		DVWC = 30.0 * OrbMech::FT2M;
		IR = false;
		NEGTIV = 0;
		WRAP = 0;
		TLO = 0.0;
	}

	SkylabLaunchWindowProcessor::SkylabLaunchWindowProcessor(OrbMech::GlobalConstants& cnst, OrbMech::SessionConstants& scnst) : constants(cnst), sesconst(scnst)
	{
		TAIP = GMTLO = 0.0;
	}

	int SkylabLaunchWindowProcessor::Calculate(const SkylabLWPInputs& in, SkylabLWPOutputs& out)
	{
		double DVNCC[3][2], DVNC1[3][2], TLO[3][2], hh, mm, ss;
		int M, I, K;

		inp = in;

		out.OutputPrint.clear();

		BuildManeuverConstraintsTable();

		OMP::OrbitalManeuverProcessor omp(constants, sesconst);
		OMP::OMPInputs omp_in;
		OMP::OMPOutputs omp_out;

		omp_in.TARGET = inp.sv_T;
		omp_in.ProjectFolder = in.ProjectFolder;
		omp_in.OMPChaserFile = "CSM";
		omp_in.OMPTargetFile = "Skylab";
		omp_in.OMPMCTFile = "SLWP";

		M = inp.MI;

		if (inp.IR)
		{
			//Input time
			I = 2;
			GMTLO = inp.TLO;
		}
		else
		{
			//Launch window
			LWT();
			I = 0;
		}
		K = 0;

		while (1)
		{
			//Launch targeting
			RLOT(I == 0);

			while (1)
			{
				//Settings for the OMP
				omp_in.CHASER = sv_C;
				omp_in.ManeuverConstraintsTable = ManeuverConstraintsTable;
				omp_in.PRINT = (in.IR || (I == 0 && K == 0));

				sesconst.GMTLO = GMTLO;

				hh = floor(GMTLO / 3600.0);
				mm = floor(fmod(GMTLO, 3600.0) / 60.0);
				ss = fmod(GMTLO, 60.0);

				sesconst.Hours = (int)hh;
				sesconst.Minutes = (int)mm;
				sesconst.launchdateSec = ss;

				omp.Calculate(omp_in, omp_out);

				if (omp_out.Error) return 1;

				//Save outputs
				DVNCC[I][K] = omp_out.ManeuverEvaluationTable.Maneuvers[3].DV.x * OrbMech::FT2M;
				DVNC1[I][K] = omp_out.ManeuverEvaluationTable.Maneuvers[1].DV.x * OrbMech::FT2M;
				TLO[I][K] = GMTLO;

				if (omp_out.OutputPrint.size() > 0)
				{
					out.OutputPrint.push_back(omp_out.OutputPrint[0]);
				}

				if (M < inp.MF)
				{
					M = inp.MF;
					K = 1;
					SetMLine(M);
				}
				else break;
			}
			if (I < 1)
			{
				GMTLO += 60.0;
				M = inp.MI;
				SetMLine(M);
				I = 1;
				K = 0;
			}
			else if (I == 2)
			{
				//Display for input time
				return 0;
			}
			else break;
		}

		//Launch window calculations
		double SMIO, MIOPEN, SMIC, MICLOS, DMTO, DMTC, MFOPEN, MFCLOS;

		SMIO = (DVNCC[1][0] - DVNCC[0][0]) / (TLO[1][0] - TLO[0][0]);
		MIOPEN = TLO[1][0] + (inp.DVWO - DVNCC[1][0]) / SMIO;
		SMIC = (DVNC1[1][0] - DVNC1[0][0]) / (TLO[1][0] - TLO[0][0]);
		MICLOS = TLO[1][0] + (inp.DVWC - DVNC1[1][0]) / SMIC;

		DMTO = DMTC = MFOPEN = MFCLOS = 0.0;

		if (inp.MI < inp.MF)
		{
			double SMFO, SMFC;

			SMFO = (DVNCC[1][1] - DVNCC[0][1]) / (TLO[1][1] - TLO[0][1]);
			MFOPEN = TLO[1][1] + (inp.DVWO - DVNCC[1][1]) / SMFO;
			SMFC = (DVNC1[1][1] - DVNC1[0][1]) / (TLO[1][1] - TLO[0][1]);
			MFCLOS = TLO[1][1] + (inp.DVWC - DVNC1[1][1]) / SMFC;

			if (inp.MI + 1 < inp.MF)
			{
				//More than 1 rev difference
				double DM;

				DM = (double)(inp.MF - inp.MI);
				DMTO = (MFOPEN - MIOPEN) / DM;
				DMTC = (MFCLOS - MICLOS) / DM;
			}
		}

		std::vector<double> MO, MC;
		int siz;

		siz = 1 + inp.MF - inp.MI;
		if (siz < 2) siz = 2;

		MO.resize(siz);
		MC.resize(siz);

		MO[0] = MIOPEN;
		MC[0] = MICLOS;

		if (inp.MI < inp.MF)
		{
			int J;

			J = inp.MF - inp.MI;
			MO[J] = MFOPEN;
			MC[J] = MFCLOS;

			if (inp.MI + 1 < inp.MF)
			{
				int N, L;

				N = 1;
				L = 0;

				while (1)
				{
					MO[N] = MO[L] + DMTO;
					MC[N] = MC[L] + DMTC;

					if (N < J)
					{
						L++;
						N++;
					}
					else break;
				}
			}
		}

		double TOP, TOPEN, TCLOSE;
		char Buffer[128];

		TOP = TAIP;
		TOPEN = TOP - inp.DTOPEN;
		TCLOSE = TOP + inp.DTCLOSE;

		//Output
		std::vector<std::string> outarray;
		std::string temp;

		outarray.push_back("SKYLAB LAUNCH WINDOW PROCESSOR");
		outarray.push_back("");
		outarray.push_back("PLANAR WINDOW:");

		OrbMech::GMT2String(Buffer, TOPEN, inp.DAY);
		temp = "Open:    ";
		temp.append(Buffer);
		outarray.push_back(temp);

		OrbMech::GMT2String(Buffer, TOP, inp.DAY);
		temp = "Inplane: ";
		temp.append(Buffer);
		outarray.push_back(temp);

		OrbMech::GMT2String(Buffer, TCLOSE, inp.DAY);
		temp = "Close:   ";
		temp.append(Buffer);
		outarray.push_back(temp);
		outarray.push_back("PHASE WINDOW:");

		for (unsigned i = 0; i < MO.size(); i++)
		{
			temp = "M=" + std::to_string(inp.MI + i);
			outarray.push_back(temp);

			OrbMech::GMT2String(Buffer, MO[i], inp.DAY);
			temp = "Open: ";
			temp.append(Buffer);
			outarray.push_back(temp);

			OrbMech::GMT2String(Buffer, MC[i], inp.DAY);
			temp = "Close: ";
			temp.append(Buffer);
			outarray.push_back(temp);
		}

		out.OutputPrint.push_back(outarray);

		return 0;
	}

	void SkylabLaunchWindowProcessor::LWT()
	{
		//TBD
	}

	int SkylabLaunchWindowProcessor::RLOT(bool init)
	{
		//init: true = find optimum launch time, false = launch time input
		LWP::LWPInputs lwp_in;
		LWP::LWPOutputs lwp_out;

		lwp_in.TRGVEC = inp.sv_T;
		lwp_in.CKFactor = inp.CKFactor;
		lwp_in.CAREA = inp.CAREA;
		lwp_in.CWHT = inp.CWHT;
		lwp_in.NS = inp.NS;
		lwp_in.DAY = inp.DAY;
		lwp_in.LPT = 0;
		lwp_in.STABLE = false;
		lwp_in.LATLS = inp.LATLS;
		lwp_in.LONGLS = inp.LONGLS;
		lwp_in.PFA = inp.PFA;
		lwp_in.PFT = inp.PFT;
		lwp_in.YSMAX = inp.YSMAX;
		lwp_in.DTOPT = inp.DTOP;
		lwp_in.RINS = inp.RINS;
		lwp_in.VINS = inp.VINS;
		lwp_in.GAMINS = inp.GAMINS;
		lwp_in.INSCO = 1;
		lwp_in.TRANS = 0.0;
		lwp_in.NEGTIV = inp.NEGTIV;
		lwp_in.WRAP = inp.WRAP;
		lwp_in.DELNOF = true;
		lwp_in.DELNO = 0.0;
		lwp_in.PRINT = false;

		if (init)
		{
			lwp_in.LW = 2;
			lwp_in.LOT = 6;
			lwp_in.TPLANE = GMTLO;
		}
		else
		{
			lwp_in.LW = 1;
			lwp_in.LOT = 1;
			lwp_in.GMTLOR = GMTLO;
		}

		LWP::LaunchWindowProcessor lwp(constants, sesconst);

		lwp.Calculate(lwp_in, lwp_out);

		if (lwp_out.SUMTAB.Error) return 1;

		GMTLO = lwp_out.GMTLO;
		sv_C = lwp_out.PVTABC;
		if (init) TAIP = GMTLO;

		return 0;
	}

	void SkylabLaunchWindowProcessor::BuildManeuverConstraintsTable()
	{
		ManeuverConstraintsTable.clear();

		OMP::ManeuverConstraints temp;
		OMP::SecData sectemp;

		//SEP
		temp.name = "SEP";
		temp.threshold = OMP::OMPDefs::THRESHOLD::THRES_T;
		temp.thresh_num = inp.PFT + inp.DTSEP;
		temp.type = OMP::OMPDefs::MANTYPE::EXDV;
		
		sectemp.type = OMP::OMPDefs::SECONDARIES::DVLV;
		sectemp.value = inp.DVSEP.x;
		temp.secondaries.push_back(sectemp);
		sectemp.value = inp.DVSEP.y;
		temp.secondaries.push_back(sectemp);
		sectemp.value = inp.DVSEP.z;
		temp.secondaries.push_back(sectemp);

		ManeuverConstraintsTable.push_back(temp);

		//NC1
		temp = OMP::ManeuverConstraints();
		temp.name = "NC1";
		temp.threshold = OMP::OMPDefs::THRESHOLD::THRES_T;
		temp.thresh_num = inp.PFT + inp.DTSEP + 60.0;
		temp.type = OMP::OMPDefs::MANTYPE::NC;

		sectemp.type = OMP::OMPDefs::SECONDARIES::APO;
		sectemp.value = 2.0;
		temp.secondaries.push_back(sectemp);

		ManeuverConstraintsTable.push_back(temp);

		//NH
		temp = OMP::ManeuverConstraints();
		temp.name = "NC2";
		temp.threshold = OMP::OMPDefs::THRESHOLD::THRES_M;
		temp.thresh_num = (double)inp.MI - 3.5;
		temp.type = OMP::OMPDefs::MANTYPE::NH;

		ManeuverConstraintsTable.push_back(temp);

		//NCC
		temp = OMP::ManeuverConstraints();
		temp.name = "NCC";
		temp.threshold = OMP::OMPDefs::THRESHOLD::THRES_M;
		temp.thresh_num = 0.5;
		temp.type = OMP::OMPDefs::MANTYPE::NH;

		sectemp.type = OMP::OMPDefs::SECONDARIES::DH;
		sectemp.value = 20.0;
		temp.secondaries.push_back(sectemp);

		ManeuverConstraintsTable.push_back(temp);

		//NSR
		temp = OMP::ManeuverConstraints();
		temp.name = "NSR";
		temp.threshold = OMP::OMPDefs::THRESHOLD::THRES_DT;
		temp.thresh_num = 37.0 * 60.0;
		temp.type = OMP::OMPDefs::MANTYPE::NSR;

		sectemp.type = OMP::OMPDefs::SECONDARIES::DH;
		sectemp.value = 10.0;
		temp.secondaries.push_back(sectemp);

		ManeuverConstraintsTable.push_back(temp);

		//TPI
		temp = OMP::ManeuverConstraints();
		temp.name = "TPI";
		temp.threshold = OMP::OMPDefs::THRESHOLD::THRES_DT;
		temp.thresh_num = 40.0 * 60.0;
		//temp.threshold = OMP::OMPDefs::THRESHOLD::THRES_T;
		//temp.thresh_num = 7.0 * 3600.0;
		temp.type = OMP::OMPDefs::MANTYPE::TPI;

		sectemp.type = OMP::OMPDefs::SECONDARIES::DR;
		sectemp.value = -20.0;
		temp.secondaries.push_back(sectemp);

		sectemp.type = OMP::OMPDefs::SECONDARIES::NITO;
		sectemp.value = -23.0;
		temp.secondaries.push_back(sectemp);

		ManeuverConstraintsTable.push_back(temp);

		//TPF
		temp = OMP::ManeuverConstraints();
		temp.name = "TPF";
		temp.threshold = OMP::OMPDefs::THRESHOLD::THRES_WT;
		temp.thresh_num = 130.0*OrbMech::RAD;
		temp.type = OMP::OMPDefs::MANTYPE::TPF;

		ManeuverConstraintsTable.push_back(temp);
	}

	void SkylabLaunchWindowProcessor::SetMLine(int M)
	{
		ManeuverConstraintsTable[2].thresh_num = (double)M - 3.5;
	}
}