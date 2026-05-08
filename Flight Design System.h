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

#include <wx/wx.h>
#include <wx/bookctrl.h>
#include <wx/listctrl.h>
#include <wx/grid.h>
#include <vector>

class Core;
class wxTextFile;

class MyApp : public wxApp
{
public:
    bool OnInit() override;
};

wxIMPLEMENT_APP(MyApp);

class FDSFrame : public wxFrame
{
public:
    FDSFrame(const wxString& title);
    ~FDSFrame();

    void OnButtonLWPGenerate(wxCommandEvent& event);
    void OnButtonLWPSaveStateVector(wxCommandEvent& event);
    void OnButtonOMPGenerate(wxCommandEvent& event);
    void OnButtonSkylabLWPGenerate(wxCommandEvent& event);
    void OnButtonLWP_SIB_Presets(wxCommandEvent& event);
    void OnButtonLWP_Shuttle_Presets(wxCommandEvent& event);
    void OnButton_Save(wxCommandEvent& event);
    void OnButton_Load(wxCommandEvent& event);
    void OnPageChanged(wxBookCtrlEvent& event);
    void StateVectorSelected(wxListEvent& event);
    void OnButton_StateVector_Check(wxCommandEvent& event);
    void OnButton_StateVector_Save(wxCommandEvent& event);
    void OnButton_StateVector_New(wxCommandEvent& event);
    void CalculateDayOfYear(wxCommandEvent& event);
    void CalculateEphemerisDT(wxCommandEvent& event);
    void OnButton_LWP_LVDC_Export(wxCommandEvent& event);
    void OnButton_View_MCT(wxCommandEvent& event);
    void OnButton_Save_MCT(wxCommandEvent& event);
    void OnButton_MCT_Load_Template(wxCommandEvent& event);
    void OnButton_MCT_Delete_Maneuver(wxCommandEvent& event);
    void OnButton_MCT_Insert_Maneuver(wxCommandEvent& event);
    void OnMCTEditorCellChange(wxGridEvent& event);
    void OnButton_FDOMFD_Export(wxCommandEvent& event);
    void OnCombo_ShuttleLWP_Launchpad(wxCommandEvent& event);
    void OnButton_ShuttleLWP_LWP_Execute(wxCommandEvent& event);
    void OnButton_ShuttleLWP_LTP_Execute(wxCommandEvent& event);
    void OnButton_ShuttleLWP_Export(wxCommandEvent& event);
    void OnButtonShuttleLWPSaveStateVector(wxCommandEvent& event);
    void ChangeLoadSVPageLabels(wxCommandEvent& event);
    void OnButtonLoadSVConvert(wxCommandEvent& event);
    void OnButton_LoadSV_Save(wxCommandEvent& event);
private:

    struct wxTextCtrlData
    {
        wxTextCtrl* text;
        wxString name;
    };

    struct wxChoiceData
    {
        wxChoice* choice;
        wxString name;
    };

    struct wxCheckBoxData
    {
        wxCheckBox* checkBox;
        wxString name;
    };

    void AddConfigPage();
    void AddLWPPage();
    void AddShuttleLWPPage();
    void AddOMPPage();
    void AddSkylabLWPPage();
    void AddStateVectorPage();
    void AddLoadSVPage();

    void ReloadStateVectorPage();
    int ParseStateVectorFile(wxTextFile* file);
    void UpdateOrbitData();
    int SetConstants();
    void ShuttleLWP_Execute(bool IsLW);
    void ViewMCT(const wxString &filepath);

    // Text parsing
    int GetInteger(wxTextCtrl* text, const wxString& name, int* val);
    int GetDouble(wxTextCtrl* text, const wxString& name, double* val);
    int GetDDDHHMMSS(wxTextCtrl* text, const wxString& name, double* val);
    int GetHHMMSS(wxTextCtrl* text, const wxString& name, double* val);
    int ParseTime(wxTextCtrl* text, size_t size, double& sgn, int* vals, double* secs);

    // For save/loading
    void Add(wxTextCtrl* text, const wxString& config);
    void Add(wxChoice* choice, const wxString& config);
    void Add(wxCheckBox* choice, const wxString& config);

    wxNotebook* notebook;
    wxPanel* panel1;
    wxPanel* panel2;
    wxPanel* panel3;
    wxPanel* panel4;
    wxPanel* panel5;
    wxPanel* panel_LoadSV;
    wxNotebook* nb_ShuttleLWP;

    //Config page
    wxTextCtrl* textProjectFile;
    wxChoice* comboWorld;
    wxTextCtrl* textYear;
    wxTextCtrl* textMonth;
    wxTextCtrl* textDay;
    wxTextCtrl* textDETUTC;
    wxTextCtrl* textDayOfYear;

    //LWP page
    wxTextCtrl* textLWPTargetVector;
    wxTextCtrl* textLWPCKFactor;
    wxTextCtrl* textLWPCArea;
    wxTextCtrl* textLWPCWHT;
    wxChoice* comboLWPLW;
    wxChoice* comboLWPNS;
    wxTextCtrl* textLWPDay;
    wxChoice* comboLWP_LPT;
    wxTextCtrl* textLWP_TSTART;
    wxTextCtrl* textLWP_TEND;
    wxTextCtrl* textLWP_TSTEP;
    wxChoice* comboLWP_STABLE;
    wxTextCtrl* textLWP_STARS;
    wxTextCtrl* textLWP_STARE;
    wxTextCtrl* textLWP_LATLS;
    wxTextCtrl* textLWP_LONGLS;
    wxTextCtrl* textLWP_PFT;
    wxTextCtrl* textLWP_PFA;
    wxTextCtrl* textLWP_YSMAX;
    wxTextCtrl* textLWP_DTOPT;
    wxTextCtrl* textLWP_DTGRR;
    wxTextCtrl* textLWP_RINS;
    wxTextCtrl* textLWP_VINS;
    wxTextCtrl* textLWP_GAMINS;
    wxChoice* comboLWP_LOT;
    wxTextCtrl* textLWP_GMTLOR;
    wxTextCtrl* textLWP_OFFSET;
    wxTextCtrl* textLWP_BIAS;
    wxTextCtrl* textLWP_TPLANE;
    wxTextCtrl* textLWP_TRANS;
    wxChoice* comboLWP_INSCO;
    wxTextCtrl* textLWP_DHW;
    wxTextCtrl* textLWP_DU;
    wxTextCtrl* textLWP_ANOM;
    wxChoice* comboLWP_DELNOF;
    wxTextCtrl* textLWP_DELNO;
    wxChoice* comboLWP_NEGTIV;
    wxTextCtrl* textLWP_WRAP;
    wxTextCtrl* textLWP_Actual_GMTLO;

    //Shuttle LWP init page
    wxTextCtrl* textShuttleLWPTargetVector;
    wxTextCtrl* textShuttleLWP_YSMAX;
    wxCheckBox* checkShuttleLWP_DI;
    wxTextCtrl* textShuttleLWP_CD;
    wxTextCtrl* textShuttleLWP_Area;
    wxTextCtrl* textShuttleLWP_Weight;
    wxChoice* comboShuttleLWP_Launchpad;
    wxTextCtrl* textShuttleLWP_LATLS;
    wxTextCtrl* textShuttleLWP_LONGLS;
    wxTextCtrl* textShuttleLWP_ET_Area;
    wxTextCtrl* textShuttleLWP_ET_CD;
    wxTextCtrl* textShuttleLWP_ET_WT;
    wxTextCtrl* textShuttleLWP_MPS_Dump_DVX;
    wxTextCtrl* textShuttleLWP_MPS_Dump_DVY;
    wxTextCtrl* textShuttleLWP_MPS_Dump_DVZ;
    wxTextCtrl* textShuttleLWP_MPS_Dump_DTIG;
    wxTextCtrl* text_ShuttleLWP_PFA;
    wxTextCtrl* text_ShuttleLWP_PFT;
    wxTextCtrl* text_ShuttleLWP_RAD;
    wxTextCtrl* text_ShuttleLWP_VEL;
    wxTextCtrl* text_ShuttleLWP_FPA;
    wxTextCtrl* text_ShuttleLWP_OPT;
    wxTextCtrl* text_ShuttleLWP_DTO;
    wxTextCtrl* text_ShuttleLWP_DTC;
    wxTextCtrl* textShuttleLWP_ET_Sep_DVX;
    wxTextCtrl* textShuttleLWP_ET_Sep_DVY;
    wxTextCtrl* textShuttleLWP_ET_Sep_DVZ;
    wxTextCtrl* textShuttleLWP_ET_Sep_DTIG;
    wxChoice* comboShuttleLWP_PhaseCntrlFlag;
    wxTextCtrl* textShuttleLWP_WRAP_FLAG;
    wxCheckBox* checkShuttleLWP_LAUNCH_AZ_DIR_FLAG;

    // Shuttle LWP constants page
    wxCheckBox* checkShuttleLWP_Drag;

    //Shuttle LWP lnch ref sets page
    wxTextCtrl* textShuttleLWP_OMS1_DTIG[4];
    wxTextCtrl* textShuttleLWP_OMS1_C1[4];
    wxTextCtrl* textShuttleLWP_OMS1_C2[4];
    wxTextCtrl* textShuttleLWP_OMS1_HT[4];
    wxTextCtrl* textShuttleLWP_OMS1_THETAT[4];
    wxTextCtrl* textShuttleLWP_OMS2_DTIG[4];
    wxTextCtrl* textShuttleLWP_OMS2_C1[4];
    wxTextCtrl* textShuttleLWP_OMS2_C2[4];
    wxTextCtrl* textShuttleLWP_OMS2_HT[4];
    wxTextCtrl* textShuttleLWP_OMS2_THETAT[4];

    // Shuttle LW execution page
    wxTextCtrl* textShuttleLWP_LW_Ref_Set_ID;
    wxTextCtrl* textShuttleLWP_Desired_Phase_1;
    wxTextCtrl* textShuttleLWP_Desired_Phase_2;
    wxTextCtrl* textShuttleLWP_LW_Out[10];

    // Shuttle LT execution page
    wxTextCtrl* textShuttleLWP_LT_GMTLO;
    wxTextCtrl* textShuttleLWP_LT_Ref_Set_ID;
    wxTextCtrl* textShuttleLWP_LT_Out[30];

    //OMP page
    wxTextCtrl* textOMP_GMTLO;
    wxTextCtrl* textOMP_Chaser;
    wxTextCtrl* textOMP_Target;
    wxTextCtrl* textOMP_MCT;
    wxGrid* textOMP_MCT_Editor;
    bool bMCTWasChanged;

    //Skylab LWP page
    wxTextCtrl* textSkylabLWP_TargetVector;
    wxTextCtrl* textSkylabLWP_CKFactor;
    wxTextCtrl* textSkylabLWP_CArea;
    wxTextCtrl* textSkylabLWP_CWHT;
    wxChoice* comboSkylabLWP_NS;
    wxTextCtrl* textSkylabLWP_DAY;
    wxTextCtrl* textSkylabLWP_LATLS;
    wxTextCtrl* textSkylabLWP_LONGLS;
    wxTextCtrl* textSkylabLWP_PFT;
    wxTextCtrl* textSkylabLWP_PFA;
    wxTextCtrl* textSkylabLWP_YSMAX;
    wxTextCtrl* textSkylabLWP_DTOPT;
    wxTextCtrl* textSkylabLWP_RINS;
    wxTextCtrl* textSkylabLWP_VINS;
    wxTextCtrl* textSkylabLWP_GAMINS;
    wxChoice* comboSkylabLWP_NEGTIV;
    wxTextCtrl* textSkylabLWP_WRAP;
    wxTextCtrl* textSkylabLWP_DTOPEN;
    wxTextCtrl* textSkylabLWP_DTCLOSE;
    wxChoice* comboSkylabLWP_IR;
    wxTextCtrl* textSkylabLWP_TLO;
    wxTextCtrl* textSkylabLWP_MI;
    wxTextCtrl* textSkylabLWP_MF;
    wxTextCtrl* textSkylabLWP_DTSEP;
    wxTextCtrl* textSkylabLWP_DVSEP;
    wxTextCtrl* textSkylabLWP_DVWO;
    wxTextCtrl* textSkylabLWP_DVWC;

   //State Vector page
    wxListCtrl* StateVectorList;
    wxTextCtrl* textStateVectorFileName;
    wxChoice* comboStateVectorCoordinateSystem;
    wxTextCtrl* textStateVectorData;
    wxTextCtrl* textStateVectorWeight;
    wxTextCtrl* textStateVectorArea;
    wxTextCtrl* textStateVectorKFactor;
    wxTextCtrl* textStateVectorOrbitData;

    // Load SV page
    wxChoice* comboLoadSV_REFAX;
    wxChoice* comboLoadSV_ELSET;
    wxChoice* comboLoadSV_ANGUN;
    wxChoice* comboLoadSV_DSTUN;
    wxChoice* comboLoadSV_VELUN;
    wxChoice* comboLoadSV_MASUN;
    wxChoice* comboLoadSV_LENUN;
    wxStaticText* staticLoadSV_Descriptions[15];
    wxTextCtrl* textLoadSV_Array[15];
    wxStaticText* staticLoadSV_Units[15];
    wxTextCtrl* textLoadSV_Output;

    Core* core;

    // Save data
    std::vector<wxTextCtrlData> textCtrlData;
    std::vector<wxChoiceData> choiceData;
    std::vector<wxCheckBoxData> checkBoxData;

    DECLARE_EVENT_TABLE()
};