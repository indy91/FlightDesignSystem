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

class Core;

class MyApp : public wxApp
{
public:
    bool OnInit() override;
};

wxIMPLEMENT_APP(MyApp);

class MyFrame : public wxFrame
{
public:
    MyFrame(const wxString& title);
    ~MyFrame();

    void OnButton1(wxCommandEvent& event);
    void OnButton2(wxCommandEvent& event);
    void OnButton3(wxCommandEvent& event);
    void OnButton4(wxCommandEvent& event);
    void OnButtonLWP_SIB_Presets(wxCommandEvent& event);
    void OnButtonLWP_Shuttle_Presets(wxCommandEvent& event);
    void OnButton_Save(wxCommandEvent& event);
    void OnButton_Load(wxCommandEvent& event);
    void OnPageChanged(wxBookCtrlEvent& event);
    void StateVectorSelected(wxListEvent& event);
    void OnButton_StateVector_Save(wxCommandEvent& event);
    void OnButton_StateVector_New(wxCommandEvent& event);
    void StateVectorRawDataEnter(wxCommandEvent& event);
    void CalculateDayOfYear(wxCommandEvent& event);
    void OnButton_LWP_LVDC_Export(wxCommandEvent& event);
    void OnButton_View_MCT(wxCommandEvent& event);
    void OnButton_Save_MCT(wxCommandEvent& event);
private:

    void AddConfigPage();
    void AddLWPPage();
    void AddOMPPage();
    void AddSkylabLWPPage();
    void AddStateVectorPage();

    void ReloadStateVectorPage();
    void UpdateOrbitData();
    int SetConstants();

    wxPanel* panel1;
    wxPanel* panel2;
    wxPanel* panel3;
    wxPanel* panel4;
    wxPanel* panel5;

    //Config page
    wxTextCtrl* textProjectFile;
    wxComboBox* comboWorld;
    wxTextCtrl* textYear;
    wxTextCtrl* textMonth;
    wxTextCtrl* textDay;
    wxTextCtrl* textDayOfYear;

    //LWP page
    wxTextCtrl* textLWPTargetVector;
    wxTextCtrl* textLWPCKFactor;
    wxTextCtrl* textLWPCArea;
    wxTextCtrl* textLWPCWHT;
    wxComboBox* comboLWPLW;
    wxComboBox* comboLWPNS;
    wxTextCtrl* textLWPDay;
    wxComboBox* comboLWP_LPT;
    wxTextCtrl* textLWP_TSTART;
    wxTextCtrl* textLWP_TEND;
    wxTextCtrl* textLWP_TSTEP;
    wxComboBox* comboLWP_STABLE;
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
    wxComboBox* comboLWP_LOT;
    wxTextCtrl* textLWP_GMTLOR;
    wxTextCtrl* textLWP_OFFSET;
    wxTextCtrl* textLWP_BIAS;
    wxTextCtrl* textLWP_TPLANE;
    wxTextCtrl* textLWP_TRANS;
    wxComboBox* comboLWP_INSCO;
    wxTextCtrl* textLWP_DHW;
    wxTextCtrl* textLWP_DU;
    wxTextCtrl* textLWP_ANOM;
    wxComboBox* comboLWP_DELNOF;
    wxTextCtrl* textLWP_DELNO;
    wxComboBox* comboLWP_NEGTIV;
    wxTextCtrl* textLWP_WRAP;

    //OMP page
    wxTextCtrl* textOMP_GMTLO;
    wxTextCtrl* textOMP_Chaser;
    wxTextCtrl* textOMP_Target;
    wxTextCtrl* textOMP_MCT;
    wxGrid* textOMP_MCT_Editor;

    //Skylab LWP page
    wxTextCtrl* textSkylabLWP_TargetVector;
    wxTextCtrl* textSkylabLWP_CKFactor;
    wxTextCtrl* textSkylabLWP_CArea;
    wxTextCtrl* textSkylabLWP_CWHT;
    wxComboBox* comboSkylabLWP_NS;
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
    wxComboBox* comboSkylabLWP_NEGTIV;
    wxTextCtrl* textSkylabLWP_WRAP;
    wxTextCtrl* textSkylabLWP_DTOPEN;
    wxTextCtrl* textSkylabLWP_DTCLOSE;
    wxComboBox* comboSkylabLWP_IR;
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
    wxTextCtrl* textStateVectorRawData;
    wxTextCtrl* textStateVectorOrbitData;

    Core* core;

    DECLARE_EVENT_TABLE()
};