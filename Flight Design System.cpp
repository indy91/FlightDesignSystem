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

// Flight Design System.cpp : Defines the entry point for the application.
//

#include "Flight Design System.h"
#include "Core.h"
#include <iostream>
#include <wx/notebook.h>
#include "FDSOutputDialog.h"
#include <wx/fileconf.h>
#include "wx/dir.h"
#include <wx/tokenzr.h>
#include "wx/statline.h"

enum
{
    ID_Hello = 1,
    ID_Button_LWPGenerate,
    ID_Button_LWPSaveStateVector,
    ID_Button_OMPGenerate,
    ID_Button_SkylabLWPGenerate,
    ID_TextBox_1,
    ID_TextBox_Year,
    ID_TextBox_Month,
    ID_TextBox_Day,
    ID_TextBox_EDT,
    ID_Button_EDT,
    ID_Button_LWP_SIB_Presets,
    ID_Button_LWP_Shuttle_Presets,
    ID_Button_Save,
    ID_Button_Load,
    MainContentID,
    wxID_StateVectorList,
    ID_Button_SV_Check,
    ID_Button_SV_Save,
    ID_Button_SV_New,
    ID_Button_LWP_LVDC_Export,
    ID_Button_MCT_View,
    ID_Button_MCT_Save,
    ID_Button_MCT_Load_Template,
    ID_Button_FDOMFD_Export,
    wxID_comboShuttleLWP_Launchpad,
    ID_Button_Shuttle_LWP_Execute,
    ID_Button_Shuttle_LTP_Execute,
    ID_World,
    ID_Button_ShuttleLWP_Export,
    ID_Button_ShuttleLWP_SaveStateVector,
    ID_LoadSVPageLabels,
    ID_LoadSV_Convert,
    ID_LoadSV_Save
};

BEGIN_EVENT_TABLE(FDSFrame, wxFrame)
EVT_BUTTON(ID_Button_LWPGenerate, FDSFrame::OnButtonLWPGenerate)
EVT_BUTTON(ID_Button_LWPSaveStateVector, FDSFrame::OnButtonLWPSaveStateVector)
EVT_BUTTON(ID_Button_OMPGenerate, FDSFrame::OnButtonOMPGenerate)
EVT_BUTTON(ID_Button_SkylabLWPGenerate, FDSFrame::OnButtonSkylabLWPGenerate)
EVT_BUTTON(ID_Button_LWP_SIB_Presets, FDSFrame::OnButtonLWP_SIB_Presets)
EVT_BUTTON(ID_Button_LWP_Shuttle_Presets, FDSFrame::OnButtonLWP_Shuttle_Presets)
EVT_BUTTON(ID_Button_Save, FDSFrame::OnButton_Save)
EVT_BUTTON(ID_Button_Load, FDSFrame::OnButton_Load)
EVT_NOTEBOOK_PAGE_CHANGED(MainContentID, FDSFrame::OnPageChanged)
EVT_LIST_ITEM_SELECTED(wxID_StateVectorList, FDSFrame::StateVectorSelected)
EVT_BUTTON(ID_Button_SV_Check, FDSFrame::OnButton_StateVector_Check)
EVT_BUTTON(ID_Button_SV_Save, FDSFrame::OnButton_StateVector_Save)
EVT_BUTTON(ID_Button_SV_New, FDSFrame::OnButton_StateVector_New)
EVT_TEXT(ID_TextBox_Year, FDSFrame::CalculateDayOfYear)
EVT_TEXT(ID_TextBox_Month, FDSFrame::CalculateDayOfYear)
EVT_TEXT(ID_TextBox_Day, FDSFrame::CalculateDayOfYear)
EVT_TEXT(ID_TextBox_EDT, FDSFrame::CalculateDayOfYear)
EVT_BUTTON(ID_Button_EDT, FDSFrame::CalculateEphemerisDT)
EVT_CHOICE(ID_World, FDSFrame::CalculateDayOfYear)
EVT_BUTTON(ID_Button_LWP_LVDC_Export, FDSFrame::OnButton_LWP_LVDC_Export)
EVT_BUTTON(ID_Button_MCT_View, FDSFrame::OnButton_View_MCT)
EVT_BUTTON(ID_Button_MCT_Save, FDSFrame::OnButton_Save_MCT)
EVT_BUTTON(ID_Button_MCT_Load_Template, FDSFrame::OnButton_MCT_Load_Template)
EVT_BUTTON(ID_Button_FDOMFD_Export, FDSFrame::OnButton_FDOMFD_Export)
EVT_CHOICE(wxID_comboShuttleLWP_Launchpad, FDSFrame::OnCombo_ShuttleLWP_Launchpad)
EVT_BUTTON(ID_Button_Shuttle_LWP_Execute, FDSFrame::OnButton_ShuttleLWP_LWP_Execute)
EVT_BUTTON(ID_Button_Shuttle_LTP_Execute, FDSFrame::OnButton_ShuttleLWP_LTP_Execute)
EVT_BUTTON(ID_Button_ShuttleLWP_Export, FDSFrame::OnButton_ShuttleLWP_Export)
EVT_BUTTON(ID_Button_ShuttleLWP_SaveStateVector, FDSFrame::OnButtonShuttleLWPSaveStateVector)
EVT_CHOICE(ID_LoadSVPageLabels, FDSFrame::ChangeLoadSVPageLabels)
EVT_BUTTON(ID_LoadSV_Convert, FDSFrame::OnButtonLoadSVConvert)
EVT_BUTTON(ID_LoadSV_Save, FDSFrame::OnButton_LoadSV_Save)
END_EVENT_TABLE()

bool MyApp::OnInit()
{
    FDSFrame* frame = new FDSFrame("Flight Design System");
    frame->Show(true);
    return true;
}

FDSFrame::FDSFrame(const wxString& title)
    : wxFrame(NULL, wxID_ANY, title)
{
    core = NULL;

    // Create a top-level panel to hold all the contents of the frame
    wxPanel* panel = new wxPanel(this, wxID_ANY);

    // Create the wxNotebook widget
    notebook = new wxNotebook(panel, MainContentID);

    // Add pages to the wxNotebook widget
    panel1 = new wxPanel(notebook, wxID_ANY, wxDefaultPosition, wxDefaultSize, 2621440L, L"Tab 1 Contents");
    panel2 = new wxPanel(notebook, wxID_ANY, wxDefaultPosition, wxDefaultSize, 2621440L, L"Tab 2 Contents");
    nb_ShuttleLWP = new wxNotebook(notebook, wxID_ANY);
    AddShuttleLWPPage();
    panel3 = new wxPanel(notebook, wxID_ANY, wxDefaultPosition, wxDefaultSize, 2621440L, L"Tab 3 Contents");
    panel4 = new wxPanel(notebook, wxID_ANY, wxDefaultPosition, wxDefaultSize, 2621440L, L"Skylab LWP Contents");
    panel5 = new wxPanel(notebook, wxID_ANY, wxDefaultPosition, wxDefaultSize, 2621440L, L"SV Contents");
    panel_LoadSV = new wxPanel(notebook, wxID_ANY, wxDefaultPosition, wxDefaultSize, 2621440L, L"Load SV Contents");

    notebook->AddPage(panel1, L"Config");
    notebook->AddPage(panel2, L"Generic LWP");
    notebook->AddPage(nb_ShuttleLWP, L"Shuttle LWP");
    notebook->AddPage(panel3, L"OMP");
    notebook->AddPage(panel4, L"Skylab LWP");
    notebook->AddPage(panel5, L"State Vectors");
    notebook->AddPage(panel_LoadSV, "Load SV");

    // Set up the sizer for the panel
    wxBoxSizer* panelSizer = new wxBoxSizer(wxHORIZONTAL);
    panelSizer->Add(notebook, 1, wxEXPAND);
    panel->SetSizer(panelSizer);

    // Set up the sizer for the frame and resize the frame
    // according to its contents
    wxBoxSizer* topSizer = new wxBoxSizer(wxHORIZONTAL);
    topSizer->SetMinSize(1024, 598);
    topSizer->Add(panel, 1, wxEXPAND);
    SetSizerAndFit(topSizer);

    // Create a status bar just for fun
    CreateStatusBar(1);
    SetStatusText(wxT("Welcome to the Flight Design System!"));

    AddConfigPage();
    AddLWPPage();
    AddOMPPage();   
    AddSkylabLWPPage();
    AddStateVectorPage();
    AddLoadSVPage();

    // Initialize with default inputs
    SetConstants();
}

FDSFrame::~FDSFrame()
{
    if (core)
    {
        delete core;
        core = NULL;
    }
}

void FDSFrame::AddConfigPage()
{
    wxArrayString strings;
    int minX, minY, diffX, diffY, counter, difftext, Y, diffunit;

    minX = 10;
    minY = 10;
    diffX = 70;
    diffY = 32;
    counter = 0;
    difftext = 4;
    diffunit = 200;

    Y = minY;

    new wxStaticText(panel1, wxID_ANY, "Project", wxPoint(minX, Y + difftext));
    textProjectFile = new wxTextCtrl(panel1, ID_TextBox_1, "Test", wxPoint(minX + diffX, Y));
    Y += diffY;

   new wxButton(panel1, ID_Button_Save, wxT("Save"),
        wxPoint(minX + diffX, Y), wxDefaultSize);
   Y += diffY;

   new wxButton(panel1, ID_Button_Load, wxT("Load"),
       wxPoint(minX + diffX, Y), wxDefaultSize);
   Y += diffY;
   Y += diffY;

    new wxStaticText(panel1, wxID_ANY, "World", wxPoint(minX, Y + difftext));
    strings.Add(wxT("NASSP"));
    strings.Add(wxT("SSV"));
    strings.Add(wxT("Real"));
    Add(comboWorld = new wxChoice(panel1, ID_World, wxPoint(minX + diffX, Y), wxDefaultSize, strings), "/Config/World");
    comboWorld->SetToolTip(wxT("Select world"));
    comboWorld->SetSelection(0);
    strings.clear();
    Y += diffY;

    new wxStaticText(panel1, wxID_ANY, "Year", wxPoint(minX, Y + difftext));
    Add(textYear = new wxTextCtrl(panel1, ID_TextBox_Year, "1973", wxPoint(minX + diffX, Y)), "/Config/Year");
    textYear->SetToolTip(wxT("Liftoff year"));
    Y += diffY;

    new wxStaticText(panel1, wxID_ANY, "Month", wxPoint(minX, Y + difftext));
    Add(textMonth = new wxTextCtrl(panel1, ID_TextBox_Month, "5", wxPoint(minX + diffX, Y)), "/Config/Month");
    textMonth->SetToolTip(wxT("Liftoff month"));
    Y += diffY;

    new wxStaticText(panel1, wxID_ANY, "Day", wxPoint(minX, Y + difftext));
    Add(textDay = new wxTextCtrl(panel1, ID_TextBox_Day, "15", wxPoint(minX + diffX, Y)), "/Config/Day");
    textDay->SetToolTip(wxT("Liftoff day"));
    Y += diffY;

    new wxStaticText(panel1, wxID_ANY, "EDT", wxPoint(minX, Y + difftext));
    Add(textDETUTC = new wxTextCtrl(panel1, ID_TextBox_EDT, "0.0", wxPoint(minX + diffX, Y)), "/Config/DETUTC");
    textDETUTC->SetToolTip(wxT("Ephemeris Time (ET/TT/TDB) minus Universal Time (UTC)"));
    new wxStaticText(panel1, wxID_ANY, "sec", wxPoint(minX + diffunit, Y + difftext));

    new wxButton(panel1, ID_Button_EDT, "Calculate EDT", wxPoint(minX + diffunit + 100, Y));

    Y += diffY;
    Y += diffY;

    new wxStaticText(panel1, wxID_ANY, "DOY", wxPoint(minX, Y + difftext));
    Add(textDayOfYear = new wxTextCtrl(panel1, wxID_ANY, wxEmptyString, wxPoint(minX + diffX, Y), wxDefaultSize, wxTE_READONLY), "/Config/DOY");
    textDayOfYear->SetToolTip(wxT("Day of year"));
    Y += diffY;
}

void FDSFrame::AddLWPPage()
{
    wxArrayString strings;
    int minX, minY, diffX, diffY, counter, difftext, Y, diffunit;

    minX = 10;
    minY = 10;
    diffX = 70;
    diffY = 32;
    counter = 0;
    difftext = 4;
    diffunit = 200;

    Y = minY;

    new wxStaticText(panel2, wxID_ANY, "TRGVEC", wxPoint(minX, Y + difftext));
    Add(textLWPTargetVector = new wxTextCtrl(panel2, wxID_ANY, "Skylab TLE2.txt", wxPoint(minX + diffX, Y)), "/LWP/TargetVector");
    textLWPTargetVector->SetToolTip(wxT("Target vehicle state vector"));
    Y += diffY;

    new wxStaticText(panel2, wxID_ANY, "CKFACTOR", wxPoint(minX, Y + difftext));
    Add(textLWPCKFactor = new wxTextCtrl(panel2, wxID_ANY, "1.0", wxPoint(minX + diffX, Y)), "/LWP/CKFactor");
    textLWPCKFactor->SetToolTip(wxT("Chaser vehicle drag multiplier"));
    Y += diffY;

    new wxStaticText(panel2, wxID_ANY, "CAREA", wxPoint(minX, Y + difftext));
    Add(textLWPCArea = new wxTextCtrl(panel2, wxID_ANY, "129.4", wxPoint(minX + diffX, Y)), "/LWP/CArea");
    textLWPCArea->SetToolTip(wxT("Chaser vehicle reference area"));
    new wxStaticText(panel2, wxID_ANY, "sq ft", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel2, wxID_ANY, "CWHT", wxPoint(minX, Y + difftext));
    Add(textLWPCWHT = new wxTextCtrl(panel2, wxID_ANY, "300000.0", wxPoint(minX + diffX, Y)), "LWP/CWHT");
    textLWPCWHT->SetToolTip(wxT("Chaser vehicle weight at insertion"));
    new wxStaticText(panel2, wxID_ANY, "lbs", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel2, wxID_ANY, "LW", wxPoint(minX, Y + difftext));
   
    strings.Add(wxT("Launch Window"));
    strings.Add(wxT("Launch Targeting"));
    strings.Add(wxT("Both"));
    Add(comboLWPLW = new wxChoice(panel2, wxID_ANY, wxPoint(minX + diffX, Y), wxDefaultSize, strings), "/LWP/LW");
    comboLWPLW->SetToolTip(wxT("Launch window/launch targeting options"));
    comboLWPLW->SetSelection(2);
    strings.clear();
    Y += diffY;

    new wxStaticText(panel2, wxID_ANY, "NS", wxPoint(minX, Y + difftext));
    strings.Add(wxT("North"));
    strings.Add(wxT("South"));
    strings.Add(wxT("Both"));
    Add(comboLWPNS = new wxChoice(panel2, wxID_ANY, wxPoint(minX + diffX, Y), wxDefaultSize, strings), "/LWP/NS");
    comboLWPNS->SetToolTip(wxT("Inplane launch window opening and closing times option"));
    comboLWPNS->SetSelection(0);
    strings.clear();
    Y += diffY;

    new wxStaticText(panel2, wxID_ANY, "DAY", wxPoint(minX, Y + difftext));
    Add(textLWPDay = new wxTextCtrl(panel2, wxID_ANY, "0", wxPoint(minX + diffX, Y)), "LWP/LWPDay");
    textLWPDay->SetToolTip(wxT("Day on which launch window times are computed, relative to base date"));
    Y += diffY;

    new wxStaticText(panel2, wxID_ANY, "LPT", wxPoint(minX, Y + difftext));
    strings.Add(wxT("Do not generate table"));
    strings.Add(wxT("Around opening"));
    strings.Add(wxT("Around closing"));
    strings.Add(wxT("Around both"));
    strings.Add(wxT("Entire window"));
    Add(comboLWP_LPT = new wxChoice(panel2, wxID_ANY, wxPoint(minX + diffX, Y), wxDefaultSize, strings), "/LWP/LPT");
    comboLWP_LPT->SetToolTip(wxT("Launch window parameter table options"));
    comboLWP_LPT->SetSelection(0);
    strings.clear();
    Y += diffY;

    new wxStaticText(panel2, wxID_ANY, "TSTART", wxPoint(minX, Y + difftext));
    Add(textLWP_TSTART = new wxTextCtrl(panel2, wxID_ANY, "300.0", wxPoint(minX + diffX, Y)), "/LWP/TSTART");
    textLWP_TSTART->SetToolTip(wxT("Delta time prior to inplane time to start parameter table"));
    new wxStaticText(panel2, wxID_ANY, "sec", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel2, wxID_ANY, "TEND", wxPoint(minX, Y + difftext));
    Add(textLWP_TEND = new wxTextCtrl(panel2, wxID_ANY, "300.0", wxPoint(minX + diffX, Y)), "/LWP/TEND");
    textLWP_TEND->SetToolTip(wxT("Delta time after inplane time to start parameter table"));
    new wxStaticText(panel2, wxID_ANY, "sec", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel2, wxID_ANY, "TSTEP", wxPoint(minX, Y + difftext));
    Add(textLWP_TSTEP = new wxTextCtrl(panel2, wxID_ANY, "300.0", wxPoint(minX + diffX, Y)), "/LWP/TSTEP");
    textLWP_TSTEP->SetToolTip(wxT("Time step in parameter table"));
    new wxStaticText(panel2, wxID_ANY, "sec", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel2, wxID_ANY, "STABLE", wxPoint(minX, Y + difftext));
    strings.Add(wxT("Do not compute table"));
    strings.Add(wxT("Compute table"));
    Add(comboLWP_STABLE = new wxChoice(panel2, wxID_ANY, wxPoint(minX + diffX, Y), wxDefaultSize, strings), "/LWP/STABLE");
    comboLWP_STABLE->SetToolTip(wxT("GMTLO* table flag"));
    comboLWP_STABLE->SetSelection(0);
    strings.clear();
    Y += diffY;

    new wxStaticText(panel2, wxID_ANY, "STARS", wxPoint(minX, Y + difftext));
    Add(textLWP_STARS = new wxTextCtrl(panel2, wxID_ANY, "300.0", wxPoint(minX + diffX, Y)), "/LWP/STARS");
    textLWP_STARS->SetToolTip(wxT("Delta time prior to each inplane launch point to start GMTLO* search"));
    new wxStaticText(panel2, wxID_ANY, "sec", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel2, wxID_ANY, "STARE", wxPoint(minX, Y + difftext));
    Add(textLWP_STARE = new wxTextCtrl(panel2, wxID_ANY, "300.0", wxPoint(minX + diffX, Y)), "/LWP/STARE");
    textLWP_STARE->SetToolTip(wxT("Delta time after each inplane launch point to start GMTLO* search"));
    new wxStaticText(panel2, wxID_ANY, "sec", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    minX += 256;
    Y = minY;

    new wxStaticText(panel2, wxID_ANY, "LATLS", wxPoint(minX, Y + difftext));
    Add(textLWP_LATLS = new wxTextCtrl(panel2, wxID_ANY, "28.627", wxPoint(minX + diffX, Y)), "/LWP/LATLS");
    textLWP_LATLS->SetToolTip(wxT("Geocentric latitude of launch site"));
    new wxStaticText(panel2, wxID_ANY, "deg", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel2, wxID_ANY, "LONGLS", wxPoint(minX, Y + difftext));
    Add(textLWP_LONGLS = new wxTextCtrl(panel2, wxID_ANY, "279.379", wxPoint(minX + diffX, Y)), "/LWP/LONGLS");
    textLWP_LONGLS->SetToolTip(wxT("Geographic longitude of launch site"));
    new wxStaticText(panel2, wxID_ANY, "deg", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel2, wxID_ANY, "PFT", wxPoint(minX, Y + difftext));
    Add(textLWP_PFT = new wxTextCtrl(panel2, wxID_ANY, "519.0", wxPoint(minX + diffX, Y)), "/LWP/PFT");
    textLWP_PFT->SetToolTip(wxT("Powered flight time"));
    new wxStaticText(panel2, wxID_ANY, "sec", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel2, wxID_ANY, "PFA", wxPoint(minX, Y + difftext));
    Add(textLWP_PFA = new wxTextCtrl(panel2, wxID_ANY, "14.4", wxPoint(minX + diffX, Y)), "/LWP/PFA");
    textLWP_PFA->SetToolTip(wxT("Powered flight arc"));
    new wxStaticText(panel2, wxID_ANY, "deg", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel2, wxID_ANY, "YSMAX", wxPoint(minX, Y + difftext));
    Add(textLWP_YSMAX = new wxTextCtrl(panel2, wxID_ANY, "14.0", wxPoint(minX + diffX, Y)), "/LWP/YSMAX");
    textLWP_YSMAX->SetToolTip(wxT("Yaw steering limit"));
    new wxStaticText(panel2, wxID_ANY, "deg", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel2, wxID_ANY, "DTOPT", wxPoint(minX, Y + difftext));
    Add(textLWP_DTOPT = new wxTextCtrl(panel2, wxID_ANY, "360.0", wxPoint(minX + diffX, Y)), "/LWP/DTOPT");
    textLWP_DTOPT->SetToolTip(wxT("Delta time to be subtracted from analytical inplane launch time to obtain empirical launch time"));
    new wxStaticText(panel2, wxID_ANY, "sec", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel2, wxID_ANY, "DTGRR", wxPoint(minX, Y + difftext));
    Add(textLWP_DTGRR = new wxTextCtrl(panel2, wxID_ANY, "0.0", wxPoint(minX + diffX, Y)), "/LWP/DTGRR");
    textLWP_DTGRR->SetToolTip(wxT("DT from lift-off, which defines the time of guidance reference release"));
    new wxStaticText(panel2, wxID_ANY, "sec", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel2, wxID_ANY, "RINS", wxPoint(minX, Y + difftext));
    Add(textLWP_RINS = new wxTextCtrl(panel2, wxID_ANY, "21417907.0", wxPoint(minX + diffX, Y)), "/LWP/RINS");
    textLWP_RINS->SetToolTip(wxT("Radius of insertion"));
    new wxStaticText(panel2, wxID_ANY, "ft", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel2, wxID_ANY, "VINS", wxPoint(minX, Y + difftext));
    Add(textLWP_VINS = new wxTextCtrl(panel2, wxID_ANY, "25705.8", wxPoint(minX + diffX, Y)), "/LWP/VINS");
    textLWP_VINS->SetToolTip(wxT("Velocity of insertion"));
    new wxStaticText(panel2, wxID_ANY, "ft/s", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel2, wxID_ANY, "GAMINS", wxPoint(minX, Y + difftext));
    Add(textLWP_GAMINS = new wxTextCtrl(panel2, wxID_ANY, "0.0", wxPoint(minX + diffX, Y)), "/LWP/GAMINS");
    textLWP_GAMINS->SetToolTip(wxT("Flightpath angle of insertion"));
    new wxStaticText(panel2, wxID_ANY, "deg", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    minX += 256;
    Y = minY;

    new wxStaticText(panel2, wxID_ANY, "LOT", wxPoint(minX, Y + difftext));
    strings.Add(wxT("Input time"));
    strings.Add(wxT("Desired phase angle"));
    strings.Add(wxT("Time from zero phase (GMTLOR)"));
    strings.Add(wxT("Time from zero phase (TPLANE)"));
    strings.Add(wxT("Inplane time"));
    strings.Add(wxT("Zero yaw steering time"));
    Add(comboLWP_LOT = new wxChoice(panel2, wxID_ANY, wxPoint(minX + diffX, Y), wxDefaultSize, strings), "/LWP/LOT");
    comboLWP_LOT->SetToolTip(wxT("Lift-off time options for launch targeting"));
    comboLWP_LOT->SetSelection(5);
    strings.clear();
    Y += diffY;

    new wxStaticText(panel2, wxID_ANY, "GMTLOR", wxPoint(minX, Y + difftext));
    Add(textLWP_GMTLOR = new wxTextCtrl(panel2, wxID_ANY, "00:00:00.000", wxPoint(minX + diffX, Y)), "/LWP/GMTLOR");
    textLWP_GMTLOR->SetToolTip(wxT("Recommended or threshold lift-off time"));
    new wxStaticText(panel2, wxID_ANY, "GMT", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel2, wxID_ANY, "OFFSET", wxPoint(minX, Y + difftext));
    Add(textLWP_OFFSET = new wxTextCtrl(panel2, wxID_ANY, "0.0", wxPoint(minX + diffX, Y)), "/LWP/OFFSET");
    textLWP_OFFSET->SetToolTip(wxT("Phase angle desired at insertion"));
    new wxStaticText(panel2, wxID_ANY, "deg", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel2, wxID_ANY, "BIAS", wxPoint(minX, Y + difftext));
    Add(textLWP_BIAS = new wxTextCtrl(panel2, wxID_ANY, "0.0", wxPoint(minX + diffX, Y)), "/LWP/BIAS");
    textLWP_BIAS->SetToolTip(wxT("Delta time added to GMTLO* to produce lift-off time"));
    new wxStaticText(panel2, wxID_ANY, "sec", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel2, wxID_ANY, "TPLANE", wxPoint(minX, Y + difftext));
    Add(textLWP_TPLANE = new wxTextCtrl(panel2, wxID_ANY, "00:00:00.000", wxPoint(minX + diffX, Y)), "/LWP/TPLANE");
    textLWP_TPLANE->SetToolTip(wxT("Greenwich mean time of in-plane lift-off (computed internally if LW is run)"));
    new wxStaticText(panel2, wxID_ANY, "GMT", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel2, wxID_ANY, "TRANS", wxPoint(minX, Y + difftext));
    Add(textLWP_TRANS = new wxTextCtrl(panel2, wxID_ANY, "0.0", wxPoint(minX + diffX, Y)), "/LWP/TRANS");
    textLWP_TRANS->SetToolTip(wxT("Delta time added in in-plane time to obtain lift-off time"));
    new wxStaticText(panel2, wxID_ANY, "sec", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel2, wxID_ANY, "INSCO", wxPoint(minX, Y + difftext));
    strings.Add(wxT("Input VINS, GAMINS, RINS"));
    strings.Add(wxT("Input GAMINS, RINS and height diff"));
    strings.Add(wxT("Input GAMINS, RINS and altitude"));
    Add(comboLWP_INSCO = new wxChoice(panel2, wxID_ANY, wxPoint(minX + diffX, Y), wxDefaultSize, strings), "/LWP/INSCO");
    comboLWP_INSCO->SetToolTip(wxT("Insertion cutoff conditions option flag"));
    comboLWP_INSCO->SetSelection(0);
    strings.clear();
    Y += diffY;

    new wxStaticText(panel2, wxID_ANY, "DHW", wxPoint(minX, Y + difftext));
    Add(textLWP_DHW = new wxTextCtrl(panel2, wxID_ANY, "0.0", wxPoint(minX + diffX, Y)), "/LWP/DHW");
    textLWP_DHW->SetToolTip(wxT("Desired height difference between chaser and target, or altitude of chaser, at input angle from insertion"));
    new wxStaticText(panel2, wxID_ANY, "NM", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel2, wxID_ANY, "DU", wxPoint(minX, Y + difftext));
    Add(textLWP_DU = new wxTextCtrl(panel2, wxID_ANY, "0.0", wxPoint(minX + diffX, Y)), "/LWP/DU");
    textLWP_DU->SetToolTip(wxT("Angle from insertion to obtain a given altitude, or delta altitude"));
    new wxStaticText(panel2, wxID_ANY, "deg", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel2, wxID_ANY, "ANOM", wxPoint(minX, Y + difftext));
    Add(textLWP_ANOM = new wxTextCtrl(panel2, wxID_ANY, "0.0", wxPoint(minX + diffX, Y)), "/LWP/ANOM");
    textLWP_ANOM->SetToolTip(wxT("Nominal semimajor axis at insertion"));
    new wxStaticText(panel2, wxID_ANY, "NM", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel2, wxID_ANY, "DELNOF", wxPoint(minX, Y + difftext));
    strings.Add(wxT("Input DELNO"));
    strings.Add(wxT("Compute DELNO"));
    Add(comboLWP_DELNOF = new wxChoice(panel2, wxID_ANY, wxPoint(minX + diffX, Y), wxDefaultSize, strings), "/LWP/DELNOF");
    comboLWP_DELNOF->SetToolTip(wxT("Flag for option to compute differential nodal regression from insertion to rendezvous"));
    comboLWP_DELNOF->SetSelection(1),
    strings.clear();
    Y += diffY;

    new wxStaticText(panel2, wxID_ANY, "DELNO", wxPoint(minX, Y + difftext));
    Add(textLWP_DELNO = new wxTextCtrl(panel2, wxID_ANY, "0.0", wxPoint(minX + diffX, Y)), "/LWP/DELNO");
    textLWP_DELNO->SetToolTip(wxT("Angle that is added to the target descending node to account for differential nodal regression"));
    new wxStaticText(panel2, wxID_ANY, "deg", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel2, wxID_ANY, "NEGTIV", wxPoint(minX, Y + difftext));
    strings.Add(wxT("Use positive value"));
    strings.Add(wxT("Use negative value"));
    strings.Add(wxT("Use lowest absolute value"));
    Add(comboLWP_NEGTIV = new wxChoice(panel2, wxID_ANY, wxPoint(minX + diffX, Y), wxDefaultSize, strings), "/LWP/NEGTIV");
    comboLWP_NEGTIV->SetToolTip(wxT("Initial phase angle control flag"));
    comboLWP_NEGTIV->SetSelection(0);
    strings.clear();
    Y += diffY;

    new wxStaticText(panel2, wxID_ANY, "WRAP", wxPoint(minX, Y + difftext));
    Add(textLWP_WRAP = new wxTextCtrl(panel2, wxID_ANY, "0", wxPoint(minX + diffX, Y)), "/LWP/WRAP");
    textLWP_WRAP->SetToolTip(wxT("Flag to wrap initial phase angle"));
    Y += diffY;

    wxButton* button = new wxButton(panel2, ID_Button_LWPGenerate, wxT("Generate"),
        wxPoint(10, 514), wxDefaultSize);

    button = new wxButton(panel2, ID_Button_LWPSaveStateVector, wxT("Save SV"),
        wxPoint(550, 514), wxDefaultSize);

    minX += 256 + 64;
    Y = minY;

    new wxButton(panel2, ID_Button_LWP_SIB_Presets, wxT("Saturn IB Presets"), wxPoint(minX, Y), wxDefaultSize);
    Y += diffY;
    new wxButton(panel2, ID_Button_LWP_Shuttle_Presets, wxT("Shuttle Presets"), wxPoint(minX, Y), wxDefaultSize);

    Y = 300;
    new wxStaticText(panel2, wxID_ANY, "Actual GMTLO", wxPoint(minX, Y + difftext));
    Y += diffY;
    textLWP_Actual_GMTLO = new wxTextCtrl(panel2, wxID_ANY, "", wxPoint(minX, Y),wxDefaultSize, wxTE_READONLY);
    Y += diffY;
    new wxButton(panel2, ID_Button_LWP_LVDC_Export, wxT("Export LVDC"), wxPoint(minX, Y), wxDefaultSize);
}

void FDSFrame::AddShuttleLWPPage()
{
    // Add notebook
    nb_ShuttleLWP = new wxNotebook(notebook, wxID_ANY);

    // Add pages
    wxPanel* panel_Init = new wxPanel(nb_ShuttleLWP, wxID_ANY);
    wxPanel* panel_Constants = new wxPanel(nb_ShuttleLWP, wxID_ANY);
    wxPanel* panel_LNCH_REF_TGT_SETS = new wxPanel(nb_ShuttleLWP, wxID_ANY);
    wxPanel* panel_LWP_EXECUTION = new wxPanel(nb_ShuttleLWP, wxID_ANY);
    wxPanel* panel_LTP_EXECUTION = new wxPanel(nb_ShuttleLWP, wxID_ANY);

    nb_ShuttleLWP->AddPage(panel_Init, L"Launch W/T Init");
    nb_ShuttleLWP->AddPage(panel_Constants, L"Constants");
    nb_ShuttleLWP->AddPage(panel_LNCH_REF_TGT_SETS, "LNCH REF TGT SETS");
    nb_ShuttleLWP->AddPage(panel_LWP_EXECUTION, "LWP EXECUTION");
    nb_ShuttleLWP->AddPage(panel_LTP_EXECUTION, "LTP EXECUTION");

    wxArrayString strings;
    int minX, minY, diffX, diffY, counter, difftext, Y, diffunit;

    minX = 10;
    minY = 10;
    diffX = 70;
    diffY = 26;// 32;
    counter = 0;
    difftext = 4;
    diffunit = 200;

    Y = minY;

    new wxStaticText(panel_Init, wxID_ANY, "TRGVEC", wxPoint(minX, Y + difftext));
    Add(textShuttleLWPTargetVector = new wxTextCtrl(panel_Init, wxID_ANY, "ISS.txt", wxPoint(minX + diffX, Y)), "/ShuttleLWP/TargetVector");
    textShuttleLWPTargetVector->SetToolTip(wxT("Target vehicle state vector"));
    Y += diffY;

    new wxStaticText(panel_Init, wxID_ANY, "YS", wxPoint(minX, Y + difftext));
    Add(textShuttleLWP_YSMAX = new wxTextCtrl(panel_Init, wxID_ANY, "14.0", wxPoint(minX + diffX, Y)), "/ShuttleLWP/YS");
    textShuttleLWP_YSMAX->SetToolTip(wxT("Yaw steering limit"));
    new wxStaticText(panel_Init, wxID_ANY, "deg", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel_Init, wxID_ANY, "INSERTION MODE", wxPoint(minX, Y + difftext));
    Add(checkShuttleLWP_DI = new wxCheckBox(panel_Init, wxID_ANY, "DI", wxPoint(minX + diffX + 50, Y + difftext), wxDefaultSize, wxALIGN_LEFT), "/ShuttleLWP/DI");
    checkShuttleLWP_DI->SetValue(true);
    Y += diffY;

    new wxStaticText(panel_Init, wxID_ANY, "CHASER VEHICLE DATA", wxPoint(minX, Y + difftext));
    Y += diffY;

    new wxStaticText(panel_Init, wxID_ANY, "CD", wxPoint(minX, Y + difftext));
    Add(textShuttleLWP_CD = new wxTextCtrl(panel_Init, wxID_ANY, "2.0", wxPoint(minX + diffX, Y)), "/ShuttleLWP/CD");
    textShuttleLWP_CD->SetToolTip("Drag coefficient");
    Y += diffY;

    new wxStaticText(panel_Init, wxID_ANY, "AREA", wxPoint(minX, Y + difftext));
    Add(textShuttleLWP_Area = new wxTextCtrl(panel_Init, wxID_ANY, "2500.0", wxPoint(minX + diffX, Y)), "/ShuttleLWP/AREA");
    textShuttleLWP_Area->SetToolTip("Vehicle area");
    new wxStaticText(panel_Init, wxID_ANY, "ft2", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel_Init, wxID_ANY, "WEIGHT", wxPoint(minX, Y + difftext));
    Add(textShuttleLWP_Weight = new wxTextCtrl(panel_Init, wxID_ANY, "251679", wxPoint(minX + diffX, Y)), "/ShuttleLWP/WEIGHT");
    textShuttleLWP_Weight->SetToolTip("Vehicle weight");
    new wxStaticText(panel_Init, wxID_ANY, "lbm", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel_Init, wxID_ANY, "LAUNCH SITE", wxPoint(minX, Y + difftext));
    strings.Add(wxT("LC-39A"));
    strings.Add(wxT("LC-39B"));
    strings.Add(wxT("SLC-6"));
    Add(comboShuttleLWP_Launchpad = new wxChoice(panel_Init, wxID_comboShuttleLWP_Launchpad, wxPoint(minX + diffX + 50, Y), wxDefaultSize, strings), "/ShuttleLWP/Launchpad");
    comboShuttleLWP_Launchpad->SetToolTip(wxT("Launchpad"));
    comboShuttleLWP_Launchpad->SetSelection(0);
    strings.clear();
    Y += diffY;

    new wxStaticText(panel_Init, wxID_ANY, "LATLS", wxPoint(minX, Y + difftext));
    Add(textShuttleLWP_LATLS = new wxTextCtrl(panel_Init, wxID_ANY, "28.608385", wxPoint(minX + diffX, Y)), "/ShuttleLWP/LATLS");
    textShuttleLWP_LATLS->SetToolTip(wxT("Geocentric latitude of launch site"));
    new wxStaticText(panel_Init, wxID_ANY, "deg", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel_Init, wxID_ANY, "LONGLS", wxPoint(minX, Y + difftext));
    Add(textShuttleLWP_LONGLS = new wxTextCtrl(panel_Init, wxID_ANY, "279.395928", wxPoint(minX + diffX, Y)), "/ShuttleLWP/LONGLS");
    textShuttleLWP_LONGLS->SetToolTip(wxT("Geographic longitude of launch site"));
    new wxStaticText(panel_Init, wxID_ANY, "deg", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    minX += 256;
    Y = minY;

    new wxStaticText(panel_Init, wxID_ANY, "MPS Dump Data", wxPoint(minX + diffX, Y + difftext));
    Y += diffY;
    new wxStaticText(panel_Init, wxID_ANY, "PEG 7", wxPoint(minX + diffX, Y + difftext));
    Y += diffY;

    new wxStaticText(panel_Init, wxID_ANY, "DVX", wxPoint(minX, Y + difftext));
    Add(textShuttleLWP_MPS_Dump_DVX = new wxTextCtrl(panel_Init, wxID_ANY, "8.1", wxPoint(minX + diffX, Y)), "/ShuttleLWP/MPS_Dump_DVX");
    textShuttleLWP_MPS_Dump_DVX->SetToolTip("MPS dump Delta V X in feet per second");
    Y += diffY;

    new wxStaticText(panel_Init, wxID_ANY, "DVY", wxPoint(minX, Y + difftext));
    Add(textShuttleLWP_MPS_Dump_DVY = new wxTextCtrl(panel_Init, wxID_ANY, "0.5", wxPoint(minX + diffX, Y)), "/ShuttleLWP/MPS_Dump_DVY");
    textShuttleLWP_MPS_Dump_DVY->SetToolTip("MPS dump Delta V Y in feet per second");
    Y += diffY;

    new wxStaticText(panel_Init, wxID_ANY, "DVZ", wxPoint(minX, Y + difftext));
    Add(textShuttleLWP_MPS_Dump_DVZ = new wxTextCtrl(panel_Init, wxID_ANY, "1.6", wxPoint(minX + diffX, Y)), "/ShuttleLWP/MPS_Dump_DVZ");
    textShuttleLWP_MPS_Dump_DVZ->SetToolTip("MPS dump Delta V Z in feet per second");
    Y += diffY;

    new wxStaticText(panel_Init, wxID_ANY, "DTIG", wxPoint(minX, Y + difftext));
    Add(textShuttleLWP_MPS_Dump_DTIG = new wxTextCtrl(panel_Init, wxID_ANY, "0:00:01:54", wxPoint(minX + diffX, Y)), "/ShuttleLWP/MPS_Dump_DTIG");
    textShuttleLWP_MPS_Dump_DTIG->SetToolTip("Time from MECO to MPS dump");
    new wxStaticText(panel_Init, wxID_ANY, "sec", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel_Init, wxID_ANY, "ET DATA", wxPoint(minX + diffX, Y + difftext));
    Y += diffY;

    new wxStaticText(panel_Init, wxID_ANY, "AREA", wxPoint(minX, Y + difftext));
    Add(textShuttleLWP_ET_Area = new wxTextCtrl(panel_Init, wxID_ANY, "598.0", wxPoint(minX + diffX, Y)), "/ShuttleLWP/ET_Area");
    textShuttleLWP_ET_Area->SetToolTip("External tank area");
    new wxStaticText(panel_Init, wxID_ANY, "ft2", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel_Init, wxID_ANY, "CD", wxPoint(minX, Y + difftext));
    Add(textShuttleLWP_ET_CD = new wxTextCtrl(panel_Init, wxID_ANY, "2.0", wxPoint(minX + diffX, Y)), "/ShuttleLWP/ET_CD");
    textShuttleLWP_ET_CD->SetToolTip("External tank drag coefficient");
    Y += diffY;

    new wxStaticText(panel_Init, wxID_ANY, "WEIGHT", wxPoint(minX, Y + difftext));
    Add(textShuttleLWP_ET_WT = new wxTextCtrl(panel_Init, wxID_ANY, "75000.0", wxPoint(minX + diffX, Y)), "/ShuttleLWP/ET_Weight");
    textShuttleLWP_ET_WT->SetToolTip("Vehicle weight");
    new wxStaticText(panel_Init, wxID_ANY, "lbm", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;
    Y += diffY;

    new wxStaticText(panel_Init, wxID_ANY, "PFA", wxPoint(minX, Y + difftext));
    Add(text_ShuttleLWP_PFA = new wxTextCtrl(panel_Init, wxID_ANY, "14.4", wxPoint(minX + diffX, Y)), "/ShuttleLWP/PFA");
    text_ShuttleLWP_PFA->SetToolTip(wxT("Powered flight arc"));
    new wxStaticText(panel_Init, wxID_ANY, "deg", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel_Init, wxID_ANY, "PFT", wxPoint(minX, Y + difftext));
    Add(text_ShuttleLWP_PFT = new wxTextCtrl(panel_Init, wxID_ANY, "0:00:08:39.0", wxPoint(minX + diffX, Y)), "/ShuttleLWP/PFT");
    text_ShuttleLWP_PFT->SetToolTip(wxT("Powered flight time"));
    new wxStaticText(panel_Init, wxID_ANY, "sec", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel_Init, wxID_ANY, "RAD", wxPoint(minX, Y + difftext));
    Add(text_ShuttleLWP_RAD = new wxTextCtrl(panel_Init, wxID_ANY, "21241700.0", wxPoint(minX + diffX, Y)), "/ShuttleLWP/RAD");
    text_ShuttleLWP_RAD->SetToolTip(wxT("Radius of insertion"));
    new wxStaticText(panel_Init, wxID_ANY, "ft", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel_Init, wxID_ANY, "VEL", wxPoint(minX, Y + difftext));
    Add(text_ShuttleLWP_VEL = new wxTextCtrl(panel_Init, wxID_ANY, "25819.0", wxPoint(minX + diffX, Y)), "/ShuttleLWP/VEL");
    text_ShuttleLWP_VEL->SetToolTip(wxT("Velocity of insertion"));
    new wxStaticText(panel_Init, wxID_ANY, "fps", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel_Init, wxID_ANY, "FPA", wxPoint(minX, Y + difftext));
    Add(text_ShuttleLWP_FPA = new wxTextCtrl(panel_Init, wxID_ANY, "0.6", wxPoint(minX + diffX, Y)), "/ShuttleLWP/FPA");
    text_ShuttleLWP_FPA->SetToolTip(wxT("Flightpath angle of insertion"));
    new wxStaticText(panel_Init, wxID_ANY, "deg", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel_Init, wxID_ANY, "OPT", wxPoint(minX, Y + difftext));
    Add(text_ShuttleLWP_OPT = new wxTextCtrl(panel_Init, wxID_ANY, "-0:00:05:40.0", wxPoint(minX + diffX, Y)), "/ShuttleLWP/OPT");
    text_ShuttleLWP_OPT->SetToolTip(wxT("Delta time to be subtracted from analytical inplane launch time to obtain empirical launch time"));
    new wxStaticText(panel_Init, wxID_ANY, "sec", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel_Init, wxID_ANY, "DTO", wxPoint(minX, Y + difftext));
    Add(text_ShuttleLWP_DTO = new wxTextCtrl(panel_Init, wxID_ANY, "-0:00:05:00.0", wxPoint(minX + diffX, Y)), "/ShuttleLWP/DTO");
    text_ShuttleLWP_DTO->SetToolTip(wxT("Time from optimum launch time to launch window opening"));
    new wxStaticText(panel_Init, wxID_ANY, "sec", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel_Init, wxID_ANY, "DTC", wxPoint(minX, Y + difftext));
    Add(text_ShuttleLWP_DTC = new wxTextCtrl(panel_Init, wxID_ANY, "0:00:05:00.0", wxPoint(minX + diffX, Y)), "/ShuttleLWP/DTC");
    text_ShuttleLWP_DTC->SetToolTip(wxT("Time from optimum launch time to launch window closing"));
    new wxStaticText(panel_Init, wxID_ANY, "sec", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    minX += 256;
    Y = minY;

    new wxStaticText(panel_Init, wxID_ANY, "ET SEP PARAMETERS", wxPoint(minX + diffX, Y + difftext));
    Y += diffY;

    new wxStaticText(panel_Init, wxID_ANY, "DVX", wxPoint(minX, Y + difftext));
    Add(textShuttleLWP_ET_Sep_DVX = new wxTextCtrl(panel_Init, wxID_ANY, "5.0", wxPoint(minX + diffX, Y)), "/ShuttleLWP/ET_Sep_DVX");
    textShuttleLWP_ET_Sep_DVX->SetToolTip("ET separation Delta V X in feet per second");
    new wxStaticText(panel_Init, wxID_ANY, "fps", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel_Init, wxID_ANY, "DVY", wxPoint(minX, Y + difftext));
    Add(textShuttleLWP_ET_Sep_DVY = new wxTextCtrl(panel_Init, wxID_ANY, "0.0", wxPoint(minX + diffX, Y)), "/ShuttleLWP/ET_Sep_DVY");
    textShuttleLWP_ET_Sep_DVY->SetToolTip("ET separation Delta V Y in feet per second");
    new wxStaticText(panel_Init, wxID_ANY, "fps", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel_Init, wxID_ANY, "DVZ", wxPoint(minX, Y + difftext));
    Add(textShuttleLWP_ET_Sep_DVZ = new wxTextCtrl(panel_Init, wxID_ANY, "-5.0", wxPoint(minX + diffX, Y)), "/ShuttleLWP/ET_Sep_DVZ");
    textShuttleLWP_ET_Sep_DVZ->SetToolTip("ET separation Delta V Z in feet per second");
    new wxStaticText(panel_Init, wxID_ANY, "fps", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel_Init, wxID_ANY, "DTIG", wxPoint(minX, Y + difftext));
    Add(textShuttleLWP_ET_Sep_DTIG = new wxTextCtrl(panel_Init, wxID_ANY, "0:00:00:12.0", wxPoint(minX + diffX, Y)), "/ShuttleLWP/ET_Sep_DTIG");
    textShuttleLWP_ET_Sep_DTIG->SetToolTip("Time from MECO to ET separation");
    new wxStaticText(panel_Init, wxID_ANY, "sec", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel_Init, wxID_ANY, "MISC PARAMETERS", wxPoint(minX + diffX, Y + difftext));
    Y += diffY;

    new wxStaticText(panel_Init, wxID_ANY, "PHASE CNTRL FLAG", wxPoint(minX, Y + difftext));
    strings.Add(wxT("0"));
    strings.Add(wxT("1"));
    strings.Add(wxT("2"));
    Add(comboShuttleLWP_PhaseCntrlFlag = new wxChoice(panel_Init, wxID_ANY, wxPoint(minX + diffX + 50, Y), wxDefaultSize, strings), "/ShuttleLWP/Phase_Control_Flag");
    comboShuttleLWP_PhaseCntrlFlag->SetToolTip(wxT("Phase control"));
    comboShuttleLWP_PhaseCntrlFlag->SetSelection(0);
    strings.clear();
    Y += diffY;

    new wxStaticText(panel_Init, wxID_ANY, "WRAP FLAG", wxPoint(minX, Y + difftext));
    Add(textShuttleLWP_WRAP_FLAG = new wxTextCtrl(panel_Init, wxID_ANY, "0", wxPoint(minX + diffX, Y)), "/ShuttleLWP/WRAP");
    textShuttleLWP_WRAP_FLAG->SetToolTip("Multiples of 360 degrees of phase angle");
    Y += diffY;

    new wxStaticText(panel_Init, wxID_ANY, "LAUNCH AZ DIR FLAG", wxPoint(minX, Y + difftext));
    Add(checkShuttleLWP_LAUNCH_AZ_DIR_FLAG = new wxCheckBox(panel_Init, wxID_ANY, "N", wxPoint(minX + diffX + 70, Y + difftext), wxDefaultSize, wxALIGN_LEFT), "/ShuttleLWP/Launch_Azimuth_Dir_Flag");
    checkShuttleLWP_LAUNCH_AZ_DIR_FLAG->SetValue(true);
    Y += diffY;

    // CONSTANTS

    minX = 10;
    Y = minY;

    new wxStaticText(panel_Constants, wxID_ANY, "Use drag?", wxPoint(minX, Y + difftext));
    Add(checkShuttleLWP_Drag = new wxCheckBox(panel_Constants, wxID_ANY, "YES", wxPoint(minX + diffX + 70, Y + difftext), wxDefaultSize, wxALIGN_LEFT), "/ShuttleLWP/Drag_Flag");
    checkShuttleLWP_Drag->SetValue(true);
    Y += diffY;

    // LNCH REF SETS
    int diffXSets = 150;

    minX = 10;
    Y = minY;

    new wxStaticText(panel_LNCH_REF_TGT_SETS, wxID_ANY, "REF SET 1", wxPoint(minX + diffX + 30, Y));
    new wxStaticText(panel_LNCH_REF_TGT_SETS, wxID_ANY, "REF SET 2", wxPoint(minX + diffX + 30 + diffXSets, Y));
    new wxStaticText(panel_LNCH_REF_TGT_SETS, wxID_ANY, "REF SET 3", wxPoint(minX + diffX + 30 + diffXSets * 2, Y));
    new wxStaticText(panel_LNCH_REF_TGT_SETS, wxID_ANY, "REF SET 4", wxPoint(minX + diffX + 30 + diffXSets * 3, Y));
    Y += diffY;

    new wxStaticText(panel_LNCH_REF_TGT_SETS, wxID_ANY, "OMS-1 TGTS", wxPoint(minX, Y + difftext));
    Y += diffY;

    new wxStaticText(panel_LNCH_REF_TGT_SETS, wxID_ANY, "DTIG", wxPoint(minX, Y + difftext));
    for (int i = 0; i < 4; i++)
    {
        Add(textShuttleLWP_OMS1_DTIG[i] = new wxTextCtrl(panel_LNCH_REF_TGT_SETS, wxID_ANY, "0:00:02:00.000", wxPoint(minX + diffX + diffXSets * i, Y)), "/ShuttleLWP/OMS1_DTIG_" + wxString::Format("%d", i));
        textShuttleLWP_OMS1_DTIG[i]->SetToolTip("Time from MECO to OMS-1");
    }
    Y += diffY;

    new wxStaticText(panel_LNCH_REF_TGT_SETS, wxID_ANY, "C1", wxPoint(minX, Y + difftext));
    for (int i = 0; i < 4; i++)
    {
        Add(textShuttleLWP_OMS1_C1[i] = new wxTextCtrl(panel_LNCH_REF_TGT_SETS, wxID_ANY, "0.0", wxPoint(minX + diffX + diffXSets * i, Y)), "/ShuttleLWP/OMS1_C1_" + wxString::Format("%d", i));
        textShuttleLWP_OMS1_C1[i]->SetToolTip("C1 for OMS-1");
    }
    Y += diffY;

    new wxStaticText(panel_LNCH_REF_TGT_SETS, wxID_ANY, "C2", wxPoint(minX, Y + difftext));
    for (int i = 0; i < 4; i++)
    {
        Add(textShuttleLWP_OMS1_C2[i] = new wxTextCtrl(panel_LNCH_REF_TGT_SETS, wxID_ANY, "0.0", wxPoint(minX + diffX + diffXSets * i, Y)), "/ShuttleLWP/OMS1_C2_" + wxString::Format("%d", i));
        textShuttleLWP_OMS1_C2[i]->SetToolTip("C2 for OMS-1");
    }
    Y += diffY;

    new wxStaticText(panel_LNCH_REF_TGT_SETS, wxID_ANY, "HT", wxPoint(minX, Y + difftext));
    for (int i = 0; i < 4; i++)
    {
        Add(textShuttleLWP_OMS1_HT[i] = new wxTextCtrl(panel_LNCH_REF_TGT_SETS, wxID_ANY, "120.0", wxPoint(minX + diffX + diffXSets * i, Y)), "/ShuttleLWP/OMS1_HT_" + wxString::Format("%d", i));
        textShuttleLWP_OMS1_HT[i]->SetToolTip("Height for OMS-1");
    }
    Y += diffY;

    new wxStaticText(panel_LNCH_REF_TGT_SETS, wxID_ANY, "THETA-T", wxPoint(minX, Y + difftext));
    for (int i = 0; i < 4; i++)
    {
        Add(textShuttleLWP_OMS1_THETAT[i] = new wxTextCtrl(panel_LNCH_REF_TGT_SETS, wxID_ANY, "133.0", wxPoint(minX + diffX + diffXSets * i, Y)), "/ShuttleLWP/OMS1_THETAT_" + wxString::Format("%d", i));
        textShuttleLWP_OMS1_THETAT[i]->SetToolTip("Theta-T for OMS-1");
    }
    Y += diffY;

    new wxStaticText(panel_LNCH_REF_TGT_SETS, wxID_ANY, "OMS-2 TGTS", wxPoint(minX, Y + difftext));
    Y += diffY;

    new wxStaticText(panel_LNCH_REF_TGT_SETS, wxID_ANY, "DTIG", wxPoint(minX, Y + difftext));
    for (int i = 0; i < 4; i++)
    {
        Add(textShuttleLWP_OMS2_DTIG[i] = new wxTextCtrl(panel_LNCH_REF_TGT_SETS, wxID_ANY, "0:00:31:17.000", wxPoint(minX + diffX + diffXSets * i, Y)), "/ShuttleLWP/OMS2_DTIG_" + wxString::Format("%d", i));
        textShuttleLWP_OMS2_DTIG[i]->SetToolTip("Time from MECO to OMS-1");
    }
    Y += diffY;

    new wxStaticText(panel_LNCH_REF_TGT_SETS, wxID_ANY, "C1", wxPoint(minX, Y + difftext));
    for (int i = 0; i < 4; i++)
    {
        Add(textShuttleLWP_OMS2_C1[i] = new wxTextCtrl(panel_LNCH_REF_TGT_SETS, wxID_ANY, "0.0", wxPoint(minX + diffX + diffXSets * i, Y)), "/ShuttleLWP/OMS2_C1_" + wxString::Format("%d", i));
        textShuttleLWP_OMS2_C1[i]->SetToolTip("C1 for OMS-2");
    }
    Y += diffY;

    new wxStaticText(panel_LNCH_REF_TGT_SETS, wxID_ANY, "C2", wxPoint(minX, Y + difftext));
    for (int i = 0; i < 4; i++)
    {
        Add(textShuttleLWP_OMS2_C2[i] = new wxTextCtrl(panel_LNCH_REF_TGT_SETS, wxID_ANY, "0.0", wxPoint(minX + diffX + diffXSets * i, Y)), "/ShuttleLWP/OMS2_C2_" + wxString::Format("%d", i));
        textShuttleLWP_OMS2_C2[i]->SetToolTip("C2 for OMS-2");
    }
    Y += diffY;

    new wxStaticText(panel_LNCH_REF_TGT_SETS, wxID_ANY, "HT", wxPoint(minX, Y + difftext));
    for (int i = 0; i < 4; i++)
    {
        Add(textShuttleLWP_OMS2_HT[i] = new wxTextCtrl(panel_LNCH_REF_TGT_SETS, wxID_ANY, "93.0", wxPoint(minX + diffX + diffXSets * i, Y)), "/ShuttleLWP/OMS2_HT_" + wxString::Format("%d", i));
        textShuttleLWP_OMS2_HT[i]->SetToolTip("Height for OMS-2");
    }
    Y += diffY;

    new wxStaticText(panel_LNCH_REF_TGT_SETS, wxID_ANY, "THETA-T", wxPoint(minX, Y + difftext));
    for (int i = 0; i < 4; i++)
    {
        Add(textShuttleLWP_OMS2_THETAT[i] = new wxTextCtrl(panel_LNCH_REF_TGT_SETS, wxID_ANY, "326.0", wxPoint(minX + diffX + diffXSets * i, Y)), "/ShuttleLWP/OMS2_THETAT_" + wxString::Format("%d", i));
        textShuttleLWP_OMS2_THETAT[i]->SetToolTip("Theta-T for OMS-2");
    }
    Y += diffY;

    // LWP EXECUTION

    minX = 10;
    Y = minY;

    new wxStaticText(panel_LWP_EXECUTION, wxID_ANY, "REF SET ID", wxPoint(minX, Y + difftext));
    Add(textShuttleLWP_LW_Ref_Set_ID = new wxTextCtrl(panel_LWP_EXECUTION, wxID_ANY, "1", wxPoint(minX + diffX * 3, Y), wxSize(20, -1)), "/ShuttleLWP/LW_Ref_Set_ID");
    Y += diffY;

    new wxStaticText(panel_LWP_EXECUTION, wxID_ANY, "The following are all optional inputs for LWP:", wxPoint(minX, Y + difftext));
    Y += diffY;
    new wxStaticText(panel_LWP_EXECUTION, wxID_ANY, "Desired OMS-2 Phase Angle(s)", wxPoint(minX, Y + difftext));
    Add(textShuttleLWP_Desired_Phase_1 = new wxTextCtrl(panel_LWP_EXECUTION, wxID_ANY, "", wxPoint(minX + diffX * 3, Y)), "/ShuttleLWP/Desired_Phase_1");
    Y += diffY;
    new wxStaticText(panel_LWP_EXECUTION, wxID_ANY, "(DEG, D:M:S, or D.D)", wxPoint(minX, Y + difftext));
    Add(textShuttleLWP_Desired_Phase_2 = new wxTextCtrl(panel_LWP_EXECUTION, wxID_ANY, "", wxPoint(minX + diffX * 3, Y)), "/ShuttleLWP/Desired_Phase_2");
    Y += diffY;

    new wxButton(panel_LWP_EXECUTION, ID_Button_Shuttle_LWP_Execute, wxT("Execute"),
        wxPoint(minX + diffX + 150, Y), wxDefaultSize);
    Y += diffY;
    new wxStaticLine(panel_LWP_EXECUTION, wxID_STATIC,
        wxPoint(minX, Y), wxSize(500, -1), wxLI_HORIZONTAL);
    Y += diffY;
    new wxStaticText(panel_LWP_EXECUTION, wxID_ANY, "LAUNCH WINDOW PROCESSOR OUTPUT", wxPoint(minX + diffX, Y + difftext));
    Y += diffY;
    new wxStaticText(panel_LWP_EXECUTION, wxID_ANY, "OPTIMUM L/O", wxPoint(minX, Y + difftext));
    textShuttleLWP_LW_Out[0] = new wxTextCtrl(panel_LWP_EXECUTION, wxID_ANY, "", wxPoint(minX + 96, Y), wxDefaultSize, wxTE_READONLY);
    Y += diffY;
    new wxStaticText(panel_LWP_EXECUTION, wxID_ANY, "PHASE", wxPoint(minX + diffX, Y + difftext));
    textShuttleLWP_LW_Out[1] = new wxTextCtrl(panel_LWP_EXECUTION, wxID_ANY, "", wxPoint(minX + 128, Y), wxSize(60, -1), wxTE_READONLY);
    new wxStaticText(panel_LWP_EXECUTION, wxID_ANY, "deg", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    int PlanarDiff = 256;

    new wxStaticText(panel_LWP_EXECUTION, wxID_ANY, "PLANAR OPEN", wxPoint(minX, Y + difftext));
    new wxStaticText(panel_LWP_EXECUTION, wxID_ANY, "PLANAR CLOSE", wxPoint(minX + PlanarDiff, Y + difftext));
    Y += diffY;
    new wxStaticText(panel_LWP_EXECUTION, wxID_ANY, "L/O", wxPoint(minX, Y + difftext));
    textShuttleLWP_LW_Out[2] = new wxTextCtrl(panel_LWP_EXECUTION, wxID_ANY, "", wxPoint(minX + diffX, Y), wxDefaultSize, wxTE_READONLY);
    new wxStaticText(panel_LWP_EXECUTION, wxID_ANY, "L/O", wxPoint(minX + PlanarDiff, Y + difftext));
    textShuttleLWP_LW_Out[4] = new wxTextCtrl(panel_LWP_EXECUTION, wxID_ANY, "", wxPoint(minX + PlanarDiff + diffX, Y), wxDefaultSize, wxTE_READONLY);
    Y += diffY;
    new wxStaticText(panel_LWP_EXECUTION, wxID_ANY, "PHASE", wxPoint(minX, Y + difftext));
    textShuttleLWP_LW_Out[3] = new wxTextCtrl(panel_LWP_EXECUTION, wxID_ANY, "", wxPoint(minX + diffX, Y), wxDefaultSize, wxTE_READONLY);
    new wxStaticText(panel_LWP_EXECUTION, wxID_ANY, "deg", wxPoint(minX + diffunit, Y + difftext));
    new wxStaticText(panel_LWP_EXECUTION, wxID_ANY, "PHASE", wxPoint(minX + PlanarDiff, Y + difftext));
    textShuttleLWP_LW_Out[5] = new wxTextCtrl(panel_LWP_EXECUTION, wxID_ANY, "", wxPoint(minX + PlanarDiff + diffX, Y), wxDefaultSize, wxTE_READONLY);
    new wxStaticText(panel_LWP_EXECUTION, wxID_ANY, "deg", wxPoint(minX + PlanarDiff + diffunit, Y + difftext));
    Y += diffY;
    new wxStaticText(panel_LWP_EXECUTION, wxID_ANY, "DESIRED OMS-2 PHASE ANGLE(S)", wxPoint(minX, Y + difftext));
    new wxStaticText(panel_LWP_EXECUTION, wxID_ANY, "L/O TO ACHIEVE DESIRED PHASE", wxPoint(minX + PlanarDiff, Y + difftext));
    Y += diffY;
    textShuttleLWP_LW_Out[6] = new wxTextCtrl(panel_LWP_EXECUTION, wxID_ANY, "", wxPoint(minX  + diffX, Y), wxDefaultSize, wxTE_READONLY);
    new wxStaticText(panel_LWP_EXECUTION, wxID_ANY, "deg", wxPoint(minX + diffunit, Y + difftext));
    textShuttleLWP_LW_Out[7] = new wxTextCtrl(panel_LWP_EXECUTION, wxID_ANY, "", wxPoint(minX + PlanarDiff + diffX, Y), wxDefaultSize, wxTE_READONLY);
    Y += diffY;
    textShuttleLWP_LW_Out[8] = new wxTextCtrl(panel_LWP_EXECUTION, wxID_ANY, "", wxPoint(minX + diffX, Y), wxDefaultSize, wxTE_READONLY);
    new wxStaticText(panel_LWP_EXECUTION, wxID_ANY, "deg", wxPoint(minX + diffunit, Y + difftext));
    textShuttleLWP_LW_Out[9] = new wxTextCtrl(panel_LWP_EXECUTION, wxID_ANY, "", wxPoint(minX + PlanarDiff + diffX, Y), wxDefaultSize, wxTE_READONLY);

    // LTP EXECUTION
    minX = 10;
    Y = minY;

    new wxStaticText(panel_LTP_EXECUTION, wxID_ANY, "GMT L/O", wxPoint(minX, Y + difftext));
    Add(textShuttleLWP_LT_GMTLO = new wxTextCtrl(panel_LTP_EXECUTION, wxID_ANY, "00:00:00.000", wxPoint(minX + diffX * 3, Y)), "/ShuttleLWP/LT_GMTLO");
    Y += diffY;

    new wxStaticText(panel_LTP_EXECUTION, wxID_ANY, "REF SET ID", wxPoint(minX, Y + difftext));
    Add(textShuttleLWP_LT_Ref_Set_ID = new wxTextCtrl(panel_LTP_EXECUTION, wxID_ANY, "1", wxPoint(minX + diffX * 3, Y), wxSize(20, -1)), "/ShuttleLWP/LT_Ref_Set_ID");
    Y += diffY;

    new wxButton(panel_LTP_EXECUTION, ID_Button_Shuttle_LTP_Execute, wxT("Execute"),
        wxPoint(minX + diffX + 150, Y), wxDefaultSize);
    Y += diffY;

    new wxStaticLine(panel_LTP_EXECUTION, wxID_STATIC,
        wxPoint(minX, Y), wxSize(500, -1), wxLI_HORIZONTAL);
    Y += diffY;
    new wxStaticText(panel_LTP_EXECUTION, wxID_ANY, "LAUNCH TARGETING PROCESSOR OUTPUT", wxPoint(minX + diffX*2, Y + difftext));
    Y += diffY;
    new wxStaticText(panel_LTP_EXECUTION, wxID_ANY, "GMT L/O", wxPoint(minX + diffX, Y + difftext));
    textShuttleLWP_LT_Out[0] = new wxTextCtrl(panel_LTP_EXECUTION, wxID_ANY, "", wxPoint(minX + diffX*3, Y), wxDefaultSize, wxTE_READONLY);
    Y += diffY;
    Y += diffY;

    int minOutY = Y;

    new wxStaticText(panel_LTP_EXECUTION, wxID_ANY, "CHASER", wxPoint(minX + diffX, Y + difftext));
    Y += diffY;
    new wxStaticText(panel_LTP_EXECUTION, wxID_ANY, "MECO", wxPoint(minX + diffX, Y + difftext));
    Y += diffY;
    new wxStaticText(panel_LTP_EXECUTION, wxID_ANY, "MECO", wxPoint(minX, Y + difftext));
    textShuttleLWP_LT_Out[1] = new wxTextCtrl(panel_LTP_EXECUTION, wxID_ANY, "", wxPoint(minX + diffX, Y), wxDefaultSize, wxTE_READONLY);
    Y += diffY;
    new wxStaticText(panel_LTP_EXECUTION, wxID_ANY, "VMECO", wxPoint(minX, Y + difftext));
    textShuttleLWP_LT_Out[2] = new wxTextCtrl(panel_LTP_EXECUTION, wxID_ANY, "", wxPoint(minX + diffX, Y), wxDefaultSize, wxTE_READONLY);
    new wxStaticText(panel_LTP_EXECUTION, wxID_ANY, "fps", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;
    new wxStaticText(panel_LTP_EXECUTION, wxID_ANY, "RMECO", wxPoint(minX, Y + difftext));
    textShuttleLWP_LT_Out[3] = new wxTextCtrl(panel_LTP_EXECUTION, wxID_ANY, "", wxPoint(minX + diffX, Y), wxDefaultSize, wxTE_READONLY);
    new wxStaticText(panel_LTP_EXECUTION, wxID_ANY, "nm", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;
    new wxStaticText(panel_LTP_EXECUTION, wxID_ANY, "GMECO", wxPoint(minX, Y + difftext));
    textShuttleLWP_LT_Out[4] = new wxTextCtrl(panel_LTP_EXECUTION, wxID_ANY, "", wxPoint(minX + diffX, Y), wxDefaultSize, wxTE_READONLY);
    new wxStaticText(panel_LTP_EXECUTION, wxID_ANY, "deg", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;
    new wxStaticText(panel_LTP_EXECUTION, wxID_ANY, "IMECO", wxPoint(minX, Y + difftext));
    textShuttleLWP_LT_Out[5] = new wxTextCtrl(panel_LTP_EXECUTION, wxID_ANY, "", wxPoint(minX + diffX, Y), wxDefaultSize, wxTE_READONLY);
    new wxStaticText(panel_LTP_EXECUTION, wxID_ANY, "deg", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;
    new wxStaticText(panel_LTP_EXECUTION, wxID_ANY, "PHASE", wxPoint(minX, Y + difftext));
    textShuttleLWP_LT_Out[6] = new wxTextCtrl(panel_LTP_EXECUTION, wxID_ANY, "", wxPoint(minX + diffX, Y), wxDefaultSize, wxTE_READONLY);
    new wxStaticText(panel_LTP_EXECUTION, wxID_ANY, "deg", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;
    new wxStaticText(panel_LTP_EXECUTION, wxID_ANY, "HA", wxPoint(minX, Y + difftext));
    textShuttleLWP_LT_Out[7] = new wxTextCtrl(panel_LTP_EXECUTION, wxID_ANY, "", wxPoint(minX + diffX, Y), wxDefaultSize, wxTE_READONLY);
    new wxStaticText(panel_LTP_EXECUTION, wxID_ANY, "nm", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;
    new wxStaticText(panel_LTP_EXECUTION, wxID_ANY, "HP", wxPoint(minX, Y + difftext));
    textShuttleLWP_LT_Out[8] = new wxTextCtrl(panel_LTP_EXECUTION, wxID_ANY, "", wxPoint(minX + diffX, Y), wxDefaultSize, wxTE_READONLY);
    new wxStaticText(panel_LTP_EXECUTION, wxID_ANY, "nm", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;
    new wxStaticText(panel_LTP_EXECUTION, wxID_ANY, "LONG", wxPoint(minX, Y + difftext));
    textShuttleLWP_LT_Out[9] = new wxTextCtrl(panel_LTP_EXECUTION, wxID_ANY, "", wxPoint(minX + diffX, Y), wxDefaultSize, wxTE_READONLY);
    new wxStaticText(panel_LTP_EXECUTION, wxID_ANY, "deg", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    minX += 240;
    new wxStaticLine(panel_LTP_EXECUTION, wxID_STATIC, wxPoint(minX - 8, minOutY), wxSize(-1, 300), wxLI_VERTICAL);
    Y = minOutY;

    new wxStaticText(panel_LTP_EXECUTION, wxID_ANY, "OMS-1/MPS DUMP", wxPoint(minX + diffX, Y + difftext));
    Y += diffY;
    Y += diffY;
    new wxStaticText(panel_LTP_EXECUTION, wxID_ANY, "TIG", wxPoint(minX, Y + difftext));
    textShuttleLWP_LT_Out[10] = new wxTextCtrl(panel_LTP_EXECUTION, wxID_ANY, "", wxPoint(minX + diffX, Y), wxDefaultSize, wxTE_READONLY);
    Y += diffY;
    new wxStaticText(panel_LTP_EXECUTION, wxID_ANY, "DELTA V", wxPoint(minX, Y + difftext));
    textShuttleLWP_LT_Out[11] = new wxTextCtrl(panel_LTP_EXECUTION, wxID_ANY, "", wxPoint(minX + diffX, Y), wxDefaultSize, wxTE_READONLY);
    new wxStaticText(panel_LTP_EXECUTION, wxID_ANY, "fps", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;
    new wxStaticText(panel_LTP_EXECUTION, wxID_ANY, "HA", wxPoint(minX, Y + difftext));
    textShuttleLWP_LT_Out[12] = new wxTextCtrl(panel_LTP_EXECUTION, wxID_ANY, "", wxPoint(minX + diffX, Y), wxDefaultSize, wxTE_READONLY);
    new wxStaticText(panel_LTP_EXECUTION, wxID_ANY, "nm", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;
    new wxStaticText(panel_LTP_EXECUTION, wxID_ANY, "HP", wxPoint(minX, Y + difftext));
    textShuttleLWP_LT_Out[13] = new wxTextCtrl(panel_LTP_EXECUTION, wxID_ANY, "", wxPoint(minX + diffX, Y), wxDefaultSize, wxTE_READONLY);
    new wxStaticText(panel_LTP_EXECUTION, wxID_ANY, "nm", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    minX += 240;
    new wxStaticLine(panel_LTP_EXECUTION, wxID_STATIC, wxPoint(minX - 8, minOutY), wxSize(-1, 300), wxLI_VERTICAL);
    Y = minOutY;

    new wxStaticText(panel_LTP_EXECUTION, wxID_ANY, "OMS-2", wxPoint(minX + diffX, Y + difftext));
    Y += diffY;
    Y += diffY;
    new wxStaticText(panel_LTP_EXECUTION, wxID_ANY, "TIG", wxPoint(minX, Y + difftext));
    textShuttleLWP_LT_Out[14] = new wxTextCtrl(panel_LTP_EXECUTION, wxID_ANY, "", wxPoint(minX + diffX, Y), wxDefaultSize, wxTE_READONLY);
    Y += diffY;
    new wxStaticText(panel_LTP_EXECUTION, wxID_ANY, "DELTA V", wxPoint(minX, Y + difftext));
    textShuttleLWP_LT_Out[15] = new wxTextCtrl(panel_LTP_EXECUTION, wxID_ANY, "", wxPoint(minX + diffX, Y), wxDefaultSize, wxTE_READONLY);
    new wxStaticText(panel_LTP_EXECUTION, wxID_ANY, "fps", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;
    new wxStaticText(panel_LTP_EXECUTION, wxID_ANY, "HA", wxPoint(minX, Y + difftext));
    textShuttleLWP_LT_Out[16] = new wxTextCtrl(panel_LTP_EXECUTION, wxID_ANY, "", wxPoint(minX + diffX, Y), wxDefaultSize, wxTE_READONLY);
    new wxStaticText(panel_LTP_EXECUTION, wxID_ANY, "nm", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;
    new wxStaticText(panel_LTP_EXECUTION, wxID_ANY, "HP", wxPoint(minX, Y + difftext));
    textShuttleLWP_LT_Out[17] = new wxTextCtrl(panel_LTP_EXECUTION, wxID_ANY, "", wxPoint(minX + diffX, Y), wxDefaultSize, wxTE_READONLY);
    new wxStaticText(panel_LTP_EXECUTION, wxID_ANY, "nm", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;
    Y += diffY;
    new wxStaticText(panel_LTP_EXECUTION, wxID_ANY, "NODE", wxPoint(minX, Y + difftext));
    textShuttleLWP_LT_Out[18] = new wxTextCtrl(panel_LTP_EXECUTION, wxID_ANY, "", wxPoint(minX + diffX, Y), wxDefaultSize, wxTE_READONLY);
    new wxStaticText(panel_LTP_EXECUTION, wxID_ANY, "deg", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;
    new wxStaticText(panel_LTP_EXECUTION, wxID_ANY, "PHASE", wxPoint(minX, Y + difftext));
    textShuttleLWP_LT_Out[19] = new wxTextCtrl(panel_LTP_EXECUTION, wxID_ANY, "", wxPoint(minX + diffX, Y), wxDefaultSize, wxTE_READONLY);
    new wxStaticText(panel_LTP_EXECUTION, wxID_ANY, "deg", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;
    new wxStaticText(panel_LTP_EXECUTION, wxID_ANY, "PERIOD", wxPoint(minX, Y + difftext));
    textShuttleLWP_LT_Out[20] = new wxTextCtrl(panel_LTP_EXECUTION, wxID_ANY, "", wxPoint(minX + diffX, Y), wxDefaultSize, wxTE_READONLY);
    Y += diffY;

    minX += 240;
    new wxStaticLine(panel_LTP_EXECUTION, wxID_STATIC, wxPoint(minX - 8, minOutY), wxSize(-1, 300), wxLI_VERTICAL);
    Y = minOutY;

    new wxStaticText(panel_LTP_EXECUTION, wxID_ANY, "TGT (AT ASCN)", wxPoint(minX + diffX, Y + difftext));
    Y += diffY;
    Y += diffY;
    new wxStaticText(panel_LTP_EXECUTION, wxID_ANY, "HA", wxPoint(minX, Y + difftext));
    textShuttleLWP_LT_Out[21] = new wxTextCtrl(panel_LTP_EXECUTION, wxID_ANY, "", wxPoint(minX + diffX, Y), wxDefaultSize, wxTE_READONLY);
    new wxStaticText(panel_LTP_EXECUTION, wxID_ANY, "nm", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;
    new wxStaticText(panel_LTP_EXECUTION, wxID_ANY, "HP", wxPoint(minX, Y + difftext));
    textShuttleLWP_LT_Out[22] = new wxTextCtrl(panel_LTP_EXECUTION, wxID_ANY, "", wxPoint(minX + diffX, Y), wxDefaultSize, wxTE_READONLY);
    new wxStaticText(panel_LTP_EXECUTION, wxID_ANY, "nm", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;
    new wxStaticText(panel_LTP_EXECUTION, wxID_ANY, "LONG", wxPoint(minX, Y + difftext));
    textShuttleLWP_LT_Out[23] = new wxTextCtrl(panel_LTP_EXECUTION, wxID_ANY, "", wxPoint(minX + diffX, Y), wxDefaultSize, wxTE_READONLY);
    new wxStaticText(panel_LTP_EXECUTION, wxID_ANY, "deg", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;
    new wxStaticText(panel_LTP_EXECUTION, wxID_ANY, "DELN", wxPoint(minX, Y + difftext));
    textShuttleLWP_LT_Out[24] = new wxTextCtrl(panel_LTP_EXECUTION, wxID_ANY, "", wxPoint(minX + diffX, Y), wxDefaultSize, wxTE_READONLY);
    new wxStaticText(panel_LTP_EXECUTION, wxID_ANY, "deg", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;
    new wxStaticText(panel_LTP_EXECUTION, wxID_ANY, "PERIOD", wxPoint(minX, Y + difftext));
    textShuttleLWP_LT_Out[25] = new wxTextCtrl(panel_LTP_EXECUTION, wxID_ANY, "", wxPoint(minX + diffX, Y), wxDefaultSize, wxTE_READONLY);
    Y += diffY;

    new wxButton(panel_LTP_EXECUTION, ID_Button_ShuttleLWP_SaveStateVector, wxT("Save MPS Dump SV"),
        wxPoint(minX + 16, Y + 64), wxDefaultSize);

    new wxButton(panel_LTP_EXECUTION, ID_Button_ShuttleLWP_Export, wxT("Export"),
        wxPoint(minX + 16 + diffX + 64, Y + 64), wxDefaultSize);
}

void FDSFrame::AddOMPPage()
{
    int minX, minY, diffX, diffY, counter, difftext, Y, diffunit;

    minX = 10;
    minY = 10;
    diffX = 70;
    diffY = 32;
    counter = 0;
    difftext = 4;
    diffunit = 200;

    Y = minY;

    new wxStaticText(panel3, wxID_ANY, "GMTLO", wxPoint(minX, Y + difftext));
    Add(textOMP_GMTLO = new wxTextCtrl(panel3, wxID_ANY, "13:00:00.000", wxPoint(minX + diffX, Y)), "/OMP/GMTLO");
    textOMP_GMTLO->SetToolTip(wxT("Lift-off time"));
    new wxStaticText(panel3, wxID_ANY, "GMT", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel3, wxID_ANY, "Chaser", wxPoint(minX, Y + difftext));
    Add(textOMP_Chaser = new wxTextCtrl(panel3, wxID_ANY, "LWP.txt", wxPoint(minX + diffX, Y)), "/OMP/Chaser");
    textOMP_Chaser->SetToolTip(wxT("Chaser state vector file"));
    Y += diffY;

    new wxStaticText(panel3, wxID_ANY, "Target", wxPoint(minX, Y + difftext));
    Add(textOMP_Target = new wxTextCtrl(panel3, wxID_ANY, "Skylab.txt", wxPoint(minX + diffX, Y)), "/OMP/Target");
    textOMP_Target->SetToolTip(wxT("Target state vector file"));
    Y += diffY;

    new wxStaticText(panel3, wxID_ANY, "MCT", wxPoint(minX, Y + difftext));
    Add(textOMP_MCT = new wxTextCtrl(panel3, wxID_ANY, "SL-2 MCT.txt", wxPoint(minX + diffX, Y)), "/OMP/MCT");
    textOMP_MCT->SetToolTip(wxT("Maneuver constraints table file"));

    new wxButton(panel3, ID_Button_MCT_View, wxT("View"),
        wxPoint(minX + diffX + 150, Y), wxDefaultSize);

    Y += diffY;

    new wxButton(panel3, ID_Button_OMPGenerate, wxT("Generate"),
        wxPoint(10, 514), wxDefaultSize);

    //MCT
    new wxStaticText(panel3, wxID_ANY, "Maneuver Constraints Table", wxPoint(550, 10));
    wxPanel *temp = new wxPanel(panel3, wxID_ANY, wxPoint(350, 40), wxSize(650, 300), wxBORDER_SIMPLE);

    textOMP_MCT_Editor = new wxGrid(temp, wxID_ANY, wxDefaultPosition, wxSize(650, 300));

    textOMP_MCT_Editor->CreateGrid(40, 13);

    textOMP_MCT_Editor->SetColSize(0, 50);
    textOMP_MCT_Editor->SetColSize(1, 50);
    textOMP_MCT_Editor->SetColSize(2, 60);
    textOMP_MCT_Editor->SetColSize(3, 95);

    textOMP_MCT_Editor->SetColLabelValue(0, "Name");
    textOMP_MCT_Editor->SetColLabelValue(1, "Type");
    textOMP_MCT_Editor->SetColLabelValue(2, "Threshold");
    textOMP_MCT_Editor->SetColLabelValue(3, "Value");
    textOMP_MCT_Editor->SetColLabelValue(4, "Secondary 1");
    textOMP_MCT_Editor->SetColLabelValue(5, "Secondary 2");
    textOMP_MCT_Editor->SetColLabelValue(6, "Secondary 3");
    textOMP_MCT_Editor->SetColLabelValue(7, "Secondary 4");
    textOMP_MCT_Editor->SetColLabelValue(8, "Secondary 5");
    textOMP_MCT_Editor->SetColLabelValue(9, "Secondary 6");
    textOMP_MCT_Editor->SetColLabelValue(10, "Secondary 7");
    textOMP_MCT_Editor->SetColLabelValue(11, "Secondary 8");
    textOMP_MCT_Editor->SetColLabelValue(12, "Secondary 9");

    new wxButton(panel3, ID_Button_MCT_Load_Template, wxT("Load Template"),
        wxPoint(450, 350), wxDefaultSize);

    new wxButton(panel3, ID_Button_MCT_Save, wxT("Save"),
        wxPoint(450, 414), wxDefaultSize);

    new wxButton(panel3, ID_Button_FDOMFD_Export, wxT("Export for FDO MFD"),
        wxPoint(550, 414), wxDefaultSize);
}

void FDSFrame::AddSkylabLWPPage()
{
    wxArrayString strings;
    int minX, minY, diffX, diffY, counter, difftext, Y, diffunit;

    minX = 10;
    minY = 10;
    diffX = 70;
    diffY = 32;
    counter = 0;
    difftext = 4;
    diffunit = 200;

    Y = minY;

    new wxStaticText(panel4, wxID_ANY, "TRGVEC", wxPoint(minX, Y + difftext));
    Add(textSkylabLWP_TargetVector = new wxTextCtrl(panel4, wxID_ANY, "Skylab.txt", wxPoint(minX + diffX, Y)), "/SkylabLWP/TargetVector");
    textSkylabLWP_TargetVector->SetToolTip(wxT("Target vehicle state vector"));
    Y += diffY;

    new wxStaticText(panel4, wxID_ANY, "CKFACTOR", wxPoint(minX, Y + difftext));
    Add(textSkylabLWP_CKFactor = new wxTextCtrl(panel4, wxID_ANY, "1.0", wxPoint(minX + diffX, Y)), "/SkylabLWP/CKFactor");
    textSkylabLWP_CKFactor->SetToolTip(wxT("Chaser vehicle drag multiplier"));
    Y += diffY;

    new wxStaticText(panel4, wxID_ANY, "CAREA", wxPoint(minX, Y + difftext));
    Add(textSkylabLWP_CArea = new wxTextCtrl(panel4, wxID_ANY, "129.4", wxPoint(minX + diffX, Y)), "/SkylabLWP/CArea");
    textSkylabLWP_CArea->SetToolTip(wxT("Chaser vehicle reference area"));
    new wxStaticText(panel4, wxID_ANY, "sq ft", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel4, wxID_ANY, "CWHT", wxPoint(minX, Y + difftext));
    Add(textSkylabLWP_CWHT = new wxTextCtrl(panel4, wxID_ANY, "300000.0", wxPoint(minX + diffX, Y)), "/SkylabLWP/CWHT");
    textSkylabLWP_CWHT->SetToolTip(wxT("Chaser vehicle weight at insertion"));
    new wxStaticText(panel4, wxID_ANY, "lbs", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel4, wxID_ANY, "NS", wxPoint(minX, Y + difftext));
    strings.Add(wxT("North"));
    strings.Add(wxT("South"));
    Add(comboSkylabLWP_NS = new wxChoice(panel4, wxID_ANY, wxPoint(minX + diffX, Y), wxDefaultSize, strings), "/SkylabLWP/NS");
    comboSkylabLWP_NS->SetToolTip(wxT("Inplane launch window opening and closing times option"));
    comboSkylabLWP_NS->SetSelection(0);
    strings.clear();
    Y += diffY;

    new wxStaticText(panel4, wxID_ANY, "DAY", wxPoint(minX, Y + difftext));
    Add(textSkylabLWP_DAY = new wxTextCtrl(panel4, wxID_ANY, "0", wxPoint(minX + diffX, Y)), "/SkylabLWP/DAY");
    textSkylabLWP_DAY->SetToolTip(wxT("Day on which launch window times are computed, relative to base date"));
    Y += diffY;

    new wxStaticText(panel4, wxID_ANY, "LATLS", wxPoint(minX, Y + difftext));
    Add(textSkylabLWP_LATLS = new wxTextCtrl(panel4, wxID_ANY, "28.627", wxPoint(minX + diffX, Y)), "/SkylabLWP/LATLS");
    textSkylabLWP_LATLS->SetToolTip(wxT("Geocentric latitude of launch site"));
    new wxStaticText(panel4, wxID_ANY, "deg", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel4, wxID_ANY, "LONGLS", wxPoint(minX, Y + difftext));
    Add(textSkylabLWP_LONGLS = new wxTextCtrl(panel4, wxID_ANY, "279.379", wxPoint(minX + diffX, Y)), "/SkylabLWP/LONGLS");
    textSkylabLWP_LONGLS->SetToolTip(wxT("Geographic longitude of launch site"));
    new wxStaticText(panel4, wxID_ANY, "deg", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel4, wxID_ANY, "PFT", wxPoint(minX, Y + difftext));
    Add(textSkylabLWP_PFT = new wxTextCtrl(panel4, wxID_ANY, "600.0", wxPoint(minX + diffX, Y)), "/SkylabLWP/PFT");
    textSkylabLWP_PFT->SetToolTip(wxT("Powered flight time"));
    new wxStaticText(panel4, wxID_ANY, "sec", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel4, wxID_ANY, "PFA", wxPoint(minX, Y + difftext));
    Add(textSkylabLWP_PFA = new wxTextCtrl(panel4, wxID_ANY, "18.0", wxPoint(minX + diffX, Y)), "/SkylabLWP/PFA");
    textSkylabLWP_PFA->SetToolTip(wxT("Powered flight arc"));
    new wxStaticText(panel4, wxID_ANY, "deg", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel4, wxID_ANY, "YSMAX", wxPoint(minX, Y + difftext));
    Add(textSkylabLWP_YSMAX = new wxTextCtrl(panel4, wxID_ANY, "5.0", wxPoint(minX + diffX, Y)), "/SkylabLWP/YSMAX");
    textSkylabLWP_YSMAX->SetToolTip(wxT("Yaw steering limit"));
    new wxStaticText(panel4, wxID_ANY, "deg", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel4, wxID_ANY, "DTOPT", wxPoint(minX, Y + difftext));
    Add(textSkylabLWP_DTOPT = new wxTextCtrl(panel4, wxID_ANY, "360.0", wxPoint(minX + diffX, Y)), "/SkylabLWP/DTOPT");
    textSkylabLWP_DTOPT->SetToolTip(wxT("Delta time to be subtracted from analytical inplane launch time to obtain empirical launch time"));
    new wxStaticText(panel4, wxID_ANY, "sec", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel4, wxID_ANY, "RINS", wxPoint(minX, Y + difftext));
    Add(textSkylabLWP_RINS = new wxTextCtrl(panel4, wxID_ANY, "21417907.0", wxPoint(minX + diffX, Y)), "/SkylabLWP/RINS");
    textSkylabLWP_RINS->SetToolTip(wxT("Radius of insertion"));
    new wxStaticText(panel4, wxID_ANY, "ft", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel4, wxID_ANY, "VINS", wxPoint(minX, Y + difftext));
    Add(textSkylabLWP_VINS = new wxTextCtrl(panel4, wxID_ANY, "25705.8", wxPoint(minX + diffX, Y)), "/SkylabLWP/VINS");
    textSkylabLWP_VINS->SetToolTip(wxT("Velocity of insertion"));
    new wxStaticText(panel4, wxID_ANY, "ft/s", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel4, wxID_ANY, "GAMINS", wxPoint(minX, Y + difftext));
    Add(textSkylabLWP_GAMINS = new wxTextCtrl(panel4, wxID_ANY, "0.0", wxPoint(minX + diffX, Y)), "/SkylabLWP/GAMINS");
    textSkylabLWP_GAMINS->SetToolTip(wxT("Flightpath angle of insertion"));
    new wxStaticText(panel4, wxID_ANY, "deg", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    minX += 256;
    Y = minY;

    new wxStaticText(panel4, wxID_ANY, "NEGTIV", wxPoint(minX, Y + difftext));
    strings.Add(wxT("Use positive value"));
    strings.Add(wxT("Use negative value"));
    strings.Add(wxT("Use lowest absolute value"));
    Add(comboSkylabLWP_NEGTIV = new wxChoice(panel4, wxID_ANY, wxPoint(minX + diffX, Y), wxDefaultSize, strings), "/SkylabLWP/NEGTIV");
    comboSkylabLWP_NEGTIV->SetToolTip(wxT("Initial phase angle control flag"));
    comboSkylabLWP_NEGTIV->SetSelection(0);
    strings.clear();
    Y += diffY;

    new wxStaticText(panel4, wxID_ANY, "WRAP", wxPoint(minX, Y + difftext));
    Add(textSkylabLWP_WRAP = new wxTextCtrl(panel4, wxID_ANY, "0", wxPoint(minX + diffX, Y)), "/SkylabLWP/WRAP");
    textSkylabLWP_WRAP->SetToolTip(wxT("Flag to wrap initial phase angle"));
    Y += diffY;

    new wxStaticText(panel4, wxID_ANY, "DTOPEN", wxPoint(minX, Y + difftext));
    Add(textSkylabLWP_DTOPEN = new wxTextCtrl(panel4, wxID_ANY, "7.75", wxPoint(minX + diffX, Y)), "/SkylabLWP/DTOPEN");
    textSkylabLWP_DTOPEN->SetToolTip(wxT("Delta time from optimum inplane launch time to opening of window"));
    new wxStaticText(panel4, wxID_ANY, "min", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel4, wxID_ANY, "DTCLOSE", wxPoint(minX, Y + difftext));
    Add(textSkylabLWP_DTCLOSE = new wxTextCtrl(panel4, wxID_ANY, "7.75", wxPoint(minX + diffX, Y)), "/SkylabLWP/DTCLOSE");
    textSkylabLWP_DTCLOSE->SetToolTip(wxT("Delta time from optimum inplane launch time to closing of window"));
    new wxStaticText(panel4, wxID_ANY, "min", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel4, wxID_ANY, "IR", wxPoint(minX, Y + difftext));
    strings.Add(wxT("Compute launch window"));
    strings.Add(wxT("Input launch time only"));
    Add(comboSkylabLWP_IR = new wxChoice(panel4, wxID_ANY, wxPoint(minX + diffX, Y), wxDefaultSize, strings), "/SkylabLWP/IR");
    comboSkylabLWP_IR->SetToolTip(wxT("Liftoff time flag"));
    comboSkylabLWP_IR->SetSelection(0);
    strings.clear();
    Y += diffY;

    new wxStaticText(panel4, wxID_ANY, "TLO", wxPoint(minX, Y + difftext));
    Add(textSkylabLWP_TLO = new wxTextCtrl(panel4, wxID_ANY, "00:00:00:000", wxPoint(minX + diffX, Y)), "/SkylabLWP/TLO");
    textSkylabLWP_TLO->SetToolTip(wxT("Time of liftoff (if input)"));
    new wxStaticText(panel4, wxID_ANY, "GMT", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel4, wxID_ANY, "MI", wxPoint(minX, Y + difftext));
    Add(textSkylabLWP_MI = new wxTextCtrl(panel4, wxID_ANY, "5", wxPoint(minX + diffX, Y)), "/SkylabLWP/MI");
    textSkylabLWP_MI->SetToolTip(wxT("Initial M-line or rendezvous number"));
    Y += diffY;

    new wxStaticText(panel4, wxID_ANY, "MF", wxPoint(minX, Y + difftext));
    Add(textSkylabLWP_MF = new wxTextCtrl(panel4, wxID_ANY, "8", wxPoint(minX + diffX, Y)), "/SkylabLWP/MF");
    textSkylabLWP_MF->SetToolTip(wxT("Final M-line or rendezvous number"));
    Y += diffY;

    new wxStaticText(panel4, wxID_ANY, "DTSEP", wxPoint(minX, Y + difftext));
    Add(textSkylabLWP_DTSEP = new wxTextCtrl(panel4, wxID_ANY, "000:15:00.000", wxPoint(minX + diffX, Y)), "/SkylabLWP/DTSEP");
    textSkylabLWP_DTSEP->SetToolTip(wxT("Time from insertion to separation"));
    Y += diffY;

    new wxStaticText(panel4, wxID_ANY, "DVSEP", wxPoint(minX, Y + difftext));
    Add(textSkylabLWP_DVSEP = new wxTextCtrl(panel4, wxID_ANY, "+3.0 +0.0 +0.0", wxPoint(minX + diffX, Y)), "/SkylabLWP/DVSEP");
    textSkylabLWP_DVSEP->SetToolTip(wxT("Delta velocity vector of separation"));
    new wxStaticText(panel4, wxID_ANY, "ft/s", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel4, wxID_ANY, "DVWO", wxPoint(minX, Y + difftext));
    Add(textSkylabLWP_DVWO = new wxTextCtrl(panel4, wxID_ANY, "30.0", wxPoint(minX + diffX, Y)), "/SkylabLWP/DVWO");
    textSkylabLWP_DVWO->SetToolTip(wxT("Delta velocity of phase window opening (Minimum NCC DV)"));
    new wxStaticText(panel4, wxID_ANY, "ft/s", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel4, wxID_ANY, "DVWC", wxPoint(minX, Y + difftext));
    Add(textSkylabLWP_DVWC = new wxTextCtrl(panel4, wxID_ANY, "30.0", wxPoint(minX + diffX, Y)), "/SkylabLWP/DVWC");
    textSkylabLWP_DVWC->SetToolTip(wxT("Delta velocity of phase window closing (Minimum NC-1 DV)"));
    new wxStaticText(panel4, wxID_ANY, "ft/s", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    wxButton* button = new wxButton(panel4, ID_Button_SkylabLWPGenerate, wxT("Generate"),
        wxPoint(10, 514), wxDefaultSize);
}

void FDSFrame::AddStateVectorPage()
{
    wxArrayString strings;
    int minX, diffX, difftext, Y, diffY, diffunit;

    StateVectorList = new wxListCtrl(panel5, wxID_StateVectorList, wxPoint(0, 0), wxSize(300, 300), wxLC_LIST);

    new wxButton(panel5, ID_Button_SV_New, wxT("New"),
        wxPoint(10, 356), wxDefaultSize);

    minX = 400;
    diffX = 70;
    difftext = 4;
    Y = 10;
    diffY = 32;
    diffunit = 200;

    new wxStaticText(panel5, wxID_ANY, "File Name:", wxPoint(minX, Y));
    textStateVectorFileName = new wxTextCtrl(panel5, wxID_ANY, "", wxPoint(minX + diffX, Y), wxDefaultSize, wxTE_READONLY);
    Y += diffY;
    new wxStaticText(panel5, wxID_ANY, "Coordinates", wxPoint(minX, Y + difftext));
    strings.Clear();
    strings.Add("TEG");
    strings.Add("J2000");
    strings.Add("M50");
    strings.Add("TLE");
    comboStateVectorCoordinateSystem = new wxChoice(panel5, wxID_ANY, wxPoint(minX + diffX, Y), wxDefaultSize, strings);
    comboStateVectorCoordinateSystem->SetSelection(0);
    Y += diffY;

    new wxStaticText(panel5, wxID_ANY, "SV", wxPoint(minX, Y + difftext));
    textStateVectorData = new wxTextCtrl(panel5, wxID_ANY, wxEmptyString, wxPoint(minX + diffX, Y), wxSize(450, 80), wxTE_MULTILINE);
    textStateVectorData->SetToolTip(wxT("State vector in coordinate system specific format"));
    Y += diffY * 3;

    new wxStaticText(panel5, wxID_ANY, "Weight", wxPoint(minX, Y + difftext));
    textStateVectorWeight = new wxTextCtrl(panel5, wxID_ANY, wxEmptyString, wxPoint(minX + diffX, Y));
    textStateVectorWeight->SetToolTip(wxT("Weight in pounds"));
    new wxStaticText(panel5, wxID_ANY, "lbs", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel5, wxID_ANY, "Area", wxPoint(minX, Y + difftext));
    textStateVectorArea = new wxTextCtrl(panel5, wxID_ANY, wxEmptyString, wxPoint(minX + diffX, Y));
    textStateVectorArea->SetToolTip(wxT("Vehicle area in square feet"));
    new wxStaticText(panel5, wxID_ANY, "sq ft", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel5, wxID_ANY, "K-Factor", wxPoint(minX, Y + difftext));
    textStateVectorKFactor = new wxTextCtrl(panel5, wxID_ANY, wxEmptyString, wxPoint(minX + diffX, Y));
    textStateVectorKFactor->SetToolTip(wxT("Drag multiplier"));
    new wxStaticText(panel5, wxID_ANY, "n.d.", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;
    new wxButton(panel5, ID_Button_SV_Check, wxT("Check"),
        wxPoint(minX, Y), wxDefaultSize);
    new wxButton(panel5, ID_Button_SV_Save, wxT("Save"),
        wxPoint(minX + 100, Y), wxDefaultSize);
    Y += diffY * 2;
    new wxStaticText(panel5, wxID_ANY, "Orbit Data:", wxPoint(minX, Y));
    Y += diffY;
    textStateVectorOrbitData = new wxTextCtrl(panel5, wxID_ANY, "",
        wxPoint(minX, Y), wxSize(450, 160),
        wxTE_READONLY | wxTE_MULTILINE);
}

void FDSFrame::AddLoadSVPage()
{
    wxArrayString strings;
    int minX, minY, diffX, diffY, counter, difftext, Y, diffunit;

    minX = 10;
    minY = 10;
    diffX = 70;
    diffY = 32;
    counter = 0;
    difftext = 4;
    diffunit = 200;

    Y = minY;

    new wxStaticText(panel_LoadSV, wxID_ANY, "REFAX", wxPoint(minX, Y + difftext));
    strings.Add(wxT("TEG"));
    strings.Add(wxT("EFE"));
    comboLoadSV_REFAX = new wxChoice(panel_LoadSV, wxID_ANY, wxPoint(minX + diffX, Y), wxDefaultSize, strings);
    comboLoadSV_REFAX->SetToolTip("Reference axis. TEG = True Equator and Greenwich meridian of epoch. EFE = Earth-fixed equatorial of epoch");
    comboLoadSV_REFAX->SetSelection(0);
    strings.clear();
    Y += diffY;

    new wxStaticText(panel_LoadSV, wxID_ANY, "ELSET", wxPoint(minX, Y + difftext));
    strings.Add(wxT("Cartesian"));
    strings.Add(wxT("Elements"));
    strings.Add(wxT("Spherical"));
    strings.Add(wxT("Apsides"));
    comboLoadSV_ELSET = new wxChoice(panel_LoadSV, ID_LoadSVPageLabels, wxPoint(minX + diffX, Y), wxDefaultSize, strings);
    comboLoadSV_ELSET->SetToolTip("Element set");
    comboLoadSV_ELSET->SetSelection(0);
    strings.clear();
    Y += diffY;

    new wxStaticText(panel_LoadSV, wxID_ANY, "ANGUN", wxPoint(minX, Y + difftext));
    strings.Add(wxT("Degrees"));
    strings.Add(wxT("Radians"));
    comboLoadSV_ANGUN = new wxChoice(panel_LoadSV, ID_LoadSVPageLabels, wxPoint(minX + diffX, Y), wxDefaultSize, strings);
    comboLoadSV_ANGUN->SetToolTip("Angle units");
    comboLoadSV_ANGUN->SetSelection(0);
    strings.clear();
    Y += diffY;

    new wxStaticText(panel_LoadSV, wxID_ANY, "DSTUN", wxPoint(minX, Y + difftext));
    strings.Add(wxT("Feet"));
    strings.Add(wxT("Meters"));
    strings.Add(wxT("Nautical miles"));
    strings.Add(wxT("Earth radii"));
    strings.Add(wxT("Kilometers"));
    comboLoadSV_DSTUN = new wxChoice(panel_LoadSV, ID_LoadSVPageLabels, wxPoint(minX + diffX, Y), wxDefaultSize, strings);
    comboLoadSV_DSTUN->SetToolTip("Distance units");
    comboLoadSV_DSTUN->SetSelection(0);
    strings.clear();
    Y += diffY;

    new wxStaticText(panel_LoadSV, wxID_ANY, "VELUN", wxPoint(minX, Y + difftext));
    strings.Add(wxT("Feet/second"));
    strings.Add(wxT("Meters/second"));
    strings.Add(wxT("Nautical miles/hour"));
    strings.Add(wxT("Earth radii/hour"));
    strings.Add(wxT("Kilometers/hour"));
    comboLoadSV_VELUN = new wxChoice(panel_LoadSV, ID_LoadSVPageLabels, wxPoint(minX + diffX, Y), wxDefaultSize, strings);
    comboLoadSV_VELUN->SetToolTip("Velocity units");
    comboLoadSV_VELUN->SetSelection(0);
    strings.clear();
    Y += diffY;

    new wxStaticText(panel_LoadSV, wxID_ANY, "MASUN", wxPoint(minX, Y + difftext));
    strings.Add(wxT("Pounds"));
    strings.Add(wxT("Kilograms"));
    strings.Add(wxT("Slugs"));
    comboLoadSV_MASUN = new wxChoice(panel_LoadSV, ID_LoadSVPageLabels, wxPoint(minX + diffX, Y), wxDefaultSize, strings);
    comboLoadSV_MASUN->SetToolTip("Mass units");
    comboLoadSV_MASUN->SetSelection(0);
    strings.clear();
    Y += diffY;

    new wxStaticText(panel_LoadSV, wxID_ANY, "LENUN", wxPoint(minX, Y + difftext));
    strings.Add(wxT("Feet"));
    strings.Add(wxT("Meters"));
    strings.Add(wxT("Inches"));
    comboLoadSV_LENUN = new wxChoice(panel_LoadSV, ID_LoadSVPageLabels, wxPoint(minX + diffX, Y), wxDefaultSize, strings);
    comboLoadSV_LENUN->SetToolTip("Length units");
    comboLoadSV_LENUN->SetSelection(0);
    strings.clear();
    Y += diffY;

    minX += 256;
    Y = minY;

    staticLoadSV_Descriptions[0] = new wxStaticText(panel_LoadSV, wxID_ANY, "T", wxPoint(minX, Y + difftext));
    textLoadSV_Array[0] = new wxTextCtrl(panel_LoadSV, wxID_ANY, "00:00:00.000", wxPoint(minX + diffX, Y));
    textLoadSV_Array[0]->SetToolTip(wxT("Time in GMT"));
    staticLoadSV_Units[0] = new wxStaticText(panel_LoadSV, wxID_ANY, "GMT", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    staticLoadSV_Descriptions[1] = new wxStaticText(panel_LoadSV, wxID_ANY, "X", wxPoint(minX, Y + difftext));
    textLoadSV_Array[1] = new wxTextCtrl(panel_LoadSV, wxID_ANY, "0.0", wxPoint(minX + diffX, Y));
    //textLoadSV_Array[1]->SetToolTip(wxT("TBD"));
    staticLoadSV_Units[1] = new wxStaticText(panel_LoadSV, wxID_ANY, "feet", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    staticLoadSV_Descriptions[2] = new wxStaticText(panel_LoadSV, wxID_ANY, "Y", wxPoint(minX, Y + difftext));
    textLoadSV_Array[2] = new wxTextCtrl(panel_LoadSV, wxID_ANY, "0.0", wxPoint(minX + diffX, Y));
    //textLoadSV_Array[2]->SetToolTip(wxT("TBD"));
    staticLoadSV_Units[2] = new wxStaticText(panel_LoadSV, wxID_ANY, "feet", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    staticLoadSV_Descriptions[3] = new wxStaticText(panel_LoadSV, wxID_ANY, "Z", wxPoint(minX, Y + difftext));
    textLoadSV_Array[3] = new wxTextCtrl(panel_LoadSV, wxID_ANY, "0.0", wxPoint(minX + diffX, Y));
    //textLoadSV_Array[3]->SetToolTip(wxT("TBD"));
    staticLoadSV_Units[3] = new wxStaticText(panel_LoadSV, wxID_ANY, "feet", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    staticLoadSV_Descriptions[4] = new wxStaticText(panel_LoadSV, wxID_ANY, "XD", wxPoint(minX, Y + difftext));
    textLoadSV_Array[4] = new wxTextCtrl(panel_LoadSV, wxID_ANY, "0.0", wxPoint(minX + diffX, Y));
    staticLoadSV_Units[4] = new wxStaticText(panel_LoadSV, wxID_ANY, "ft/sec", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    staticLoadSV_Descriptions[5] = new wxStaticText(panel_LoadSV, wxID_ANY, "YD", wxPoint(minX, Y + difftext));
    textLoadSV_Array[5] = new wxTextCtrl(panel_LoadSV, wxID_ANY, "0.0", wxPoint(minX + diffX, Y));
    staticLoadSV_Units[5] = new wxStaticText(panel_LoadSV, wxID_ANY, "ft/sec", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    staticLoadSV_Descriptions[6] = new wxStaticText(panel_LoadSV, wxID_ANY, "ZD", wxPoint(minX, Y + difftext));
    textLoadSV_Array[6] = new wxTextCtrl(panel_LoadSV, wxID_ANY, "0.0", wxPoint(minX + diffX, Y));
    staticLoadSV_Units[6] = new wxStaticText(panel_LoadSV, wxID_ANY, "ft/sec", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    staticLoadSV_Descriptions[7] = new wxStaticText(panel_LoadSV, wxID_ANY, "K-Factor", wxPoint(minX, Y + difftext));
    textLoadSV_Array[7] = new wxTextCtrl(panel_LoadSV, wxID_ANY, "1.0", wxPoint(minX + diffX, Y));
    staticLoadSV_Units[7] = new wxStaticText(panel_LoadSV, wxID_ANY, "nd", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    staticLoadSV_Descriptions[8] = new wxStaticText(panel_LoadSV, wxID_ANY, "Area", wxPoint(minX, Y + difftext));
    textLoadSV_Array[8] = new wxTextCtrl(panel_LoadSV, wxID_ANY, "0.0", wxPoint(minX + diffX, Y));
    staticLoadSV_Units[8] = new wxStaticText(panel_LoadSV, wxID_ANY, "sq ft", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    staticLoadSV_Descriptions[9] = new wxStaticText(panel_LoadSV, wxID_ANY, "Weight", wxPoint(minX, Y + difftext));
    textLoadSV_Array[9] = new wxTextCtrl(panel_LoadSV, wxID_ANY, "0.0", wxPoint(minX + diffX, Y));
    staticLoadSV_Units[9] = new wxStaticText(panel_LoadSV, wxID_ANY, "lbs", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    Y += diffY;
    new wxButton(panel_LoadSV, ID_LoadSV_Convert, wxT("Convert"),
        wxPoint(minX + diffX, Y), wxDefaultSize);

    minX += 256;
    Y = minY;

    new wxStaticText(panel_LoadSV, wxID_ANY, "Output:", wxPoint(minX, Y + difftext));
    Y += diffY;

    textLoadSV_Output = new wxTextCtrl(panel_LoadSV, wxID_ANY, "",
        wxPoint(minX, Y), wxSize(450, 160),
        wxTE_READONLY | wxTE_MULTILINE);

    Y += diffY * 6;
    new wxButton(panel_LoadSV, ID_LoadSV_Save, wxT("Save"),
        wxPoint(minX + diffX, Y), wxDefaultSize);
}

std::string StringFromTextBox(wxTextCtrl* box)
{
    wxString text1 = box->GetLineText(0);
    return std::string(text1.mb_str());
}

void FDSFrame::OnButtonLWPGenerate(wxCommandEvent& event)
{
    if (core->IsInitialized() == false)
    {
        SetStatusText("Date not initialized!");
        return;
    }

    std::string inputs[39];

    inputs[0] = StringFromTextBox(textProjectFile);
    inputs[1] = StringFromTextBox(textLWPTargetVector);
    inputs[2] = StringFromTextBox(textLWPCKFactor);
    inputs[3] = StringFromTextBox(textLWPCArea);
    inputs[4] = StringFromTextBox(textLWPCWHT);

    switch (comboLWPLW->GetSelection())
    {
    case 0:
        inputs[5] = "LW";
        break;
    case 1:
        inputs[5] = "LT";
        break;
    case 2:
        inputs[5] = "WT";
        break;
    default:
        return;
    }
    switch (comboLWPNS->GetSelection())
    {
    case 0:
        inputs[6] = "OP";
        break;
    case 1:
        inputs[6] = "CL";
        break;
    case 2:
        inputs[6] = "OC";
        break;
    default:
        return;
    }

    inputs[7] = StringFromTextBox(textLWPDay);

    switch (comboLWP_LPT->GetSelection())
    {
    case 0:
        inputs[8] = "NO";
        break;
    case 1:
        inputs[8] = "OP";
        break;
    case 2:
        inputs[8] = "CL";
        break;
    case 3:
        inputs[8] = "OC";
        break;
    case 4:
        inputs[8] = "EN";
        break;
    default:
        return;
    }

    inputs[9] = StringFromTextBox(textLWP_TSTART);
    inputs[10] = StringFromTextBox(textLWP_TEND);
    inputs[11] = StringFromTextBox(textLWP_TSTEP);

    switch (comboLWP_STABLE->GetSelection())
    {
    case 0:
        inputs[12] = "NO";
        break;
    case 1:
        inputs[12] = "YE";
        break;
    default:
        return;
    }

    inputs[13] = StringFromTextBox(textLWP_STARS);
    inputs[14] = StringFromTextBox(textLWP_STARE);
    inputs[15] = StringFromTextBox(textLWP_LATLS);
    inputs[16] = StringFromTextBox(textLWP_LONGLS);
    inputs[17] = StringFromTextBox(textLWP_PFT);
    inputs[18] = StringFromTextBox(textLWP_PFA);
    inputs[19] = StringFromTextBox(textLWP_YSMAX);
    inputs[20] = StringFromTextBox(textLWP_DTOPT);
    inputs[21] = StringFromTextBox(textLWP_DTGRR);
    inputs[22] = StringFromTextBox(textLWP_RINS);
    inputs[23] = StringFromTextBox(textLWP_VINS);
    inputs[24] = StringFromTextBox(textLWP_GAMINS);

    switch (comboLWP_LOT->GetSelection())
    {
    case 0:
        inputs[25] = "GMTLOR";
        break;
    case 1:
        inputs[25] = "OFFSET";
        break;
    case 2:
        inputs[25] = "RSTAR";
        break;
    case 3:
        inputs[25] = "PSTAR";
        break;
    case 4:
        inputs[25] = "TPLANE";
        break;
    case 5:
        inputs[25] = "TYAW";
        break;
    default:
        return;
    }

    inputs[26] = StringFromTextBox(textLWP_GMTLOR);
    inputs[27] = StringFromTextBox(textLWP_OFFSET);
    inputs[28] = StringFromTextBox(textLWP_BIAS);
    inputs[29] = StringFromTextBox(textLWP_TPLANE);
    inputs[30] = StringFromTextBox(textLWP_TRANS);

    switch (comboLWP_INSCO->GetSelection())
    {
    case 0:
        inputs[31] = "VGAMR";
        break;
    case 1:
        inputs[31] = "DHGAMR";
        break;
    case 2:
        inputs[31] = "HGAMR";
        break;
    default:
        return;
    }

    inputs[32] = StringFromTextBox(textLWP_DHW);
    inputs[33] = StringFromTextBox(textLWP_DU);
    inputs[34] = StringFromTextBox(textLWP_ANOM);

    switch (comboLWP_DELNOF->GetSelection())
    {
    case 0:
        inputs[35] = "IN";
        break;
    case 1:
        inputs[35] = "CO";
        break;
    default:
        return;
    }

    inputs[36] = StringFromTextBox(textLWP_DELNO);
    
    switch (comboLWP_NEGTIV->GetSelection())
    {
    case 0:
        inputs[37] = "PO";
        break;
    case 1:
        inputs[37] = "NE";
        break;
    case 2:
        inputs[37] = "LO";
        break;
    default:
        return;
    }

    inputs[38] = StringFromTextBox(textLWP_WRAP);

    std::vector<std::vector<std::string>> data;

    int err = core->RunLWP(inputs, data);

    if (err)
    {
        wxString Output;
        Output << "LWP error code " << err;

        SetStatusText(Output);
        return;
    }

    SetStatusText("LWP success!");

    FDSOutputDialog* dialog;

    for (unsigned int i = 0; i < data.size(); i++)
    {
        dialog = new FDSOutputDialog("Output", data[i]);
        dialog->Show();
    }

    // Save actual liftoff time
    double GMTLO = core->GetLWP_GMTLO();

    char Buffer[128];
    OrbMech::GMT2String4(Buffer, GMTLO);
    textLWP_Actual_GMTLO->Clear();
    textLWP_Actual_GMTLO->AppendText(Buffer);
}

void FDSFrame::OnButtonLWPSaveStateVector(wxCommandEvent& event)
{
    if (core->IsInitialized() == false)
    {
        SetStatusText("Date not initialized!");
        return;
    }

    wxTextEntryDialog dialog(this, "Enter name for the file:", "LWP chaser state vector file", "LWP.txt", wxOK | wxCANCEL);

    if (dialog.ShowModal() == wxID_OK)
    {
        wxString str = dialog.GetValue();
        wxString project = textProjectFile->GetLineText(0);
        wxString path = "Projects/" + project + "/State Vectors/";
        wxString filepath = path + str;

        //Save file
        wxTextFile file(filepath);

        if (file.Exists())
        {
            wxMessageDialog dialog(NULL, wxT("The file already exists. Overwrite?"),
                wxT("Save file"),
                wxNO_DEFAULT | wxYES_NO);

            switch (dialog.ShowModal())
            {
            case wxID_YES:
                break;
            default:
                return;
            }
        }
        else
        {
            file.Create();
        }
        file.Close();

        core->SaveLWPStateVector(str.ToStdString());
    }
}

void FDSFrame::OnButtonOMPGenerate(wxCommandEvent& event)
{
    //Calculate OMP

    std::string inputs[5];

    inputs[0] = StringFromTextBox(textProjectFile);
    inputs[1] = StringFromTextBox(textOMP_GMTLO);
    inputs[2] = StringFromTextBox(textOMP_Chaser);
    inputs[3] = StringFromTextBox(textOMP_Target);
    inputs[4] = StringFromTextBox(textOMP_MCT);

    std::vector<std::vector<std::string>> data;

    SetStatusText("Busy...");


    std::string errormessage;

    bool err = core->RunOMP(inputs, errormessage, data);

    if (err)
    {
        wxString Output;
        Output << errormessage;

        SetStatusText(Output);
        return;
    }

    SetStatusText("OMP success!");

    FDSOutputDialog* dialog;

    for (unsigned int i = 0; i < data.size(); i++)
    {
        dialog = new FDSOutputDialog("Output", data[i]);
        dialog->Show();
    }
}

void FDSFrame::OnButtonSkylabLWPGenerate(wxCommandEvent& event)
{
    std::string inputs[28];

    inputs[0] = StringFromTextBox(textProjectFile);
    inputs[1] = StringFromTextBox(textSkylabLWP_TargetVector);
    inputs[2] = StringFromTextBox(textSkylabLWP_CKFactor);
    inputs[3] = StringFromTextBox(textSkylabLWP_CArea);
    inputs[4] = StringFromTextBox(textSkylabLWP_CWHT);

    switch (comboSkylabLWP_NS->GetSelection())
    {
    case 0:
        inputs[5] = "NO";
        break;
    case 1:
        inputs[5] = "SO";
        break;
    default:
        return;
    }

    inputs[6] = StringFromTextBox(textSkylabLWP_DAY);
    inputs[7] = StringFromTextBox(textSkylabLWP_LATLS);
    inputs[8] = StringFromTextBox(textSkylabLWP_LONGLS);
    inputs[9] = StringFromTextBox(textSkylabLWP_PFT);
    inputs[10] = StringFromTextBox(textSkylabLWP_PFA);
    inputs[11] = StringFromTextBox(textSkylabLWP_YSMAX);
    inputs[12] = StringFromTextBox(textSkylabLWP_DTOPT);
    inputs[13] = StringFromTextBox(textSkylabLWP_RINS);
    inputs[14] = StringFromTextBox(textSkylabLWP_VINS);
    inputs[15] = StringFromTextBox(textSkylabLWP_GAMINS);

    switch (comboSkylabLWP_NEGTIV->GetSelection())
    {
    case 0:
        inputs[16] = "PO";
        break;
    case 1:
        inputs[16] = "NE";
        break;
    case 2:
        inputs[16] = "LO";
        break;
    default:
        return;
    }

    inputs[17] = StringFromTextBox(textSkylabLWP_WRAP);
    inputs[18] = StringFromTextBox(textSkylabLWP_DTOPEN);
    inputs[19] = StringFromTextBox(textSkylabLWP_DTCLOSE);

    switch (comboSkylabLWP_IR->GetSelection())
    {
    case 0:
        inputs[20] = "LW";
        break;
    case 1:
        inputs[20] = "LT";
        break;
    default:
        return;
    }

    inputs[21] = StringFromTextBox(textSkylabLWP_TLO);
    inputs[22] = StringFromTextBox(textSkylabLWP_MI);
    inputs[23] = StringFromTextBox(textSkylabLWP_MF);
    inputs[24] = StringFromTextBox(textSkylabLWP_DTSEP);
    inputs[25] = StringFromTextBox(textSkylabLWP_DVSEP);
    inputs[26] = StringFromTextBox(textSkylabLWP_DVWO);
    inputs[27] = StringFromTextBox(textSkylabLWP_DVWC);

    std::vector<std::vector<std::string>> data;

    SetStatusText("Busy...");

    int err = core->RunSkylabLWP(inputs, data);

    if (err)
    {
        wxString Output;
        Output << "Skylab LWP error code " << err;

        SetStatusText(Output);
        return;
    }

    SetStatusText("Skylab LWP success!");

    FDSOutputDialog* dialog;

    for (unsigned int i = 0; i < data.size(); i++)
    {
        dialog = new FDSOutputDialog("Output", data[i]);
        dialog->Show();
    }
}

void FDSFrame::OnButtonLWP_SIB_Presets(wxCommandEvent& event)
{
    textLWPCArea->ChangeValue("129.4");
    textLWPCWHT->ChangeValue("31000.0");
    textLWP_PFT->ChangeValue("600.0");
    textLWP_PFA->ChangeValue("18.8");
    textLWP_YSMAX->ChangeValue("5.0");
    textLWP_DTOPT->ChangeValue("360.0");
    textLWP_DTGRR->ChangeValue("17.0");
    textLWP_RINS->ChangeValue("21417976.0");
    textLWP_VINS->ChangeValue("25705.8");
    textLWP_GAMINS->ChangeValue("0.0");
}

void FDSFrame::OnButtonLWP_Shuttle_Presets(wxCommandEvent& event)
{
    textLWPCArea->ChangeValue("1301.0");
    textLWPCWHT->ChangeValue("255807.0");
    textLWP_PFT->ChangeValue("519.0");
    textLWP_PFA->ChangeValue("14.4");
    textLWP_YSMAX->ChangeValue("14.0");
    textLWP_DTOPT->ChangeValue("340.0");
    textLWP_DTGRR->ChangeValue("0.0");
    textLWP_RINS->ChangeValue("21241700.0");
    textLWP_VINS->ChangeValue("25928.0");
    textLWP_GAMINS->ChangeValue("0.6");
}

void SaveTextCtrl(wxFileConfig* config, wxTextCtrl *ctrl, wxString str)
{
    config->Write(str, ctrl->GetLineText(0));
}

void LoadTextCtrl(wxFileConfig* config, wxTextCtrl* ctrl, wxString str)
{
    wxString strtemp;
    if (config->Read(str, &strtemp)) ctrl->ChangeValue(strtemp);
}

void SaveChoiceBox(wxFileConfig* config, wxChoice* ctrl, wxString str)
{
    config->Write(str, ctrl->GetSelection());
}

void LoadChoiceBox(wxFileConfig* config, wxChoice* ctrl, wxString str)
{
    int inttemp;
    if (config->Read(str, &inttemp)) ctrl->SetSelection(inttemp);
}

void SaveCheckBox(wxFileConfig* config, wxCheckBox* ctrl, wxString str)
{
    config->Write(str, ctrl->GetValue());
}

void LoadCheckBox(wxFileConfig* config, wxCheckBox* ctrl, wxString str)
{
    bool bTemp;
    if (config->Read(str, &bTemp)) ctrl->SetValue(bTemp);
}

void FDSFrame::OnButton_Save(wxCommandEvent& event)
{
    wxString project = textProjectFile->GetLineText(0);
    wxString folder = "Projects/" + project;
    wxString file = folder + "/" + project + ".ini";

    //Check if folder needs to be created
    if (CreateDirectory(folder.c_str(), NULL))
    {
        if (ERROR_ALREADY_EXISTS != GetLastError())
        {
            CreateDirectory((folder + "/Outputs").c_str(), NULL);
            CreateDirectory((folder + "/State Vectors").c_str(), NULL);
        }
    }
    else
    {
        // Failed to create directory.
        SetStatusText("Failed to create directory!");
    }

    if (wxFile::Exists(file))
    {
        wxMessageDialog dialog(NULL, wxT("The save file already exists. Overwrite?"),
            wxT("Save file"),
            wxNO_DEFAULT | wxYES_NO);

        switch (dialog.ShowModal())
        {
        case wxID_YES:
            break;
        default:
            return;
        }
    }

    wxFileConfig* config = new wxFileConfig("FDS", wxEmptyString, file, wxEmptyString, wxCONFIG_USE_LOCAL_FILE | wxCONFIG_USE_RELATIVE_PATH);

    for (unsigned i = 0; i < textCtrlData.size(); i++)
    {
        SaveTextCtrl(config, textCtrlData[i].text, textCtrlData[i].name);
    }
    for (unsigned i = 0; i < choiceData.size(); i++)
    {
        SaveChoiceBox(config, choiceData[i].choice, choiceData[i].name);
    }
    for (unsigned i = 0; i < checkBoxData.size(); i++)
    {
        SaveCheckBox(config, checkBoxData[i].checkBox, checkBoxData[i].name);
    }

    delete config;

    SetStatusText("Save successful!");
}

void FDSFrame::OnButton_Load(wxCommandEvent& event)
{
    wxString project = textProjectFile->GetLineText(0);
    wxString file = "Projects/" + project + "/" + project + ".ini";

    wxFileConfig* config = new wxFileConfig("FDS", wxEmptyString, file, wxEmptyString, wxCONFIG_USE_LOCAL_FILE | wxCONFIG_USE_RELATIVE_PATH);

    if (!config->HasGroup("Config"))
    {
        SetStatusText("Project does not exist!");
        return;
    }

    for (unsigned i = 0; i < textCtrlData.size(); i++)
    {
        LoadTextCtrl(config, textCtrlData[i].text, textCtrlData[i].name);
    }
    for (unsigned i = 0; i < choiceData.size(); i++)
    {
        LoadChoiceBox(config, choiceData[i].choice, choiceData[i].name);
    }
    for (unsigned i = 0; i < checkBoxData.size(); i++)
    {
        LoadCheckBox(config, checkBoxData[i].checkBox, checkBoxData[i].name);
    }

    delete config;

    if (SetConstants())
    {
        SetStatusText("Project loaded, but initialization failed!");
    }
    else
    {
        SetStatusText("Project loaded successfully!");
    }
}

void FDSFrame::OnPageChanged(wxBookCtrlEvent& event)
{
   int val = event.GetSelection();

   if (val == 5)
   {
       ReloadStateVectorPage();
   }
}

void FDSFrame::ReloadStateVectorPage()
{
    //State Vector page. Load list of state vectors
    StateVectorList->DeleteAllItems();

    wxString project = textProjectFile->GetLineText(0);
    wxString path = "Projects/" + project + "/State Vectors/";

    wxDir dir(path);

    if (!dir.IsOpened())
    {
        //Folder doesn't exist
        return;
    }

    wxListItem item;

    wxString filename;
    wxString filespec = wxT("*.txt");
    int flags = wxDIR_FILES;

    bool cont = dir.GetFirst(&filename, filespec, flags);

    int i = 0;

    while (cont)
    {
        //Process
        item.SetText(filename);
        //StateVectorList->InsertItem(item);

        StateVectorList->InsertItem(i, wxString::Format(filename, i));

        cont = dir.GetNext(&filename);
        i++;
    }
}

void FDSFrame::StateVectorSelected(wxListEvent& event)
{
    wxListItem item = event.GetItem();

    wxString str = item.GetText();

    wxString project = textProjectFile->GetLineText(0);
    wxString path = "Projects/" + project + "/State Vectors/";

    wxString filepath = path + str;

    if (!wxFile::Exists(filepath)) return;

    wxTextFile file(filepath);
    if (!file.Open())
        return;

    // Set file name into text box for it
    textStateVectorFileName->ChangeValue(str);
    // Parse state vector file data
    ParseStateVectorFile(&file);
    // Update orbit data
    UpdateOrbitData();

    file.Close();
}

void FDSFrame::OnButton_StateVector_Check(wxCommandEvent& event)
{
    UpdateOrbitData();
}

void FDSFrame::OnButton_StateVector_Save(wxCommandEvent& event)
{
    wxString project = textProjectFile->GetLineText(0);
    wxString filename = textStateVectorFileName->GetLineText(0);

    if (project == "" || filename == "") return;

    wxString filepath = "Projects/" + project + "/State Vectors/" + filename;

    wxTextFile file(filepath);
    if (!file.Open())
        return;

    file.Clear();

    file.AddLine(comboStateVectorCoordinateSystem->GetStringSelection());
    for (int i = 0; i < textStateVectorData->GetNumberOfLines(); i++)
    {
        file.AddLine(textStateVectorData->GetLineText(i));
    }
    file.AddLine(textStateVectorWeight->GetLineText(0));
    file.AddLine(textStateVectorArea->GetLineText(0));
    file.AddLine(textStateVectorKFactor->GetLineText(0));

    file.Write();
    file.Close();
}

void FDSFrame::OnButton_StateVector_New(wxCommandEvent& event)
{
    wxTextEntryDialog dialog(this,
        wxT("Enter name of the state vector file:"),
        wxT("State Vector File"),
        wxT("Test.txt"),
        wxOK | wxCANCEL);

    if (dialog.ShowModal() == wxID_OK)
    {
        wxString filename = dialog.GetValue();
        wxString project = textProjectFile->GetLineText(0);
        wxString filepath = "Projects/" + project + "/State Vectors/" + filename;

        wxTextFile file(filepath);

        if (file.Exists())
        {
            wxMessageBox(filename, wxT("File already exists!"));
            return;
        }

        file.Create();

        if (!file.Open())
            return;

        //Write some defaults
        file.AddLine("TEG");
        file.AddLine("6700000.0 0.0 0.0");
        file.AddLine("0.0 7700.0 0.0");
        file.AddLine("0.0");
        file.AddLine("30000.0");
        file.AddLine("129.4");
        file.AddLine("1.0");
        file.Write();

        //Write to file name text box
        textStateVectorFileName->ChangeValue(filename);

        //Write to log text boxes
        ParseStateVectorFile(&file);

        file.Close();

        ReloadStateVectorPage();
        UpdateOrbitData();

        wxMessageBox(filename, wxT("File created!"));
    }
}

int FDSFrame::ParseStateVectorFile(wxTextFile* file)
{
    // State vector formats have 6 or 7 lines
    if (file->GetLineCount() < 6) return 1;

    wxString line, sv[6];
    int coord = 0;

    // Line 1: Coordinate system
    line = file->GetFirstLine();
    if (line == "TEG")
    {
        coord = 1;
    }
    else if (line == "J2000")
    {
        coord = 2;
    }
    else if (line == "M50")
    {
        coord = 3;
    }
    else if (line == "TLE")
    {
        coord = 4;
    }
    else return 2;

    if (coord < 4)
    {
        // Line 2-4: State Vector
        sv[0] = file->GetNextLine();
        sv[1] = file->GetNextLine();
        sv[2] = file->GetNextLine();
        if (file->GetLineCount() < 7) return 1;
    }
    else
    {
        // Line 2-3: State Vector
        sv[0] = file->GetNextLine();
        sv[1] = file->GetNextLine();
    }
    // Weight
    sv[3] = file->GetNextLine();
    // Area
    sv[4] = file->GetNextLine();
    // K-Factor
    sv[5] = file->GetNextLine();

    // Output
    comboStateVectorCoordinateSystem->SetSelection(coord - 1);
    textStateVectorData->Clear();
    textStateVectorData->AppendText(sv[0]);
    textStateVectorData->AppendText("\n" + sv[1]);
    if (coord < 4) textStateVectorData->AppendText("\n" + sv[2]);
    textStateVectorWeight->Clear();
    textStateVectorWeight->AppendText(sv[3]);
    textStateVectorArea->Clear();
    textStateVectorArea->AppendText(sv[4]);
    textStateVectorKFactor->Clear();
    textStateVectorKFactor->AppendText(sv[5]);
    return 0;
}

void FDSFrame::UpdateOrbitData()
{
    wxString str;
    int coord, lines, required_lines;

    textStateVectorOrbitData->Clear();

    lines = 4 + textStateVectorData->GetNumberOfLines();

    //Coordinate system
    str = comboStateVectorCoordinateSystem->GetStringSelection();

    coord = 0;
    required_lines = 0;

    if (str == "TEG")
    {
        coord = 1;
        required_lines = 7;
    }
    else if (str == "J2000")
    {
        coord = 2;
        required_lines = 7;
    }
    else if (str == "M50")
    {
        coord = 3;
        required_lines = 7;
    }
    else if (str == "TLE")
    {
        coord = 4;
        required_lines = 6;
    }

    if (coord == 0)
    {
        textStateVectorOrbitData->Clear();
        textStateVectorOrbitData->AppendText("Unsupported coordinate system!");
        return;
    }

    if (lines < required_lines)
    {
        textStateVectorOrbitData->Clear();
        textStateVectorOrbitData->AppendText("Wrong number of lines in file!");
        return;
    }

    textStateVectorOrbitData->AppendText("Coordinate system: " + str + "\n");

    if (coord <= 3)
    {
        //TEG, J2000 and M50
        str = textStateVectorData->GetLineText(0);
        textStateVectorOrbitData->AppendText("Position vector: " + str + "\n");

        str = textStateVectorData->GetLineText(1);
        textStateVectorOrbitData->AppendText("Velocity vector: " + str + "\n");

        str = textStateVectorData->GetLineText(2);
        if (coord == 1)
        {
            //TEG
            textStateVectorOrbitData->AppendText("GMT: " + str + "\n");
        }
        else
        {
            //MJD
            textStateVectorOrbitData->AppendText("MJD: " + str + "\n");
        }
    }
    else
    {
        //TLE
        str = textStateVectorData->GetLineText(0);
        textStateVectorOrbitData->AppendText("Line 1: " + str + "\n");

        str = textStateVectorData->GetLineText(1);
        textStateVectorOrbitData->AppendText("Line 2: " + str + "\n");
    }

    //Weight
    str = textStateVectorWeight->GetLineText(0);
    textStateVectorOrbitData->AppendText("Weight: " + str + " lbs\n");

    //Area
    str = textStateVectorArea->GetLineText(0);
    textStateVectorOrbitData->AppendText("Area: " + str + " square feet\n");

    //K-Factor
    str = textStateVectorKFactor->GetLineText(0);
    textStateVectorOrbitData->AppendText("Drag multiplier: " + str + "\n");

    str = textStateVectorFileName->GetLineText(0);

    double age;

    std::string data[2];

    data[0] = StringFromTextBox(textProjectFile);
    data[1] = std::string(str.mb_str());

    if (core->AgeOfStateVector(data, age)) return;

    str = wxString::Format(wxT("%.0lf"), age / 3600.0);

    textStateVectorOrbitData->AppendText("Time from midnight to state vector is " +  str + " hours\n");
}

int FDSFrame::SetConstants()
{
    if (core == NULL)
    {
        core = new Core();
    }

    double EDT;
    int Year, Month, Day;

    textDayOfYear->ChangeValue("Invalid date!");

    if (GetInteger(textYear, "Year", &Year)) return 1;
    if (GetInteger(textMonth, "Month", &Month)) return 1;
    if (GetInteger(textDay, "Day", &Day)) return 1;
    if (GetDouble(textDETUTC, "EDT", &EDT)) return 1;

    if (core->SetConstants(comboWorld->GetSelection(), Year, Month, Day, EDT))
    {
        SetStatusText("Error during initialization!");
        return 1;
    }

    int DOY = core->GetDayOfYear();

    wxString mystring = wxString::Format(wxT("%i"), DOY);
    textDayOfYear->ChangeValue(mystring);

    SetStatusText("Initialization successful!");

    return 0;
}

int FDSFrame::GetInteger(wxTextCtrl* text, const wxString& name, int* val)
{
    if (text->GetValue().ToInt(val) == false)
    {
        SetStatusText(name + " has invalid format!");
        return 1;
    }
    return 0;
}

int FDSFrame::GetDouble(wxTextCtrl* text, const wxString& name, double* val)
{
    if (text->GetValue().ToDouble(val) == false)
    {
        SetStatusText(name + " has invalid format!");
        return 1;
    }
    return 0;
}

int FDSFrame::GetDDDHHMMSS(wxTextCtrl* text, const wxString& name, double* val)
{
    wxStringTokenizer tkz(text->GetValue(), ":");

    if (tkz.CountTokens() != 4)
    {
        SetStatusText(name + " has invalid format!");
        return 1;
    }

    int itemp, days, hours, minutes, i;
    double dtemp, seconds, sgn;
    bool err;

    sgn = 1.0;
    itemp = days = hours = minutes = i = 0;
    dtemp = seconds = 0.0;
    err = false;

    while (tkz.HasMoreTokens())
    {
        wxString token = tkz.GetNextToken();

        if (i < 3)
        {
            err = !token.ToInt(&itemp);
        }
        else
        {
            err = !token.ToDouble(&dtemp);
        }

        if (err)
        {
            SetStatusText(name + " has invalid format!");
            return 1;
        }

        switch (i)
        {
        case 0:
            days = abs(itemp);
            if (token[0] == '-') sgn = -1.0;
            break;
        case 1:
            hours = itemp;
            break;
        case 2:
            minutes = itemp;
            break;
        case 3:
            seconds = dtemp;
            break;
        }

        i++;
    }

    *val = sgn * (seconds + (double)(60 * (minutes + 60 * (hours + 24 * days))));
    return 0;
}

int FDSFrame::GetHHMMSS(wxTextCtrl* text, const wxString& name, double* val)
{
    double sgn, secs;
    int vals[2];

    if (ParseTime(text, 3, sgn, vals, &secs))
    {
        SetStatusText(name + " has invalid format!");
        return 1;
    }

    *val = sgn * (secs + (double)(60 * (vals[1] + 60 * vals[0])));
    return 0;
}

int FDSFrame::ParseTime(wxTextCtrl* text, size_t size, double& sgn, int* vals, double* secs)
{
    wxStringTokenizer tkz(text->GetValue(), ":");

    if (tkz.CountTokens() != size) return 1;

    wxArrayString strings;

    strings.resize(size);

    for (size_t i = 0; i < size; i++)
    {
        strings[i] = tkz.GetNextToken();
    }

    // Convert to int and double
    for (size_t i = 0; i < size; i++)
    {
        if ((i + 1) < size)
        {
            // Not last
            if (strings[i].ToInt(&vals[i]) == false) return 1;
        }
        else
        {
            // Last
            if (strings[i].ToDouble(secs) == false) return 1;
        }

        if (i == 0)
        {
            // First
            if (strings[0][0] == '-')
            {
                sgn = -1.0;
                vals[0] = abs(vals[0]);
            }
            else
            {
                sgn = 1.0;
            }
        }
    }

    return 0;
}

void FDSFrame::CalculateDayOfYear(wxCommandEvent& event)
{
    SetConstants();
}

void FDSFrame::CalculateEphemerisDT(wxCommandEvent& event)
{
    double EDT;
    int err;

    err = core->GetEphemerisDT(EDT);

    if (err)
    {
        SetStatusText("Date out of bounds!");
        return;
    }

    wxString mystring = wxString::Format("%.3f", EDT);

    textDETUTC->Clear();
    textDETUTC->AppendText(mystring);

    SetConstants();
}

void FDSFrame::OnButton_LWP_LVDC_Export(wxCommandEvent& event)
{
    if (core->IsInitialized() == false)
    {
        SetStatusText("Date not initialized!");
        return;
    }

    if (core->LWPExportForLVDC())
    {
        SetStatusText("Error exporting LVDC data!");
        return;
    }

    SetStatusText("LVDC data successfully exported!");
}

void FDSFrame::OnButton_View_MCT(wxCommandEvent& event)
{
    wxString str = textOMP_MCT->GetLineText(0);
    wxString project = textProjectFile->GetLineText(0);
    wxString path = "Projects/" + project + "/";
    wxString filepath = path + str;

    if (!wxFile::Exists(filepath))
    {
        SetStatusText("MCT file does not exist!");
        return;
    }

    ViewMCT(filepath);
}

void FDSFrame::ViewMCT(const wxString& filepath)
{
    wxTextFile file(filepath);
    if (!file.Open())
    {
        SetStatusText("MCT file could not be opened!");
        return;
    }

    wxString arr, token;
    wxStringTokenizer tkz;
    unsigned i, j;

    // Delete all old lines
    for (i = 0; i < 40; i++)
    {
        for (j = 0; j < 13; j++)
        {
            textOMP_MCT_Editor->SetCellValue(i, j, "");
        }
    }

    for (i = 0; i < file.GetLineCount(); i++)
    {
        tkz.SetString(file[i], ";");

        j = 0;
        while (tkz.HasMoreTokens())
        {
            token = tkz.GetNextToken();

            textOMP_MCT_Editor->SetCellValue(i, j, token);

            j++;
            if (j >= 9) break;
        }
        if (i >= 40) break;
    }

    file.Close();
}

void FDSFrame::OnButton_Save_MCT(wxCommandEvent& event)
{
    std::string line;
    std::vector<std::string> array;

    unsigned i, j;

    for (i = 0; i < 40; i++)
    {
        if (textOMP_MCT_Editor->GetCellValue(i, 0) == "") break;

        line.clear();
        for (j = 0; j < 13; j++)
        {
            if (textOMP_MCT_Editor->GetCellValue(i, j) != "")
            {
                if (j != 0)
                {
                    line += ";";
                }
                line += textOMP_MCT_Editor->GetCellValue(i, j);
            }
            else break;
        }
        array.push_back(line);
    }

    wxString str = textOMP_MCT->GetLineText(0);
    wxString project = textProjectFile->GetLineText(0);
    wxString path = "Projects/" + project + "/";
    wxString filepath = path + str;

    //Save file
    wxTextFile file(filepath);

    if (file.Exists())
    {
        wxMessageDialog dialog(NULL, wxT("The file already exists. Overwrite?"),
            wxT("Save file"),
            wxNO_DEFAULT | wxYES_NO);

        switch (dialog.ShowModal())
        {
        case wxID_YES:
            break;
        default:
            return;
        }
    }
    else
    {
        file.Create();
    }

    //Write to file
   
    if (!file.Open())
        return;

    file.Clear();

    for (i = 0; i < array.size(); i++)
    {
        file.AddLine(array[i]);
    }

    file.Write();
    file.Close();

    SetStatusText("MCT file saved!");
}

void FDSFrame::OnButton_MCT_Load_Template(wxCommandEvent& event)
{
    wxString workingDir = wxGetCwd();
    wxString caption = "Choose a MCT template";
    wxString wildcard = "Text files (*.txt)|*.txt";
    wxString defaultDir = workingDir + "\\Templates\\OMP";
    wxString defaultFilename = wxEmptyString;

    wxFileDialog dialog(this, caption, defaultDir, defaultFilename, wildcard, wxFD_OPEN | wxFD_FILE_MUST_EXIST);

    if (dialog.ShowModal() != wxID_OK) return;

    wxString path = dialog.GetPath();

    ViewMCT(path);
}

void FDSFrame::OnButton_FDOMFD_Export(wxCommandEvent& event)
{
    if (core->IsInitialized() == false)
    {
        SetStatusText("Date not initialized!");
        return;
    }

    // Ask for Shuttle name input
    wxTextEntryDialog dialog1(this, "Enter the name of the Shuttle:", "Shuttle vessel name input", wxEmptyString, wxOK | wxCANCEL);
    if (dialog1.ShowModal() != wxID_OK)
    {
        return;
    }
    wxString ShuttleName = dialog1.GetValue();

    // Ask for Target name input
    wxTextEntryDialog dialog2(this, "Enter the name of the target:", "Target vessel name input", wxEmptyString, wxOK | wxCANCEL);
    if (dialog2.ShowModal() != wxID_OK)
    {
        return;
    }
    wxString TargetName = dialog2.GetValue();

    wxString str = textOMP_MCT->GetLineText(0);
    wxString project = textProjectFile->GetLineText(0);
    wxString path = "Projects/" + project + "/";
    wxString filepath = path + str;

    if (!wxFile::Exists(filepath))
    {
        SetStatusText("MCT file does not exist!");
        return;
    }

    std::vector<OMP::ManeuverConstraintsInput> ManeuverConstraintsInput;
    std::vector <OMP::ManeuverConstraints> tab_out;
    std::string errormessage;

    if (core->ReadMCTFile(filepath.ToStdString(), ManeuverConstraintsInput))
    {
        SetStatusText("Could not read MCT file!");
        return;
    }

    OrbMech::GlobalConstants globtemp;
    OrbMech::SessionConstants sesstemp;
    OMP::OrbitalManeuverProcessor omp(globtemp, sesstemp);

    if (omp.ParseManeuverConstraintsTable(ManeuverConstraintsInput, tab_out, errormessage))
    {
        wxString Output;
        Output << errormessage;

        SetStatusText(Output);
        return;
    }

    // Read GMTLO from OMP input
    double GMTLO, hh, mm, ss;
    if (GetHHMMSS(textOMP_GMTLO, "GMTLO", &GMTLO)) return;
    OrbMech::SS2HHMMSS(GMTLO, hh, mm, ss);

    std::string line;
    std::vector<std::string> array;

    array.push_back("LAUNCHDATE0 " + std::to_string(core->GetSessionConstants()->Year));
    array.push_back("LAUNCHDATE1 " + std::to_string(core->GetSessionConstants()->DayOfYear));
    array.push_back("LAUNCHDATE2 " + std::to_string((int)(hh)));
    array.push_back("LAUNCHDATE3 " + std::to_string((int)(mm)));
    array.push_back("LAUNCHDATE4 " + std::to_string(ss));
    array.push_back("SHUTTLE " + ShuttleName.ToStdString());
    array.push_back("TARGET " + TargetName.ToStdString());
    array.push_back("NONSPHERICAL 1");
    array.push_back("START_MCT");
    for (unsigned i = 0; i < tab_out.size(); i++)
    {
        line = tab_out[i].name + " " + std::to_string(tab_out[i].type) + " " + std::to_string(tab_out[i].threshold) + " " + std::to_string(tab_out[i].thresh_num);

        for (unsigned j = 0; j < 4; j++)
        {
            if (j < tab_out[i].secondaries.size())
            {
                line += " " + OMP::GetSecondaryName(tab_out[i].secondaries[j].type) + " " + std::to_string(tab_out[i].secondaries[j].value);
            }
            else
            {
                line += " NSEC 0.000000";
            }
        }

        array.push_back(line);
    }
    array.push_back("END_MCT");

    // Save to file
    // Filename
    str = project + ".txt";
    filepath = path + "Outputs/" + str;

    wxTextFile file(filepath);

    if (!file.Exists())
    {
        file.Create();
    }
    else
    {
        wxMessageDialog dialog(NULL, wxT("The file already exists. Overwrite?"),
            wxT("Save file"),
            wxNO_DEFAULT | wxYES_NO);

        switch (dialog.ShowModal())
        {
        case wxID_YES:
            break;
        default:
            return;
        }
    }

    if (!file.Open())
    {
        SetStatusText("Could not open file for export!");
        return;
    }

    file.Clear();

    for (unsigned i = 0; i < array.size(); i++)
    {
        file.AddLine(array[i]);
    }

    file.Write();
    file.Close();

    SetStatusText("File export successful!");
}

void FDSFrame::OnCombo_ShuttleLWP_Launchpad(wxCommandEvent& event)
{
    switch (comboShuttleLWP_Launchpad->GetSelection())
    {
    case 0: // LC-39A
        textShuttleLWP_LATLS->Clear();
        textShuttleLWP_LATLS->AppendText("28.608385");
        textShuttleLWP_LONGLS->Clear();
        textShuttleLWP_LONGLS->AppendText("279.395928");
        break;
    case 1: // LC-39B
        textShuttleLWP_LATLS->Clear();
        textShuttleLWP_LATLS->AppendText("28.627215");
        textShuttleLWP_LONGLS->Clear();
        textShuttleLWP_LONGLS->AppendText("279.379138");
        break;
    case 2: // SLC-6
        textShuttleLWP_LATLS->Clear();
        textShuttleLWP_LATLS->AppendText("34.5808470");
        textShuttleLWP_LONGLS->Clear();
        textShuttleLWP_LONGLS->AppendText("239.37405");
        break;
    }
}

void FDSFrame::OnButton_ShuttleLWP_LWP_Execute(wxCommandEvent& event)
{
    ShuttleLWP_Execute(true);
}

void FDSFrame::OnButton_ShuttleLWP_LTP_Execute(wxCommandEvent& event)
{
    ShuttleLWP_Execute(false);
}

void FDSFrame::ShuttleLWP_Execute(bool IsLW)
{
    std::string strInputs[20];
    double dInputs[40];
    int iInputs[20], RefSet;

    if (IsLW)
    {
        if (GetInteger(textShuttleLWP_LW_Ref_Set_ID, "Ref Set", &RefSet)) return;
    }
    else
    {
        if (GetInteger(textShuttleLWP_LT_Ref_Set_ID, "Ref Set", &RefSet)) return;
    }
    
    if (RefSet < 1 || RefSet > 4)
    {
        SetStatusText("Invalid ref set number");
        return;
    }
    RefSet -= 1;

    strInputs[0] = StringFromTextBox(textProjectFile);
    strInputs[1] = StringFromTextBox(textShuttleLWPTargetVector);

    if (GetDouble(textShuttleLWP_YSMAX, "YSMAX", &dInputs[0])) return;
    if (GetDouble(textShuttleLWP_CD, "CD", &dInputs[1])) return;
    if (GetDouble(textShuttleLWP_Area, "Area", &dInputs[2])) return;
    if (GetDouble(textShuttleLWP_Weight, "Weight", &dInputs[3])) return;
    if (GetDouble(textShuttleLWP_LATLS, "LATLS", &dInputs[4])) return;
    if (GetDouble(textShuttleLWP_LONGLS, "LONGLS", &dInputs[5])) return;
    if (GetDouble(text_ShuttleLWP_PFA, "PFA", &dInputs[6])) return;
    if (GetDDDHHMMSS(text_ShuttleLWP_PFT, "PFT", &dInputs[7])) return;
    if (GetDouble(text_ShuttleLWP_RAD, "RAD", &dInputs[8])) return;
    if (GetDouble(text_ShuttleLWP_VEL, "VEL", &dInputs[9])) return;
    if (GetDouble(text_ShuttleLWP_FPA, "FPA", &dInputs[10])) return;
    if (GetDDDHHMMSS(text_ShuttleLWP_OPT, "OPT", &dInputs[11])) return;
    if (GetDouble(textShuttleLWP_ET_Area, "ET Area", &dInputs[12])) return;
    if (GetDouble(textShuttleLWP_ET_CD, "ET CD", &dInputs[13])) return;
    if (GetDouble(textShuttleLWP_ET_WT, "ET Weight", &dInputs[14])) return;
    if (GetDDDHHMMSS(textShuttleLWP_ET_Sep_DTIG, "ET Sep DTIG", &dInputs[15])) return;
    if (GetDouble(textShuttleLWP_ET_Sep_DVX, "ET Sep DVX", &dInputs[16])) return;
    if (GetDouble(textShuttleLWP_ET_Sep_DVY, "ET Sep DVY", &dInputs[17])) return;
    if (GetDouble(textShuttleLWP_ET_Sep_DVZ, "ET Sep DVZ", &dInputs[18])) return;
    if (GetDDDHHMMSS(text_ShuttleLWP_DTO, "DTO", &dInputs[19])) return;
    if (GetDDDHHMMSS(text_ShuttleLWP_DTC, "DTC", &dInputs[20])) return;

    if (GetDDDHHMMSS(textShuttleLWP_OMS1_DTIG[RefSet], "OMS-1 DTIG", &dInputs[21])) return;
    if (GetDouble(textShuttleLWP_OMS1_C1[RefSet], "OMS-1 C1", &dInputs[22])) return;
    if (GetDouble(textShuttleLWP_OMS1_C2[RefSet], "OMS-1 C2", &dInputs[23])) return;
    if (GetDouble(textShuttleLWP_OMS1_HT[RefSet], "OMS-1 HT", &dInputs[24])) return;
    if (GetDouble(textShuttleLWP_OMS1_THETAT[RefSet], "OMS-1 THETAT", &dInputs[25])) return;

    if (GetDDDHHMMSS(textShuttleLWP_OMS2_DTIG[RefSet], "OMS-2 DTIG", &dInputs[26])) return;
    if (GetDouble(textShuttleLWP_OMS2_C1[RefSet], "OMS-2 C1", &dInputs[27])) return;
    if (GetDouble(textShuttleLWP_OMS2_C2[RefSet], "OMS-2 C2", &dInputs[28])) return;
    if (GetDouble(textShuttleLWP_OMS2_HT[RefSet], "OMS-2 HT", &dInputs[29])) return;
    if (GetDouble(textShuttleLWP_OMS2_THETAT[RefSet], "OMS-2 THETAT", &dInputs[30])) return;

    if (GetDDDHHMMSS(textShuttleLWP_MPS_Dump_DTIG, "MPS Dump DTIG", &dInputs[34])) return;
    if (GetDouble(textShuttleLWP_MPS_Dump_DVX, "MPS Dump DVX", &dInputs[35])) return;
    if (GetDouble(textShuttleLWP_MPS_Dump_DVY, "MPS Dump DVY", &dInputs[36])) return;
    if (GetDouble(textShuttleLWP_MPS_Dump_DVZ, "MPS Dump DVZ", &dInputs[37])) return;

    iInputs[0] = checkShuttleLWP_DI->IsChecked();
    iInputs[1] = checkShuttleLWP_LAUNCH_AZ_DIR_FLAG->IsChecked();
    iInputs[2] = comboShuttleLWP_PhaseCntrlFlag->GetSelection();
    if (GetInteger(textShuttleLWP_WRAP_FLAG, "WRAP FLAG", &iInputs[3])) return;
    iInputs[6] = checkShuttleLWP_Drag->IsChecked();

    if (IsLW)
    {
        if (textShuttleLWP_Desired_Phase_1->IsEmpty())
        {
            iInputs[4] = 0;
        }
        else
        {
            iInputs[4] = 1;
            if (GetDouble(textShuttleLWP_Desired_Phase_1, "PHASE1", &dInputs[31])) return;
        }

        if (textShuttleLWP_Desired_Phase_2->IsEmpty())
        {
            iInputs[5] = 0;
        }
        else
        {
            iInputs[5] = 1;
            if (GetDouble(textShuttleLWP_Desired_Phase_2, "PHASE2", &dInputs[32])) return;
        }
    }
    else
    {
        if (GetHHMMSS(textShuttleLWP_LT_GMTLO, "GMTLO", &dInputs[33])) return;
    }

    std::vector<std::string> data;

    int err = core->RunShuttleLWP(IsLW, strInputs, dInputs, iInputs, data);

    if (err)
    {
        wxString Output;
        Output << "Shuttle LWP error code " << err;

        SetStatusText(Output);
        return;
    }

    SetStatusText("Shuttle LWP success!");

    if (IsLW)
    {
        for (unsigned i = 0; i < data.size(); i++)
        {
            textShuttleLWP_LW_Out[i]->Clear();
            textShuttleLWP_LW_Out[i]->AppendText(data[i]);
        }
    }
    else
    {
        for (unsigned i = 0; i < data.size(); i++)
        {
            textShuttleLWP_LT_Out[i]->Clear();
            textShuttleLWP_LT_Out[i]->AppendText(data[i]);
        }
    }
}

void FDSFrame::OnButton_ShuttleLWP_Export(wxCommandEvent& event)
{
    if (core->ShuttleLTPExport())
    {
        SetStatusText("Shuttle LWP export failed!");
        return;
    }

    SetStatusText("Shuttle LWP export success!");
}

void FDSFrame::OnButtonShuttleLWPSaveStateVector(wxCommandEvent& event)
{
    if (core->IsInitialized() == false)
    {
        SetStatusText("Date not initialized!");
        return;
    }

    wxTextEntryDialog dialog(this, "Enter name for the file:", "LTP post MPS dump state vector file", "LTP.txt", wxOK | wxCANCEL);

    if (dialog.ShowModal() == wxID_OK)
    {
        wxString str = dialog.GetValue();
        wxString project = textProjectFile->GetLineText(0);
        wxString path = "Projects/" + project + "/State Vectors/";
        wxString filepath = path + str;

        //Save file
        wxTextFile file(filepath);

        if (file.Exists())
        {
            wxMessageDialog dialog(NULL, wxT("The file already exists. Overwrite?"),
                wxT("Save file"),
                wxNO_DEFAULT | wxYES_NO);

            switch (dialog.ShowModal())
            {
            case wxID_YES:
                break;
            default:
                return;
            }
        }
        else
        {
            file.Create();
        }
        file.Close();

        core->SaveShuttleLWPStateVector(str.ToStdString());
    }
}

void FDSFrame::ChangeLoadSVPageLabels(wxCommandEvent& event)
{
    // Labels and tool tips
    if (comboLoadSV_ELSET->GetSelection() == 0)
    {
        // Cartesian

        staticLoadSV_Descriptions[1]->SetLabel("X");
        staticLoadSV_Descriptions[2]->SetLabel("Y");
        staticLoadSV_Descriptions[3]->SetLabel("Z");
        staticLoadSV_Descriptions[4]->SetLabel("XD");
        staticLoadSV_Descriptions[5]->SetLabel("YD");
        staticLoadSV_Descriptions[6]->SetLabel("ZD");
    }
    else if(comboLoadSV_ELSET->GetSelection() == 1)
    {
        // Orbital elements
        staticLoadSV_Descriptions[1]->SetLabel("A");
        textLoadSV_Array[1]->SetToolTip("Semimajor axis");
        staticLoadSV_Descriptions[2]->SetLabel("E");
        textLoadSV_Array[2]->SetToolTip("Eccentricity");
        staticLoadSV_Descriptions[3]->SetLabel("I");
        textLoadSV_Array[3]->SetToolTip("Inclination");
        staticLoadSV_Descriptions[4]->SetLabel("G");
        textLoadSV_Array[4]->SetToolTip("Argument of perigee");
        staticLoadSV_Descriptions[5]->SetLabel("H");
        textLoadSV_Array[5]->SetToolTip("Longitude of the ascending node");
        staticLoadSV_Descriptions[6]->SetLabel("L");
        textLoadSV_Array[6]->SetToolTip("Mean anomaly");
    }
    else if (comboLoadSV_ELSET->GetSelection() == 1)
    {
        // Spherical
        staticLoadSV_Descriptions[1]->SetLabel("RAD");
        textLoadSV_Array[1]->SetToolTip("Radius of the orbit");
        staticLoadSV_Descriptions[2]->SetLabel("VEL");
        textLoadSV_Array[2]->SetToolTip("Vehicle total velocity");
        staticLoadSV_Descriptions[3]->SetLabel("GAM");
        textLoadSV_Array[3]->SetToolTip("Flight path angle");
        staticLoadSV_Descriptions[4]->SetLabel("LAT");
        textLoadSV_Array[4]->SetToolTip("Latitude");
        staticLoadSV_Descriptions[5]->SetLabel("LNG");
        textLoadSV_Array[5]->SetToolTip("Longitude");
        staticLoadSV_Descriptions[6]->SetLabel("INC");
        textLoadSV_Array[6]->SetToolTip("Inclination (negative for descending)");
    }
    else
    {
        // Apsides
        staticLoadSV_Descriptions[1]->SetLabel("HA");
        textLoadSV_Array[1]->SetToolTip("Apogee altitude");
        staticLoadSV_Descriptions[2]->SetLabel("HP");
        textLoadSV_Array[2]->SetToolTip("Perigee altitude");
        staticLoadSV_Descriptions[3]->SetLabel("TA");
        textLoadSV_Array[3]->SetToolTip("True anomaly");
        staticLoadSV_Descriptions[4]->SetLabel("LAT");
        textLoadSV_Array[4]->SetToolTip("Latitude");
        staticLoadSV_Descriptions[5]->SetLabel("LNG");
        textLoadSV_Array[5]->SetToolTip("Longitude");
        staticLoadSV_Descriptions[6]->SetLabel("INC");
        textLoadSV_Array[6]->SetToolTip("Inclination (negative for descending)");
    }

    // Units
    std::string ANG_UNITS, DIST_UNITS, VEL_UNITS, MASS_UNITS, LENGTH_UNITS;

    if (comboLoadSV_ANGUN->GetSelection() == 0) ANG_UNITS = "deg";
    else ANG_UNITS = "rad";

    if (comboLoadSV_DSTUN->GetSelection() == 0) DIST_UNITS = "feet";
    else if (comboLoadSV_DSTUN->GetSelection() == 1) DIST_UNITS = "meters";
    else if (comboLoadSV_DSTUN->GetSelection() == 2) DIST_UNITS = "NM";
    else if (comboLoadSV_DSTUN->GetSelection() == 3) DIST_UNITS = "ER";
    else DIST_UNITS = "km";

    if (comboLoadSV_VELUN->GetSelection() == 0) VEL_UNITS = "ft/sec";
    else if (comboLoadSV_VELUN->GetSelection() == 1) VEL_UNITS = "meters/sec";
    else if (comboLoadSV_VELUN->GetSelection() == 2) VEL_UNITS = "NM/hour";
    else if (comboLoadSV_VELUN->GetSelection() == 3) VEL_UNITS = "ER/hour";
    else VEL_UNITS = "km/hour";

    if (comboLoadSV_MASUN->GetSelection() == 0) MASS_UNITS = "lbs";
    else if (comboLoadSV_MASUN->GetSelection() == 1) MASS_UNITS = "kg";
    else MASS_UNITS = "slugs";

    if (comboLoadSV_LENUN->GetSelection() == 0) LENGTH_UNITS = "sq ft";
    else if (comboLoadSV_LENUN->GetSelection() == 1) LENGTH_UNITS = "sq meters";
    else LENGTH_UNITS = "sq in";

    if (comboLoadSV_ELSET->GetSelection() == 0)
    {
        // Cartesian
        staticLoadSV_Units[1]->SetLabel(DIST_UNITS);
        staticLoadSV_Units[2]->SetLabel(DIST_UNITS);
        staticLoadSV_Units[3]->SetLabel(DIST_UNITS);
        staticLoadSV_Units[4]->SetLabel(VEL_UNITS);
        staticLoadSV_Units[5]->SetLabel(VEL_UNITS);
        staticLoadSV_Units[6]->SetLabel(VEL_UNITS);
    }
    else if (comboLoadSV_ELSET->GetSelection() == 1)
    {
        // Elements
        staticLoadSV_Units[1]->SetLabel(DIST_UNITS);
        staticLoadSV_Units[2]->SetLabel("nd");
        staticLoadSV_Units[3]->SetLabel(ANG_UNITS);
        staticLoadSV_Units[4]->SetLabel(ANG_UNITS);
        staticLoadSV_Units[5]->SetLabel(ANG_UNITS);
        staticLoadSV_Units[6]->SetLabel(ANG_UNITS);
    }
    else if (comboLoadSV_ELSET->GetSelection() == 2)
    {
        // Spherical
        staticLoadSV_Units[1]->SetLabel(DIST_UNITS);
        staticLoadSV_Units[2]->SetLabel(VEL_UNITS);
        staticLoadSV_Units[3]->SetLabel(ANG_UNITS);
        staticLoadSV_Units[4]->SetLabel(ANG_UNITS);
        staticLoadSV_Units[5]->SetLabel(ANG_UNITS);
        staticLoadSV_Units[6]->SetLabel(ANG_UNITS);
    }
    else
    {
        // Apsides
        staticLoadSV_Units[1]->SetLabel(DIST_UNITS);
        staticLoadSV_Units[2]->SetLabel(DIST_UNITS);
        staticLoadSV_Units[3]->SetLabel(ANG_UNITS);
        staticLoadSV_Units[4]->SetLabel(ANG_UNITS);
        staticLoadSV_Units[5]->SetLabel(ANG_UNITS);
        staticLoadSV_Units[6]->SetLabel(ANG_UNITS);
    }

    staticLoadSV_Units[8]->SetLabel(LENGTH_UNITS);
    staticLoadSV_Units[9]->SetLabel(MASS_UNITS);
}

void FDSFrame::OnButtonLoadSVConvert(wxCommandEvent& event)
{
    double dInputs[15];
    int iInputs[15];

    iInputs[0] = comboLoadSV_REFAX->GetSelection();
    iInputs[1] = comboLoadSV_ELSET->GetSelection();
    iInputs[2] = comboLoadSV_ANGUN->GetSelection();
    iInputs[3] = comboLoadSV_DSTUN->GetSelection();
    iInputs[4] = comboLoadSV_VELUN->GetSelection();
    iInputs[5] = comboLoadSV_MASUN->GetSelection();
    iInputs[6] = comboLoadSV_LENUN->GetSelection();

    if (GetHHMMSS(textLoadSV_Array[0], staticLoadSV_Descriptions[0]->GetLabel(), &dInputs[0])) return;
    for (int i = 1; i < 10; i++)
    {
        if (GetDouble(textLoadSV_Array[i], staticLoadSV_Descriptions[i]->GetLabel(), &dInputs[i])) return;
    }

    std::vector<std::string> data;

    int err = core->StateVectorConverter(iInputs, dInputs, data);

    if (err)
    {
        SetStatusText("Conversion error!");
        return;
    }

    textLoadSV_Output->Clear();

    for (unsigned i = 0; i < data.size(); i++)
    {
        if (i != 0) textLoadSV_Output->AppendText("\n");
        textLoadSV_Output->AppendText(data[i]);
    }

    SetStatusText("Conversion successful!");
}

void FDSFrame::OnButton_LoadSV_Save(wxCommandEvent& event)
{
    if (textLoadSV_Output->GetNumberOfLines() != 7)
    {
        SetStatusText("State vector data not valid!");
        return;
    }

    wxTextEntryDialog dialog(this, "Enter name for the file:", "LTP post MPS dump state vector file", "Test.txt", wxOK | wxCANCEL);

    if (dialog.ShowModal() != wxID_OK) return;

    wxString str = dialog.GetValue();
    wxString project = textProjectFile->GetLineText(0);
    wxString path = "Projects/" + project + "/State Vectors/";
    wxString filepath = path + str;

    //Save file
    wxTextFile file(filepath);

    if (file.Exists())
    {
        wxMessageDialog dialog(NULL, wxT("The file already exists. Overwrite?"),
            wxT("Save file"),
            wxNO_DEFAULT | wxYES_NO);

        switch (dialog.ShowModal())
        {
        case wxID_YES:
            break;
        default:
            return;
        }
    }
    else
    {
        file.Create();
    }

    file.Clear();

    for (int i = 0; i < textLoadSV_Output->GetNumberOfLines(); i++)
    {
        file.AddLine(textLoadSV_Output->GetLineText(i));
    }

    file.Write();
    file.Close();

    SetStatusText("State vector save successful!");
}

void FDSFrame::Add(wxTextCtrl* text, const wxString& config)
{
    wxTextCtrlData data;

    data.text = text;
    data.name = config;

    textCtrlData.push_back(data);
}

void FDSFrame::Add(wxChoice* choice, const wxString& config)
{
    wxChoiceData data;

    data.choice = choice;
    data.name = config;

    choiceData.push_back(data);
}

void FDSFrame::Add(wxCheckBox* choice, const wxString& config)
{
    wxCheckBoxData data;

    data.checkBox = choice;
    data.name = config;

    checkBoxData.push_back(data);
}