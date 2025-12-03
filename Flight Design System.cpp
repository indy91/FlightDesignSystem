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

enum
{
    ID_Hello = 1,
    ID_Button_1,
    ID_Button_2,
    ID_Button_3,
    ID_Button_4,
    ID_TextBox_1,
    ID_TextBox_Year,
    ID_TextBox_Month,
    ID_TextBox_Day,
    ID_Button_LWP_SIB_Presets,
    ID_Button_LWP_Shuttle_Presets,
    ID_Button_Save,
    ID_Button_Load,
    MainContentID,
    wxID_StateVectorList,
    ID_Button_SV_Save,
    ID_Button_SV_New,
    wxID_StateVectorRawData,
    ID_Button_LWP_LVDC_Export,
    ID_Button_LWP_SSV_Export,
    ID_Button_MCT_View,
    ID_Button_MCT_Save
};

BEGIN_EVENT_TABLE(MyFrame, wxFrame)
EVT_BUTTON(ID_Button_1, MyFrame::OnButton1)
EVT_BUTTON(ID_Button_2, MyFrame::OnButton2)
EVT_BUTTON(ID_Button_3, MyFrame::OnButton3)
EVT_BUTTON(ID_Button_4, MyFrame::OnButton4)
EVT_BUTTON(ID_Button_LWP_SIB_Presets, MyFrame::OnButtonLWP_SIB_Presets)
EVT_BUTTON(ID_Button_LWP_Shuttle_Presets, MyFrame::OnButtonLWP_Shuttle_Presets)
EVT_BUTTON(ID_Button_Save, MyFrame::OnButton_Save)
EVT_BUTTON(ID_Button_Load, MyFrame::OnButton_Load)
EVT_NOTEBOOK_PAGE_CHANGED(MainContentID, MyFrame::OnPageChanged)
EVT_LIST_ITEM_SELECTED(wxID_StateVectorList, MyFrame::StateVectorSelected)
EVT_BUTTON(ID_Button_SV_Save, MyFrame::OnButton_StateVector_Save)
EVT_BUTTON(ID_Button_SV_New, MyFrame::OnButton_StateVector_New)
EVT_TEXT_ENTER(wxID_StateVectorRawData, MyFrame::StateVectorRawDataEnter)
EVT_TEXT(ID_TextBox_Year, MyFrame::CalculateDayOfYear)
EVT_TEXT(ID_TextBox_Month, MyFrame::CalculateDayOfYear)
EVT_TEXT(ID_TextBox_Day, MyFrame::CalculateDayOfYear)
EVT_BUTTON(ID_Button_LWP_LVDC_Export, MyFrame::OnButton_LWP_LVDC_Export)
EVT_BUTTON(ID_Button_LWP_SSV_Export, MyFrame::OnButton_LWP_SSV_Export)
EVT_BUTTON(ID_Button_MCT_View, MyFrame::OnButton_View_MCT)
EVT_BUTTON(ID_Button_MCT_Save, MyFrame::OnButton_Save_MCT)
END_EVENT_TABLE()

bool MyApp::OnInit()
{
    MyFrame* frame = new MyFrame("Flight Design System");
    frame->Show(true);
    return true;
}

MyFrame::MyFrame(const wxString& title)
    : wxFrame(NULL, wxID_ANY, title)
{
    core = NULL;

    // Create a top-level panel to hold all the contents of the frame
    wxPanel* panel = new wxPanel(this, wxID_ANY);

    // Create the wxNotebook widget
    wxNotebook* notebook = new wxNotebook(panel, MainContentID);

    // Add 2 pages to the wxNotebook widget
    panel1 = new wxPanel(notebook, wxID_ANY, wxDefaultPosition, wxDefaultSize, 2621440L, L"Tab 1 Contents");
    notebook->AddPage(panel1, L"Config");
    panel2 = new wxPanel(notebook, wxID_ANY, wxDefaultPosition, wxDefaultSize, 2621440L, L"Tab 2 Contents");
    notebook->AddPage(panel2, L"Generic LWP");
    panel3 = new wxPanel(notebook, wxID_ANY, wxDefaultPosition, wxDefaultSize, 2621440L, L"Tab 3 Contents");
    notebook->AddPage(panel3, L"OMP");
    panel4 = new wxPanel(notebook, wxID_ANY, wxDefaultPosition, wxDefaultSize, 2621440L, L"Tab 4 Contents");
    notebook->AddPage(panel4, L"Skylab LWP");
    panel5 = new wxPanel(notebook, wxID_ANY, wxDefaultPosition, wxDefaultSize, 2621440L, L"Tab 4 Contents");
    notebook->AddPage(panel5, L"State Vectors");

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
    SetStatusText(wxT("Welcome to wxWidgets!"));

    AddConfigPage();
    AddLWPPage();
    AddOMPPage();   
    AddSkylabLWPPage();
    AddStateVectorPage();
}

MyFrame::~MyFrame()
{
    if (core)
    {
        delete core;
        core = NULL;
    }
}

void MyFrame::AddConfigPage()
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
    comboWorld = new wxComboBox(panel1, wxID_ANY, wxT("NASSP"), wxPoint(minX + diffX, Y), wxDefaultSize, strings, wxCB_READONLY);
    comboWorld->SetToolTip(wxT("Select world"));
    strings.clear();
    Y += diffY;

    new wxStaticText(panel1, wxID_ANY, "Year", wxPoint(minX, Y + difftext));
    textYear = new wxTextCtrl(panel1, ID_TextBox_Year, "1973", wxPoint(minX + diffX, Y));
    textYear->SetToolTip(wxT("Liftoff year"));
    Y += diffY;

    new wxStaticText(panel1, wxID_ANY, "Month", wxPoint(minX, Y + difftext));
    textMonth = new wxTextCtrl(panel1, ID_TextBox_Month, "5", wxPoint(minX + diffX, Y));
    textMonth->SetToolTip(wxT("Liftoff month"));
    Y += diffY;

    new wxStaticText(panel1, wxID_ANY, "Day", wxPoint(minX, Y + difftext));
    textDay = new wxTextCtrl(panel1, ID_TextBox_Day, "15", wxPoint(minX + diffX, Y));
    textDay->SetToolTip(wxT("Liftoff day"));
    Y += diffY;
    Y += diffY;

    new wxStaticText(panel1, wxID_ANY, "DOY", wxPoint(minX, Y + difftext));
    textDayOfYear = new wxTextCtrl(panel1, wxID_ANY, wxEmptyString, wxPoint(minX + diffX, Y), wxDefaultSize, wxTE_READONLY);
    textDayOfYear->SetToolTip(wxT("Day of year"));
    Y += diffY;
}

void MyFrame::AddLWPPage()
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
    textLWPTargetVector = new wxTextCtrl(panel2, wxID_ANY, "Skylab TLE2.txt", wxPoint(minX + diffX, Y));
    textLWPTargetVector->SetToolTip(wxT("Target vehicle state vector"));
    Y += diffY;

    new wxStaticText(panel2, wxID_ANY, "CKFACTOR", wxPoint(minX, Y + difftext));
    textLWPCKFactor = new wxTextCtrl(panel2, wxID_ANY, "1.0", wxPoint(minX + diffX, Y));
    textLWPCKFactor->SetToolTip(wxT("Chaser vehicle drag multiplier"));
    Y += diffY;

    new wxStaticText(panel2, wxID_ANY, "CAREA", wxPoint(minX, Y + difftext));
    textLWPCArea = new wxTextCtrl(panel2, wxID_ANY, "129.4", wxPoint(minX + diffX, Y));
    textLWPCArea->SetToolTip(wxT("Chaser vehicle reference area"));
    new wxStaticText(panel2, wxID_ANY, "sq ft", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel2, wxID_ANY, "CWHT", wxPoint(minX, Y + difftext));
    textLWPCWHT = new wxTextCtrl(panel2, wxID_ANY, "300000.0", wxPoint(minX + diffX, Y));
    textLWPCWHT->SetToolTip(wxT("Chaser vehicle weight at insertion"));
    new wxStaticText(panel2, wxID_ANY, "lbs", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel2, wxID_ANY, "LW", wxPoint(minX, Y + difftext));
   
    strings.Add(wxT("Launch Window"));
    strings.Add(wxT("Launch Targeting"));
    strings.Add(wxT("Both"));
    comboLWPLW = new wxComboBox(panel2, wxID_ANY, wxT("Both"), wxPoint(minX + diffX, Y), wxDefaultSize, strings, wxCB_READONLY);
    comboLWPLW->SetToolTip(wxT("Launch window/launch targeting options"));
    strings.clear();
    Y += diffY;

    new wxStaticText(panel2, wxID_ANY, "NS", wxPoint(minX, Y + difftext));
    strings.Add(wxT("North"));
    strings.Add(wxT("South"));
    strings.Add(wxT("Both"));
    comboLWPNS = new wxComboBox(panel2, wxID_ANY, wxT("North"), wxPoint(minX + diffX, Y), wxDefaultSize, strings, wxCB_READONLY);
    comboLWPNS->SetToolTip(wxT("Inplane launch window opening and closing times option"));
    strings.clear();
    Y += diffY;

    new wxStaticText(panel2, wxID_ANY, "DAY", wxPoint(minX, Y + difftext));
    textLWPDay = new wxTextCtrl(panel2, wxID_ANY, "0", wxPoint(minX + diffX, Y));
    textLWPDay->SetToolTip(wxT("Day on which launch window times are computed, relative to base date"));
    Y += diffY;

    new wxStaticText(panel2, wxID_ANY, "LPT", wxPoint(minX, Y + difftext));
    strings.Add(wxT("Do not generate table"));
    strings.Add(wxT("Around opening"));
    strings.Add(wxT("Around closing"));
    strings.Add(wxT("Around both"));
    strings.Add(wxT("Entire window"));
    comboLWP_LPT = new wxComboBox(panel2, wxID_ANY, wxT("Do not generate table"), wxPoint(minX + diffX, Y), wxDefaultSize, strings, wxCB_READONLY);
    comboLWP_LPT->SetToolTip(wxT("Launch window parameter table options"));
    strings.clear();
    Y += diffY;

    new wxStaticText(panel2, wxID_ANY, "TSTART", wxPoint(minX, Y + difftext));
    textLWP_TSTART = new wxTextCtrl(panel2, wxID_ANY, "300.0", wxPoint(minX + diffX, Y));
    textLWP_TSTART->SetToolTip(wxT("Delta time prior to inplane time to start parameter table"));
    new wxStaticText(panel2, wxID_ANY, "sec", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel2, wxID_ANY, "TEND", wxPoint(minX, Y + difftext));
    textLWP_TEND = new wxTextCtrl(panel2, wxID_ANY, "300.0", wxPoint(minX + diffX, Y));
    textLWP_TEND->SetToolTip(wxT("Delta time after inplane time to start parameter table"));
    new wxStaticText(panel2, wxID_ANY, "sec", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel2, wxID_ANY, "TSTEP", wxPoint(minX, Y + difftext));
    textLWP_TSTEP = new wxTextCtrl(panel2, wxID_ANY, "300.0", wxPoint(minX + diffX, Y));
    textLWP_TSTEP->SetToolTip(wxT("Time step in parameter table"));
    new wxStaticText(panel2, wxID_ANY, "sec", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel2, wxID_ANY, "STABLE", wxPoint(minX, Y + difftext));
    strings.Add(wxT("Do not compute table"));
    strings.Add(wxT("Compute table"));
    comboLWP_STABLE = new wxComboBox(panel2, wxID_ANY, wxT("Do not compute table"), wxPoint(minX + diffX, Y), wxDefaultSize, strings, wxCB_READONLY);
    comboLWP_STABLE->SetToolTip(wxT("GMTLO* table flag"));
    strings.clear();
    Y += diffY;

    new wxStaticText(panel2, wxID_ANY, "STARS", wxPoint(minX, Y + difftext));
    textLWP_STARS = new wxTextCtrl(panel2, wxID_ANY, "300.0", wxPoint(minX + diffX, Y));
    textLWP_STARS->SetToolTip(wxT("Delta time prior to each inplane launch point to start GMTLO* search"));
    new wxStaticText(panel2, wxID_ANY, "sec", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel2, wxID_ANY, "STARE", wxPoint(minX, Y + difftext));
    textLWP_STARE = new wxTextCtrl(panel2, wxID_ANY, "300.0", wxPoint(minX + diffX, Y));
    textLWP_STARE->SetToolTip(wxT("Delta time after each inplane launch point to start GMTLO* search"));
    new wxStaticText(panel2, wxID_ANY, "sec", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    minX += 256;
    Y = minY;

    new wxStaticText(panel2, wxID_ANY, "LATLS", wxPoint(minX, Y + difftext));
    textLWP_LATLS = new wxTextCtrl(panel2, wxID_ANY, "28.627", wxPoint(minX + diffX, Y));
    textLWP_LATLS->SetToolTip(wxT("Geocentric latitude of launch site"));
    new wxStaticText(panel2, wxID_ANY, "deg", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel2, wxID_ANY, "LONGLS", wxPoint(minX, Y + difftext));
    textLWP_LONGLS = new wxTextCtrl(panel2, wxID_ANY, "279.379", wxPoint(minX + diffX, Y));
    textLWP_LONGLS->SetToolTip(wxT("Geographic longitude of launch site"));
    new wxStaticText(panel2, wxID_ANY, "deg", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel2, wxID_ANY, "PFT", wxPoint(minX, Y + difftext));
    textLWP_PFT = new wxTextCtrl(panel2, wxID_ANY, "519.0", wxPoint(minX + diffX, Y));
    textLWP_PFT->SetToolTip(wxT("Powered flight time"));
    new wxStaticText(panel2, wxID_ANY, "sec", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel2, wxID_ANY, "PFA", wxPoint(minX, Y + difftext));
    textLWP_PFA = new wxTextCtrl(panel2, wxID_ANY, "14.4", wxPoint(minX + diffX, Y));
    textLWP_PFA->SetToolTip(wxT("Powered flight arc"));
    new wxStaticText(panel2, wxID_ANY, "deg", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel2, wxID_ANY, "YSMAX", wxPoint(minX, Y + difftext));
    textLWP_YSMAX = new wxTextCtrl(panel2, wxID_ANY, "14.0", wxPoint(minX + diffX, Y));
    textLWP_YSMAX->SetToolTip(wxT("Yaw steering limit"));
    new wxStaticText(panel2, wxID_ANY, "deg", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel2, wxID_ANY, "DTOPT", wxPoint(minX, Y + difftext));
    textLWP_DTOPT = new wxTextCtrl(panel2, wxID_ANY, "360.0", wxPoint(minX + diffX, Y));
    textLWP_DTOPT->SetToolTip(wxT("Delta time to be subtracted from analytical inplane launch time to obtain empirical launch time"));
    new wxStaticText(panel2, wxID_ANY, "sec", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel2, wxID_ANY, "DTGRR", wxPoint(minX, Y + difftext));
    textLWP_DTGRR = new wxTextCtrl(panel2, wxID_ANY, "0.0", wxPoint(minX + diffX, Y));
    textLWP_DTGRR->SetToolTip(wxT("DT from lift-off, which defines the time of guidance reference release"));
    new wxStaticText(panel2, wxID_ANY, "sec", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel2, wxID_ANY, "RINS", wxPoint(minX, Y + difftext));
    textLWP_RINS = new wxTextCtrl(panel2, wxID_ANY, "21417907.0", wxPoint(minX + diffX, Y));
    textLWP_RINS->SetToolTip(wxT("Radius of insertion"));
    new wxStaticText(panel2, wxID_ANY, "ft", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel2, wxID_ANY, "VINS", wxPoint(minX, Y + difftext));
    textLWP_VINS = new wxTextCtrl(panel2, wxID_ANY, "25705.8", wxPoint(minX + diffX, Y));
    textLWP_VINS->SetToolTip(wxT("Velocity of insertion"));
    new wxStaticText(panel2, wxID_ANY, "ft/s", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel2, wxID_ANY, "GAMINS", wxPoint(minX, Y + difftext));
    textLWP_GAMINS = new wxTextCtrl(panel2, wxID_ANY, "0.0", wxPoint(minX + diffX, Y));
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
    comboLWP_LOT = new wxComboBox(panel2, wxID_ANY, wxT("Zero yaw steering time"), wxPoint(minX + diffX, Y), wxDefaultSize, strings, wxCB_READONLY);
    comboLWP_LOT->SetToolTip(wxT("Lift-off time options for launch targeting"));
    strings.clear();
    Y += diffY;

    new wxStaticText(panel2, wxID_ANY, "GMTLOR", wxPoint(minX, Y + difftext));
    textLWP_GMTLOR = new wxTextCtrl(panel2, wxID_ANY, "00:00:00.000", wxPoint(minX + diffX, Y));
    textLWP_GMTLOR->SetToolTip(wxT("Recommended or threshold lift-off time"));
    new wxStaticText(panel2, wxID_ANY, "GMT", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel2, wxID_ANY, "OFFSET", wxPoint(minX, Y + difftext));
    textLWP_OFFSET = new wxTextCtrl(panel2, wxID_ANY, "0.0", wxPoint(minX + diffX, Y));
    textLWP_OFFSET->SetToolTip(wxT("Phase angle desired at insertion"));
    new wxStaticText(panel2, wxID_ANY, "deg", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel2, wxID_ANY, "BIAS", wxPoint(minX, Y + difftext));
    textLWP_BIAS = new wxTextCtrl(panel2, wxID_ANY, "0.0", wxPoint(minX + diffX, Y));
    textLWP_BIAS->SetToolTip(wxT("Delta time added to GMTLO* to produce lift-off time"));
    new wxStaticText(panel2, wxID_ANY, "sec", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel2, wxID_ANY, "TPLANE", wxPoint(minX, Y + difftext));
    textLWP_TPLANE = new wxTextCtrl(panel2, wxID_ANY, "00:00:00.000", wxPoint(minX + diffX, Y));
    textLWP_TPLANE->SetToolTip(wxT("Greenwich mean time of in-plane lift-off (computed internally if LW is run)"));
    new wxStaticText(panel2, wxID_ANY, "GMT", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel2, wxID_ANY, "TRANS", wxPoint(minX, Y + difftext));
    textLWP_TRANS = new wxTextCtrl(panel2, wxID_ANY, "0.0", wxPoint(minX + diffX, Y));
    textLWP_TRANS->SetToolTip(wxT("Delta time added in in-plane time to obtain lift-off time"));
    new wxStaticText(panel2, wxID_ANY, "sec", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel2, wxID_ANY, "INSCO", wxPoint(minX, Y + difftext));
    strings.Add(wxT("Input VINS, GAMINS, RINS"));
    strings.Add(wxT("Input GAMINS, RINS and height diff"));
    strings.Add(wxT("Input GAMINS, RINS and altitude"));
    comboLWP_INSCO = new wxComboBox(panel2, wxID_ANY, wxT("Input VINS, GAMINS, RINS"), wxPoint(minX + diffX, Y), wxDefaultSize, strings, wxCB_READONLY);
    comboLWP_INSCO->SetToolTip(wxT("Insertion cutoff conditions option flag"));
    strings.clear();
    Y += diffY;

    new wxStaticText(panel2, wxID_ANY, "DHW", wxPoint(minX, Y + difftext));
    textLWP_DHW = new wxTextCtrl(panel2, wxID_ANY, "0.0", wxPoint(minX + diffX, Y));
    textLWP_DHW->SetToolTip(wxT("Desired height difference between chaser and target, or altitude of chaser, at input angle from insertion"));
    new wxStaticText(panel2, wxID_ANY, "NM", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel2, wxID_ANY, "DU", wxPoint(minX, Y + difftext));
    textLWP_DU = new wxTextCtrl(panel2, wxID_ANY, "0.0", wxPoint(minX + diffX, Y));
    textLWP_DU->SetToolTip(wxT("Angle from insertion to obtain a given altitude, or delta altitude"));
    new wxStaticText(panel2, wxID_ANY, "deg", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel2, wxID_ANY, "ANOM", wxPoint(minX, Y + difftext));
    textLWP_ANOM = new wxTextCtrl(panel2, wxID_ANY, "0.0", wxPoint(minX + diffX, Y));
    textLWP_ANOM->SetToolTip(wxT("Nominal semimajor axis at insertion"));
    new wxStaticText(panel2, wxID_ANY, "NM", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel2, wxID_ANY, "DELNOF", wxPoint(minX, Y + difftext));
    strings.Add(wxT("Input DELNO"));
    strings.Add(wxT("Compute DELNO"));
    comboLWP_DELNOF = new wxComboBox(panel2, wxID_ANY, wxT("Compute DELNO"), wxPoint(minX + diffX, Y), wxDefaultSize, strings, wxCB_READONLY);
    comboLWP_DELNOF->SetToolTip(wxT("Flag for option to compute differential nodal regression from insertion to rendezvous"));
    strings.clear();
    Y += diffY;

    new wxStaticText(panel2, wxID_ANY, "DELNO", wxPoint(minX, Y + difftext));
    textLWP_DELNO = new wxTextCtrl(panel2, wxID_ANY, "0.0", wxPoint(minX + diffX, Y));
    textLWP_DELNO->SetToolTip(wxT("Angle that is added to the target descending node to account for differential nodal regression"));
    new wxStaticText(panel2, wxID_ANY, "deg", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel2, wxID_ANY, "NEGTIV", wxPoint(minX, Y + difftext));
    strings.Add(wxT("Use positive value"));
    strings.Add(wxT("Use negative value"));
    strings.Add(wxT("Use lowest absolute value"));
    comboLWP_NEGTIV = new wxComboBox(panel2, wxID_ANY, wxT("Use positive value"), wxPoint(minX + diffX, Y), wxDefaultSize, strings, wxCB_READONLY);
    comboLWP_NEGTIV->SetToolTip(wxT("Initial phase angle control flag"));
    strings.clear();
    Y += diffY;

    new wxStaticText(panel2, wxID_ANY, "WRAP", wxPoint(minX, Y + difftext));
    textLWP_WRAP = new wxTextCtrl(panel2, wxID_ANY, "0", wxPoint(minX + diffX, Y));
    textLWP_WRAP->SetToolTip(wxT("Flag to wrap initial phase angle"));
    Y += diffY;

    wxButton* button = new wxButton(panel2, ID_Button_1, wxT("Generate"),
        wxPoint(10, 514), wxDefaultSize);

    button = new wxButton(panel2, ID_Button_2, wxT("Save SV"),
        wxPoint(550, 514), wxDefaultSize);

    minX += 256 + 64;
    Y = minY;

    new wxButton(panel2, ID_Button_LWP_SIB_Presets, wxT("Saturn IB Presets"), wxPoint(minX, Y), wxDefaultSize);
    Y += diffY;
    new wxButton(panel2, ID_Button_LWP_Shuttle_Presets, wxT("Shuttle Presets"), wxPoint(minX, Y), wxDefaultSize);

    button = new wxButton(panel2, ID_Button_LWP_LVDC_Export, wxT("Export LVDC"), wxPoint(minX, 300), wxDefaultSize);
    new wxButton(panel2, ID_Button_LWP_SSV_Export, wxT("Export SSV"), wxPoint(minX, 300 + diffY), wxDefaultSize);
}

void MyFrame::AddOMPPage()
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
    textOMP_GMTLO = new wxTextCtrl(panel3, wxID_ANY, "13:00:00.000", wxPoint(minX + diffX, Y));
    textOMP_GMTLO->SetToolTip(wxT("Lift-off time"));
    new wxStaticText(panel3, wxID_ANY, "GMT", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel3, wxID_ANY, "Chaser", wxPoint(minX, Y + difftext));
    textOMP_Chaser = new wxTextCtrl(panel3, wxID_ANY, "LWP.txt", wxPoint(minX + diffX, Y));
    textOMP_Chaser->SetToolTip(wxT("Chaser state vector file"));
    Y += diffY;

    new wxStaticText(panel3, wxID_ANY, "Target", wxPoint(minX, Y + difftext));
    textOMP_Target = new wxTextCtrl(panel3, wxID_ANY, "Skylab.txt", wxPoint(minX + diffX, Y));
    textOMP_Target->SetToolTip(wxT("Target state vector file"));
    Y += diffY;

    new wxStaticText(panel3, wxID_ANY, "MCT", wxPoint(minX, Y + difftext));
    textOMP_MCT = new wxTextCtrl(panel3, wxID_ANY, "SL-2 MCT.txt", wxPoint(minX + diffX, Y));
    textOMP_MCT->SetToolTip(wxT("Maneuver constraints table file"));

    new wxButton(panel3, ID_Button_MCT_View, wxT("View"),
        wxPoint(minX + diffX + 150, Y), wxDefaultSize);

    Y += diffY;

    new wxButton(panel3, ID_Button_3, wxT("Generate"),
        wxPoint(10, 514), wxDefaultSize);


    //MCT
    new wxStaticText(panel3, wxID_ANY, "Maneuver Constraints Table", wxPoint(500, 10));
    wxPanel *temp = new wxPanel(panel3, wxID_ANY, wxPoint(400, 40), wxSize(550, 300), wxBORDER_SIMPLE);

    textOMP_MCT_Editor = new wxGrid(temp, wxID_ANY, wxDefaultPosition, wxSize(550, 300));

    textOMP_MCT_Editor->CreateGrid(41, 13);
    textOMP_MCT_Editor->SetCellValue(0, 0, wxT("Name"));
    textOMP_MCT_Editor->SetCellValue(0, 1, wxT("Type"));
    textOMP_MCT_Editor->SetCellValue(0, 2, wxT("Threshold"));
    textOMP_MCT_Editor->SetCellValue(0, 3, wxT("Value"));
    textOMP_MCT_Editor->SetCellValue(0, 4, wxT("Secondary 1"));
    textOMP_MCT_Editor->SetCellValue(0, 5, wxT("Secondary 2"));
    textOMP_MCT_Editor->SetCellValue(0, 6, wxT("Secondary 3"));
    textOMP_MCT_Editor->SetCellValue(0, 7, wxT("Secondary 4"));
    textOMP_MCT_Editor->SetCellValue(0, 8, wxT("Secondary 5"));
    textOMP_MCT_Editor->SetCellValue(0, 9, wxT("Secondary 6"));
    textOMP_MCT_Editor->SetCellValue(0, 10, wxT("Secondary 7"));
    textOMP_MCT_Editor->SetCellValue(0, 11, wxT("Secondary 8"));
    textOMP_MCT_Editor->SetCellValue(0, 12, wxT("Secondary 9"));

    for (int i = 0; i < 13; i++)
    {
        textOMP_MCT_Editor->SetReadOnly(0, i);
    }

    textOMP_MCT_Editor->SetColSize(0, 50);
    textOMP_MCT_Editor->SetColSize(1, 50);
    textOMP_MCT_Editor->SetColSize(2, 60);
    textOMP_MCT_Editor->SetColSize(3, 95);

    new wxButton(panel3, ID_Button_MCT_Save, wxT("Save"),
        wxPoint(450, 414), wxDefaultSize);
}

void MyFrame::AddSkylabLWPPage()
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
    textSkylabLWP_TargetVector = new wxTextCtrl(panel4, wxID_ANY, "Skylab.txt", wxPoint(minX + diffX, Y));
    textSkylabLWP_TargetVector->SetToolTip(wxT("Target vehicle state vector"));
    Y += diffY;

    new wxStaticText(panel4, wxID_ANY, "CKFACTOR", wxPoint(minX, Y + difftext));
    textSkylabLWP_CKFactor = new wxTextCtrl(panel4, wxID_ANY, "1.0", wxPoint(minX + diffX, Y));
    textSkylabLWP_CKFactor->SetToolTip(wxT("Chaser vehicle drag multiplier"));
    Y += diffY;

    new wxStaticText(panel4, wxID_ANY, "CAREA", wxPoint(minX, Y + difftext));
    textSkylabLWP_CArea = new wxTextCtrl(panel4, wxID_ANY, "129.4", wxPoint(minX + diffX, Y));
    textSkylabLWP_CArea->SetToolTip(wxT("Chaser vehicle reference area"));
    new wxStaticText(panel4, wxID_ANY, "sq ft", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel4, wxID_ANY, "CWHT", wxPoint(minX, Y + difftext));
    textSkylabLWP_CWHT = new wxTextCtrl(panel4, wxID_ANY, "300000.0", wxPoint(minX + diffX, Y));
    textSkylabLWP_CWHT->SetToolTip(wxT("Chaser vehicle weight at insertion"));
    new wxStaticText(panel4, wxID_ANY, "lbs", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel4, wxID_ANY, "NS", wxPoint(minX, Y + difftext));
    strings.Add(wxT("North"));
    strings.Add(wxT("South"));
    comboSkylabLWP_NS = new wxComboBox(panel4, wxID_ANY, wxT("North"), wxPoint(minX + diffX, Y), wxDefaultSize, strings, wxCB_READONLY);
    comboSkylabLWP_NS->SetToolTip(wxT("Inplane launch window opening and closing times option"));
    strings.clear();
    Y += diffY;

    new wxStaticText(panel4, wxID_ANY, "DAY", wxPoint(minX, Y + difftext));
    textSkylabLWP_DAY = new wxTextCtrl(panel4, wxID_ANY, "0", wxPoint(minX + diffX, Y));
    textSkylabLWP_DAY->SetToolTip(wxT("Day on which launch window times are computed, relative to base date"));
    Y += diffY;

    new wxStaticText(panel4, wxID_ANY, "LATLS", wxPoint(minX, Y + difftext));
    textSkylabLWP_LATLS = new wxTextCtrl(panel4, wxID_ANY, "28.627", wxPoint(minX + diffX, Y));
    textSkylabLWP_LATLS->SetToolTip(wxT("Geocentric latitude of launch site"));
    new wxStaticText(panel4, wxID_ANY, "deg", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel4, wxID_ANY, "LONGLS", wxPoint(minX, Y + difftext));
    textSkylabLWP_LONGLS = new wxTextCtrl(panel4, wxID_ANY, "279.379", wxPoint(minX + diffX, Y));
    textSkylabLWP_LONGLS->SetToolTip(wxT("Geographic longitude of launch site"));
    new wxStaticText(panel4, wxID_ANY, "deg", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel4, wxID_ANY, "PFT", wxPoint(minX, Y + difftext));
    textSkylabLWP_PFT = new wxTextCtrl(panel4, wxID_ANY, "600.0", wxPoint(minX + diffX, Y));
    textSkylabLWP_PFT->SetToolTip(wxT("Powered flight time"));
    new wxStaticText(panel4, wxID_ANY, "sec", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel4, wxID_ANY, "PFA", wxPoint(minX, Y + difftext));
    textSkylabLWP_PFA = new wxTextCtrl(panel4, wxID_ANY, "18.0", wxPoint(minX + diffX, Y));
    textSkylabLWP_PFA->SetToolTip(wxT("Powered flight arc"));
    new wxStaticText(panel4, wxID_ANY, "deg", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel4, wxID_ANY, "YSMAX", wxPoint(minX, Y + difftext));
    textSkylabLWP_YSMAX = new wxTextCtrl(panel4, wxID_ANY, "5.0", wxPoint(minX + diffX, Y));
    textSkylabLWP_YSMAX->SetToolTip(wxT("Yaw steering limit"));
    new wxStaticText(panel4, wxID_ANY, "deg", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel4, wxID_ANY, "DTOPT", wxPoint(minX, Y + difftext));
    textSkylabLWP_DTOPT = new wxTextCtrl(panel4, wxID_ANY, "360.0", wxPoint(minX + diffX, Y));
    textSkylabLWP_DTOPT->SetToolTip(wxT("Delta time to be subtracted from analytical inplane launch time to obtain empirical launch time"));
    new wxStaticText(panel4, wxID_ANY, "sec", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel4, wxID_ANY, "RINS", wxPoint(minX, Y + difftext));
    textSkylabLWP_RINS = new wxTextCtrl(panel4, wxID_ANY, "21417907.0", wxPoint(minX + diffX, Y));
    textSkylabLWP_RINS->SetToolTip(wxT("Radius of insertion"));
    new wxStaticText(panel4, wxID_ANY, "ft", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel4, wxID_ANY, "VINS", wxPoint(minX, Y + difftext));
    textSkylabLWP_VINS = new wxTextCtrl(panel4, wxID_ANY, "25705.8", wxPoint(minX + diffX, Y));
    textSkylabLWP_VINS->SetToolTip(wxT("Velocity of insertion"));
    new wxStaticText(panel4, wxID_ANY, "ft/s", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel4, wxID_ANY, "GAMINS", wxPoint(minX, Y + difftext));
    textSkylabLWP_GAMINS = new wxTextCtrl(panel4, wxID_ANY, "0.0", wxPoint(minX + diffX, Y));
    textSkylabLWP_GAMINS->SetToolTip(wxT("Flightpath angle of insertion"));
    new wxStaticText(panel4, wxID_ANY, "deg", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    minX += 256;
    Y = minY;

    new wxStaticText(panel4, wxID_ANY, "NEGTIV", wxPoint(minX, Y + difftext));
    strings.Add(wxT("Use positive value"));
    strings.Add(wxT("Use negative value"));
    strings.Add(wxT("Use lowest absolute value"));
    comboSkylabLWP_NEGTIV = new wxComboBox(panel4, wxID_ANY, wxT("Use positive value"), wxPoint(minX + diffX, Y), wxDefaultSize, strings, wxCB_READONLY);
    comboSkylabLWP_NEGTIV->SetToolTip(wxT("Initial phase angle control flag"));
    strings.clear();
    Y += diffY;

    new wxStaticText(panel4, wxID_ANY, "WRAP", wxPoint(minX, Y + difftext));
    textSkylabLWP_WRAP = new wxTextCtrl(panel4, wxID_ANY, "0", wxPoint(minX + diffX, Y));
    textSkylabLWP_WRAP->SetToolTip(wxT("Flag to wrap initial phase angle"));
    Y += diffY;

    new wxStaticText(panel4, wxID_ANY, "DTOPEN", wxPoint(minX, Y + difftext));
    textSkylabLWP_DTOPEN = new wxTextCtrl(panel4, wxID_ANY, "7.75", wxPoint(minX + diffX, Y));
    textSkylabLWP_DTOPEN->SetToolTip(wxT("Delta time from optimum inplane launch time to opening of window"));
    new wxStaticText(panel4, wxID_ANY, "min", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel4, wxID_ANY, "DTCLOSE", wxPoint(minX, Y + difftext));
    textSkylabLWP_DTCLOSE = new wxTextCtrl(panel4, wxID_ANY, "7.75", wxPoint(minX + diffX, Y));
    textSkylabLWP_DTCLOSE->SetToolTip(wxT("Delta time from optimum inplane launch time to closing of window"));
    new wxStaticText(panel4, wxID_ANY, "min", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel4, wxID_ANY, "IR", wxPoint(minX, Y + difftext));
    strings.Add(wxT("Compute launch window"));
    strings.Add(wxT("Input launch time only"));
    comboSkylabLWP_IR = new wxComboBox(panel4, wxID_ANY, wxT("Compute launch window"), wxPoint(minX + diffX, Y), wxDefaultSize, strings, wxCB_READONLY);
    comboSkylabLWP_IR->SetToolTip(wxT("Liftoff time flag"));
    strings.clear();
    Y += diffY;

    new wxStaticText(panel4, wxID_ANY, "TLO", wxPoint(minX, Y + difftext));
    textSkylabLWP_TLO = new wxTextCtrl(panel4, wxID_ANY, "00:00:00:000", wxPoint(minX + diffX, Y));
    textSkylabLWP_TLO->SetToolTip(wxT("Time of liftoff (if input)"));
    new wxStaticText(panel4, wxID_ANY, "GMT", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel4, wxID_ANY, "MI", wxPoint(minX, Y + difftext));
    textSkylabLWP_MI = new wxTextCtrl(panel4, wxID_ANY, "5", wxPoint(minX + diffX, Y));
    textSkylabLWP_MI->SetToolTip(wxT("Initial M-line or rendezvous number"));
    Y += diffY;

    new wxStaticText(panel4, wxID_ANY, "MF", wxPoint(minX, Y + difftext));
    textSkylabLWP_MF = new wxTextCtrl(panel4, wxID_ANY, "8", wxPoint(minX + diffX, Y));
    textSkylabLWP_MF->SetToolTip(wxT("Final M-line or rendezvous number"));
    Y += diffY;

    new wxStaticText(panel4, wxID_ANY, "DTSEP", wxPoint(minX, Y + difftext));
    textSkylabLWP_DTSEP = new wxTextCtrl(panel4, wxID_ANY, "000:15:00.000", wxPoint(minX + diffX, Y));
    textSkylabLWP_DTSEP->SetToolTip(wxT("Time from insertion to separation"));
    Y += diffY;

    new wxStaticText(panel4, wxID_ANY, "DVSEP", wxPoint(minX, Y + difftext));
    textSkylabLWP_DVSEP = new wxTextCtrl(panel4, wxID_ANY, "+3.0 +0.0 +0.0", wxPoint(minX + diffX, Y));
    textSkylabLWP_DVSEP->SetToolTip(wxT("Delta velocity vector of separation"));
    new wxStaticText(panel4, wxID_ANY, "ft/s", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel4, wxID_ANY, "DVWO", wxPoint(minX, Y + difftext));
    textSkylabLWP_DVWO = new wxTextCtrl(panel4, wxID_ANY, "30.0", wxPoint(minX + diffX, Y));
    textSkylabLWP_DVWO->SetToolTip(wxT("Delta velocity of phase window opening (Minimum NCC DV)"));
    new wxStaticText(panel4, wxID_ANY, "ft/s", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    new wxStaticText(panel4, wxID_ANY, "DVWC", wxPoint(minX, Y + difftext));
    textSkylabLWP_DVWC = new wxTextCtrl(panel4, wxID_ANY, "30.0", wxPoint(minX + diffX, Y));
    textSkylabLWP_DVWC->SetToolTip(wxT("Delta velocity of phase window closing (Minimum NC-1 DV)"));
    new wxStaticText(panel4, wxID_ANY, "ft/s", wxPoint(minX + diffunit, Y + difftext));
    Y += diffY;

    wxButton* button = new wxButton(panel4, ID_Button_4, wxT("Generate"),
        wxPoint(10, 514), wxDefaultSize);
}

void MyFrame::AddStateVectorPage()
{
    StateVectorList = new wxListCtrl(panel5, wxID_StateVectorList, wxPoint(0, 0), wxSize(300, 300), wxLC_LIST);

    new wxButton(panel5, ID_Button_SV_New, wxT("New"),
        wxPoint(10, 356), wxDefaultSize);

    new wxStaticText(panel5, wxID_ANY, "File Name:", wxPoint(400, 10));
    textStateVectorFileName = new wxTextCtrl(panel5, wxID_ANY, "", wxPoint(400, 30), wxDefaultSize, wxTE_READONLY);

    new wxStaticText(panel5, wxID_ANY, "Raw Data:", wxPoint(400, 60));

    textStateVectorRawData = new wxTextCtrl(panel5, wxID_StateVectorRawData, "",
        wxPoint(400, 80), wxSize(450, 160),
        wxTE_MULTILINE | wxSUNKEN_BORDER | wxTE_PROCESS_ENTER);

    new wxButton(panel5, ID_Button_SV_Save, wxT("Save"),
        wxPoint(400, 256), wxDefaultSize);

    new wxStaticText(panel5, wxID_ANY, "Orbit Data:", wxPoint(400, 300));

    textStateVectorOrbitData = new wxTextCtrl(panel5, wxID_ANY, "",
        wxPoint(400, 320), wxSize(450, 160),
        wxTE_READONLY | wxTE_MULTILINE);
}

std::string StringFromTextBox(wxTextCtrl* box)
{
    wxString text1 = box->GetLineText(0);
    return std::string(text1.mb_str());
}

void MyFrame::OnButton1(wxCommandEvent& event)
{
    if (SetConstants())
    {
        //Error output
        SetStatusText("Error during initialization!");
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
}

void MyFrame::OnButton2(wxCommandEvent& event)
{
    if (core == NULL) return;

    wxTextEntryDialog dialog(this, "Enter name for the file:", "LWP chaser state vector file", "LWP", wxOK | wxCANCEL);

    if (dialog.ShowModal() == wxID_OK)
    {
        wxString str = dialog.GetValue();
        core->SaveLWPStateVector(str.ToStdString());
    }  
}

void MyFrame::OnButton3(wxCommandEvent& event)
{
    //Calculate OMP

    if (SetConstants())
    {
        //Error output
        SetStatusText(wxT("Error during initialization!"));
        return;
    }

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

void MyFrame::OnButton4(wxCommandEvent& event)
{
    if (SetConstants())
    {
        //Error output
        SetStatusText(wxT("Error during initialization!"));
        return;
    }

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

void MyFrame::OnButtonLWP_SIB_Presets(wxCommandEvent& event)
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

void MyFrame::OnButtonLWP_Shuttle_Presets(wxCommandEvent& event)
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

void SaveComboBox(wxFileConfig* config, wxComboBox* ctrl, wxString str)
{
    config->Write(str, ctrl->GetSelection());
}

void LoadComboBox(wxFileConfig* config, wxComboBox* ctrl, wxString str)
{
    int inttemp;
    if (config->Read(str, &inttemp)) ctrl->SetSelection(inttemp);
}

void MyFrame::OnButton_Save(wxCommandEvent& event)
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

    config->SetPath("/Config");
    SaveComboBox(config, comboWorld, "World");
    SaveTextCtrl(config, textYear, "Year");
    SaveTextCtrl(config, textMonth, "Month");
    SaveTextCtrl(config, textDay, "Day");
    SaveTextCtrl(config, textDayOfYear, "DOY");

    config->SetPath("/LWP");
    SaveTextCtrl(config, textLWPTargetVector, "TargetVector");
    SaveTextCtrl(config, textLWPCKFactor, "CKFactor");
    SaveTextCtrl(config, textLWPCArea, "CArea");
    SaveTextCtrl(config, textLWPCWHT, "CWHT");
    SaveComboBox(config, comboLWPLW, "LW");
    SaveComboBox(config, comboLWPNS, "NS");
    SaveTextCtrl(config, textLWPDay, "LWPDay");
    SaveComboBox(config, comboLWP_LPT, "LPT");
    SaveTextCtrl(config, textLWP_TSTART, "TSTART");
    SaveTextCtrl(config, textLWP_TEND, "TEND");
    SaveTextCtrl(config, textLWP_TSTEP, "TSTEP");
    SaveComboBox(config, comboLWP_STABLE, "STABLE");
    SaveTextCtrl(config, textLWP_STARS, "STARS");
    SaveTextCtrl(config, textLWP_STARE, "STARE");
    SaveTextCtrl(config, textLWP_LATLS, "LATLS");
    SaveTextCtrl(config, textLWP_LONGLS, "LONGLS");
    SaveTextCtrl(config, textLWP_PFT, "PFT");
    SaveTextCtrl(config, textLWP_PFA, "PFA");
    SaveTextCtrl(config, textLWP_YSMAX, "YSMAX");
    SaveTextCtrl(config, textLWP_DTOPT, "DTOPT");
    SaveTextCtrl(config, textLWP_DTGRR, "DTGRR");
    SaveTextCtrl(config, textLWP_RINS, "RINS");
    SaveTextCtrl(config, textLWP_VINS, "VINS");
    SaveTextCtrl(config, textLWP_GAMINS, "GAMINS");
    SaveComboBox(config, comboLWP_LOT, "LOT");
    SaveTextCtrl(config, textLWP_GMTLOR, "GMTLOR");
    SaveTextCtrl(config, textLWP_OFFSET, "OFFSET");
    SaveTextCtrl(config, textLWP_BIAS, "BIAS");
    SaveTextCtrl(config, textLWP_TPLANE, "TPLANE");
    SaveTextCtrl(config, textLWP_TRANS, "TRANS");
    SaveComboBox(config, comboLWP_INSCO, "INSCO");
    SaveTextCtrl(config, textLWP_DHW, "DHW");
    SaveTextCtrl(config, textLWP_DU, "DU");
    SaveTextCtrl(config, textLWP_ANOM, "ANOM");
    SaveComboBox(config, comboLWP_DELNOF, "DELNOF");
    SaveTextCtrl(config, textLWP_DELNO, "DELNO");
    SaveComboBox(config, comboLWP_NEGTIV, "NEGTIV");
    SaveTextCtrl(config, textLWP_WRAP, "WRAP");

    config->SetPath("/OMP");
    SaveTextCtrl(config, textOMP_GMTLO, "GMTLO");
    SaveTextCtrl(config, textOMP_Chaser, "Chaser");
    SaveTextCtrl(config, textOMP_Target, "Target");
    SaveTextCtrl(config, textOMP_MCT, "MCT");

    config->SetPath("/SkylabLWP");
    SaveTextCtrl(config, textSkylabLWP_TargetVector, "TargetVector");
    SaveTextCtrl(config, textSkylabLWP_CKFactor, "CKFactor");
    SaveTextCtrl(config, textSkylabLWP_CArea, "CArea");
    SaveTextCtrl(config, textSkylabLWP_CWHT, "CWHT");
    SaveComboBox(config, comboSkylabLWP_NS, "NS");
    SaveTextCtrl(config, textSkylabLWP_DAY, "DAY");
    SaveTextCtrl(config, textSkylabLWP_LATLS, "LATLS");
    SaveTextCtrl(config, textSkylabLWP_LONGLS, "LONGLS");
    SaveTextCtrl(config, textSkylabLWP_PFT, "PFT");
    SaveTextCtrl(config, textSkylabLWP_PFA, "PFA");
    SaveTextCtrl(config, textSkylabLWP_YSMAX, "YSMAX");
    SaveTextCtrl(config, textSkylabLWP_DTOPT, "DTOPT");
    SaveTextCtrl(config, textSkylabLWP_RINS, "RINS");
    SaveTextCtrl(config, textSkylabLWP_VINS, "VINS");
    SaveTextCtrl(config, textSkylabLWP_GAMINS, "GAMINS");
    SaveComboBox(config, comboSkylabLWP_NEGTIV, "NEGTIV");
    SaveTextCtrl(config, textSkylabLWP_WRAP, "WRAP");
    SaveTextCtrl(config, textSkylabLWP_DTOPEN, "DTOPEN");
    SaveTextCtrl(config, textSkylabLWP_DTCLOSE, "DTCLOSE");
    SaveComboBox(config, comboSkylabLWP_IR, "IR");
    SaveTextCtrl(config, textSkylabLWP_TLO, "TLO");
    SaveTextCtrl(config, textSkylabLWP_MI, "MI");
    SaveTextCtrl(config, textSkylabLWP_MF, "MF");
    SaveTextCtrl(config, textSkylabLWP_DTSEP, "DTSEOP");
    SaveTextCtrl(config, textSkylabLWP_DVSEP, "DVSEP");
    SaveTextCtrl(config, textSkylabLWP_DVWO, "DVWO");
    SaveTextCtrl(config, textSkylabLWP_DVWC, "DVWC");

    delete config;

    SetStatusText("Save successful!");
}

void MyFrame::OnButton_Load(wxCommandEvent& event)
{
    wxString project = textProjectFile->GetLineText(0);
    wxString file = "Projects/" + project + "/" + project + ".ini";

    wxFileConfig* config = new wxFileConfig("FDS", wxEmptyString, file, wxEmptyString, wxCONFIG_USE_LOCAL_FILE | wxCONFIG_USE_RELATIVE_PATH);

    if (!config->HasGroup("Config"))
    {
        SetStatusText("Project does not exist!");
        return;
    }

    config->SetPath("/Config");
    LoadComboBox(config, comboWorld, "World");
    LoadTextCtrl(config, textYear, "Year");
    LoadTextCtrl(config, textMonth, "Month");
    LoadTextCtrl(config, textDay, "Day");
    LoadTextCtrl(config, textDayOfYear, "DOY");

    config->SetPath("/LWP");
    LoadTextCtrl(config, textLWPTargetVector, "TargetVector");
    LoadTextCtrl(config, textLWPCKFactor, "CKFactor");
    LoadTextCtrl(config, textLWPCArea, "CArea");
    LoadTextCtrl(config, textLWPCWHT, "CWHT");
    LoadComboBox(config, comboLWPLW, "LW");
    LoadComboBox(config, comboLWPNS, "NS");
    LoadTextCtrl(config, textLWPDay, "LWPDay");
    LoadComboBox(config, comboLWP_LPT, "LPT");
    LoadTextCtrl(config, textLWP_TSTART, "TSTART");
    LoadTextCtrl(config, textLWP_TEND, "TEND");
    LoadTextCtrl(config, textLWP_TSTEP, "TSTEP");
    LoadComboBox(config, comboLWP_STABLE, "STABLE");
    LoadTextCtrl(config, textLWP_STARS, "STARS");
    LoadTextCtrl(config, textLWP_STARE, "STARE");
    LoadTextCtrl(config, textLWP_LATLS, "LATLS");
    LoadTextCtrl(config, textLWP_LONGLS, "LONGLS");
    LoadTextCtrl(config, textLWP_PFT, "PFT");
    LoadTextCtrl(config, textLWP_PFA, "PFA");
    LoadTextCtrl(config, textLWP_YSMAX, "YSMAX");
    LoadTextCtrl(config, textLWP_DTOPT, "DTOPT");
    LoadTextCtrl(config, textLWP_DTGRR, "DTGRR");
    LoadTextCtrl(config, textLWP_RINS, "RINS");
    LoadTextCtrl(config, textLWP_VINS, "VINS");
    LoadTextCtrl(config, textLWP_GAMINS, "GAMINS");
    LoadComboBox(config, comboLWP_LOT, "LOT");
    LoadTextCtrl(config, textLWP_GMTLOR, "GMTLOR");
    LoadTextCtrl(config, textLWP_OFFSET, "OFFSET");
    LoadTextCtrl(config, textLWP_BIAS, "BIAS");
    LoadTextCtrl(config, textLWP_TPLANE, "TPLANE");
    LoadTextCtrl(config, textLWP_TRANS, "TRANS");
    LoadComboBox(config, comboLWP_INSCO, "INSCO");
    LoadTextCtrl(config, textLWP_DHW, "DHW");
    LoadTextCtrl(config, textLWP_DU, "DU");
    LoadTextCtrl(config, textLWP_ANOM, "ANOM");
    LoadComboBox(config, comboLWP_DELNOF, "DELNOF");
    LoadTextCtrl(config, textLWP_DELNO, "DELNO");
    LoadComboBox(config, comboLWP_NEGTIV, "NEGTIV");
    LoadTextCtrl(config, textLWP_WRAP, "WRAP");

    config->SetPath("/OMP");
    LoadTextCtrl(config, textOMP_GMTLO, "GMTLO");
    LoadTextCtrl(config, textOMP_Chaser, "Chaser");
    LoadTextCtrl(config, textOMP_Target, "Target");
    LoadTextCtrl(config, textOMP_MCT, "MCT");

    config->SetPath("/SkylabLWP");
    LoadTextCtrl(config, textSkylabLWP_TargetVector, "TargetVector");

    delete config;

    SetStatusText("Project loaded successfully!");
}

void MyFrame::OnPageChanged(wxBookCtrlEvent& event)
{
   int val = event.GetSelection();

   if (val == 4)
   {
       ReloadStateVectorPage();
   }
}

void MyFrame::ReloadStateVectorPage()
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

void MyFrame::StateVectorSelected(wxListEvent& event)
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

    textStateVectorFileName->ChangeValue(str);
    textStateVectorRawData->Clear();

    wxString arr;

    size_t i;
    for (i = 0; i < file.GetLineCount(); i++)
    {
        arr.append(file[i]);
        arr.append("\n");
    }

    textStateVectorRawData->ChangeValue(arr);

    UpdateOrbitData();

    file.Close();
}

void MyFrame::OnButton_StateVector_Save(wxCommandEvent& event)
{
    wxString project = textProjectFile->GetLineText(0);
    wxString filename = textStateVectorFileName->GetLineText(0);

    if (project == "" || filename == "") return;

    wxString filepath = "Projects/" + project + "/State Vectors/" + filename;

    wxTextFile file(filepath);
    if (!file.Open())
        return;

    file.Clear();

    for (int i = 0; i < textStateVectorRawData->GetNumberOfLines(); i++)
    {
        file.AddLine(textStateVectorRawData->GetLineText(i));
    }

    file.Write();
    file.Close();
}

void MyFrame::OnButton_StateVector_New(wxCommandEvent& event)
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

        //Write to log text box
        textStateVectorRawData->Clear();

        wxString arr;

        size_t i;
        for (i = 0; i < file.GetLineCount(); i++)
        {
            arr.append(file[i]);
            arr.append("\n");
        }

        textStateVectorRawData->ChangeValue(arr);

        file.Close();

        ReloadStateVectorPage();
        UpdateOrbitData();

        wxMessageBox(filename, wxT("File created!"));
    }
}

void MyFrame::UpdateOrbitData()
{
    wxString str;
    int coord, lines, required_lines;

    textStateVectorOrbitData->Clear();

    lines = textStateVectorRawData->GetNumberOfLines();

    //Coordinate system
    str = textStateVectorRawData->GetLineText(0);

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
    else if (str == "TLE")
    {
        coord = 3;
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

    int i;

    if (coord <= 2)
    {
        //TEG and J2000
        str = textStateVectorRawData->GetLineText(1);
        textStateVectorOrbitData->AppendText("Position vector: " + str + "\n");

        str = textStateVectorRawData->GetLineText(2);
        textStateVectorOrbitData->AppendText("Velocity vector: " + str + "\n");

        str = textStateVectorRawData->GetLineText(3);
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
        i = 4;
    }
    else
    {
        //TLE
        str = textStateVectorRawData->GetLineText(1);
        textStateVectorOrbitData->AppendText("Line 1: " + str + "\n");

        str = textStateVectorRawData->GetLineText(2);
        textStateVectorOrbitData->AppendText("Line 2: " + str + "\n");

        i = 3;
    }

    //Weight
    str = textStateVectorRawData->GetLineText(i);
    textStateVectorOrbitData->AppendText("Weight: " + str + " lbs\n");

    //Area
    str = textStateVectorRawData->GetLineText(i + 1);
    textStateVectorOrbitData->AppendText("Area: " + str + " square feet\n");

    //K-Factor
    str = textStateVectorRawData->GetLineText(i + 2);
    textStateVectorOrbitData->AppendText("Drag multiplier: " + str + "\n");

    //Age of state vector
    if (SetConstants()) return;

    str = textStateVectorFileName->GetLineText(0);

    double age;

    std::string data[2];

    data[0] = StringFromTextBox(textProjectFile);
    data[1] = std::string(str.mb_str());

    if (core->AgeOfStateVector(data, age)) return;

    str = wxString::Format(wxT("%.0lf"), age / 3600.0);

    textStateVectorOrbitData->AppendText("Time from midnight to state vector is " +  str + " hours\n");
}

void MyFrame::StateVectorRawDataEnter(wxCommandEvent& event)
{
    UpdateOrbitData();
}

int MyFrame::SetConstants()
{
    if (core == NULL)
    {
        core = new Core();
    }

    std::string inputs[3];

    inputs[0] = StringFromTextBox(textYear);
    inputs[1] = StringFromTextBox(textMonth);
    inputs[2] = StringFromTextBox(textDay);

    int Year, Month, Day;

    Year = std::stoi(inputs[0]);
    Month = std::stoi(inputs[1]);
    Day = std::stoi(inputs[2]);

    return core->SetConstants(comboWorld->GetSelection(), Year, Month, Day);
}

void MyFrame::CalculateDayOfYear(wxCommandEvent& event)
{
    if (core == NULL)
    {
        core = new Core();
    }

    std::string inputs[3];

    inputs[0] = StringFromTextBox(textYear);
    inputs[1] = StringFromTextBox(textMonth);
    inputs[2] = StringFromTextBox(textDay);

    int Year, Month, Day, DOY;

    try
    {
        Year = std::stoi(inputs[0]);
        Month = std::stoi(inputs[1]);
        Day = std::stoi(inputs[2]);
    }
    catch (std::invalid_argument)
    {
        textDayOfYear->ChangeValue("Invalid date!");
        return;
    }

    if (core->CalculateDayOfYear(Year, Month, Day, DOY))
    {
        textDayOfYear->ChangeValue("Invalid date!");
        return;
    }

    wxString mystring = wxString::Format(wxT("%i"), DOY);
    textDayOfYear->ChangeValue(mystring);
}

void MyFrame::OnButton_LWP_LVDC_Export(wxCommandEvent& event)
{
    if (core == NULL)
    {
        SetStatusText("Launch day not initialized!");
        return;
    }

    if (core->LWPExportForLVDC())
    {
        SetStatusText("Error exporting LVDC data!");
        return;
    }

    SetStatusText("LVDC data successfully exported!");
}

void MyFrame::OnButton_LWP_SSV_Export(wxCommandEvent& event)
{
    if (core == NULL)
    {
        SetStatusText("Launch day not initialized!");
        return;
    }

    if (core->LWPExportForSSV())
    {
        SetStatusText("Error exporting SSV data!");
        return;
    }

    SetStatusText("SSV data successfully exported!");
}

void MyFrame::OnButton_View_MCT(wxCommandEvent& event)
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

    wxTextFile file(filepath);
    if (!file.Open())
    {
        SetStatusText("MCT file could not be opened!");
        return;
    }

    wxString arr, token;
    wxStringTokenizer tkz;
    unsigned i, j;
   
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

void MyFrame::OnButton_Save_MCT(wxCommandEvent& event)
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

    if (wxFile::Exists(filepath))
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

    //Write to file
    wxTextFile file(filepath);
    if (!file.Open())
        return;

    file.Clear();

    for (i = 0; i < array.size(); i++)
    {
        file.AddLine(array[i]);
    }

    file.Write();
    file.Close();
}