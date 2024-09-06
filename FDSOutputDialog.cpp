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

#include "FDSOutputDialog.h"
#include <wx/stattext.h>
#include <wx/statline.h>

FDSOutputDialog::FDSOutputDialog(const wxString& title, const std::vector<std::string>& data) : wxDialog(NULL, wxID_ANY, title,
    wxDefaultPosition, wxSize(768, 768), wxDEFAULT_DIALOG_STYLE | wxRESIZE_BORDER)
{

    wxString arr;

    for (unsigned i = 0; i < data.size(); i++)
    {
        arr.append(data[i]);
        arr.append("\n");
    }

    wxTextCtrl* text = new wxTextCtrl(this, wxID_ANY, arr, wxDefaultPosition, wxDefaultSize, wxTE_MULTILINE | wxTE_READONLY);

    wxFont font = text->GetFont();

    font.SetFamily(wxFONTFAMILY_TELETYPE);

    text->SetFont(font);
}