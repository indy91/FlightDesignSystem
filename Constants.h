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

namespace OrbMech
{
	const double PI = 3.14159265358979323846;	/// pi (180 deg)
	const double PI05 = 1.57079632679489661923;	/// pi/2 (90 deg)
	const double PI2 = 6.28318530717958647693;	/// pi*2 (360 deg)
	const double PI302 = PI * 1.5;				/// 3*pi/2 (270 deg)
	const double RAD = PI / 180.0;				/// factor to map degrees to radians
	const double DEG = 180.0 / PI;				/// factor to map radians to degrees
	const double NM = 1852.0;					/// meters in nautical miles
	const double AU = 149597870700.0;			/// Astronomical unit in degrees
	const double LBS = 0.45359237;				/// 1 lbs in kg
	const double FT2M = 0.3048;					/// 1 ft in meters
}