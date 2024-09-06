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

#include "cmath"

//Orbiter Space Flight Simulator vector math code

typedef union {
	double data[3];               ///< array data interface
	struct { double x, y, z; };   ///< named data interface
} VECTOR3;

typedef union {
	double data[9];               ///< array data interface (row-sorted)
	struct { double m11, m12, m13, m21, m22, m23, m31, m32, m33; }; ///< named data interface
} MATRIX3;

inline VECTOR3 _V(double x, double y, double z)
{
	VECTOR3 vec = { x,y,z }; return vec;
}

inline double length(const VECTOR3& a)
{
	return sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
}

inline double dotp(const VECTOR3& a, const VECTOR3& b)
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline VECTOR3 operator+ (const VECTOR3& a, const VECTOR3& b)
{
	VECTOR3 c;
	c.x = a.x + b.x;
	c.y = a.y + b.y;
	c.z = a.z + b.z;
	return c;
}

inline VECTOR3 operator- (const VECTOR3& a, const VECTOR3& b)
{
	VECTOR3 c;
	c.x = a.x - b.x;
	c.y = a.y - b.y;
	c.z = a.z - b.z;
	return c;
}

inline VECTOR3 operator* (const VECTOR3& a, const double f)
{
	VECTOR3 c;
	c.x = a.x * f;
	c.y = a.y * f;
	c.z = a.z * f;
	return c;
}

inline VECTOR3 operator/ (const VECTOR3& a, const double f)
{
	VECTOR3 c;
	c.x = a.x / f;
	c.y = a.y / f;
	c.z = a.z / f;
	return c;
}

inline VECTOR3& operator+= (VECTOR3& a, const VECTOR3& b)
{
	a.x += b.x;
	a.y += b.y;
	a.z += b.z;
	return a;
}

inline VECTOR3& operator-= (VECTOR3& a, const VECTOR3& b)
{
	a.x -= b.x;
	a.y -= b.y;
	a.z -= b.z;
	return a;
}

inline VECTOR3& operator*= (VECTOR3& a, const double f)
{
	a.x *= f;
	a.y *= f;
	a.z *= f;
	return a;
}

inline VECTOR3& operator/= (VECTOR3& a, const double f)
{
	a.x /= f;
	a.y /= f;
	a.z /= f;
	return a;
}

inline VECTOR3 operator- (const VECTOR3& a)
{
	VECTOR3 c;
	c.x = -a.x;
	c.y = -a.y;
	c.z = -a.z;
	return c;
}

inline VECTOR3 unit(const VECTOR3& a)
{
	return a / length(a);
}

inline VECTOR3 crossp(const VECTOR3& a, const VECTOR3& b)
{
	return _V(a.y * b.z - b.y * a.z, a.z * b.x - b.z * a.x, a.x * b.y - b.x * a.y);
}

inline MATRIX3 _M(double m11, double m12, double m13,
	double m21, double m22, double m23,
	double m31, double m32, double m33)
{
	MATRIX3 mat = { m11,m12,m13,  m21,m22,m23,  m31,m32,m33 };
	return mat;
}

inline VECTOR3 mul(const MATRIX3& A, const VECTOR3& b)
{
	return _V(
		A.m11 * b.x + A.m12 * b.y + A.m13 * b.z,
		A.m21 * b.x + A.m22 * b.y + A.m23 * b.z,
		A.m31 * b.x + A.m32 * b.y + A.m33 * b.z);
}

inline MATRIX3 mul(const MATRIX3& A, const MATRIX3& B)
{
	MATRIX3 mat = {
		A.m11 * B.m11 + A.m12 * B.m21 + A.m13 * B.m31, A.m11 * B.m12 + A.m12 * B.m22 + A.m13 * B.m32, A.m11 * B.m13 + A.m12 * B.m23 + A.m13 * B.m33,
		A.m21 * B.m11 + A.m22 * B.m21 + A.m23 * B.m31, A.m21 * B.m12 + A.m22 * B.m22 + A.m23 * B.m32, A.m21 * B.m13 + A.m22 * B.m23 + A.m23 * B.m33,
		A.m31 * B.m11 + A.m32 * B.m21 + A.m33 * B.m31, A.m31 * B.m12 + A.m32 * B.m22 + A.m33 * B.m32, A.m31 * B.m13 + A.m32 * B.m23 + A.m33 * B.m33
	};
	return mat;
}

inline VECTOR3 tmul(const MATRIX3& A, const VECTOR3& b)
{
	return _V(
		A.m11 * b.x + A.m21 * b.y + A.m31 * b.z,
		A.m12 * b.x + A.m22 * b.y + A.m32 * b.z,
		A.m13 * b.x + A.m23 * b.y + A.m33 * b.z);
}

static inline double sign(double x)
{
	if (x >= 0.0) return 1.0;
	else return -1.0;
}