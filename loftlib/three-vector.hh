//  Copyright (C) 2022 Sam Varner
//
//  This file is part of Laft.
//
//  Loft is free software: you can redistribute it and/or modify it under the terms of
//  the GNU General Public License as published by the Free Software Foundation, either
//  version 3 of the License, or (at your option) any later version.
//
//  Vamos is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
//  without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
//  PURPOSE.  See the GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License along with Vamos.
//  If not, see <http://www.gnu.org/licenses/>.

#ifndef LOFT_LOFTLIB_THREE_VECTOR_HH_INCLUDED
#define LOFT_LOFTLIB_THREE_VECTOR_HH_INCLUDED

#include <cmath>
#include <iostream>
#include <tuple>

struct V3
{
    double x, y, z;
    double const& operator[](std::size_t i) const;
    double& operator[](std::size_t i);
    friend bool operator==(V3 const& v1, V3 const& v2) = default;
};

struct M3
{
    V3 x, y, z; // rows
    V3 const& operator[](std::size_t i) const;
    V3& operator[](std::size_t i);
    friend bool operator==(M3 const& m1, M3 const& m2) = default;
};

double dot(V3 const& v1, V3 const& v2);
double square(V3 const& v1);
V3 cross(V3 const& v1, V3 const& v2);
M3 outer(V3 const& v1, V3 const& v2);
double mag(V3 const& v);
V3 unit(V3 const& v);
// @return Vector v rotated about a by ||a|| radians.
V3 rot(V3 const& v, V3 const& a);
// @return Orientation matrix m rotated about a by ||a|| radians.
M3 rot(M3 const& m, V3 const& a);
double det(M3 const& m);
M3 inv(M3 const& m);
M3 tr(M3 const& m);
// @return An axis vector and an angle in radians.  The length of the vector is arbitrary.
std::tuple<V3, double> axis_angle(M3 const& m);

V3 operator-(V3 const& v);

V3 operator+(V3 const& v1, V3 const& v2);
V3 operator-(V3 const& v1, V3 const& v2);
V3 operator*(double c, V3 const& v);
V3 operator*(V3 const& v, double c);
V3 operator/(V3 const& v, double c);

V3& operator+=(V3& v1, V3 const& v2);
V3& operator-=(V3& v1, V3 const& v2);

M3 operator+(M3 const& m1, M3 const& m2);
M3 operator-(M3 const& m1, M3 const& m2);
M3 operator*(M3 const& m, double c);
M3 operator*(double c, M3 const& m);
V3 operator*(M3 const& m, V3 const& v);
V3 operator*(V3 const& v, M3 const& m);
M3 operator*(M3 const& m1, M3 const& m2);
M3 operator/(M3 const& v, double c);

M3& operator+=(M3& m1, M3 const& m2);

std::ostream& operator<<(std::ostream& os, V3 const& v);
std::ostream& operator<<(std::ostream& os, M3 const& m);

static constexpr V3 V0(0.0, 0.0, 0.0);
static constexpr V3 Vx(1.0, 0.0, 0.0);
static constexpr V3 Vy(0.0, 1.0, 0.0);
static constexpr V3 Vz(0.0, 0.0, 1.0);

static constexpr M3 M0(V0, V0, V0);
static constexpr M3 M1(Vx, Vy, Vz);

#endif // LOFT_LOFTLIB_THREE_VECTOR_HH_INCLUDED
