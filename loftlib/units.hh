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

#ifndef LOFT_LOFTLIB_UNITS_HH_INCLUDED
#define LOFT_LOFTLIB_UNITS_HH_INCLUDED

#include "three-vector.hh"

#include <numbers>

/// Functions that convert to internal units.  The function names give the units to
/// convert from.  Internal units are m, kg, s, rad.
namespace units
{
    /// @param degrees Angle in degrees
    /// @return Angle in radians.
    constexpr double deg(double degrees)
    {
        using namespace std::numbers;
        return degrees*pi/180.0;
    }
    constexpr double dms(double degrees, double minutes, double seconds)
    {
        if (degrees < 0.0)
        {
            minutes = -std::abs(minutes);
            seconds = -std::abs(seconds);
        }
        return deg(degrees + minutes/60 + seconds/3600);
    }

    /// @param day A span of time in siderial Earth days.
    /// @return Time in seconds
    constexpr double day(double days)
    {
        return days*86'164.1;
    }

    M3 I_cylinder_shell(double m, double r, double l);
    M3 I_cylinder_solid(double m, double r, double l);
    double V_cylinder(double r, double l);
}

namespace consts
{
    static constexpr double G = 6.67430e-11; ///< Gravitational constant: m³/s²kg
    static constexpr double m_earth = 5.972e24; ///< Earth mass: kg
    static constexpr double r_earth = 6.361e6; ///< Mean Earth radius: m
    static constexpr double m_moon = 7.342e22; ///< Moon mass: kg
    static constexpr double r_moon = 1.737e6; ///< Mean Moon radius: m
}
#endif // LOFT_LOFTLIB_UNITS_HH_INCLUDED
