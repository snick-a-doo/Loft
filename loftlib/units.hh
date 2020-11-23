#ifndef LOFT_LOFTLIB_UNITS_HH_INCLUDED
#define LOFT_LOFTLIB_UNITS_HH_INCLUDED

#include <numbers>

using namespace std::numbers;

/// Functions that convert to internal units.  The function names give the units to
/// convert from.  Internal units are m, kg, s, rad.
namespace units
{
    /// @param degrees Angle in degrees
    /// @return Angle in radians.
    constexpr double deg(double degrees)
    {
        return degrees*pi/180.0;
    }
    constexpr double dms(double degrees, double minutes, double seconds)
    {
        if (degrees < 0)
        {
            minutes = -std::abs(minutes);
            seconds = -std::abs(seconds);
        }
        return degrees + minutes/60 + seconds/3600;
    }

    /// @param day A span of time in siderial Earth days.
    /// @return Time in seconds
    constexpr double day(double days)
    {
        return days*8'6164.1;
    }
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
