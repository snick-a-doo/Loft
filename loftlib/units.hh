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

    /// @param day A span of time in siderial Earth days.
    /// @return Time in seconds
    constexpr double day(double days)
    {
        return days*8'6164.1;
    }
}
#endif // LOFT_LOFTLIB_UNITS_HH_INCLUDED
