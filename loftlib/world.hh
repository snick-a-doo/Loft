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

#ifndef LOFT_LOFTLIB_WORLD_HH_INCLUDED
#define LOFT_LOFTLIB_WORLD_HH_INCLUDED

#include "body.hh"

#include <tuple>

/// A large spherical body, such as a planet or moon.
class World : public Body
{
public:
    /// @param period The period of rotation in seconds. units::day() can be used to
    /// convert from days.
    World(double mass, double radius, V3 const& r, V3 const& v,
          M3 const& orientation, double period);
    virtual ~World() = default;

    /// @return The body's radius.
    double radius() const;
    virtual bool intersects(Body const& b) const override;
    /// @return Absolute coordinates and orientation for the given latitude, longitude,
    /// and altitude.  The orientation has z normal to the surface and y north.
    std::tuple<V3, M3> locate(double lat, double lon, double alt) const;
    /// @return Latitude, longitude, and altitude for a given position.
    std::tuple<double, double, double> location(V3 const& r);

private:
    double m_radius;
};

#endif // LOFT_LOFTLIB_WORLD_HH_INCLUDED
