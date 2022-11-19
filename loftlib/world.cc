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

#include "units.hh"
#include "world.hh"

#include <numbers>

using namespace std::numbers;

World::World(double mass, double radius, V3 const& r, V3 const& v,
             M3 const& orientation, double period)
    // The orientation matrix aligns the z-axis with omega.
    : Body(mass, M0, r, v, orientation, orientation*(2*pi/period)*Vz),
      m_radius(radius)
{
}

double World::radius() const
{
    return m_radius;
}

bool World::intersects(Body const& b) const
{
    return mag(b.r() - r()) < m_radius;
}

std::tuple<V3, M3> World::locate(double lat, double lon, double alt) const
{
    // Zero longitude is in the y-direction to match the gluSphere texture origin.
    auto r{transform_out(rot(rot((m_radius + alt)*Vy, lat*Vx), lon*Vz))};
    // Construct the matrix for z up and x east.
    auto m{rot(rot(rot(M1, lon*Vz), (lat - pi/2)*Vx), pi*Vz)};
    return {r, orientation()*m};
}

std::tuple<double, double, double> World::location(V3 const& r)
{
    auto r_in{rotate_in(r)};
    auto r_xy{mag(V3(r_in.x, r_in.y, 0.0))};
    return {std::atan2(r_in.z, r_xy), std::atan2(-r_in.x, r_in.y), mag(r_in) - m_radius};
}
