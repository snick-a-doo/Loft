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

M3 units::I_cylinder_shell(double m, double r, double l)
{
    return M3(m*(6*r*r + l*l)/12*Vx, m*(6*r*r + l*l)/12*Vy, m*r*r*Vz);
}

M3 units::I_cylinder_solid(double m, double r, double l)
{
    return M3(m*(3*r*r + l*l)/12*Vx, m*(3*r*r + l*l)/12*Vy, m*r*r/2*Vz);
}

double units::V_cylinder(double r, double l)
{
    using namespace std::numbers;
    return pi*r*r*l;
}
