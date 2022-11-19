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

#ifndef LOFT_LOFTLIB_ROCKET_HH_INCLUDED
#define LOFT_LOFTLIB_ROCKET_HH_INCLUDED

#include "body.hh"

#include <memory>

class Engine;
class Fuel;
class V3;

/// A liquid-fueled rocket with orientable engine.
class Rocket : public Body
{
public:
    /// @param shell_mass The mass of the rocket minus fuel and engine.
    /// @param engine_mass The mass of the engine.
    /// @param radius The radius of the cylindrical rocket body.
    /// @param length The length of the cylindrical rocket body.
    /// @param fuel_density The density of the fuel.
    /// @param specific_impulse The maximum achievable impulse attainable by directing the
    /// products of burning one mass unit of fuel.
    /// @param fuel_rate The maximum volume rate of fuel usage.
    Rocket(double shell_mass, double engine_mass,
           double radius, double length,
           double fuel_density, double specific_impulse, double fuel_rate,
           V3 const& position, M3 const& orientation);
    ~Rocket() = default;

    /// Set the engine throttle.
    /// @param frac Fraction of full throttle.
    void throttle(double frac);
    /// Set the direction of engine thrust.
    /// @param v Rotate thrust from the z-direction about the rocket-frame axis v by the
    /// amount equal to the length of v in radians.  Call with V0 to set the thrust
    /// direction to the rocket's z-axis.
    void orient_thrust(V3 const& v);
    /// @return The volume of fuel left in the tank.
    double fuel_volume() const;
    virtual void step(double time) override;

private:
    std::shared_ptr<Engine> m_engine;
    std::shared_ptr<Fuel> m_fuel;
};

#endif // LOFT_LOFTLIB_ROCKET_HH_INCLUDED
