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

#include "rocket.hh"
#include "units.hh"

#include <numbers>

using namespace std::numbers;

/// A cylindrical volume of fuel.  The fuel is treated as a solid slug that remains at the
/// bottom (minimum z) of the fuel tank.
class Fuel : public Body
{
public:
    /// @param radius The inner radius of the fuel tank.
    /// @param depth The initial depth of fuel in the tank.
    /// @param density The density of the fuel.
    Fuel(double radius, double depth, double density, double impulse);
    virtual ~Fuel() = default;

    /// Get the impulse attainable from a requested volume of fuel.
    /// @param volume The amount of fuel to use.  The actual amount of fuel consumed will
    /// be the amount in the tank if that's less than the argument.
    /// @return The impulse attainable from the actual amount of fuel consumed.
    double get_impulse(double volume);
    double volume() const { return m_depth*m_area; }

private:
    const double m_density;
    const double m_radius;
    const double m_area;
    const double m_full_depth; ///< The initial depth of fuel.
    double m_depth; ///< The current depth of fuel.
    const double m_impulse;
};

Fuel::Fuel(double radius, double depth, double density, double impulse)
    : Body(density*units::V_cylinder(radius, depth), M1, V0, V0, M1, V0),
      m_density(density),
      m_radius(radius),
      m_area(pi*radius*radius),
      m_full_depth(depth),
      m_depth(depth),
      m_impulse(impulse)
{
    // Set the correct inertia tensor now that the mass is available.
    set_inertia(units::I_cylinder_solid(m(), radius, depth));
}

double Fuel::get_impulse(double volume)
{
    auto V{m_depth*m_area};
    auto dV{std::min(V, volume)};
    V -= dV;
    set_mass(V*m_density);
    m_depth = V/m_area;
    // Keep the fuel at the bottom of the tank.
    set_r((m_depth - m_full_depth)/2*Vz);
    set_inertia(units::I_cylinder_solid(m(), m_radius, m_depth));
    return m_impulse*m_density*dV;
}


/// A steerable, throttleable rocket engine.
class Engine : public Body
{
public:
    /// @param mass The mass of the engine.
    /// @param fuel_rate The volume rate of fuel usage at full throttle.
    /// @param efficiency The fraction of available impulse converted to thrust.
    Engine(double mass, double fuel_rate, double efficiency);

    /// @param frac The fraction of full throttle.
    void throttle(double frac);
    /// Set the direction of engine thrust.
    /// @param v Rotate thrust from the z-direction about the rocket-frame axis and angle.
    void orient(V3 const& v);
    /// @return The amount of fuel consumed in the given time interval.
    double consumed(double time) const;
    /// @return The impulse vector to apply to the rocket.
    /// @param max_impulse The maximum impulse available from the fuel.
    V3 get_impulse(double max_impulse) const;

private:
    double m_fuel_rate;
    double m_efficiency;
    double m_throttle = 0.0;
};

Engine::Engine(double mass, double fuel_rate, double efficiency)
    : Body(mass, M1, V0, V0, M1, V0),
      m_fuel_rate(fuel_rate),
      m_efficiency(efficiency)
{
}

void Engine::throttle(double k)
{
    m_throttle = k;
}

void Engine::orient(V3 const& v)
{
    // The engine always produces thrust in its z-direction.
    set_orientation(rot(M1, v));
}

double Engine::consumed(double time) const
{
    return m_throttle*m_fuel_rate*time;
}

V3 Engine::get_impulse(double max_impulse) const
{
    return max_impulse*m_efficiency*rotate_out(Vz);
}


Rocket::Rocket(double shell_mass, double engine_mass, double radius, double length,
               double fuel_density, double spec_impulse, double fuel_rate,
               V3 const& position, M3 const& orientation)
    : Body(shell_mass,
           units::I_cylinder_shell(shell_mass, radius, length),
           position, V0, M1, V0),
      m_engine(std::make_shared<Engine>(engine_mass, fuel_rate, 1.0)),
      m_fuel(std::make_shared<Fuel>(radius, length, fuel_density, spec_impulse))
{
    capture(m_engine);
    capture(m_fuel);
    // Set position relative to the rocket after capturing.
    m_engine->set_r(-length/2*Vz);
    set_orientation(orientation);
}

void Rocket::throttle(double frac)
{
    m_engine->throttle(frac);
}

void Rocket::orient_thrust(V3 const& v)
{
    m_engine->orient(v);
}

double Rocket::fuel_volume() const
{
    return m_fuel->volume();
}

void Rocket::step(double time)
{
    if (is_free())
    {
        auto volume{m_engine->consumed(time)};
        auto imp{m_engine->get_impulse(m_fuel->get_impulse(volume))};
        impulse(imp, transform_out(m_engine->r_cm()));
    }
    Body::step(time);
}
