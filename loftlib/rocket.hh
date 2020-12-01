#ifndef LOFT_LOFTLIB_ROCKET_HH_INCLUDED
#define LOFT_LOFTLIB_ROCKET_HH_INCLUDED

#include "body.hh"

class Engine;
class Fuel;

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
           const V3& position, const M3& orientation);
    ~Rocket() = default;

    /// Set the engine throttle.
    /// @param frac Fraction of full throttle.
    void throttle(double frac);
    /// Set the direction of engine thrust.
    /// @param v Rotate thrust from the z-direction about the rocket-frame axis v by the
    /// amount equal to the length of v in radians.  Call with V0 to set the thrust
    /// direction to the rocket's z-axis.
    void orient_thrust(const V3& v);
    /// @return The volume of fuel left in the tank.
    double fuel_volume() const;
    virtual void step(double time) override;

private:
    std::shared_ptr<Engine> m_engine;
    std::shared_ptr<Fuel> m_fuel;
};

#endif // LOFT_LOFTLIB_ROCKET_HH_INCLUDED
