#include <rocket.hh>
#include <units.hh>

#include "test.hh"

#include "doctest.h"

#include <numbers>

using namespace std::numbers;

TEST_CASE("fuel")
{
    double m_s = 10;
    double m_e = 50;
    double r = 0.5;
    double l = 10;
    double rho = 1.5;
    double impulse = 0; // no thrust
    double rate = 0.01;
    Rocket rocket(m_s, m_e, r, l, rho, impulse, rate, V0, M1);

    double V = units::V_cylinder(r, l);
    double m_fuel = rho*V;
    double mass = m_s + m_e + m_fuel;
    CHECK(rocket.m() == mass);
    CHECK(rocket.r_cm() == -m_e*l/2*Vz/mass); // Σmr/Σm

    rocket.step(10);
    CHECK(rocket.r_cm() == -m_e*l/2*Vz/mass); // Σmr/Σm
    CHECK(rocket.m() == mass);

    // Use half the fuel.
    rocket.throttle(1.0);
    rocket.step(V/2/rate);
    m_fuel /= 2;
    mass = m_s + m_e + m_fuel;
    // Fuel stays at the bottom.  r_cm() is absolute, but since impulse=0, the rocket
    // doesn't go anywhere.
    CHECK(rocket.r_cm() == -(m_e*l/2 + m_fuel*l/4)*Vz/mass);
    CHECK(rocket.m() == mass);

    // Use half the fuel remaining.
    rocket.throttle(0.5);
    rocket.step(V/2/rate);
    m_fuel /= 2;
    mass = m_s + m_e + m_fuel;
    CHECK(rocket.r_cm() == -(m_e*l/2 + m_fuel*l*3/8)*Vz/mass);
    CHECK(rocket.m() == mass);

    // Use the rest of the fuel.
    rocket.step(V/2/rate);
    mass = m_s + m_e;
    CHECK(rocket.r_cm() == -m_e*l/2*Vz/mass);
    CHECK(rocket.m() == mass);
    rocket.step(100);
    CHECK(rocket.r_cm() == -m_e*l/2*Vz/mass);
    CHECK(rocket.m() == mass);
}

TEST_CASE("thrust")
{
    double m_s = 10;
    double m_e = 50;
    double r = 0.5;
    double l = 10;
    double rho = 1.5;
    double impulse = 1e3;
    double rate = 0.01;
    auto o = rot(M1, pi/2*Vy);
    Rocket rocket(m_s, m_e, r, l, rho, impulse, rate, V0, o);

    // Rocket is oriented along +x
    auto r_cm = rocket.r_cm();
    auto v_cm = rocket.v_cm();
    CHECK(r_cm.x < 0); // CM is negative because of the engine.
    CHECK(r_cm.y == 0);
    CHECK(close(r_cm.z, 0.0, 1e-9));
    CHECK(v_cm == V0);
    CHECK(rocket.orientation() == o);

    // Rocket moves in +x
    rocket.throttle(0.5);
    rocket.step(10);
    r_cm = rocket.r_cm();
    v_cm = rocket.v_cm();
    CHECK(r_cm.x > 0);
    CHECK(r_cm.y == 0);
    CHECK(close(r_cm.z, 0.0, 1e-9));
    CHECK(v_cm.x > 0);
    CHECK(v_cm.y == 0);
    CHECK(close(v_cm.z, 0.0, 1e-9));
    CHECK(rocket.orientation() == o);

    // Thrust is in the rocket's frame.  Rotate from absolute +x toward -y, causing
    // rotation about +z.
    rocket.orient_thrust(units::deg(2)*Vx);
    for (int i = 0; i < 10; ++i)
        rocket.step(1);
    r_cm = rocket.r_cm();
    v_cm = rocket.v_cm();
    CHECK(r_cm.x > 1);
    CHECK(r_cm.y < -1e-3);
    CHECK(close(r_cm.z, 0.0, 1e-9));
    CHECK(v_cm.x > 1);
    CHECK(v_cm.y < -1e-3);
    CHECK(close(v_cm.z, 0.0, 1e-9));
    auto x = rocket.rotate_in(Vx);
    CHECK(close(x.x, 0.0, 1e-9));
    CHECK(x.y < -1e-3);
    CHECK(x.z > 0.9);
    CHECK(close(rocket.omega().x, 0.0, 1e-9));
    CHECK(close(rocket.omega().y, 0.0, 1e-9));
    CHECK(rocket.omega().z > 1e-3);
}
