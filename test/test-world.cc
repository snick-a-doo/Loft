#include "test.hh"
#include "units.hh"
#include "universe.hh"
#include "world.hh"

#include "doctest.h"

#include <numbers>

using namespace std::numbers;
using namespace consts;

TEST_CASE("Earth")
{
    auto orientation = rot(M1, units::deg(23.44)*Vy);
    auto earth = std::make_shared<World>(m_earth, r_earth, V0, V0, orientation,
                                         units::day(1.0));
    double theta = 23.44*(pi/180.0);
    CHECK(earth->orientation() == rot(M1, theta*Vy));
    earth->step(8616.41); // 0.1 siderial day
    CHECK(close(earth->orientation(), rot(rot(M1, theta*Vy), 0.2*pi*Vz), 1e-9));
}

TEST_CASE("Earth and Moon")
{
    auto orientation = rot(M1, units::deg(23.44)*Vy);
    auto earth = std::make_shared<World>(m_earth, r_earth, V0, V0, orientation,
                                         units::day(1.0));
    double apogee = 4.054e8;
    double perigee = 3.626e8;
    auto moon = std::make_shared<World>(m_moon, r_moon, 4.054e8*Vx, 0.97e3*Vy, M1,
                                         units::day(27.32));
    Universe all;
    all.add(earth);
    all.add(moon);

    CHECK(unit(moon->rotate_in(earth->r() - moon->r())) == -Vx);

    for (int i; i < 2732; ++i)
        all.step(units::day(1.0)/200.0);
    auto r_me = moon->r() - earth->r();
    // Don't expect too much accuracy.
    CHECK(close(moon->rotate_in(r_me), perigee*Vx, 0.05e8));

    for (int i; i < 2732; ++i)
        all.step(units::day(1.0)/200.0);
    r_me = moon->r() - earth->r();
    CHECK(close(moon->rotate_in(r_me), apogee*Vx, 0.05e8));
}

TEST_CASE("Body on Earth")
{
    auto orientation = rot(M1, units::deg(23.44)*Vy);
    auto earth = std::make_shared<World>(m_earth, r_earth, V0, V0, orientation,
                                         units::day(1.0));
    auto b = std::make_shared<Body>(2.0, M1, r_earth*Vx, V0, M1, V0);
    Universe all;
    all.add(earth);
    all.add(b);
    earth->capture(b);
    auto axis = earth->rotate_out(Vz);
    CHECK(axis == V3(sin(units::deg(23.44)), 0, cos(units::deg(23.44))));
    all.step(8616.41); // 0.1 siderial day
    auto axis2 = earth->rotate_out(Vz);
    // Check that the axis of rotation is fixed.
    CHECK(close(axis2, axis, 1e-9));
    // Check that the body moves with the surface of the earth.
    CHECK(close(earth->transform_out(b->r()), rot(r_earth*Vx, 0.2*pi*axis), 1e-9));
}

TEST_CASE("locate")
{
    using units::deg;
    auto tilt_0 = std::make_shared<World>(m_earth, r_earth, V0, V0, M1, 0);
    // Zero longitude is in the y-direction to match the gluSphere texture origin.
    SUBCASE("origin")
    {
        auto [r, o] = tilt_0->locate(0.0, 0.0, 0.0);
        CHECK(close(r, r_earth*Vy, 1e-9));
        CHECK(close(o*Vx, -Vx, 1e-9));
        CHECK(close(o*Vy, Vz, 1e-9));
        CHECK(close(o*Vz, Vy, 1e-9));
    }
    SUBCASE("lat 45N, lon 0")
    {
        auto [r, o] = tilt_0->locate(deg(45.0), 0.0, 0.0);
        CHECK(close(r, sqrt2/2*r_earth*V3(0, 1, 1), 1e-9));
        CHECK(close(o*Vx, -Vx, 1e-9));
        CHECK(close(o*Vy, sqrt2/2*V3(0, -1, 1), 1e-9));
        CHECK(close(o*Vz, sqrt2/2*V3(0, 1, 1), 1e-9));
    }
    SUBCASE("lat 45S, lon 90E")
    {
        auto [r, o] = tilt_0->locate(deg(-45.0), deg(90.0), 0.0);
        CHECK(close(r, sqrt2/2*r_earth*V3(-1, 0, -1), 1e-9));
        CHECK(close(o*Vx, -Vy, 1e-9));
        CHECK(close(o*Vy, sqrt2/2*V3(-1, 0, 1), 1e-9));
        CHECK(close(o*Vz, sqrt2/2*V3(-1, 0, -1), 1e-9));
    }
    SUBCASE("lat 45S, lon 90W")
    {
        auto [r, o] = tilt_0->locate(deg(-45.0), deg(-90.0), r_earth);
        CHECK(close(r, sqrt2*r_earth*V3(1, 0, -1), 1e-6));
        CHECK(close(o*Vx, Vy, 1e-9));
        CHECK(close(o*Vy, sqrt2/2*V3(1, 0, 1), 1e-9));
        CHECK(close(o*Vz, sqrt2/2*V3(1, 0, -1), 1e-9));
    }

    auto orientation = rot(M1, deg(45)*Vy);
    auto tilt_45 = std::make_shared<World>(m_earth, r_earth, V0, V0, orientation, 4);
    SUBCASE("tilt origin")
    {
        auto [r, o] = tilt_45->locate(0.0, 0.0, 0.0);
        CHECK(close(r, r_earth*Vy, 1e-9));
        CHECK(close(o*Vx, sqrt2/2*V3(-1, 0, 1), 1e-9));
        CHECK(close(o*Vy, sqrt2/2*V3(1, 0, 1), 1e-9));
        CHECK(close(o*Vz, Vy, 1e-9));
    }
    SUBCASE("tilt lat 0, lon 90E")
    {
        auto [r, o] = tilt_45->locate(0.0, deg(90.0), 0.0);
        CHECK(close(r, sqrt2/2*r_earth*V3(-1, 0, 1), 1e-6));
        CHECK(close(o*Vx, -Vy, 1e-9));
        CHECK(close(o*Vy, sqrt2/2*V3(1, 0, 1), 1e-9));
        CHECK(close(o*Vz, sqrt2/2*V3(-1, 0, 1), 1e-9));
    }
    SUBCASE("tilt lat 45S, lon 90E")
    {
        auto [r, o] = tilt_45->locate(deg(-45.0), deg(90.0), 0.0);
        CHECK(close(r, -r_earth*Vx, 1e-6));
        CHECK(close(o*Vx, -Vy, 1e-9));
        CHECK(close(o*Vy, Vz, 1e-9));
        CHECK(close(o*Vz, -Vx, 1e-9));
    }
    SUBCASE("tilt lat 45N, lon 90W")
    {
        auto [r, o] = tilt_45->locate(deg(45.0), deg(-90.0), r_earth);
        CHECK(close(r, 2*r_earth*Vx, 1e-6));
        CHECK(close(o*Vx, Vy, 1e-9));
        CHECK(close(o*Vy, Vz, 1e-9));
        CHECK(close(o*Vz, Vx, 1e-9));
    }
    SUBCASE("tilt lat 45S, lon 0 quarter day")
    {
        // Same as 45S, lon 90E
        tilt_45->step(1);
        auto [r, o] = tilt_45->locate(deg(-45.0), 0.0, 0.0);
        CHECK(close(r, -r_earth*Vx, 1e-6));
        CHECK(close(o*Vx, -Vy, 1e-9));
        CHECK(close(o*Vy, Vz, 1e-9));
        CHECK(close(o*Vz, -Vx, 1e-9));
    }
}
