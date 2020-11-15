#include "test.hh"
#include "units.hh"
#include "universe.hh"
#include "world.hh"

#include "doctest.h"

#include <fstream>
#include <numbers>

using namespace std::numbers;

TEST_CASE("Earth")
{
    auto orientation = rot(M1, units::deg(23.44)*Vy);
    auto earth = std::make_shared<World>(5.972e24, 6.371e6, V0, V0, orientation,
                                         units::day(1.0));
    double theta = 23.44*(pi/180.0);
    CHECK(earth->orientation() == rot(M1, theta*Vy));
    earth->step(8616.41); // 0.1 siderial day
    CHECK(close(earth->orientation(), rot(rot(M1, 0.2*pi*Vz), theta*Vy), 1e-9));
}

TEST_CASE("Earth and Moon")
{
    auto orientation = rot(M1, units::deg(23.44)*Vy);
    auto earth = std::make_shared<World>(5.972e24, 6.371e6, V0, V0, orientation,
                                         units::day(1.0));
    double apogee = 4.054e8;
    double perigee = 3.626e8;
    auto moon = std::make_shared<World>(7.342e22, 1.737e6, 4.054e8*Vx, 0.97e3*Vy, M1,
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
