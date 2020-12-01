#include "body.hh"
#include "test.hh"
#include "units.hh"

#include "doctest.h"

using units::deg;

TEST_CASE("origin")
{
    auto b1 = std::make_shared<Body>(1.0, M1, V0, V0, M1, V0);
    CHECK(b1->rotate_in(Vx) == Vx);
    CHECK(b1->rotate_in(Vy) == Vy);
    CHECK(b1->rotate_in(Vz) == Vz);
    CHECK(b1->rotate_out(Vx) == Vx);
    CHECK(b1->rotate_out(Vy) == Vy);
    CHECK(b1->rotate_out(Vz) == Vz);
    CHECK(b1->transform_in(Vx) == Vx);
    CHECK(b1->transform_in(Vy) == Vy);
    CHECK(b1->transform_in(Vz) == Vz);
    CHECK(b1->transform_out(Vx) == Vx);
    CHECK(b1->transform_out(Vy) == Vy);
    CHECK(b1->transform_out(Vz) == Vz);
}

TEST_CASE("translate")
{
    auto b1 = std::make_shared<Body>(1.0, M1, V3(1, 2, 3), V0, M1, V0);
    CHECK(b1->rotate_in(Vx) == Vx);
    CHECK(b1->rotate_in(Vy) == Vy);
    CHECK(b1->rotate_in(Vz) == Vz);
    CHECK(b1->rotate_out(Vx) == Vx);
    CHECK(b1->rotate_out(Vy) == Vy);
    CHECK(b1->rotate_out(Vz) == Vz);
    CHECK(b1->transform_in(Vx) == V3(0, -2, -3));
    CHECK(b1->transform_in(Vy) == V3(-1, -1, -3));
    CHECK(b1->transform_in(Vz) == V3(-1, -2, -2));
    CHECK(b1->transform_out(Vx) == V3(2, 2, 3));
    CHECK(b1->transform_out(Vy) == V3(1, 3, 3));
    CHECK(b1->transform_out(Vz) == V3(1, 2, 4));
}

TEST_CASE("rotate diagonal")
{
    auto r = V3(1, 2, 3);
    // x -> y, y -> z, z ->x
    auto o = rot(M1, deg(120)*unit(V3(1, 1, 1)));
    auto b1 = std::make_shared<Body>(1.0, M1, r, V0, o, V0);
    CHECK(close(b1->rotate_in(Vx), Vz, 1e-9));
    CHECK(close(b1->rotate_in(Vy), Vx, 1e-9));
    CHECK(close(b1->rotate_in(Vz), Vy, 1e-9));
    CHECK(close(b1->rotate_out(Vx), Vy, 1e-9));
    CHECK(close(b1->rotate_out(Vy), Vz, 1e-9));
    CHECK(close(b1->rotate_out(Vz), Vx, 1e-9));
    CHECK(close(b1->transform_in(Vx), V3(-2, -3, 0), 1e-9));
    CHECK(close(b1->transform_in(Vy), V3(-1, -3, -1), 1e-9));
    CHECK(close(b1->transform_in(Vz), V3(-2, -2, -1), 1e-9));
    CHECK(close(b1->transform_out(Vx), V3(1, 3, 3), 1e-9));
    CHECK(close(b1->transform_out(Vy), V3(1, 2, 4), 1e-9));
    CHECK(close(b1->transform_out(Vz), V3(2, 2, 3), 1e-9));
}

TEST_CASE("2-body rotate diagonal")
{
    auto r = V3(1, 2, 3);
    // x -> y, y -> z, z ->x
    auto o1 = rot(M1, deg(120)*unit(V3(1, 1, 1)));
    auto b1 = std::make_shared<Body>(1.0, M1, r, V0, o1, V0);
    // b2 is (-3, -2, -1) relative to b1 after capture.
    // Absolute position will be (-2, 0, 2)
    r = r + V3(-3, -2, -1);
    // b2 is rotated about b1's z, i.e. about absolute x.
    // b2's rotation is applied first
    auto o2 = o1 * rot(M1, pi/2*Vz);
    auto b2 = std::make_shared<Body>(1.0, M1, r, V0, o2, V0);
    b1->capture(b2);

    CHECK(close(b2->rotate_in(Vx), Vz, 1e-9));
    CHECK(close(b2->rotate_in(Vy), -Vy, 1e-9));
    CHECK(close(b2->rotate_in(Vz), Vx, 1e-9));
    CHECK(close(b2->rotate_out(Vx), Vz, 1e-9));
    CHECK(close(b2->rotate_out(Vy), -Vy, 1e-9));
    CHECK(close(b2->rotate_out(Vz), Vx, 1e-9));
    // CHECK(close(b1->transform_in(Vx), V3(-2, -3, 0), 1e-9));
    // CHECK(close(b1->transform_in(Vy), V3(-1, -3, -1), 1e-9));
    // CHECK(close(b1->transform_in(Vz), V3(-2, -2, -1), 1e-9));
    // CHECK(close(b1->transform_out(Vx), V3(1, 3, 3), 1e-9));
    // CHECK(close(b1->transform_out(Vy), V3(1, 2, 4), 1e-9));
    // CHECK(close(b1->transform_out(Vz), V3(2, 2, 3), 1e-9));

    // Set b2 to be rotated pi/2 about b1's x-axis instead of b1's z-axis.
    b2->set_orientation(rot(M1, pi/2*Vx));
    CHECK(close(b2->rotate_in(Vx), Vy, 1e-9));
    CHECK(close(b2->rotate_in(Vy), Vx, 1e-9));
    CHECK(close(b2->rotate_in(Vz), -Vz, 1e-9));
    CHECK(close(b2->rotate_out(Vx), Vy, 1e-9));
    CHECK(close(b2->rotate_out(Vy), Vx, 1e-9));
    CHECK(close(b2->rotate_out(Vz), -Vz, 1e-9));
}
