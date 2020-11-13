#include "body.hh"
#include "universe.hh"
#include "doctest.h"

#include <iostream>
#include <numbers>

template<typename T> bool close(const T& actual, const T& expected, double tol)
{
    if (close(actual.x, expected.x, tol)
        && close(actual.y, expected.y, tol)
        && close(actual.z, expected.z, tol))
        return true;
    std::cout << actual << " != " << expected << " ±" << tol << std::endl;
    return false;
}
template<> bool close(const double& actual, const double& expected, double tol)
{
    if (std::abs(expected - actual) < tol)
        return true;
    std::cout << actual << " != " << expected << " ±" << tol << std::endl;
    return false;
}

const V3 V111 = V3(1, 1, 1);

using namespace std::numbers;
auto Mz = rot(M1, pi/2*Vz);
auto My = rot(M1, pi/2*Vy);
auto Myz = rot(My, pi/2*Vz); // == My*Mz (passive rotations)

TEST_CASE("single rest")
{
    auto b = std::make_shared<Body>(2.0, M1, Vx, V0, M1, V0);
    CHECK(b->m() == 2.0);
    CHECK(b->I() == M1);
    CHECK(b->r_cm() == Vx);
    CHECK(b->v_cm() == V0);
    CHECK(b->r() == Vx);
    CHECK(b->orientation() == M1);
    CHECK(b->omega() == V0);
    b->step(100.0);
    CHECK(b->m() == 2.0);
    CHECK(b->I() == M1);
    CHECK(b->r_cm() == Vx);
    CHECK(b->v_cm() == V0);
    CHECK(b->r() == Vx);
    CHECK(b->orientation() == M1);
    CHECK(b->omega() == V0);
}

TEST_CASE("single move")
{
    auto b = std::make_shared<Body>(2.0, M1, Vx, V111, M1, Vz);
    CHECK(b->m() == 2.0);
    CHECK(b->I() == M1);
    CHECK(b->r_cm() == Vx);
    CHECK(b->v_cm() == V111);
    CHECK(b->r() == Vx);
    CHECK(b->orientation() == M1);
    CHECK(b->omega() == Vz);
    b->step(1.0);
    CHECK(b->m() == 2.0);
    CHECK(b->I() == M1);
    CHECK(b->r_cm() == V3(2, 1, 1));
    CHECK(b->v_cm() == V111);
    CHECK(b->r() == V3(2, 1, 1));
    CHECK(b->orientation()*Vx == V3(cos(1), sin(1), 0));
    CHECK(b->omega() == Vz);
}

TEST_CASE("2-point static")
{
    auto b1 = std::make_shared<Body>(2.0, M1, 6*Vz, V0, My, V0);
    auto b2 = std::make_shared<Body>(6.0, M1, 2*Vz, V0, Myz, V0);
    CHECK(close(b1->orientation()*Vx, -Vz, 1e-9));
    CHECK(close(b1->orientation()*Vy, Vy, 1e-9));
    CHECK(close(b1->orientation()*Vz, Vx, 1e-9));
    CHECK(close(b2->orientation()*Vx, Vy, 1e-9));
    CHECK(close(b2->orientation()*Vy, Vz, 1e-9));
    CHECK(close(b2->orientation()*Vz, Vx, 1e-9));
    auto check_init = [&](){
        CHECK(b1->m() == 8.0);
        CHECK(b2->m() == 6.0);
        // I comes from orbit (Σmr² = 2*9 + 6) + spin (ΣI = 1+1)
        CHECK(b1->I() == M3{26*Vx, 26*Vy, 2*Vz});
        CHECK(b2->I() == M1);
        CHECK(b1->r_cm() == 3*Vz);
        CHECK(close(b2->r_cm(), 4*Vx, 1e-9)); // b2 is in b1's x-direction
        CHECK(b1->v_cm() == V0);
        CHECK(b2->v_cm() == V0);
        CHECK(b1->r() == 6*Vz);
        CHECK(close(b2->r(), 4*Vx, 1e-9));
        CHECK(b1->orientation() == My);
        CHECK(b2->orientation() == Mz);
        CHECK(b1->omega() == V0);
        CHECK(b2->omega() == V0);
    };
    b1->capture(b2);
    check_init();
    b1->step(1.0);
    check_init();

    b1->release(b2);
    CHECK(b1->m() == 2.0);
    CHECK(b2->m() == 6.0);
    CHECK(b1->I() == M1);
    CHECK(b2->I() == M1);
    CHECK(b1->r_cm() == 6*Vz);
    CHECK(b2->r_cm() == 2*Vz);
    CHECK(b1->v_cm() == V0);
    CHECK(b2->v_cm() == V0);
    CHECK(b1->r() == 6*Vz);
    CHECK(b2->r() == 2*Vz);
    CHECK(b1->orientation() == My);
    CHECK(b2->orientation() == Myz);
    CHECK(b1->omega() == V0);
    CHECK(b2->omega() == V0);
}

TEST_CASE("2-point translate")
{
    auto b1 = std::make_shared<Body>(2.0, M1, 6*Vz, -4*Vz, My, V0);
    auto b2 = std::make_shared<Body>(6.0, M1, 2*Vz, V0, Myz, V0);
    b1->capture(b2);
    CHECK(b1->m() == 8.0);
    CHECK(b2->m() == 6.0);
    CHECK(b1->I() == M3{26*Vx, 26*Vy, 2*Vz});
    CHECK(b2->I() == M1);
    CHECK(b1->r_cm() == 3*Vz);
    CHECK(close(b2->r_cm(), 4*Vx, 1e-9));
    CHECK(b1->v_cm() == -Vz);
    CHECK(b2->v_cm() == V0);
    CHECK(b1->r() == 6*Vz);
    CHECK(close(b2->r(), 4*Vx, 1e-9));
    CHECK(b1->orientation() == My);
    CHECK(b2->orientation() == Mz);
    CHECK(b1->omega() == V0);
    CHECK(b2->omega() == V0);

    b1->step(1.0);
    CHECK(b1->r() == 5*Vz);
    CHECK(b1->r_cm() == 2*Vz);
    CHECK(b1->v_cm() == -Vz);
    CHECK(b1->orientation() == My);
    CHECK(b1->omega() == V0);

    b1->release(b2);
    CHECK(b1->r() == 5*Vz);
    CHECK(b2->r() == Vz);
    CHECK(b1->r_cm() == 5*Vz);
    CHECK(b2->r_cm() == Vz);
    CHECK(b1->v_cm() == -Vz);
    CHECK(b2->v_cm() == -Vz);
    CHECK(b1->orientation() == My);
    CHECK(b2->orientation() == Myz);
    CHECK(b1->omega() == V0);
    CHECK(b2->omega() == V0);
}

TEST_CASE("2-point rotate")
{
    auto b1 = std::make_shared<Body>(2.0, M1, 6*Vz, 3*Vx, My, V0);
    auto b2 = std::make_shared<Body>(6.0, M1, 2*Vz, -Vx, Myz, V0);
    double w = 12.0/13.0;
    auto r1_hat = V3(sin(w), 0, cos(w));

    b1->capture(b2);
    CHECK(b1->r_cm() == 3*Vz);
    CHECK(b1->v_cm() == V0); // total linear momentum is zero.
    CHECK(b1->r() == 6*Vz);
    CHECK(close(b2->r_cm(), 4*Vx, 1e-9));
    CHECK(b1->orientation() == My);
    CHECK(close(b1->orientation()*Vz, Vx, 1e-9));
    CHECK(close(b2->orientation()*Vx, Vy, 1e-9));
    CHECK(close(b2->orientation()*Vz, Vz, 1e-9));
    CHECK(b1->omega() == w*Vy);

    b1->step(1.0);
    CHECK(b1->r() == 3*Vz + 3*r1_hat);
    CHECK(close(b2->r_cm(), 4*Vx, 1e-9));
    CHECK(close(b2->r(), 4*Vx, 1e-9));
    CHECK(b1->r_cm() == 3*Vz);
    CHECK(b1->v_cm() == V0);
    CHECK(close(b1->orientation()*Vz, V3(cos(w), 0, -sin(w)), 1e-9));
    CHECK(close(b2->orientation()*Vx, Vy, 1e-9));
    CHECK(b2->orientation() == Mz);
    CHECK(b1->omega() == w*Vy);
    CHECK(b2->omega() == V0);

    b1->release(b2);
    CHECK(b1->r() == 3*Vz + 3*r1_hat);
    CHECK(b2->r() == 3*Vz - r1_hat);
    CHECK(b1->r_cm() == b1->r());
    CHECK(b2->r_cm() == b2->r());
    CHECK(close(b1->v_cm(), cross(w*Vy, 3*r1_hat), 1e-9));
    CHECK(close(b2->v_cm(), cross(w*Vy, -r1_hat), 1e-9));
    CHECK(close(b1->orientation()*Vz, V3(cos(w), 0, -sin(w)), 1e-9));
    CHECK(close(b2->orientation()*Vx, Vy, 1e-9));
    CHECK(close(b2->orientation()*Vy, V3(sin(w), 0, cos(w)), 1e-9));
    CHECK(close(b2->orientation()*Vz, V3(cos(w), 0, -sin(w)), 1e-9));
    CHECK(b1->omega() == w*Vy);
    CHECK(b2->omega() == w*Vy);
}

TEST_CASE("2-point rotate and translate")
{
    auto b1 = std::make_shared<Body>(2.0, M1, 6*Vz, 6*Vx, My, V0);
    auto b2 = std::make_shared<Body>(6.0, M1, 2*Vz, 2*Vx, Myz, V0);
    double w = 12.0/13.0;
    auto r1_hat = V3(sin(w), 0, cos(w));

    b1->capture(b2);
    CHECK(b1->r_cm() == 3*Vz);
    CHECK(b1->v_cm() == 3*Vx);
    CHECK(b1->r() == 6*Vz);
    CHECK(close(b2->r(), 4*Vx, 1e-9));
    CHECK(b1->orientation() == My);
    CHECK(b1->omega() == w*Vy);

    b1->step(1.0);
    CHECK(b1->r() == V3(3, 0, 3) + 3*r1_hat);
    CHECK(close(b2->r(), 4*Vx, 1e-9));
    CHECK(close(b1->r_cm(), V3(3, 0, 3), 1e-9));
    CHECK(b1->v_cm() == 3*Vx);
    CHECK(close(b1->orientation()*Vz, V3(cos(w), 0, -sin(w)), 1e-9));
    CHECK(b2->orientation() == Mz);
    CHECK(b1->omega() == (12.0/13.0)*Vy);
    CHECK(b2->omega() == V0);

    b1->release(b2);
    CHECK(close(b1->r(), V3(3, 0, 3) + 3*r1_hat, 1e-9));
    CHECK(close(b2->r(), V3(3, 0, 3) - r1_hat, 1e-9));
    CHECK(b1->r_cm() == b1->r());
    CHECK(b2->r_cm() == b2->r());
    CHECK(close(b1->v_cm(), 3*Vx + cross(w*Vy, 3*r1_hat), 1e-9));
    CHECK(close(b2->v_cm(), 3*Vx + cross(w*Vy, -r1_hat), 1e-9));
    CHECK(close(b1->orientation()*Vz, V3(cos(w), 0, -sin(w)), 1e-9));
    CHECK(close(b2->orientation()*Vx, Vy, 1e-9));
    CHECK(close(b2->orientation()*Vy, V3(sin(w), 0, cos(w)), 1e-9));
    CHECK(close(b2->orientation()*Vz, V3(cos(w), 0, -sin(w)), 1e-9));
    CHECK(b1->omega() == w*Vy);
    CHECK(b2->omega() == w*Vy);
}

// TEST_CASE("3-point")
// {
//     auto b1 = std::make_shared<Body>(1.0, 1.0, M1);
//     auto b2 = std::make_shared<Body>(1.0, 1.0, M1);
//     auto b3 = std::make_shared<Body>(1.0, 1.0, M1);
//     b1->set_r(1*Vx);
//     b2->set_r(2*Vx);
//     b3->set_r(3*Vx);
//     b1->set_v_cm(Vy);

//     auto check = [b1](){
//         CHECK(b1->r_cm() == 2*Vx);
//         CHECK(b1->v_cm() == Vy/3);
//         CHECK(b1->I() == M3(3*Vx, 5*Vy, 5*Vz));
//         CHECK(b1->omega() == -Vz/5);
//     };

//     SUBCASE("top-down")
//     {
//         b1->capture(b2);
//         CHECK(b1->r_cm() == 1.5*Vx);
//         CHECK(b1->v_cm() == Vy/2);
//         CHECK(b1->I() == M3(2*Vx, 2.5*Vy, 2.5*Vz));
//         CHECK(b1->omega() == -Vz/5);
//         b2->capture(b3);
//         CHECK(b1->r_cm() == 2*Vx);
//         CHECK(b1->v_cm() == Vy/3);
//         CHECK(b1->I() == M3(3*Vx, 5*Vy, 5*Vz));
//         CHECK(b1->omega() == -Vz/5);
//         check();
//     }
//     SUBCASE("bottom-up")
//     {
//         b2->capture(b3);
//         b1->capture(b2);
//         check();
//     }
//     SUBCASE("2-level")
//     {
//         b1->capture(b2);
//         b1->capture(b3);
//         check();
//     }
// }

// TEST_CASE("cube")
// {
//     auto b000 = std::make_shared<Body>(1.0, 1.0, M1);
//     auto b100 = std::make_shared<Body>(1.0, 1.0, M1);
//     auto b010 = std::make_shared<Body>(1.0, 1.0, M1);
//     auto b001 = std::make_shared<Body>(1.0, 1.0, M1);
//     auto b110 = std::make_shared<Body>(1.0, 1.0, M1);
//     auto b011 = std::make_shared<Body>(1.0, 1.0, M1);
//     auto b101 = std::make_shared<Body>(1.0, 1.0, M1);
//     auto b111 = std::make_shared<Body>(1.0, 1.0, M1);
//     b000->set_r(V0);
//     b100->set_r(Vx);
//     b010->set_r(Vy);
//     b001->set_r(Vz);
//     b110->set_r(V3(1,1,0));
//     b011->set_r(V3(0,1,1));
//     b101->set_r(V3(1,0,1));
//     b111->set_r(V3(1,1,1));

//     auto check = [b000]() {
//         V3 V111 = V3(1, 1, 1);
//         b000->set_v_cm(0.25*V111);
//         CHECK(b000->m() == 8.0);
//         CHECK(b000->r() == V0);
//         CHECK(b000->r_cm() == 0.5*V111);
//         CHECK(b000->v_cm() == 0.25*V111);
//         b000->step(2.0);
//         CHECK(b000->r_cm() == V111);
//         CHECK(b000->v_cm() == 0.25*V111);
//     };

//     SUBCASE("bottom-up")
//     {
//         // Build up aggregates from bodies, then add aggregates to the head.
//         b110->capture(b111);
//         b100->capture(b110);
//         b010->capture(b011);
//         b001->capture(b101);
//         b000->capture(b100);
//         b000->capture(b010);
//         b000->capture(b001);
//         check();
//     }
//     SUBCASE("top-down")
//     {
//         // Add to sub-bodies after adding them to the head.
//         b000->capture(b100);
//         b000->capture(b010);
//         b000->capture(b001);
//         b100->capture(b110);
//         b010->capture(b011);
//         b001->capture(b101);
//         b110->capture(b111);
//         check();
//     }
// }

// TEST_CASE("spinning cube")
// {
//     auto b000 = std::make_shared<Body>(1.0, 1.0, M1);
//     auto b100 = std::make_shared<Body>(1.0, 1.0, M1);
//     auto b010 = std::make_shared<Body>(1.0, 1.0, M1);
//     auto b001 = std::make_shared<Body>(1.0, 1.0, M1);
//     auto b110 = std::make_shared<Body>(1.0, 1.0, M1);
//     auto b011 = std::make_shared<Body>(1.0, 1.0, M1);
//     auto b101 = std::make_shared<Body>(1.0, 1.0, M1);
//     auto b111 = std::make_shared<Body>(1.0, 1.0, M1);
//     b000->set_r(V0);
//     b100->set_r(Vx);
//     b010->set_r(Vy);
//     b001->set_r(Vz);
//     b110->set_r(V3(1,1,0));
//     b011->set_r(V3(0,1,1));
//     b101->set_r(V3(1,0,1));
//     b111->set_r(V3(1,1,1));
//     b000->set_v_cm(2*Vx);

//     auto check = [b000]() {
//         CHECK(b000->v_cm() == 0.25*Vx);
//         CHECK(close(b000->omega(), (1.0/12.0)*V3(0, -1, 1), 1e-9));
//     };

//     SUBCASE("flat")
//     {
//         // Attach all bodies to the head.
//         b000->capture(b111);
//         b000->capture(b110);
//         b000->capture(b011);
//         b000->capture(b101);
//         b000->capture(b100);
//         b000->capture(b010);
//         b000->capture(b001);
//         check();
//     }
//     SUBCASE("bottom-up")
//     {
//         // Build up aggregates from bodies, then add aggregates to the head.
//         b110->capture(b111);
//         b100->capture(b110);
//         b010->capture(b011);
//         b001->capture(b101);
//         b000->capture(b100);
//         b000->capture(b010);
//         b000->capture(b001);
//         check();
//     }
//     SUBCASE("top-down")
//     {
//         // Add to sub-bodies after adding them to the head.
//         b000->capture(b100);
//         b000->capture(b010);
//         b000->capture(b001);
//         b100->capture(b110);
//         b010->capture(b011);
//         b001->capture(b101);
//         b110->capture(b111);
//         check();
//     }
// }
