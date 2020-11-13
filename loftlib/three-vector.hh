#ifndef LOFT_LOFTLIB_THREE_VECTOR_HH_INCLUDED
#define LOFT_LOFTLIB_THREE_VECTOR_HH_INCLUDED

#include <cmath>
#include <iostream>

struct V3
{
    double x, y, z;
    const double& operator[](std::size_t i) const;
    double& operator[](std::size_t i);
    friend bool operator==(const V3& v1, const V3& v2) = default;
};

struct M3
{
    V3 x, y, z; // rows
    const V3& operator[](std::size_t i) const;
    V3& operator[](std::size_t i);
    friend bool operator==(const M3& m1, const M3& m2) = default;
};

double dot(const V3& v1, const V3& v2);
double square(const V3& v1);
V3 cross(const V3& v1, const V3& v2);
M3 outer(const V3& v1, const V3& v2);
double mag(const V3& v);
V3 unit(const V3& v);
M3 rot(const M3& m, const V3& a);
double det(const M3& m);
M3 inv(const M3& m);
M3 tr(const M3& m);

V3 operator-(const V3& v);

V3 operator+(const V3& v1, const V3& v2);
V3 operator-(const V3& v1, const V3& v2);
V3 operator*(double c, const V3& v);
V3 operator*(const V3& v, double c);
V3 operator/(const V3& v, double c);

V3& operator+=(V3& v1, const V3& v2);
V3& operator-=(V3& v1, const V3& v2);

M3 operator+(const M3& m1, const M3& m2);
M3 operator-(const M3& m1, const M3& m2);
M3 operator*(const M3& m, double c);
M3 operator*(double c, const M3& m);
V3 operator*(const M3& m, const V3& v);
V3 operator*(const V3& v, const M3& m);
M3 operator*(const M3& m1, const M3& m2);
M3 operator/(const M3& v, double c);

M3& operator+=(M3& m1, const M3& m2);

namespace std
{
    ostream& operator<<(ostream& os, const V3& v);
    ostream& operator<<(ostream& os, const M3& m);
}

static constexpr V3 V0(0.0, 0.0, 0.0);
static constexpr V3 Vx(1.0, 0.0, 0.0);
static constexpr V3 Vy(0.0, 1.0, 0.0);
static constexpr V3 Vz(0.0, 0.0, 1.0);

static constexpr M3 M0(V0, V0, V0);
static constexpr M3 M1(Vx, Vy, Vz);

#endif // LOFT_LOFTLIB_THREE_VECTOR_HH_INCLUDED
