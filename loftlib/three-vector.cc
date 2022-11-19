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

#include "three-vector.hh"

#include <cassert>
#include <ranges>

using std::views::iota;

double const& V3::operator[](std::size_t i) const
{
    switch(i)
    {
    case 0: return x;
    case 1: return y;
    default: return z;
    }
}
double& V3::operator[](std::size_t i)
{
    return const_cast<double&>(const_cast<V3 const*>(this)->operator[](i));
}

V3 const& M3::operator[](std::size_t i) const
{
    switch(i)
    {
    case 0: return x;
    case 1: return y;
    default: return z;
    }
}
V3& M3::operator[](std::size_t i)
{
    return const_cast<V3&>(const_cast<M3 const*>(this)->operator[](i));
}

double dot(V3 const& v1, V3 const& v2)
{
    return v1.x*v2.x + v1.y*v2.y + v1.z*v2.z;
}

double square(V3 const& v)
{
    return dot(v, v);
}

V3 cross(V3 const& v1, V3 const& v2)
{
    return V3{v1.y*v2.z - v1.z*v2.y, v1.z*v2.x - v1.x*v2.z, v1.x*v2.y - v1.y*v2.x};
}

M3 outer(V3 const& v1, V3 const& v2)
{
    M3 prod;
    for (int i : iota(0, 3))
        for (int j : iota(0, 3))
            prod[i][j] = v1[i]*v2[j];
    return prod;
}

double mag(V3 const& v)
{
    return std::sqrt(dot(v, v));
}

V3 unit(V3 const& v)
{
    auto r{mag(v)};
    return r == 0.0 ? Vz : v/mag(v);
}

V3 operator-(V3 const& v)
{
    return -1.0*v;
}

V3 operator+(V3 const& v1, V3 const& v2)
{
    return V3{v1.x+v2.x, v1.y+v2.y, v1.z+v2.z};
}

V3 operator-(V3 const& v1, V3 const& v2)
{
    return V3{v1.x-v2.x, v1.y-v2.y, v1.z-v2.z};
}

V3 operator*(double c, V3 const& v)
{
    return V3{v.x*c, v.y*c, v.z*c};
}
V3 operator*(V3 const& v, double c)
{
    return c*v;
}

V3 operator/(V3 const& v, double c)
{
    return V3{v.x/c, v.y/c, v.z/c};
}

V3& operator+=(V3& v1, V3 const& v2)
{
    v1.x += v2.x;
    v1.y += v2.y;
    v1.z += v2.z;
    return v1;
}

V3& operator-=(V3& v1, V3 const& v2)
{
    v1.x -= v2.x;
    v1.y -= v2.y;
    v1.z -= v2.z;
    return v1;
}

std::ostream& operator<<(std::ostream& os, V3 const& v)
{
    return os << '(' << v.x << " " << v.y << " " << v.z << ')';
}

std::ostream& operator<<(std::ostream& os, M3 const& m)
{
    return os << '[' << m.x << " " << m.y << " " << m.z << ']';
}

V3 rot(V3 const& v, V3 const& a)
{
    return rot(M1, a)*v;
}

M3 rot(M3 const& m, V3 const& a)
{
    if (a == V0)
        return m;
    const auto angle{0.5*mag(a)};
    const auto e{unit(a)*sin(angle)};
    const auto w{cos(angle)};

    // This tranformation matrix is derived from quaternion analysis.
    const auto wx{w*e.x};
    const auto wy{w*e.y};
    const auto wz{w*e.z};

    const auto xx{e.x*e.x};
    const auto xy{e.x*e.y};
    const auto xz{e.x*e.z};

    const auto yy{e.y*e.y};
    const auto yz{e.y*e.z};

    const auto zz{e.z*e.z};

    M3 R{{1.0 - 2.0*(yy + zz), 2.0*(xy - wz), 2.0*(xz + wy)},
         {2.0*(xy + wz), 1.0 - 2.0*(xx + zz), 2.0*(yz - wx)},
         {2.0*(xz - wy), 2.0*(yz + wx), 1.0 - 2.0*(xx + yy)}};
    return m * R;
}

double det(M3 const& m)
{
    return m.x.x*(m.y.y*m.z.z - m.y.z*m.z.y)
        + m.x.y*(m.y.z*m.z.x - m.y.x*m.z.z)
        + m.x.z*(m.y.x*m.z.y - m.y.y*m.z.x);
}

M3 inv(M3 const& m)
{
    auto d{det(m)};
    if (d == 0.0)
        return M0;

    return M3(V3(m.y.y*m.z.z - m.y.z*m.z.y, m.z.y*m.x.z - m.z.z*m.x.y, m.x.y*m.y.z - m.x.z*m.y.y),
              V3(m.y.z*m.z.x - m.y.x*m.z.z, m.z.z*m.x.x - m.z.x*m.x.z, m.x.z*m.y.x - m.x.x*m.y.z),
              V3(m.y.x*m.z.y - m.y.y*m.z.x, m.z.x*m.x.y - m.z.y*m.x.x, m.x.x*m.y.y - m.x.y*m.y.x))
        /d;
}

M3 tr(M3 const& m)
{
    return M3(V3(m.x.x, m.y.x, m.z.x),
              V3(m.x.y, m.y.y, m.z.y),
              V3(m.x.z, m.y.z, m.z.z));
}

M3 operator+(M3 const& m1, M3 const& m2)
{
    M3 sum;
    for (int i : iota(0, 3))
        for (int j : iota(0, 3))
            sum[i][j] = m1[i][j] + m2[i][j];
    return sum;
}

M3 operator-(M3 const& m1, M3 const& m2)
{
    M3 diff;
    for (int i : iota(0, 3))
        for (int j : iota(0, 3))
            diff[i][j] = m1[i][j] - m2[i][j];
    return diff;
}

M3 operator*(M3 const& m, double c)
{
    M3 prod;
    for (int i : iota(0, 3))
        for (int j : iota(0, 3))
            prod[i][j] = m[i][j]*c;
    return prod;
}

M3 operator*(double c, M3 const& m)
{
    return m*c;
}

V3 operator*(M3 const& m, V3 const& v)
{
    auto prod{V0};
    for (int i : iota(0, 3))
        for (int j : iota(0, 3))
            prod[i] += m[i][j]*v[j];
    return prod;
}

V3 operator*(V3 const& v, M3 const& m)
{
    auto prod{V0};
    for (int i : iota(0, 3))
        for (int j : iota(0, 3))
            prod[i] += v[i]*m[i][j];
    return prod;
}

M3 operator*(M3 const& m1, M3 const& m2)
{
    auto prod{M0};
    for (int i : iota(0, 3))
        for (int j : iota(0, 3))
            for (int k : iota(0, 3))
                prod[i][j] += m1[i][k]*m2[k][j];
    return prod;
}

M3 operator/(M3 const& m, double c)
{
    auto M{m};
    for (int i : iota(0, 3))
        for (int j : iota(0, 3))
            M[i][j] /= c;
    return M;
}

M3& operator+=(M3& m1, M3 const& m2)
{
    return m1 = m1 + m2;
}

std::tuple<V3, double> axis_angle(M3 const& m)
{
    // To convert the rotation matrix representation of the body's orientation to an
    // axis-angle orientation, we transform first to a quaternion representation.  The
    // matrix-to-quaternion and quaternion-to-axis-angle transformations are described in
    // the Matrix and Quaternion FAQ (matrixfaq.htm) in the doc directory.

    // Convert from matrix to quaternion
    auto trace{m.x.x +  m.y.y +  m.z.z + 1.0};
    double s, w, x, y, z;
    s = w = x = y = z = 0.0;
    if (trace > 0.0)
    {
        s = 0.5/sqrt(trace);
        w = 0.25/s;
        x = (m.z.y -  m.y.z)*s;
        y = (m.x.z -  m.z.x)*s;
        z = (m.y.x -  m.x.y)*s;
    }
    else
    {
        // Find the largest diagonal element and do the appropriate transformation.
        auto largest{m.x.x};
        auto index{0};
        if (m.y.y > largest)
        {
            largest = m.y.y;
            index = 1;
        }

        if (m.z.z > largest)
        {
            largest =  m.z.z;
            s = sqrt(1.0 -  m.x.x -  m.y.y +  m.z.z) * 2.0;
            w = (m.x.y +  m.y.x)/s;
            x = (m.x.z +  m.z.x)/s;
            y = (m.y.z +  m.z.y)/s;
            z = 0.5/s;
        }
        else if (index == 0)
        {
            s = sqrt(1.0 +  m.x.x -  m.y.y -  m.z.z) * 2.0;
            w = (m.y.z +  m.z.y)/s;
            x = 0.5/s;
            y = (m.x.y +  m.y.x)/s;
            z = (m.x.z +  m.z.x)/s;
        }
        else
        {
            assert(index == 1);
            s = sqrt(1.0 -  m.x.x +  m.y.y -  m.z.z) * 2.0;
            w = (m.x.z +  m.z.x)/s;
            x = (m.x.y +  m.y.x)/s;
            y = 0.5/s;
            z = (m.y.z +  m.z.y)/s;
        }
    }
    return {V3(x, y, z), acos(w) * 2.0};
}
