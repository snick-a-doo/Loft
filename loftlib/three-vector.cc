#include "three-vector.hh"

#include <ranges>

using std::views::iota;

const double& V3::operator[](std::size_t i) const
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
    return const_cast<double&>(const_cast<const V3*>(this)->operator[](i));
}

const V3& M3::operator[](std::size_t i) const
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
    return const_cast<V3&>(const_cast<const M3*>(this)->operator[](i));
}

double dot(const V3& v1, const V3& v2)
{
    return v1.x*v2.x + v1.y*v2.y + v1.z*v2.z;
}

double square(const V3& v)
{
    return dot(v, v);
}

V3 cross(const V3& v1, const V3& v2)
{
    return V3{v1.y*v2.z - v1.z*v2.y, v1.z*v2.x - v1.x*v2.z, v1.x*v2.y - v1.y*v2.x};
}

M3 outer(const V3& v1, const V3& v2)
{
    M3 prod;
    for (int i : iota(0, 3))
        for (int j : iota(0, 3))
            prod[i][j] = v1[i]*v2[j];
    return prod;
}

double mag(const V3& v)
{
    return std::sqrt(dot(v, v));
}

V3 unit(const V3& v)
{
    return v/mag(v);
}

V3 operator-(const V3& v)
{
    return -1.0*v;
}

V3 operator+(const V3& v1, const V3& v2)
{
    return V3{v1.x+v2.x, v1.y+v2.y, v1.z+v2.z};
}

V3 operator-(const V3& v1, const V3& v2)
{
    return V3{v1.x-v2.x, v1.y-v2.y, v1.z-v2.z};
}

V3 operator*(double c, const V3& v)
{
    return V3{v.x*c, v.y*c, v.z*c};
}
V3 operator*(const V3& v, double c)
{
    return c*v;
}

V3 operator/(const V3& v, double c)
{
    return V3{v.x/c, v.y/c, v.z/c};
}

V3& operator+=(V3& v1, const V3& v2)
{
    v1.x += v2.x;
    v1.y += v2.y;
    v1.z += v2.z;
    return v1;
}

V3& operator-=(V3& v1, const V3& v2)
{
    v1.x -= v2.x;
    v1.y -= v2.y;
    v1.z -= v2.z;
    return v1;
}

namespace std
{
    ostream& operator<<(ostream& os, const V3& v)
    {
        return os << '(' << v.x << " " << v.y << " " << v.z << ')';
    }
    ostream& operator<<(ostream& os, const M3& m)
    {
        return os << '[' << m.x << " " << m.y << " " << m.z << ']';
    }
}

M3 rot(const M3& m, const V3& a)
{
    if (a == V0)
        return m;
    const double angle = 0.5*mag(a);
    const auto e = unit(a)*sin(angle);
    const double w = cos(angle);

    // This tranformation matrix is derived from quaternion analysis.
    const double wx = w*e.x;
    const double wy = w*e.y;
    const double wz = w*e.z;

    const double xx = e.x*e.x;
    const double xy = e.x*e.y;
    const double xz = e.x*e.z;

    const double yy = e.y*e.y;
    const double yz = e.y*e.z;

    const double zz = e.z*e.z;

    M3 R{{1.0 - 2.0*(yy + zz), 2.0*(xy - wz), 2.0*(xz + wy)},
         {2.0*(xy + wz), 1.0 - 2.0*(xx + zz), 2.0*(yz - wx)},
         {2.0*(xz - wy), 2.0*(yz + wx), 1.0 - 2.0*(xx + yy)}};
    return m * R;
}

double det(const M3& m)
{
    return m.x.x*(m.y.y*m.z.z - m.y.z*m.z.y)
        + m.x.y*(m.y.z*m.z.x - m.y.x*m.z.z)
        + m.x.z*(m.y.x*m.z.y - m.y.y*m.z.x);
}

M3 inv(const M3& m)
{
    double d = det(m);
    if (d == 0.0)
        return M0;

    return M3(V3(m.y.y*m.z.z - m.y.z*m.z.y, m.z.y*m.x.z - m.z.z*m.x.y, m.x.y*m.y.z - m.x.z*m.y.y),
              V3(m.y.z*m.z.x - m.y.x*m.z.z, m.z.z*m.x.x - m.z.x*m.x.z, m.x.z*m.y.x - m.x.x*m.y.z),
              V3(m.y.x*m.z.y - m.y.y*m.z.x, m.z.x*m.x.y - m.z.y*m.x.x, m.x.x*m.y.y - m.x.y*m.y.x))
        /d;
}

M3 tr(const M3& m)
{
    return M3(V3(m.x.x, m.y.x, m.z.x),
              V3(m.x.y, m.y.y, m.z.y),
              V3(m.x.z, m.y.z, m.z.z));
}

M3 operator+(const M3& m1, const M3& m2)
{
    M3 sum;
    for (int i : iota(0, 3))
        for (int j : iota(0, 3))
            sum[i][j] = m1[i][j] + m2[i][j];
    return sum;
}

M3 operator-(const M3& m1, const M3& m2)
{
    M3 diff;
    for (int i : iota(0, 3))
        for (int j : iota(0, 3))
            diff[i][j] = m1[i][j] - m2[i][j];
    return diff;
}

M3 operator*(const M3& m, double c)
{
    M3 prod;
    for (int i : iota(0, 3))
        for (int j : iota(0, 3))
            prod[i][j] = m[i][j]*c;
    return prod;
}

M3 operator*(double c, const M3& m)
{
    return m*c;
}

V3 operator*(const M3& m, const V3& v)
{
    V3 prod = V0;
    for (int i : iota(0, 3))
        for (int j : iota(0, 3))
            prod[i] += m[i][j]*v[j];
    return prod;
}

V3 operator*(const V3& v, const M3& m)
{
    V3 prod = V0;
    for (int i : iota(0, 3))
        for (int j : iota(0, 3))
            prod[i] += v[i]*m[i][j];
    return prod;
}

M3 operator*(const M3& m1, const M3& m2)
{
    M3 prod = M0;
    for (int i : iota(0, 3))
        for (int j : iota(0, 3))
            for (int k : iota(0, 3))
                prod[i][j] += m1[i][k]*m2[k][j];
    return prod;
}

M3 operator/(const M3& m, double c)
{
    M3 M = m;
    for (int i : iota(0, 3))
        for (int j : iota(0, 3))
            M[i][j] /= c;
    return M;
}

M3& operator+=(M3& m1, const M3& m2)
{
    m1 = m1 + m2;
    return m1;
}
