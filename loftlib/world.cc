#include "units.hh"
#include "world.hh"

World::World(double mass, double radius, const V3& r, const V3& v,
             const M3& orientation, double period)
    // The orientation matrix aligns the z-axis with omega.
    : Body(mass, M0, r, v, orientation, orientation*(2*pi/period)*Vz),
      m_radius(radius)
{
}

double World::radius() const
{
    return m_radius;
}

bool World::intersects(const Body& b) const
{
    return mag(b.r() - r()) < m_radius;
}

std::tuple<V3, M3> World::locate(double lat, double lon, double alt) const
{
    // Zero longitude is in the y-direction to match the gluSphere texture origin.
    auto r = transform_out(rot(rot((m_radius + alt)*Vy, lat*Vx), lon*Vz));
    // Construct the matrix for z up and x east.
    auto m = rot(rot(rot(M1, lon*Vz), (lat - pi/2)*Vx), pi*Vz);
    return std::make_tuple(r, orientation()*m);
}

std::tuple<double, double, double> World::location(const V3& r)
{
    auto r_in = rotate_in(r);
    auto r_xy = mag(V3(r_in.x, r_in.y, 0));
    return std::make_tuple(std::atan2(r_in.z, r_xy),
                           std::atan2(-r_in.x, r_in.y),
                           mag(r_in) - m_radius);
}
