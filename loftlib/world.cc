#include "world.hh"

World::World(double mass, double radius, const V3& r, const V3& v,
             const M3& orientation, double period)
    // The orientation matrix aligns the z-axis with omega.
    : Body(mass, M0, r, v, orientation, (2*pi/period)*Vz),
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

V3 World::locate(double lat, double lon, double alt) const
{
    // Zero longitude is in the y-direction to match the gluSphere texture origin.
    return transform_out(rot(rot((m_radius + alt)*Vy, units::deg(lat)*Vx),
                             units::deg(lon)*Vz));
}
