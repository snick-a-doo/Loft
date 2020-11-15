#include "world.hh"

World::World(double mass, double radius, const V3& r, const V3& v,
             const M3& orientation, double period)
    // The orientation matrix aligns the z-axis with omega.
    : Body(mass, M0, r, v, orientation, (2*pi/period)*tr(orientation)*Vz)
{
}
