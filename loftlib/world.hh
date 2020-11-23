#ifndef LOFT_LOFTLIB_WORLD_HH_INCLUDED
#define LOFT_LOFTLIB_WORLD_HH_INCLUDED

#include "body.hh"
#include "units.hh"

class World : public Body
{
public:
    World(double mass, double radius, const V3& r, const V3& v,
          const M3& orientation, double period);
    virtual ~World() = default;

    double radius() const;
    virtual bool intersects(const Body& b) const override;
    /// @return Absolute coordinates for the given latitude, longitude, and altitude.
    /// @param lat Latitude in degrees.
    /// @param lat Longitude in degrees.
    V3 locate(double lat, double lon, double alt) const;

private:
    double m_radius;
};

#endif // LOFT_LOFTLIB_WORLD_HH_INCLUDED
