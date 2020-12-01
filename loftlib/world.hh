#ifndef LOFT_LOFTLIB_WORLD_HH_INCLUDED
#define LOFT_LOFTLIB_WORLD_HH_INCLUDED

#include "body.hh"

#include <tuple>

/// A large spherical body, such as a planet or moon.
class World : public Body
{
public:
    /// @param period The period of rotation in seconds. units::day() can be used to
    /// convert from days.
    World(double mass, double radius, const V3& r, const V3& v,
          const M3& orientation, double period);
    virtual ~World() = default;

    /// @return The body's radius.
    double radius() const;
    virtual bool intersects(const Body& b) const override;
    /// @return Absolute coordinates and orientation for the given latitude, longitude,
    /// and altitude.  The orientation has z normal to the surface and y north.
    std::tuple<V3, M3> locate(double lat, double lon, double alt) const;
    /// @return Latitude, longitude, and altitude for a given position.
    std::tuple<double, double, double> location(const V3& r);

private:
    double m_radius;
};

#endif // LOFT_LOFTLIB_WORLD_HH_INCLUDED
