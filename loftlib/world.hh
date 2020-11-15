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

private:
};

#endif // LOFT_LOFTLIB_WORLD_HH_INCLUDED
