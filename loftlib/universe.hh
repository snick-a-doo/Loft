#ifndef LOFT_LOFTLIB_UNIVERSE_HH_INCLUDED
#define LOFT_LOFTLIB_UNIVERSE_HH_INCLUDED

#include <memory>
#include <list>

class Body;

using Body_ptr = std::shared_ptr<Body>;

class Universe
{
public:
    void add(Body_ptr bp);
    void step(double time);

    double time() const;

private:
    double m_time = 0.0;
    std::list<Body_ptr> m_body;
};

#endif // LOFT_LOFTLIB_UNIVERSE_HH_INCLUDED
