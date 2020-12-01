#include "body.hh"
#include "units.hh"
#include "universe.hh"

#include <cassert>

V3 gravity(const Body& p1, const Body& p2)
{
    auto r = p2.r_cm() - p1.r_cm();
    return unit(r)*consts::G*p1.m()*p2.m()/dot(r, r);
}

void Universe::add(Body_ptr bp)
{
    m_body.push_back(bp);
}

void Universe::step(double time)
{
    // Change velocities due to gravity.
    for(auto it1 = m_body.begin(); it1 != m_body.end(); ++it1)
    {
        if (!(*it1)->is_free())
            continue;
        auto it2 = it1;
        for(++it2; it2 != m_body.end(); ++it2)
        {
            assert(it1 != it2);
            if (!(*it2)->is_free())
                continue;
            auto& p1 = **it1;
            auto& p2 = **it2;
            auto force = gravity(p1, p2);
            auto imp = force*time;
            p1.impulse(imp);
            p2.impulse(-imp);
        }
    }

    for (auto& b : m_body)
        b->step(time);

    // Check for collisions.
    for(auto it1 = m_body.begin(); it1 != m_body.end(); ++it1)
    {
        if (!(*it1)->is_free())
            continue;
        auto it2 = it1;
        for(++it2; it2 != m_body.end(); ++it2)
        {
            if (!(*it2)->is_free())
                continue;
            auto& p1 = **it1;
            const auto& p2 = **it2;
            if (p1.intersects(p2) && dot(p1.v_cm(), p2.v_cm()) < 0.0)
                p1.capture(*it2);
        }
    }
    m_time += time;
}

double Universe::time() const
{
    return m_time;
}
;
