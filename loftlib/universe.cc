#include "body.hh"
#include "universe.hh"

static constexpr double G = 6.67430e-11;

V3 gravity(const Body& p1, const Body& p2)
{
    auto r = p2.r_cm() - p1.r_cm();
    return unit(r)*G*p1.m()*p2.m()/dot(r, r);
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
        auto it2 = it1;
        for(++it2; it2 != m_body.end(); ++it2)
        {
            auto& p1 = **it1;
            auto& p2 = **it2;
            auto force = gravity(p1, p2);
            auto imp = force*time;
            p1.impulse(imp);
            p2.impulse(-imp);
        }
    }
    // Move the bodies.
    for(auto p : m_body)
        p->step(time);
    // Check for collisions.
    for(auto it1 = m_body.begin(); it1 != m_body.end(); ++it1)
    {
        auto it2 = it1;
        for(++it2; it2 != m_body.end(); ++it2)
        {
            // auto& p1 = **it1;
            // auto& p2 = **it2;
            // if(p1.intersects(p2))
            // {
            //     p1.add(*it2);
            //     m_body.remove(*it2);
            // }
        }
    }
    m_time += time;
}

double Universe::time() const
{
    return m_time;
}
