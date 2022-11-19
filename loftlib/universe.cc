//  Copyright (C) 2022 Sam Varner
//
//  This file is part of Laft.
//
//  Loft is free software: you can redistribute it and/or modify it under the terms of
//  the GNU General Public License as published by the Free Software Foundation, either
//  version 3 of the License, or (at your option) any later version.
//
//  Vamos is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
//  without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
//  PURPOSE.  See the GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License along with Vamos.
//  If not, see <http://www.gnu.org/licenses/>.

#include "body.hh"
#include "units.hh"
#include "universe.hh"

#include <cassert>

V3 gravity(const Body& p1, const Body& p2)
{
    auto r{p2.r_cm() - p1.r_cm()};
    auto r2{dot(r, r)};
    return r2 < 1e-3 ? V0 : unit(r)*consts::G*p1.m()*p2.m()/dot(r, r);
}

Universe::Universe(bool handle_collision)
    : m_handle_collision{handle_collision}
{
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
        auto it2{it1};
        for(auto it2{std::next(it1)}; it2 != m_body.end(); ++it2)
        {
            assert(it1 != it2);
            if (!(*it2)->is_free())
                continue;
            auto& p1{**it1};
            auto& p2{**it2};
            if (p1.m() == p2.m())
                continue;
            auto force{gravity(p1, p2)};
            auto imp{force*time};
            p1.impulse(imp);
            p2.impulse(-imp);
        }
    }

    for (auto& b : m_body)
        b->step(time);
    m_time += time;

    if (!m_handle_collision)
        return;

    // Check for collisions.
    for(auto it1 = m_body.begin(); it1 != m_body.end(); ++it1)
    {
        if (!(*it1)->is_free())
            continue;
        for(auto it2{std::next(it1)}; it2 != m_body.end(); ++it2)
        {
            if (!(*it2)->is_free())
                continue;
            auto& p1{**it1};
            auto const& p2{**it2};
            if (p1.intersects(p2) && dot(p1.v_cm(), p2.v_cm()) < 0.0)
                p1.capture(*it2);
        }
    }
}

double Universe::time() const
{
    return m_time;
}
