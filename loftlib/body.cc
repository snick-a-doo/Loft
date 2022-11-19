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

#include <algorithm>
#include <cassert>
#include <numeric>

Body::Body(double mass, M3 const& inertia,
           const V3 r, const V3 v, M3 const& orientation, const V3 omega)
    : m_mass(mass),
      m_inertia(inertia),
      m_r(r),
      m_v_cm(v),
      m_orientation(orientation),
      m_omega(omega)
{
}

void Body::capture(Body_ptr part)
{
    add_momentum(part);
    m_subs.push_back(part);
    part->m_parent = this;

    // Set the body's frame relative to the parent's frame.
    part->m_r = transform_in(part->m_r);
    part->m_orientation = tr(m_orientation)*part->m_orientation;
    part->m_v_cm = V0;
    part->m_omega = V0;
}

void Body::release(Body_ptr part)
{
    auto it{std::find(m_subs.begin(), m_subs.end(), part)};
    assert(it != m_subs.end());
    if (it == m_subs.end())
        return;

    auto cm{r_cm()};
    part->m_parent = nullptr;
    m_subs.erase(it);

    part->m_r = transform_out(part->m_r);
    part->m_orientation = m_orientation*part->m_orientation;
    part->m_v_cm = m_v_cm + cross(m_omega, part->m_r - cm);
    m_v_cm += cross(m_omega, m_r - cm);
    part->m_omega = m_omega;
}

void Body::add_momentum(Body_ptr const part)
{
    // Assume constant omega (infinite inertia) if the inertia matrix is singular.
    if (det(m_inertia) == 0)
        return;
    if (m_parent)
        return m_parent->add_momentum(part);

    // Set head CM velocity and omega to conserve momentum.
    // 1. Find the new v_cm from head's and part's masses and their v_cms.
    // 2. Find new cm from head's aggregate and part's CMs
    // 3. Transform head's and part's I to the new CM.
    // 4. Find the new omega.
    // 5. Set part's position and orientation relative to this body's.
    // 6. Set velocity and omega to 0.

    // part must not already be a part.
    assert(!part->m_parent);
    auto head_m{m()};
    auto part_m{part->m()};
    auto v_cm{m_v_cm};
    m_v_cm = (head_m*v_cm + part_m*part->v_cm())/(head_m + part_m);

    auto new_cm{(head_m*r_cm() + part_m*part->r_cm())/(head_m + part_m)};
    auto r_head{r_cm() - new_cm};
    auto L_spin_head{I()*m_omega};
    auto L_orbit_head{head_m*cross(r_head, v_cm)};
    auto r_part = part->r_cm() - new_cm;
    auto L_spin_part{part->I()*part->omega()};
    auto L_orbit_part{part_m*cross(r_part, part->v_cm())};
    m_omega = inv(I(new_cm) + part->I(new_cm))
        *(L_spin_head + L_orbit_head + L_spin_part + L_orbit_part);
}

V3 Body::rotate_in(V3 const& v) const
{
    // Innermost rotation is done last.
    return tr(m_orientation)*(m_parent ? m_parent->rotate_in(v) : v);
}

V3 Body::transform_in(V3 const& v) const
{
    auto v_in{tr(m_orientation)*(v - m_r)};
    return m_parent ? m_parent->transform_in(v_in) : v_in;
}

V3 Body::rotate_out(V3 const& v) const
{
    // Innermost rotation is done first.
    auto v_out{m_orientation*v};
    return m_parent ? m_parent->rotate_out(v_out) : v_out;
}

V3 Body::transform_out(V3 const& v) const
{
    auto v_out{m_r + m_orientation*v};
    return m_parent ? m_parent->transform_out(v_out) : v_out;
}

bool Body::is_free() const
{
    return !m_parent;
}

bool Body::intersects(Body const&) const
{
    return false;
}

double Body::m() const
{
    return std::accumulate(m_subs.begin(), m_subs.end(), m_mass,
                           [](double m, Body_ptr b){ return m + b->m(); });
}

M3 Body::I()
{
    return I(m_parent ? m_parent->transform_out(r_cm()) : r_cm());
}

M3 Body::I(V3 const& center)
{
    V3 r{(m_parent ? m_parent->transform_out(m_r) : m_r) - center};
    return std::accumulate(m_subs.begin(), m_subs.end(),
                           m_inertia + m_mass*(square(r)*M1 - outer(r, r)),
                           [center](M3 const& i, const Body_ptr b){
                               return i + b->I(center); });
}

V3 Body::r() const
{
    return m_r;
}

V3 Body::r_cm() const
{
    // Head position is added after dividing by total mass.
    auto total{m()};
    if (total < 1e-9)
        return m_r;
    return m_r + std::accumulate(m_subs.begin(), m_subs.end(), V0,
                                 [this](V3 const& rm, Body_ptr b){
                                     return rm + rotate_out(b->r_cm())*b->m(); })
        /total;
}

V3 Body::v_cm() const
{
    return m_v_cm;
}

M3 const& Body::orientation() const
{
    return m_orientation;
}

V3 const& Body::omega() const
{
    return m_omega;
}

void Body::impulse(V3 const& imp)
{
    m_v_cm += imp/m();
}

void Body::impulse(V3 const& imp, V3 const& r)
{
    Body::impulse(imp);
    m_omega += cross(r - r_cm(), imp)*inv(I());
}

void Body::step(double time)
{
    // The origin of the body, m_r, is generally not at the CM.  Find the new origin after
    // rotation by transforming CM - m_r into the body's frame before rotating the body,
    // and then transforming back out of the body's frame.
    auto cm{r_cm()};
    auto dr{rotate_in(cm - m_r)};
    m_orientation = rot(m_orientation, rotate_in(m_omega)*time);
    m_r = cm + m_v_cm*time - rotate_out(dr);
    for (auto b : m_subs)
        b->step(time);
}

void Body::set_r(V3 const& r)
{
    m_r = r;
}

void Body::set_orientation(M3 const& o)
{
    m_orientation = o;
}

void Body::set_mass(double m)
{
    m_mass = m;
}

void Body::set_inertia(M3 const& i)
{
    m_inertia = i;
}
