#include "body.hh"

#include <cassert>
#include <numbers>
#include <numeric>
#include <ranges>

Body::Body(double mass, const M3& inertia,
           const V3 r, const V3 v, const M3& orientation, const V3 omega)
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
    auto it = std::find(m_subs.begin(), m_subs.end(), part);
    assert(it != m_subs.end());
    if (it == m_subs.end())
        return;

    auto cm = r_cm();
    part->m_parent = nullptr;
    m_subs.erase(it);

    part->m_r = transform_out(part->m_r);
    part->m_orientation = m_orientation*part->m_orientation;
    part->m_v_cm = m_v_cm + cross(m_omega, part->m_r - cm);
    m_v_cm += cross(m_omega, m_r - cm);
    part->m_omega = m_omega;
}

void Body::add_momentum(const Body_ptr part)
{
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
    auto head_m = m();
    auto part_m = part->m();
    auto v_cm = m_v_cm;
    m_v_cm = (head_m*v_cm + part_m*part->v_cm())/(head_m + part_m);

    auto new_cm = (head_m*r_cm() + part_m*part->r_cm())/(head_m + part_m);
    auto r_head = r_cm() - new_cm;
    auto L_spin_head = I()*m_omega;
    auto L_orbit_head = head_m*cross(r_head, v_cm);
    auto r_part = part->r_cm() - new_cm;
    auto L_spin_part = part->I()*part->omega();
    auto L_orbit_part = part_m*cross(r_part, part->v_cm());
    // std::cout << new_cm << ' ' << r_head << ' ' << r_part << std::endl;
    // std::cout << I_head << ' ' << I_part << std::endl;
    // std::cout << m_omega << std::endl;
    // std::cout << v_cm << ' ' << part->v_cm() << std::endl;
    // std::cout << L_spin_head << ' ' << L_orbit_head << ' '
    //           << L_spin_part << ' ' << L_orbit_part << std::endl;
    m_omega = inv(I(new_cm) + part->I(new_cm))
        *(L_spin_head + L_orbit_head + L_spin_part + L_orbit_part);
}

V3 Body::rotate_in(const V3& v) const
{
    V3 v_in = tr(m_orientation)*v;
    return m_parent ? m_parent->rotate_in(v_in) : v_in;
}

V3 Body::transform_in(const V3& v) const
{
    V3 v_in = tr(m_orientation)*(v - m_r);
    return m_parent ? m_parent->transform_in(v_in) : v_in;
}

V3 Body::rotate_out(const V3& v) const
{
    V3 v_out = m_orientation*v;
    return m_parent ? m_parent->rotate_out(v_out) : v_out;
}

V3 Body::transform_out(const V3& v) const
{
    V3 v_out = m_r + m_orientation*v;
    return m_parent ? m_parent->transform_out(v_out) : v_out;
}

const Body* Body::top() const
{
    return m_parent ? m_parent->top() : this;
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

M3 Body::I(const V3& center)
{
    V3 r = (m_parent ? m_parent->transform_out(m_r) : m_r) - center;
    return std::accumulate(m_subs.begin(), m_subs.end(),
                           m_inertia + m_mass*(square(r)*M1 - outer(r, r)),
                           [center](const M3& i, const Body_ptr b){
                               return i + b->I(center); });
}

V3 Body::r() const
{
    return m_r;
}

V3 Body::r_cm() const
{
    // Head position is added after dividing by total mass.
    return m_r + std::accumulate(m_subs.begin(), m_subs.end(), V0,
                                 [this](const V3& rm, Body_ptr b){
                                     return rm + rotate_out(b->r_cm())*b->m(); })
        /m();
}

V3 Body::v_cm() const
{
    return m_v_cm;
}

const M3& Body::orientation() const
{
    return m_orientation;
}

const V3& Body::omega() const
{
    return m_omega;
}

void Body::step(double time)
{
    // The origin of the body, m_r, is generally not at the CM.  Find the new origin after
    // rotation by transforming CM - m_r into the body's frame before rotating the body,
    // and then transforming back out of the body's frame.
    auto cm = r_cm();
    auto dr = rotate_in(cm - m_r);
    m_orientation = rot(m_orientation, m_omega*time);
    m_r = cm + m_v_cm*time - rotate_out(dr);

    for (auto b : m_subs)
        b->step(time);
}
