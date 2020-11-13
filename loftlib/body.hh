#ifndef LOFT_LOFTLIB_BODY_HH_INCLUDED
#define LOFT_LOFTLIB_BODY_HH_INCLUDED

#include "three-vector.hh"

#include <list>
#include <memory>

/// A rigid body or an aggregate of bodies.  A Body has a tree structure.  A body may have
/// sub-bodies that are part of its tree.  A body may be a sub-body of another tree.
class Body
{
    using Body_ptr = std::shared_ptr<Body>;

public:
    /// Create a spherical body that responds to forces and torques.
    Body(double mass, const M3& inertia,
         const V3 r, const V3 v, const M3& orientation, const V3 omega);
    virtual ~Body() = default;

    /// Attach a body at its current position conserving linear and angular momentum.  The
    /// attached body becomes fixed in location and orientation relative to this body.
    void capture(Body_ptr part);
    /// Remove the given body conserving linear and angular momentum.
    void release(Body_ptr part);

    // * Aggregate properties calculated from this body and its sub-bodies.
    /// @return Total mass
    double m() const;
    /// @return Total rotational inertia about the center of mass.
    M3 I();
    /// @return Position of the center of mass.
    V3 r_cm() const;
    /// @return Velocity of the center of mass.
    V3 v_cm() const;
    /// The matrix that rotates out of the body frame.
    const M3& orientation() const;
    /// @return A vector parallel to the axis of rotation whose magnitude is the body's
    /// angular velocity.
    const V3& omega() const;

    /// @return Position of this body relative to the parent if there is one, else the
    /// absolute position.
    V3 r() const;

    /// Update the body's position, velocity, orientation, and angular momentum.
    void step(double time);

private:
    /// Rotate an absolute point to this body's frame.
    V3 rotate_in(const V3& v) const;
    /// Transform an absolute position vector to this body's frame.
    V3 transform_in(const V3& v) const;
    /// Rotate a point in this body's frame to the absolute frame.
    V3 rotate_out(const V3& v) const;
    /// Transform a position vector in this body's frame to the absolute frame.
    V3 transform_out(const V3& v) const;

    /// @return Total rotational inertia about a point.
    M3 I(const V3& center);
    /// Take care of conservation of linear and angular momentum when a body is added.
    void add_momentum(const Body_ptr part);
    /// @return The top-level body at the root of the tree structure that this body
    /// belongs to.
    const Body* top() const;
    /// The body that has this body as one of its sub-bodies, or nullptr if this is the
    /// top-level body.
    Body* m_parent = nullptr;
    /// The sub-bodies of this body.
    std::list<Body_ptr> m_subs;

    // * Properties that are unchanged when the body becomes part of an aggregate.
    /// This body's mass, not including sub-bodies.
    double m_mass;
    /// This body's rotational inertia, not including sub-bodies.
    M3 m_inertia;

    // * Properties that become relative to the parent when aggregated.
    /// Position relative to the parent if one is present, else absolute position.
    V3 m_r;
    /// The velocity of the center of mass of this sub-body.  Zero when there's a parent.
    V3 m_v_cm;
    /// The matrix that rotates vectors from the body frame to the parent's frame.  A
    /// vector in the direction v in the body frame is in the direction m_orientation*v in
    /// the parent frame.
    M3 m_orientation;
    /// The angular velocity of the center of mass of this sub-body.  Zero when there's a
    /// parent.
    V3 m_omega;
};

#endif // LOFT_LOFTLIB_BODY_HH_INCLUDED
