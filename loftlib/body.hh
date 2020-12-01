#ifndef LOFT_LOFTLIB_BODY_HH_INCLUDED
#define LOFT_LOFTLIB_BODY_HH_INCLUDED

#include "three-vector.hh"

#include <list>
#include <memory>

/// A rigid body in three-dimensional space.  A body has physical properties (mass, inertia
/// and physical extent) and state (position, velocity, orientation and angular velocity).
/// A body may be captured by another body, in which case the capturing body becomes an
/// aggregate; its physical properties become the properties of the system.  Its state
/// changes to conserve linear and angular momentum.  A captured body may be released.
class Body
{
    using Body_ptr = std::shared_ptr<Body>;

public:
    /// Create a spherical body with initial physical properties and state.
    Body(double mass, const M3& inertia,
         const V3 r, const V3 v, const M3& orientation, const V3 omega);
    virtual ~Body() = default;

    /// Attach a body at its current position conserving linear and angular momentum.  The
    /// attached body becomes fixed in location and orientation relative to this body.
    void capture(Body_ptr part);
    /// Remove the given body conserving linear and angular momentum.
    void release(Body_ptr part);

    // * Physical properties calculated from this body and its sub-bodies.
    /// @return Total mass
    double m() const;
    /// @return Total rotational inertia about the center of mass.
    M3 I();
    /// @return Position of the center of mass.
    V3 r_cm() const;
    /// @return Velocity of the center of mass.
    V3 v_cm() const;
    /// The matrix that rotates the parent frame to this one.
    const M3& orientation() const;
    /// @return A vector parallel to the axis of rotation whose magnitude is the body's
    /// angular velocity.
    const V3& omega() const;

    /// @return Position of this body relative to the parent if there is one, else the
    /// absolute position.
    V3 r() const;

    /// Impart an impulse at the center of mass; change linear momentum but not angular
    /// momentum.
    /// @param imp Absolute impulse vector.
    void impulse(const V3& imp);
    /// Impart an impulse at a given position.  In general, linear momentum and angular
    /// momentum change.
    void impulse(const V3& imp, const V3& r);

    /// Update the body's properties and state.
    virtual void step(double time);

    /// Rotate an absolute vector to this body's frame.
    V3 rotate_in(const V3& v) const;
    /// Transform an absolute position vector to this body's frame.
    V3 transform_in(const V3& v) const;
    /// Rotate a vector in this body's frame to the absolute frame.
    V3 rotate_out(const V3& v) const;
    /// Transform a position vector in this body's frame to the absolute frame.
    V3 transform_out(const V3& v) const;

    /// @return True if this body is not captured by another body.
    bool is_free() const;
    /// @return True if this body occupies some of the same space as another.
    virtual bool intersects(const Body& b) const;

    // * Direct manipulation.  Useful for construction and for updating as properties
    //change, e.g. fuel is consumed.
    /// Set the body's position.
    void set_r(const V3& r);
    /// Set the body's orientation.
    void set_orientation(const M3& o);
    /// Set the body's mass.
    void set_mass(double mass);
    /// Set the body's inertia tensor.
    void set_inertia(const M3& i);

protected:
    // * Physical properties
    /// This body's mass, not including sub-bodies.
    double m_mass;
    /// This body's rotational inertia, not including sub-bodies.
    M3 m_inertia;

private:
    /// @return Total rotational inertia about a point.
    M3 I(const V3& center);
    /// Take care of conservation of linear and angular momentum when a body is added.
    void add_momentum(const Body_ptr part);

    /// The body that has this body as one of its sub-bodies, or nullptr if this is the
    /// top-level body.
    Body* m_parent = nullptr;
    /// The sub-bodies of this body.
    std::list<Body_ptr> m_subs;

    // * State
    /// Position relative to the enclosing frame.
    V3 m_r;
    /// Velocity of the center of mass in the enclosing frame.
    V3 m_v_cm;
    /// The matrix that rotates vectors from the body frame to the parent's frame.  A
    /// vector in the direction v in the body frame is in the direction m_orientation*v in
    /// the parent frame.  Alternatively, m_orientation can be though of as rotating the
    /// parent axes to the body's frame.
    M3 m_orientation;
    /// The angular velocity vector of this body in the enclosing frame.
    V3 m_omega;
};

#endif // LOFT_LOFTLIB_BODY_HH_INCLUDED
