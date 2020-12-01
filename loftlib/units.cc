#include "units.hh"

M3 units::I_cylinder_shell(double m, double r, double l)
{
    return M3(m*(6*r*r + l*l)/12*Vx, m*(6*r*r + l*l)/12*Vy, m*r*r*Vz);
}

M3 units::I_cylinder_solid(double m, double r, double l)
{
    return M3(m*(3*r*r + l*l)/12*Vx, m*(3*r*r + l*l)/12*Vy, m*r*r/2*Vz);
}

double units::V_cylinder(double r, double l)
{
    return pi*r*r*l;
}
