#ifndef LOFT_TEST_TEST_HH_INCLUDED
#define LOFT_TEST_TEST_HH_INCLUDED

#include <iostream>

template<typename T> bool close(const T& actual, const T& expected, double tol)
{
    if (close(actual.x, expected.x, tol)
        && close(actual.y, expected.y, tol)
        && close(actual.z, expected.z, tol))
        return true;
    std::cout << actual << " != " << expected << " ±" << tol << std::endl;
    return false;
}

template<> bool close(const double& actual, const double& expected, double tol);

#endif // LOFT_TEST_TEST_HH_INCLUDED
