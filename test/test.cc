#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "test.hh"

#include "doctest.h"

template<> bool close(const double& actual, const double& expected, double tol)
{
    if (std::abs(expected - actual) < tol)
        return true;
    std::cout << actual << " != " << expected << " ±" << tol << std::endl;
    return false;
}
