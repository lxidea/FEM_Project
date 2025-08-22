#include <gtest/gtest.h>
#include "Material.h"
#include <Eigen/Dense>

// Test that we can create a material and retrieve its properties.
TEST(MaterialTest, PropertiesAreSetCorrectly) {
    // Properties for steel (e.g., in Pascals)
    double youngsModulus = 200e9;
    double poissonsRatio = 0.3;

    Material steel(youngsModulus, poissonsRatio);

    // Check that the getters return the correct values
    ASSERT_DOUBLE_EQ(steel.getE(), 200e9);
    ASSERT_DOUBLE_EQ(steel.getNu(), 0.3);
}

TEST(MaterialTest, DMatrixCalculation) {
    // Properties for steel
    double E = 210e9; // Young's Modulus in Pascals
    double nu = 0.3;  // Poisson's Ratio
    Material steel(E, nu);

    Eigen::Matrix<double, 6, 6> D = steel.getDMatrix();

    // Check a few key values of the D-matrix based on the formula
    double prefactor = E / ((1.0 + nu) * (1.0 - 2.0 * nu));
    
    // D(0,0) should be prefactor * (1 - nu)
    ASSERT_NEAR(D(0, 0), prefactor * (1.0 - nu), 1e3);

    // D(0,1) should be prefactor * nu
    ASSERT_NEAR(D(0, 1), prefactor * nu, 1e3);

    // D(3,3) (a shear term) should be prefactor * (1 - 2*nu) / 2
    ASSERT_NEAR(D(3, 3), prefactor * (1.0 - 2.0 * nu) / 2.0, 1e3);
}