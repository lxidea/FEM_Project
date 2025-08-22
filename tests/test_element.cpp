#include <gtest/gtest.h>
#include "Tet4Element.h"
#include "Node.h"
#include "Material.h"
#include <vector>

TEST(Tet4ElementTest, VolumeCalculation) {
    // Create nodes for a simple right-angled tetrahedron at the origin
    // Its volume should be (1/6) * base * height = (1/6) * (0.5 * 2 * 3) * 4 = 2.0
    std::vector<Node> nodes = {
        {1, 0.0, 0.0, 0.0},
        {2, 2.0, 0.0, 0.0},
        {3, 0.0, 3.0, 0.0},
        {4, 0.0, 0.0, 4.0}
    };

    Tet4Element element(nodes);

    // The volume of a tetrahedron is 1/6th of the determinant of the matrix formed by its edge vectors
    ASSERT_NEAR(element.getVolume(), 4.0, 1e-9);
}

TEST(Tet4ElementTest, StiffnessMatrixRigidBodyMotion) {
    // 1. Define the element's geometry and material
    std::vector<Node> nodes = {
        {1, 0.2, 0.3, 0.1}, {2, 1.5, 0.5, 0.8},
        {3, 0.9, 1.7, 0.6}, {4, 0.4, 0.6, 1.4}
    };
    Material material(210e9, 0.3);
    Tet4Element element(nodes);

    // 2. Calculate the element stiffness matrix
    Eigen::Matrix<double, 12, 12> ke = element.calculateStiffnessMatrix(material);

    // 3. Define a rigid-body translation vector (e.g., move 1.0 unit in x)
    Eigen::Matrix<double, 12, 1> u = Eigen::Matrix<double, 12, 1>::Zero();
    for (int i = 0; i < 4; ++i) {
        u(i * 3 + 0) = 1.0; // Set all x-displacements to 1.0
    }

    // 4. Calculate nodal forces: F = K * u
    Eigen::Matrix<double, 12, 1> F = ke * u;

    // 5. Assert that the resulting forces are close to zero
    for (int i = 0; i < 12; ++i) {
        ASSERT_NEAR(F(i), 0.0, 1e-4);
    }
}

TEST(Tet4ElementTest, StressCalculation) {
    // 1. Define a simple unit element and material
    std::vector<Node> nodes = {
        {1, 0, 0, 0}, {2, 1, 0, 0}, {3, 0, 1, 0}, {4, 0, 0, 1}
    };
    Material material(1.0, 0.25); // Use simple values for easy calculation
    Tet4Element element(nodes);

    // 2. Define a simple displacement vector (uniaxial strain)
    // We displace node 2 by 0.1 in the x-direction.
    // This creates a uniform strain of 0.1 in the x-direction.
    Eigen::Matrix<double, 12, 1> u_e = Eigen::Matrix<double, 12, 1>::Zero();
    u_e(3) = 0.1; // Displacement of Node 2 (index 1) in X (dof 3)

    // 3. Calculate the stress
    Eigen::Matrix<double, 6, 1> stress = element.calculateStress(u_e, material);

    // 4. Verify the result
    // For uniaxial strain (exx=0.1), stress_x should be D(0,0)*0.1
    Eigen::Matrix<double, 6, 6> D = material.getDMatrix();
    double expected_stress_x = D(0, 0) * 0.1;
    double expected_stress_y = D(1, 0) * 0.1;
    
    ASSERT_NEAR(stress(0), expected_stress_x, 1e-9); // Check sigma_x
    ASSERT_NEAR(stress(1), expected_stress_y, 1e-9); // Check sigma_y
    ASSERT_NEAR(stress(3), 0.0, 1e-9);               // Shear stress should be zero
}