#include <gtest/gtest.h>
#include "Assembler.h"
#include "Mesh.h"
#include "Material.h"
#include "Tet4Element.h"
#include <vector>

TEST(AssemblerTest, TwoElementAssemblyCheck) {
    // 1. Create a mesh in memory: 5 nodes, 2 tet elements sharing a face
    Mesh mesh;
    mesh.addNode(1, 0, 0, 0);
    mesh.addNode(2, 1, 0, 0);
    mesh.addNode(3, 1, 1, 0);
    mesh.addNode(4, 0, 1, 0);
    mesh.addNode(5, 0.5, 0.5, 1); // Peak of a "tent"

    // Element 1 uses nodes {1, 2, 4, 5} (IDs are 1-based)
    mesh.addElement({1, 2, 4, 5});
    // Element 2 uses nodes {2, 3, 4, 5}
    mesh.addElement({2, 3, 4, 5});

    // 2. Define material and assembler
    Material material(210e9, 0.3);
    Assembler assembler;

    // 3. Assemble the global stiffness matrix
    Eigen::SparseMatrix<double> K = assembler.assembleGlobalStiffness(mesh, material);

    // 4. Perform Checks
    // The total number of degrees of freedom should be 5 nodes * 3 DOFs/node = 15
    ASSERT_EQ(K.rows(), 15);
    ASSERT_EQ(K.cols(), 15);

    // Check a value for a node that is NOT shared.
    // Node 1 (index 0) is only in element 1. Its K(0,0) entry should match the
    // local ke(0,0) from element 1.
    Tet4Element elem1({mesh.getNodes()[0], mesh.getNodes()[1], mesh.getNodes()[3], mesh.getNodes()[4]});
    auto ke1 = elem1.calculateStiffnessMatrix(material);
    ASSERT_NEAR(K.coeff(0, 0), ke1(0, 0), 1e-6);

    // Check a value for a node that IS shared.
    // Node 5 (index 4) is in both elements. Its K(12,12) entry should be the
    // sum of the contributions from both elements.
    Tet4Element elem2({mesh.getNodes()[1], mesh.getNodes()[2], mesh.getNodes()[3], mesh.getNodes()[4]});
    auto ke2 = elem2.calculateStiffnessMatrix(material);
    // Node 5 is the 4th node listed in the connectivity for both elements.
    // Its local DOFs start at index 3*3=9. So we check the (9,9) entry of each ke.
    double expected_k_val = ke1(9, 9) + ke2(9, 9);
    // The global DOFs for node 5 (index 4) start at index 4*3=12.
    ASSERT_NEAR(K.coeff(12, 12), expected_k_val, 1e-6);

    // Check a zero entry. Nodes 1 and 3 are not in the same element.
    // The block connecting them in K should be zero.
    ASSERT_NEAR(K.coeff(0, 6), 0.0, 1e-9);
}