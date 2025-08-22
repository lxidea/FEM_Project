#include <iostream>
#include <fstream>
#include <vector>
#include "Mesh.h"
#include "Material.h"
#include "Assembler.h"
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>
#include <Eigen/SparseLU>

// A simple function to save the results to a VTK file for visualization
void save_vtk(const std::string& filename, const Mesh& mesh, const Eigen::VectorXd& displacements) {
    std::ofstream vtk_file(filename);
    const auto& nodes = mesh.getNodes();
    const auto& elements = mesh.getElements();

    vtk_file << "# vtk DataFile Version 3.0\n";
    vtk_file << "FEM Deformation\n";
    vtk_file << "ASCII\n";
    vtk_file << "DATASET UNSTRUCTURED_GRID\n";

    // Write nodal positions
    vtk_file << "POINTS " << nodes.size() << " double\n";
    for (size_t i = 0; i < nodes.size(); ++i) {
        vtk_file << (nodes[i].x + displacements(i * 3 + 0)) << " "
                 << (nodes[i].y + displacements(i * 3 + 1)) << " "
                 << (nodes[i].z + displacements(i * 3 + 2)) << "\n";
    }

    // Write element connectivity
    vtk_file << "CELLS " << elements.size() << " " << elements.size() * 5 << "\n";
    for (const auto& elem : elements) {
        vtk_file << "4"; // 4 nodes per tetrahedron
        for (int node_id : elem.connectivity) {
            vtk_file << " " << (node_id - 1); // VTK uses 0-based indexing
        }
        vtk_file << "\n";
    }

    // Write element types (VTK_TETRA = 10)
    vtk_file << "CELL_TYPES " << elements.size() << "\n";
    for (size_t i = 0; i < elements.size(); ++i) {
        vtk_file << "10\n";
    }

    // Write displacement vectors for coloring
    vtk_file << "POINT_DATA " << nodes.size() << "\n";
    vtk_file << "VECTORS Displacements double\n";
    for (size_t i = 0; i < nodes.size(); ++i) {
        vtk_file << displacements(i * 3 + 0) << " "
                 << displacements(i * 3 + 1) << " "
                 << displacements(i * 3 + 2) << "\n";
    }

    vtk_file.close();
    std::cout << "Successfully saved results to " << filename << std::endl;
}

int main() {
    // === 1. SETUP ===
    std::cout << "1. Setting up simulation..." << std::endl;
    Mesh mesh;
    // Load the new, correct mesh file
    if (!mesh.loadFromFile("single_tet.mesh")) {
        return -1;
    }
    Material steel(210e9, 0.3);
    Assembler assembler;

    // === 2. ASSEMBLE ===
    std::cout << "2. Assembling global stiffness matrix..." << std::endl;
    Eigen::SparseMatrix<double> K = assembler.assembleGlobalStiffness(mesh, steel);

    // === 3. DEFINE BCs AND LOADS ===
    std::cout << "3. Defining boundary conditions and loads..." << std::endl;
    size_t total_dofs = mesh.getNodes().size() * 3;
    Eigen::VectorXd F = Eigen::VectorXd::Zero(total_dofs);
    Eigen::VectorXd U = Eigen::VectorXd::Zero(total_dofs);

    // Apply a downward force on the free node (ID=4, index=3)
    int loaded_node_index = 3;
    double force_magnitude = -1e7; // 10 MegaNewtons downwards
    F(loaded_node_index * 3 + 2) = force_magnitude; // Apply in Z-direction

    // Identify fixed nodes (nodes 1, 2, and 3 are on the z=0 plane)
    std::vector<int> fixed_dofs;
    fixed_dofs.push_back(0 * 3 + 0); fixed_dofs.push_back(0 * 3 + 1); fixed_dofs.push_back(0 * 3 + 2); // Node 1
    fixed_dofs.push_back(1 * 3 + 0); fixed_dofs.push_back(1 * 3 + 1); fixed_dofs.push_back(1 * 3 + 2); // Node 2
    fixed_dofs.push_back(2 * 3 + 0); fixed_dofs.push_back(2 * 3 + 1); fixed_dofs.push_back(2 * 3 + 2); // Node 3

    // === 4. MODIFY SYSTEM FOR BCs ===
    std::cout << "4. Applying boundary conditions..." << std::endl;
    double penalty = 1e12 * K.diagonal().mean();
    for (int dof : fixed_dofs) {
        K.coeffRef(dof, dof) += penalty;
        F(dof) = 0.0;
    }

    // === 5. SOLVE ===
    std::cout << "5. Solving the linear system..." << std::endl;
    Eigen::SparseLU<Eigen::SparseMatrix<double>> solver;
    solver.compute(K);
    if(solver.info() != Eigen::Success) {
        std::cerr << "Error: Matrix decomposition failed." << std::endl;
        return -1;
    }
    U = solver.solve(F);
    if(solver.info() != Eigen::Success) {
        std::cerr << "Error: Linear system solve failed." << std::endl;
        return -1;
    }

    // === VALIDATION STEP ===
    std::cout << "\n--- Result Validation ---" << std::endl;
    loaded_node_index = 3; // Node 4 is at index 3
    double dx = U(loaded_node_index * 3 + 0);
    double dy = U(loaded_node_index * 3 + 1);
    double dz = U(loaded_node_index * 3 + 2);

    std::cout << "Displacement of loaded node (Node 4):" << std::endl;
    std::cout << "dx = " << dx << " m" << std::endl;
    std::cout << "dy = " << dy << " m" << std::endl;
    std::cout << "dz = " << dz << " m" << std::endl;

    // Sanity checks
    if (dz < 0) {
        std::cout << "Check PASSED: Displacement in Z is negative (downward)." << std::endl;
    } else {
        std::cout << "Check FAILED: Displacement in Z should be negative." << std::endl;
    }
    if (std::abs(dz) > std::abs(dx) && std::abs(dz) > std::abs(dy)) {
        std::cout << "Check PASSED: Z-displacement is the largest component." << std::endl;
    } else {
        std::cout << "Check FAILED: Z-displacement should be the largest component." << std::endl;
    }

    // === 6. SAVE RESULTS ===
    std::cout << "6. Saving results..." << std::endl;
    save_vtk("result.vtk", mesh, U);

    std::cout << "\nSimulation finished successfully!" << std::endl;
    return 0;
}