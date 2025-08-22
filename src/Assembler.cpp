#include "Assembler.h"
#include "Tet4Element.h"
#include <vector>

Eigen::SparseMatrix<double> Assembler::assembleGlobalStiffness(const Mesh& mesh, const Material& mat) const {
    const auto& nodes = mesh.getNodes();
    const auto& elements = mesh.getElements();

    if (nodes.empty()) {
        return Eigen::SparseMatrix<double>(0, 0);
    }

    size_t total_dofs = nodes.size() * 3; // 3 DOFs (x,y,z) per node
    Eigen::SparseMatrix<double> K(total_dofs, total_dofs);
    
    // Use a triplet list to efficiently build the sparse matrix
    std::vector<Eigen::Triplet<double>> triplet_list;

    for (const auto& elem_data : elements) {
        // 1. Get the nodes for the current element
        std::vector<Node> elem_nodes;
        std::vector<size_t> global_dof_map;
        elem_nodes.reserve(elem_data.connectivity.size());
        global_dof_map.reserve(elem_data.connectivity.size() * 3);

        for (int node_id : elem_data.connectivity) {
            // Node IDs in file are 1-based, vector indices are 0-based
            elem_nodes.push_back(nodes[node_id - 1]);
            global_dof_map.push_back((node_id - 1) * 3 + 0); // Global X DOF
            global_dof_map.push_back((node_id - 1) * 3 + 1); // Global Y DOF
            global_dof_map.push_back((node_id - 1) * 3 + 2); // Global Z DOF
        }

        // 2. Calculate the element's stiffness matrix [ke]
        if (elem_nodes.size() == 4) { // For now, only handle Tet4
            Tet4Element tet(elem_nodes);
            Eigen::Matrix<double, 12, 12> ke = tet.calculateStiffnessMatrix(mat);

            // 3. Add [ke] into the global triplet list
            for (int i = 0; i < 12; ++i) {
                for (int j = 0; j < 12; ++j) {
                    if (ke(i, j) != 0.0) {
                        triplet_list.emplace_back(global_dof_map[i], global_dof_map[j], ke(i, j));
                    }
                }
            }
        }
    }

    // 4. Build the sparse matrix from the triplets
    K.setFromTriplets(triplet_list.begin(), triplet_list.end());
    return K;
}