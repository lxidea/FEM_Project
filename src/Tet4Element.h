#pragma once

#include <vector>
#include <Eigen/Dense>
#include "Node.h"
#include "Material.h"

class Tet4Element {
public:
    Tet4Element(const std::vector<Node>& nodes);
    double getVolume() const;
    Eigen::Matrix<double, 12, 12> calculateStiffnessMatrix(const Material& mat) const;
    
    // --- NEW METHODS ---
    Eigen::Matrix<double, 6, 1> calculateStrain(const Eigen::Matrix<double, 12, 1>& element_displacements) const;
    Eigen::Matrix<double, 6, 1> calculateStress(const Eigen::Matrix<double, 12, 1>& element_displacements, const Material& mat) const;

private:
    Eigen::Matrix<double, 4, 3> node_coords_;

    // --- NEW PRIVATE HELPER ---
    Eigen::Matrix<double, 6, 12> calculateBMatrix() const;
};