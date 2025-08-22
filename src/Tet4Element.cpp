#include "Tet4Element.h"
#include <stdexcept>
#include <iostream>

Tet4Element::Tet4Element(const std::vector<Node>& nodes) {
    if (nodes.size() != 4) {
        throw std::invalid_argument("Tet4Element must be initialized with 4 nodes.");
    }
    for (size_t i = 0; i < 4; ++i) {
        node_coords_(i, 0) = nodes[i].x;
        node_coords_(i, 1) = nodes[i].y;
        node_coords_(i, 2) = nodes[i].z;
    }
}

double Tet4Element::getVolume() const {
    Eigen::Matrix4d m;
    m.block<4, 3>(0, 0) = node_coords_;
    m.col(3).setOnes();
    return std::abs(m.determinant()) / 6.0;
}

Eigen::Matrix<double, 6, 12> Tet4Element::calculateBMatrix() const {
    Eigen::Matrix4d C;
    C.block<4,3>(0,0) = node_coords_;
    C.col(3).setOnes();
    
    // If the determinant is near zero, the element is degenerate
    if (std::abs(C.determinant()) < 1e-12) {
        return Eigen::Matrix<double, 6, 12>::Zero();
    }

    Eigen::Matrix4d C_inv_T = C.inverse().transpose();
    Eigen::Matrix<double, 4, 3> dN_dxyz = C_inv_T.block<4,3>(0,0);

    Eigen::Matrix<double, 6, 12> B = Eigen::Matrix<double, 6, 12>::Zero();
    for (int i = 0; i < 4; ++i) {
        double dN_dx = dN_dxyz(i, 0);
        double dN_dy = dN_dxyz(i, 1);
        double dN_dz = dN_dxyz(i, 2);

        B(0, i * 3 + 0) = dN_dx;
        B(1, i * 3 + 1) = dN_dy;
        B(2, i * 3 + 2) = dN_dz;
        B(3, i * 3 + 0) = dN_dy; B(3, i * 3 + 1) = dN_dx;
        B(4, i * 3 + 1) = dN_dz; B(4, i * 3 + 2) = dN_dy;
        B(5, i * 3 + 0) = dN_dz; B(5, i * 3 + 2) = dN_dx;
    }
    return B;
}


Eigen::Matrix<double, 12, 12> Tet4Element::calculateStiffnessMatrix(const Material& mat) const {
    Eigen::Matrix<double, 6, 6> D = mat.getDMatrix();
    Eigen::Matrix<double, 6, 12> B = this->calculateBMatrix();
    double volume = getVolume();
    
    return B.transpose() * D * B * volume;
}

Eigen::Matrix<double, 6, 1> Tet4Element::calculateStrain(const Eigen::Matrix<double, 12, 1>& element_displacements) const {
    Eigen::Matrix<double, 6, 12> B = this->calculateBMatrix();
    return B * element_displacements;
}

Eigen::Matrix<double, 6, 1> Tet4Element::calculateStress(const Eigen::Matrix<double, 12, 1>& element_displacements, const Material& mat) const {
    Eigen::Matrix<double, 6, 6> D = mat.getDMatrix();
    Eigen::Matrix<double, 6, 1> strain = this->calculateStrain(element_displacements);
    return D * strain;
}