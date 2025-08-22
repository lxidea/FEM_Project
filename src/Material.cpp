#include "Material.h"

Material::Material(double youngsModulus, double poissonsRatio)
    : E_(youngsModulus), nu_(poissonsRatio) {}

double Material::getE() const {
    return E_;
}

double Material::getNu() const {
    return nu_;
}

Eigen::Matrix<double, 6, 6> Material::getDMatrix() const {
    Eigen::Matrix<double, 6, 6> D = Eigen::Matrix<double, 6, 6>::Zero();

    // Prefactor for 3D isotropic material
    double prefactor = E_ / ((1.0 + nu_) * (1.0 - 2.0 * nu_));

    D(0, 0) = D(1, 1) = D(2, 2) = prefactor * (1.0 - nu_);
    D(0, 1) = D(1, 0) = prefactor * nu_;
    D(0, 2) = D(2, 0) = prefactor * nu_;
    D(1, 2) = D(2, 1) = prefactor * nu_;
    D(3, 3) = D(4, 4) = D(5, 5) = prefactor * (1.0 - 2.0 * nu_) / 2.0;

    return D;
}