#pragma once
#include <Eigen/Dense> // Include Eigen

class Material {
public:
    // Constructor to initialize properties
    Material(double youngsModulus, double poissonsRatio);

    // Public getters
    double getE() const;  // Young's Modulus
    double getNu() const; // Poisson's Ratio

    // --- NEW METHOD ---
    Eigen::Matrix<double, 6, 6> getDMatrix() const;

private:
    double E_;  // Young's Modulus
    double nu_; // Poisson's Ratio
};