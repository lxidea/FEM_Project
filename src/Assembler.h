#pragma once

#include "Mesh.h"
#include "Material.h"
#include <Eigen/Sparse>

class Assembler {
public:
    Eigen::SparseMatrix<double> assembleGlobalStiffness(const Mesh& mesh, const Material& mat) const;
};