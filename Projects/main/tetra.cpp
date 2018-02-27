//
// Created by emilyhvo on 2/27/18.
//

#include "tetra.h"
template <typename T, int dim>
void Tetra::computeDm() { // material's rest state
    // assume that this is called at frame 0
    Eigen::Matrix<T, 3, 3> x1x4 = p->getPosition(x1) - p->getPosition(x4);
    Eigen::Matrix<T, 3, 3> x2x4 = p->getPosition(x2) - p->getPosition(x4);
    Eigen::Matrix<T, 3, 3> x3x4 = p->getPosition(x3) - p->getPosition(x4);

    // first column = x1 - x4
    Dm(0, 0) = x1x4[0];
    Dm(1, 0) = x1x4[1];
    Dm(2, 0) = x1x4[2];

    // second column = x2 - x4
    Dm(0, 1) = x2x4[0];
    Dm(1, 1) = x2x4[1];
    Dm(2, 1) = x2x4[2];

    // third column = x3 - x4
    Dm(0, 2) = x3x4[0];
    Dm(1, 2) = x3x4[1];
    Dm(2, 2) = x3x4[2];
};

template <typename T, int dim>
void Tetra::computeDs() { // Ds is the deformed state
    Eigen::Matrix<T, 3, 3> x1x4 = p->getPosition(x1) - p->getPosition(x4);
    Eigen::Matrix<T, 3, 3> x2x4 = p->getPosition(x2) - p->getPosition(x4);
    Eigen::Matrix<T, 3, 3> x3x4 = p->getPosition(x3) - p->getPosition(x4);

    // first column = x1 - x4
    Ds(0, 0) = x1x4[0];
    Ds(1, 0) = x1x4[1];
    Ds(2, 0) = x1x4[2];

    // second column = x2 - x4
    Ds(0, 1) = x2x4[0];
    Ds(1, 1) = x2x4[1];
    Ds(2, 1) = x2x4[2];

    // third column = x3 - x4
    Ds(0, 2) = x3x4[0];
    Ds(1, 2) = x3x4[1];
    Ds(2, 2) = x3x4[2];
};

template <typename T, int dim>
void Tetra::computeDefGrad() { // F = DsDm^(-1)
    F = Ds * Dm.inverse();
};

template <typename T, int dim>
void Tetra::computeW() { // volume of the tetra
    W = (T(1.0) / T(6.0)) * Dm.determinant();
};

template <typename T, int dim>
void Tetra::computeP() { // strain energy density
    P = (k / T(2.0)) * (F - Eigen::Matrix<T, 3, 3>::Identity()).norm();
};

template <typename T, int dim>
void Tetra::computeElasticForces() { // simple model for now
    computeDs();
    computeDefGrad();
    computeP();

    // compute H
    Eigen::Matrix<T, 3, 3> H;

    // add forces to p1, p2, p3, p4
    p->addForce(x1, H.col(0));
    p->addForce(x2, H.col(1));
    p->addForce(x3, H.col(2));
    p->addForce(x4, -H.col(0) - H.col(1) - H.col(2));
};