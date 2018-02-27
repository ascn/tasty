#pragma once
#include <stdlib.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "particles.h"
/*
 * this class represents a single tetrahedra.
 */
template <typename T, int dim>
class Tetra {
public:
    Tetra(int x1, int x2, int x3, int x4, T k) : W(T(0.0)),
              // strainE(T(0.0)),
              F(Eigen::Matrix<T, 3, 3>()),
              Ds(Eigen::Matrix<T, 3, 3>()),
              Dm(Eigen::Matrix<T, 3, 3>()),
              x1(x1), x2(x2), x3(x3), x4(x4)
                {
                    computeDm(); // pre-compute the rest state
                    computeW();
                }

    void computeDm(); // material's rest state
    void computeDs(); // Ds is the deformed state
    void computeDefGrad(); // F = DsDm^(-1)
    void computeW();
    void computeP();
    void computeElasticForces(); // simple model for now

    T W; // undeformed volume of the tetrahedra
    T k;
    // T strainE; // strain energy
    Eigen::Matrix<T, 3, 3> F; // deformation gradient
    Eigen::Matrix<T, 3, 3> Ds; // deformed state
    Eigen::Matrix<T, 3, 3> Dm; // material state
    Eigen::Matrix<T, 3, 3> P;
    int x1;
    int x2;
    int x3;
    int x4;
    Particles<T, dim> *p;

};

template <typename T, int dim>
void Tetra<T, dim>::computeDm() { // material's rest state
    // assume that this is called at frame 0
    Eigen::Matrix<T, 3, 1> x1x4 = p->getPosition(x1) - p->getPosition(x4);
    Eigen::Matrix<T, 3, 1> x2x4 = p->getPosition(x2) - p->getPosition(x4);
    Eigen::Matrix<T, 3, 1> x3x4 = p->getPosition(x3) - p->getPosition(x4);

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
void Tetra<T, dim>::computeDs() { // Ds is the deformed state
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
void Tetra<T, dim>::computeDefGrad() { // F = DsDm^(-1)
    F = Ds * Dm.inverse();
};

template <typename T, int dim>
void Tetra<T, dim>::computeW() { // volume of the tetra
    W = (T(1.0) / T(6.0)) * Dm.determinant();
};

template <typename T, int dim>
void Tetra<T, dim>::computeP() { // strain energy density
    P = (k / T(2.0)) * (F - Eigen::Matrix<T, 3, 3>::Identity()).norm();
};

template <typename T, int dim>
void Tetra<T, dim>::computeElasticForces() { // simple model for now
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
