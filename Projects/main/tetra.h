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
              F(Eigen::Matrix<T, 3, 3>(T(0.0))),
              Ds(Eigen::Matrix<T, 3, 3>(T(0.0))),
              Dm(Eigen::Matrix<T, 3, 3>(T(0.0f))),
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

