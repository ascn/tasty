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
    Tetra(int x1, int x2, int x3, int x4, T k, T nu, Particles<T, dim> *p) :
			W(T(0.0)),
			// strainE(T(0.0)),
			  k(k), nu(nu),
              F(Eigen::Matrix<T, 3, 3>()),
              Ds(Eigen::Matrix<T, 3, 3>()),
              Dm(Eigen::Matrix<T, 3, 3>()),
		      P(Eigen::Matrix<T, 3, 3>()),
              x1(x1), x2(x2), x3(x3), x4(x4), p(p)
                {
                    computeDm(); // pre-compute the rest state
                    computeW();
					computeParticleMass();
                }

    void computeSphereCenter();
    void computeSphereRadius();
    void computeDm(); // material's rest state
    void computeDs(); // Ds is the deformed state
    void computeDefGrad(); // F = DsDm^(-1)
    void computeW();
	void computeParticleMass();
    void computeP();
    void computeElasticForces(); // simple model for now

    T W; // undeformed volume of the tetrahedra
    T k;
	T nu;
    T sphereRadius; // Bounding sphere's radius
    Eigen::Matrix<T, 3, 3> B;
    Eigen::Matrix<T, 3, 3> F; // deformation gradient
    Eigen::Matrix<T, 3, 3> Ds; // deformed state
    Eigen::Matrix<T, 3, 3> Dm; // material state
    Eigen::Matrix<T, 3, 3> P;
    Eigen::Matrix<T, 3, 1> X; // Center of the tetrahedron's bounding sphere
    int x1;
    int x2;
    int x3;
    int x4;
    Particles<T, dim> *p;
};

// Compute the (x,y,z) position of this tetrahedron's bounding sphere
template <typename T, int dim>
void Tetra<T, dim>::computeSphereCenter() {
    Eigen::Matrix<T, 3, 1> A = p->getPosition(x1);
    Eigen::Matrix<T, 3, 1> B = p->getPosition(x2);
    Eigen::Matrix<T, 3, 1> C = p->getPosition(x3);
    Eigen::Matrix<T, 3, 1> D = p->getPosition(x4);
    Eigen::Matrix<T, 3, 3> M = Eigen::Matrix<T, 3, 3>::Zero();

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            if (i == 0) {
                M[i][j] = A[j] - B[j];
            }
            else if (i == 1) {
                M[i][j] = B[j] - C[j];
            }
            else {
                M[i][j] = C[j] - D[j];
            }
        }
    }

    // Really just in here for posterity; we shouldn't ever hit this case for a properly tetrahedralized mesh
    if (M.determinant() == 0) {
        throw std::logic_error("All points are coplanar; cannot find a unique solution.");
    }

    Eigen::Matrix<T, 3, 1> y;
    y << (A.squaredNorm() - B.squaredNorm()), (B.squaredNorm() - C.squaredNorm()), (C.squaredNorm() - D.squaredNorm());

    X = M.inverse() * (0.5 * y);
};

// Compute the radius of this tetrahedron's bounding sphere, given the center has been found
template <typename T, int dim>
void Tetra<T, dim>::computeSphereRadius() {
    Eigen::Matrix<T, 3, 1> R = X - (p->getPosition(x1));
    sphereRadius = R.norm();
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
    B = Dm.inverse();
};

template <typename T, int dim>
void Tetra<T, dim>::computeDs() { // Ds is the deformed state
    Eigen::Matrix<T, 3, 1> x1x4 = p->getPosition(x1) - p->getPosition(x4);
    Eigen::Matrix<T, 3, 1> x2x4 = p->getPosition(x2) - p->getPosition(x4);
    Eigen::Matrix<T, 3, 1> x3x4 = p->getPosition(x3) - p->getPosition(x4);

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
    for (unsigned int i = 0; i < 3; ++i)
    {
        for (unsigned int j = 0; j < 3; ++j)
        {
            if (std::abs(F(i,j)) < 1e-12)
            {
                F(i,j) = 0.f;
            }
        }
    }
};

template <typename T, int dim>
void Tetra<T, dim>::computeW() { // volume of the undeformed tetra
	W = std::abs((1.0 / 6.0) * Dm.determinant());
};

enum class StrainModel {
	Linear,
	StVenant_Kirchhoff,
	CorotatedLinear,
	Neohookean,
	pba
};

template <typename T, int dim>
void Tetra<T, dim>::computeParticleMass() {
	float density = 1000.f;
	float tetMass = density * W;
	p->ms[x1] += tetMass / 4.f;
	p->ms[x2] += tetMass / 4.f;
	p->ms[x3] += tetMass / 4.f;
	p->ms[x4] += tetMass / 4.f;
}

template <typename T, int dim>
void Tetra<T, dim>::computeP() { // strain energy density
	double mu = k / (2 * (1.0 + nu));
	double lambda = (k * nu) / ((1.0 + nu) * (1.0 - (2.0 * nu)));
	Mat3d E = (1.0 / 2.0) * (F.transpose() * F - Mat3d::Identity());

	StrainModel model = StrainModel::pba;
	switch (model) {
	case StrainModel::Linear:
		P = mu * (F + F.transpose() - 2 * Mat3d::Identity()) + lambda * (F - Mat3d::Identity()).trace() * Mat3d::Identity();
		break;
	case StrainModel::StVenant_Kirchhoff:
		P = F * (2.f * mu * E + lambda * E.trace() * Mat3d::Identity());
		break;
	case StrainModel::CorotatedLinear: {
		Eigen::JacobiSVD<Mat3d> svd(F, Eigen::ComputeFullU | Eigen::ComputeFullV);
		Mat3d U = svd.matrixU();
		Mat3d V = svd.matrixV();
		Mat3d R = U * V.transpose();
		P = 2 * mu * (F - R) + lambda * (R.transpose() * F - Mat3d::Identity()).trace() * R;
		break;
	}
	case StrainModel::Neohookean: {
		Eigen::JacobiSVD<Mat3d> svd(F, Eigen::ComputeFullU | Eigen::ComputeFullV);
		Mat3d F_invTrans = F.inverse().transpose();
		float J = F.determinant();
		float logJ = std::log(J);
		P = mu * (F - mu * F_invTrans) + lambda * logJ * F_invTrans;
		break;
	}
	case StrainModel::pba: {
		Eigen::JacobiSVD<Mat3d> svd(F, Eigen::ComputeFullU | Eigen::ComputeFullV);
		Mat3d U = svd.matrixU();
		Mat3d V = svd.matrixV();
		if (U.determinant() < 0) { U.col(2) *= -1; }
		if (V.determinant() < 0) { V.col(2) *= -1; }

		Mat3d R = U * V.transpose();

        Mat3d JFinvT;
        JFinvT(0, 0) = F(1, 1) * F(2, 2) - F(1, 2) * F(2, 1);
        JFinvT(0, 1) = F(1, 2) * F(2, 0) - F(1, 0) * F(2, 2);
        JFinvT(0, 2) = F(1, 0) * F(2, 1) - F(1, 1) * F(2, 0);
        JFinvT(1, 0) = F(0, 2) * F(2, 1) - F(0, 1) * F(2, 2);
        JFinvT(1, 1) = F(0, 0) * F(2, 2) - F(0, 2) * F(2, 0);
        JFinvT(1, 2) = F(0, 1) * F(2, 0) - F(0, 0) * F(2, 1);
        JFinvT(2, 0) = F(0, 1) * F(1, 2) - F(0, 2) * F(1, 1);
        JFinvT(2, 1) = F(0, 2) * F(1, 0) - F(0, 0) * F(1, 2);
        JFinvT(2, 2) = F(0, 0) * F(1, 1) - F(0, 1) * F(1, 0);
        double J = F.determinant();
        P = 2 * mu * (F - R) + lambda * (J - 1.f) * JFinvT;
	}
	}
};

template <typename T, int dim>
void Tetra<T, dim>::computeElasticForces() {
    computeDs();
    computeDefGrad();
    computeP();

    // compute H
    Eigen::Matrix<T, 3, 3> H = -W * P * B.transpose();

    // add forces to p1, p2, p3, p4
    p->addForce(x1, H.col(0));
    p->addForce(x2, H.col(1));
    p->addForce(x3, H.col(2));
    p->addForce(x4, -H.col(0) - H.col(1) - H.col(2));
};