#ifndef __DEFOBJ_H__
#define __DEFOBJ_H__

#include "globalincludes.h"
#include "tastyio.h"
#include "particles.h"
#include "tetra.h"

template <typename T, int dim>
class DefObj {
public:
	DefObj(const std::string &name, T k, T nu, const std::string &inFiles);
	void resetForces();
	void addForce(Eigen::Matrix<T, dim, 1> force);
	void addGravity();
	void calculateInternalForces();
	void tick(T time);

	std::string name;
	T k, nu;
	Particles<T, dim> particles;
	std::vector<Tetra<T, dim>> tetras;
};

template <typename T, int dim>
DefObj<T, dim>::DefObj(const std::string &name, T k, T nu, const std::string &inFiles) :
	name(name),
	k(k), nu(nu),
	particles(), tetras() {
	std::string nodeFile = inFiles + ".node";
	std::string eleFile = inFiles + ".ele";
	TastyIO::parseNodeFile(nodeFile, particles);
	TastyIO::parseEleFile(eleFile, tetras, k, nu, particles);
}

template <typename T, int dim>
void DefObj<T, dim>::resetForces() {
	particles.resetForces();
}

template <typename T, int dim>
void DefObj<T, dim>::addGravity() {
	for (unsigned int i = 0; i < particles.size; ++i) {
		particles.addForce(i, Vector3d(0.0, -9.81 * particles.ms[i], 0.0));
	}
}

template <typename T, int dim>
void DefObj<T, dim>::addForce(Eigen::Matrix<T, dim, 1> force) {
	for (unsigned int i = 0; i < particles.size; ++i) {
		particles.addForce(i, force);
	}
}

template <typename T, int dim>
void DefObj<T, dim>::calculateInternalForces() {
	for (Tetra<T, dim> tet : tetras) {
		tet.computeElasticForces();
	}
}

template <typename T, int dim>
void DefObj<T, dim>::tick(T time) {
	particles.tick(time);
}

#endif // __DEFOBJ_H__