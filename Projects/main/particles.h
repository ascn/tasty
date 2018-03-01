#pragma once
#include "globalincludes.h"

template <typename T, int dim>
class Particles {
public:
	Particles() : size(0) {}

	std::vector<Eigen::Matrix<T, dim, 1>> xs;
	std::vector<Eigen::Matrix<T, dim, 1>> vs;
	std::vector<T> ms;
	std::vector<Eigen::Matrix<T, dim, 1>> fs;
	std::vector<Eigen::Matrix<T, dim, 1>> dfs;
	std::vector<Eigen::Matrix<T, dim, 1>> as;
	std::vector<Eigen::Matrix<float, 3, 1>> colors;
	std::vector<std::tuple<float, float>> uvs;

	void setInitialTransform(Vector3d t, Vector3d r, Vector3d s) {
		Mat4d tMat, rxMat, ryMat, rzMat, rMat, sMat;
		tMat << 1.f, 0.f, 0.f, t[0],
				0.f, 1.f, 0.f, t[1],
				0.f, 0.f, 1.f, t[2],
				0.f, 0.f, 0.f, 1.f;
		float crx = std::cos(r[0]);
		float srx = std::sin(r[0]);
		float cry = std::cos(r[1]);
		float sry = std::sin(r[1]);
		float crz = std::cos(r[2]);
		float srz = std::sin(r[2]);
		rxMat << 1.f, 0.f, 0.f, 0.f,
				 0.f, crx, -srx, 0.f,
				 0.f, srx, crx, 0.f,
				 0.f, 0.f, 0.f, 1.f;
		ryMat << cry, 0.f, sry, 0.f,
				 0.f, 1.f, 0.f, 0.f,
				 -sry, 0.f, cry, 0.f,
				 0.f, 0.f, 0.f, 1.f;
		rzMat << crz, -srz, 0.f, 0.f,
				 srz, crz, 0.f, 0.f,
				 0.f, 0.f, 1.f, 0.f,
				 0.f, 0.f, 0.f, 1.f;
		rMat = rzMat * ryMat * rxMat;
		sMat << s[0], 0.f, 0.f, 0.f,
				0.f, s[1], 0.f, 0.f,
				0.f, 0.f, s[2], 0.f,
				0.f, 0.f, 0.f, 1.f;
		Mat4d transform = tMat * rMat * sMat;
		for (unsigned int i = 0; i < xs.size(); ++i) {
			Vector4d tmp;
			tmp << xs[i][0], xs[i][1], xs[i][2], 1.f;
			tmp = transform * tmp;
			xs[i] = Vector3d(tmp[0], tmp[1], tmp[2]);
		}
	}

	Eigen::Matrix<T, dim, 1> getForce(int idx) {
		return fs[idx];
	}

	Eigen::Matrix<T, dim, 1> getDampingForce(int idx) {
		return dfs[idx];
	}

	Eigen::Matrix<T, dim, 1> getPosition(int idx) {
		return xs[idx];
	}

	Eigen::Matrix<T, dim, 1> getVelocity(int idx) {
		return vs[idx];
	}

	void addForce(int idx, Eigen::Matrix<T, dim, 1> force) {
		fs[idx] += force;
	}

	void addUniformForce(Eigen::Matrix<T, dim, 1> force) {
		for (unsigned int i = 0; i < fs.size(); ++i) {
			fs[i] += force;
		}
	}

	void addDampingForce(int idx, Eigen::Matrix<T, dim, 1> force) {
		dfs[idx] += force;
	}

	void resetForces() {
		for (unsigned int i = 0; i < fs.size(); ++i) {
			for (unsigned int j = 0; j < dim; ++j) {
				fs[i][j] == 0;
			}
		}
	}

	void addParticle(Eigen::Matrix<T, dim, 1> x,
					 Eigen::Matrix<T, dim, 1> v);

	void addParticle(Eigen::Matrix<T, dim, 1> x,
					 Eigen::Matrix<T, dim, 1> v,
					 T m,
					 Eigen::Matrix<T, dim, 1> f,
					 Eigen::Matrix<T, dim, 1> a);

	void addParticle(Eigen::Matrix<T, dim, 1> x,
					 Eigen::Matrix<T, dim, 1> v,
					 T m,
					 Eigen::Matrix<T, dim, 1> f,
					 Eigen::Matrix<T, dim, 1> a,
					 Eigen::Matrix<float, 3, 1> color);

	void tick(T time);

	void writePartio(const std::string&particleFile);

	int size;
};

enum class Integrator {
	Euler, RungeKutta2
};

template <typename T, int dim>
void Particles<T, dim>::tick(T time) {
	Integrator integrator = Integrator::Euler;
	for (unsigned i = 0; i < xs.size(); i++) {
		switch (integrator) {
		case Integrator::Euler:
			as[i] = fs[i] / ms[i];
			vs[i] += as[i] * time; // simple position and velocity case
			xs[i] += vs[i] * time;
			break;
		case Integrator::RungeKutta2:
			Eigen::Matrix<T, dim, 1> d1 = time * vs[i];
			Eigen::Matrix<T, dim, 1> d2 = xs[i] + time * vs[i];
			xs[i] = xs[i] + (d1 + d2)/2.0;

			Eigen::Matrix<T, dim, 1> d3 = time * as[i];
			Eigen::Matrix<T, dim, 1> d4 = vs[i] + time * as[i];
			vs[i] = vs[i] + (d3 + d4)/2.0;

			as[i] = fs[i] / ms[i];
			break;
		}
	}
};

template <typename T, int dim>
void Particles<T, dim>::addParticle(Eigen::Matrix<T, dim, 1> x,
									Eigen::Matrix<T, dim, 1> v) {
	xs.push_back(x);
	vs.push_back(v);
	ms.push_back(T(0.0));
	dfs.push_back(Eigen::Matrix<T, dim, 1>().setZero());
	fs.push_back(Eigen::Matrix<T, dim, 1>().setZero());
	as.push_back(Eigen::Matrix<T, dim, 1>().setZero());
	size++;
}

template <typename T, int dim>
void Particles<T, dim>::addParticle(Eigen::Matrix<T, dim, 1> x,
									Eigen::Matrix<T, dim, 1> v,
									T m,
									Eigen::Matrix<T, dim, 1> f,
									Eigen::Matrix<T, dim, 1> a) {
	xs.push_back(x);
	vs.push_back(v);
	ms.push_back(m);
	fs.push_back(f);
	as.push_back(a);
	dfs.push_back(Eigen::Matrix<T, dim, 1>().setZero());
	size++;
}

template <typename T, int dim>
void Particles<T, dim>::addParticle(Eigen::Matrix<T, dim, 1> x,
									Eigen::Matrix<T, dim, 1> v,
									T m,
									Eigen::Matrix<T, dim, 1> f,
									Eigen::Matrix<T, dim, 1> a,
									Eigen::Matrix<float, 3, 1> color) {
	xs.push_back(x);
	vs.push_back(v);
	ms.push_back(m);
	fs.push_back(f);
	as.push_back(a);
	dfs.push_back(Eigen::Matrix<T, dim, 1>().setZero());
	colors.push_back(color);
	size++;
}



template <class T, int dim>
void Particles<T, dim>::writePartio(const std::string& particleFile)
{
	Partio::ParticlesDataMutable* parts = Partio::create();
	Partio::ParticleAttribute posH, vH, mH, fH, cH;
	mH = parts->addAttribute("m", Partio::VECTOR, 1);
	posH = parts->addAttribute("position", Partio::VECTOR, 3);
	vH = parts->addAttribute("v", Partio::VECTOR, 3);
	fH = parts->addAttribute("f", Partio::VECTOR, 3);
	cH = parts->addAttribute("color", Partio::VECTOR, 3);
	for (unsigned i=0; i< xs.size(); i++){
		int idx = parts->addParticle();
		float* m = parts->dataWrite<float>(mH, idx);
		float* p = parts->dataWrite<float>(posH, idx);
		float* v = parts->dataWrite<float>(vH, idx);
		float* f = parts->dataWrite<float>(fH, idx);
		float* c = parts->dataWrite<float>(cH, idx);
		m[0] = ms[i];
		for (int k = 0; k < 3; k++)
			p[k] = (T)xs[i][k];
		for (int k = 0; k < 3; k++)
			v[k] = (T)vs[i][k];
		for (int k = 0; k < 3; k++)
			f[k] = (T)fs[i][k] / 10.f;
			//c[k] = (T)colors[i][k];
	}

	Partio::write(particleFile.c_str(), *parts);
	parts->release();
}
