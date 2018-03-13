#ifndef __SIMULATION_H__
#define __SIMULATION_H__

#include "globalincludes.h"
#include "defobj.h"

template <typename T, int dim>
class Simulation {
public:
	Simulation(double dt = 1e-4, double seconds = 20.0, double fps = 24.0);
	int addObject(DefObj<T, dim> &obj);
	void addForce(int idx, Eigen::Matrix<T, dim, 1> force);
	void setTransform(int idx, Eigen::Matrix<T, dim, 1> t, Eigen::Matrix<T, dim, 1> r, Eigen::Matrix<T, dim, 1> s);
	void run();

	std::string bgeo_dir;

	double dt;
	double seconds;
	double fps;
	double frames;
	std::vector<DefObj<T, dim>> objects;

private:
	void detectCollisions();
};

template <typename T, int dim>
Simulation::Simulation(double dt, double seconds, double fps) :
	dt(dt), seconds(seconds), fps(fps), frames(fps * seconds),
	objects() {}

template <typename T, int dim>
int Simulation::addObject(DefObj<T, dim> &obj) {
	objects.push_back(obj);
	return objects.size() - 1;
}

template <typename T, int dim>
void Simulation::addForce(int idx, Eigen::Matrix<T, dim, 1> force) {
	objects[idx].addForce(force);
}

template <typename T, int dim>
void Simulation::setTransform(int idx, Eigen::Matrix<T, dim, 1> t, Eigen::Matrix<T, dim, 1> r,
							  Eigen::Matrix<T, dim, 1> s) {
	objects[idx].particles.setInitialTransform(t, r, s);
}

template <typename T, int dim>
void Simulation::run() {
	std::cout << "Beginning simulation..." << std::endl;
	double cumTime = 0.0;
	double simTime = 0.0;
	for (int frame = 0; frame < frames; ++frame) {
		std::cout << "  Writing frame " << std::setw(3) << std::setfill('0') << frame + 1 <<
				  " at timestep " << cumTime << std::endl;
		if (frame != 0) {
			double avg = simTime * 1e-6 / frame;
			std::cout << "    Estimated time remaining: " << avg * (frames - frame) << " s" << std::endl;
		}
		auto start = std::chrono::high_resolution_clock::now();
		for (DefObj<T, dim> obj : objects) {
			std::stringstream ss;
			ss << std::setw(3) << std::setfill('0') << frame + 1;
			obj.particles.writePartio(bgeo_dir + obj.name + "_frame_" + ss.str() + ".bgeo");
		}

		for (double timestep = 0; timestep < 1.0 / fps; timestep += dt, cumTime += dt) {
			for (DefObj<T, dim> obj : objects) { obj.resetForces(); }
			for (DefObj<T, dim> obj : objects) { obj.addGravity(); }
			for (DefObj<T, dim> obj : objects) { obj.calculateInternalForces(); }
			detectCollisions();
		}

		auto finish = std::chrono::high_resolution_clock::now();
		auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(finish - start);
		simTime += microseconds.count();
	}
}

template <typename T, int dim>
void Simulation::detectCollisions() {
	// Detect collisions with rigid bodies and other deformable objects
}

#endif // __SIMULATION_H__