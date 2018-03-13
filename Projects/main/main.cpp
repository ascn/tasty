#include "globalincludes.h"
#include "tetra.h"
#include "particles.h"
#include "tastyio.h"

#define BGEO_DIR "frames/"
#define TET_OUTNAME "cubeout"
#define BGEO_EXT ".bgeo"
#define BGEO_FNAME_PREFIX "frame_"
#define OBJ_FNAME "sphere.obj"

constexpr double dt = 1e-4;
constexpr double seconds = 20;
constexpr double fps = 24.0;
constexpr int frames = fps * seconds;
int main(int argc, char **argv) {
	// Tetrahedralize .obj and write to ele, node, face files
	tetgenio in, out;
	std::cout << "Reading " OBJ_FNAME "..." << std::endl;
	TastyIO::readOBJ(OBJ_FNAME, in);

	std::cout << "Tetrahedralizing " OBJ_FNAME "..." << std::endl;
	//tetrahedralize("pq1.1/0a0.1", &in, &out);
	tetrahedralize("pq1.414/0a0.1", &in, &out);

	out.save_nodes(TET_OUTNAME);
	out.save_elements(TET_OUTNAME);
	out.save_faces(TET_OUTNAME);

	//create a list of tetrahedra and an instance of particles from the .node and .ele files
    float k = 0.000001; //Young's modulus of jello
    k = 1000000;
	float nu = 0.3;
	Particles<double, 3> ptickles = Particles<double, 3>();
    std::vector<Tetra<double, 3>> tets;

	std::cout << "\nReading " TET_OUTNAME ".node..." << std::endl;
    TastyIO::parseNodeFile(TET_OUTNAME ".node", ptickles);
	std::cout << "Read " << ptickles.xs.size() << " particles" << std::endl;

	std::cout << "Reading " TET_OUTNAME ".ele..." << std::endl;
    TastyIO::parseEleFile(TET_OUTNAME ".ele", tets, k, nu, ptickles);
	std::cout << "Read " << tets.size() << " tetrahedrons" << std::endl;

	ptickles.setInitialTransform(Vector3d(0.f, 1.5f, 0.f),
								 Vector3d(0.f, 0.f, 0.f),
								 Vector3d(1.f, 1.f, 1.f));

	std::cout << "\nBeginning simulation..." << std::endl;
	double cumTime = 0.f;
	double simTime = 0.f;
	for (int frame = 0; frame < frames; ++frame) {
		std::cout << "  Writing frame " << std::setw(3) << std::setfill('0') << frame + 1 <<
				  " at timestep " << cumTime << std::endl;
		if (frame != 0) {
			double avg = simTime * 1e-6 / frame;
			std::cout << "    Estimated time remaining: " << avg * (frames - frame) << " s\n";
		}
		auto start = std::chrono::high_resolution_clock::now();
		std::stringstream ss;
		ss << std::setw(3) << std::setfill('0') << frame + 1;
		ptickles.writePartio(std::string(BGEO_DIR) + std::string(BGEO_FNAME_PREFIX) + ss.str() + std::string(BGEO_EXT));

		for (double timestep = 0; timestep < 1.f / fps; timestep += dt, cumTime += dt) {
			ptickles.resetForces();
			for (unsigned int i = 0; i < ptickles.fs.size(); ++i) {
				ptickles.addForce(i, Vector3d(0.f, -9.81 * ptickles.ms[i], 0.f));
			}
			for (Tetra<double, 3> tet : tets) {
				tet.computeElasticForces();
			}
			ptickles.tick(dt);
		}
		auto finish = std::chrono::high_resolution_clock::now();
		auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(finish - start);
		simTime += microseconds.count();
	}

	std::cout << "Simulation completed in " << simTime * 1e-6 << " s\n";
	std::cout << "Average frame time: " << simTime * 1e-6 / frames << " s\n";

	return 0;
}