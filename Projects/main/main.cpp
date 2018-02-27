#include "globalincludes.h"
#include "tetra.h"
#include "particles.h"
#include "tastyio.h"

int main(int argc, char **argv) {
	// Tetrahedralize .obj and write to ele, node, face files
	tetgenio in, out;
	TastyIO::readOBJ("cube.obj", in);

	tetrahedralize("pq1.1/0a0.1", &in, &out);

	out.save_nodes("cubeout");
	out.save_elements("cubeout");
	out.save_faces("cubeout");

	//create a list of tetrahedra and an instance of particles from the .node and .ele files
    Particles<float, 3> ptickles = Particles<float, 3>();
    std::vector<Tetra<float, 3>> tets;

    float k = 0.000001; //Young's modulus of jello

	// call computeElasticForces every frame for every tetrahedra
    // call writePartio for each frame

	return 0;
}