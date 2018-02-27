#include "globalincludes.h"
#include "tetra.h"
#include "particles.h"
#include "tastyio.h"
constexpr int frames = 120;
int main(int argc, char **argv) {
	// Tetrahedralize .obj and write to ele, node, face files
	tetgenio in, out;
	TastyIO::readOBJ("cube.obj", in);

	tetrahedralize("pq1.1/0a0.1", &in, &out);

	out.save_nodes("cubeout");
	out.save_elements("cubeout");
	out.save_faces("cubeout");

    /*
     * void readOBJ(const std::string &objFilename, tetgenio &out);
     * void parseNodeFile(const std::string &inputNodeFile, Particles<float, 3> &ps);
     * void parseEleFile(const std::string &inputEleFile, std::vector<Tetra<float, 3>> &tets, float k);
     */

	//create a list of tetrahedra and an instance of particles from the .node and .ele files
    float k = 0.000001; //Young's modulus of jello
    Particles<float, 3> ptickles = Particles<float, 3>();
    std::vector<Tetra<float, 3>> tets;
    TastyIO::parseNodeFile("cubeout.node", ptickles);
    TastyIO::parseEleFile("cubeout.ele", tets, k, ptickles);

    // call writePartio for each frame
    for (int i = 0; i < frames; i++) {
        // call computeElasticForces every frame for every tetrahedra
        for (Tetra<float, 3> tet : tets) {
            tet.computeElasticForces();
        }
        ptickles.writePartio(std::to_string(i) + ".bgeo");
    }

	return 0;
}