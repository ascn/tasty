#include "globalincludes.h"
#include "tetra.h"
#include "particles.h"
#include "tastyio.h"

#define BGEO_DIR "frames/"
#define TET_OUTNAME "cubeout"
#define BGEO_EXT ".bgeo"
#define BGEO_FNAME_PREFIX "frame_"

constexpr int frames = 120;
int main(int argc, char **argv) {
	// Tetrahedralize .obj and write to ele, node, face files
	tetgenio in, out;
	TastyIO::readOBJ("cube.obj", in);

	tetrahedralize("pq1.1/0a0.1", &in, &out);

	out.save_nodes(TET_OUTNAME);
	out.save_elements(TET_OUTNAME);
	out.save_faces(TET_OUTNAME);

	//create a list of tetrahedra and an instance of particles from the .node and .ele files
    float k = 0.000001; //Young's modulus of jello
    Particles<float, 3> ptickles = Particles<float, 3>();
    std::vector<Tetra<float, 3>> tets;
    TastyIO::parseNodeFile(TET_OUTNAME ".node", ptickles);
    TastyIO::parseEleFile(TET_OUTNAME ".ele", tets, k, ptickles);

    // call writePartio for each frame
    for (int i = 0; i < frames; i++) {
        // call computeElasticForces every frame for every tetrahedra
        for (Tetra<float, 3> tet : tets) {
            tet.computeElasticForces();
        }
		for (auto &t : tets) {
			t.tick(1.f / 24.f);
		}
		std::stringstream ss;
		ss << std::setw(3) << std::setfill('0') << i;
        ptickles.writePartio(std::string(BGEO_DIR) + std::string(BGEO_FNAME_PREFIX) + ss.str() + std::string(BGEO_EXT));
    }

	return 0;
}