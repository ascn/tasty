#include "globalincludes.h"
#include "tetra.h"
#include "particles.h"

void parseNodeFile(const std::string &inputNodeFile, Particles<float, 3> &ps) {
	std::string text;
	std::ifstream file(inputNodeFile);

	getline(file, text); //skipping over first line

	while(getline(file, text)) //each line of the file has the index of the particle then its x, y, and z coordinates
	{
		std::vector<float> vals;
		unsigned int idx = 0;
		while (!text.empty()) {
			idx = text.find(" ");
			std::string x = text.substr(0, idx);
			vals.push_back(std::stof(x));
			text.erase(0, idx + 1);
		}

		Eigen::Matrix<float, 3, 1> x;
		x[0] = vals[1]; x[1] = vals[2]; x[2] = vals[3];

        Eigen::Matrix<float, 3, 1> v;
        v[0] = 0.f; v[1] = 0.f; v[2] = 0.f;

        Eigen::Matrix<float, 3, 1> f;
        f[0] = 0.f; f[1] = 0.f; f[2] = 0.f;

        Eigen::Matrix<float, 3, 1> a;
        a[0] = 0.f; a[1] = -9.8; a[2] = 0.f;

        ps.addParticle(x, v, 1.f, f, a);
	}
}

void parseEleFile(const std::string &inputEleFile, std::vector<Tetra<float, 3>> &tets, float k){
	std::string text;
	std::ifstream file(inputEleFile);

	getline(file, text); //skipping over first line

	while(getline(file, text)) //each line of the file has the index of the tetra then the indices of the four particles that make up the tetra
	{
		std::vector<float> vals;
		unsigned int idx = 0;
		while (!text.empty()) {
			idx = text.find(" ");
			std::string x = text.substr(0, idx);
			vals.push_back(std::stoi(x));
			text.erase(0, idx + 1);
		}

		Tetra<float, 3> t = Tetra<float, 3>(vals[1], vals[2], vals[3], vals[4], k);
		tets.push_back(t);
	}
}

int main(int argc, char **argv) {
	//create a list of tetrahedra and an instance of particles from the .node and .ele files
    Particles<float, 3> ptickles = Particles<float, 3>();
    std::vector<Tetra<float, 3>> tets;

    float k = 0.000001; //Young's modulus of jello

	// call computeElasticForces every frame for every tetrahedra
    // call writePartio for each frame
	return 0;
}