#include <tetgen.h>
#include "tastyio.h"

namespace {

void split(const std::string &s, char delim, std::vector<std::string> &v) {
	auto i = 0;
	auto pos = s.find(delim);
	while (pos != std::string::npos) {
		v.push_back(s.substr(i, pos-i));
		i = ++pos;
		pos = s.find(delim, pos);

		if (pos == std::string::npos)
			v.push_back(s.substr(i, s.length()));
	}
}

}

void TastyIO::readOBJ(const std::string &objFilename, tetgenio &out) {
	std::ifstream objFile;
	objFile.open(objFilename);
	if (!objFile.is_open()) { return; }
	std::vector<Vector3f> verts;
	std::vector<std::vector<int>> faces;
	std::string line;
	while (getline(objFile, line)) {
		std::vector<std::string> lineTokens;
		split(line, ' ', lineTokens);
		if (lineTokens.size() == 0) { continue; }
		if (lineTokens[0] == "v") {
			Vector3f vertPos;
			vertPos[0] = std::stof(lineTokens[1]);
			vertPos[1] = std::stof(lineTokens[2]);
			vertPos[2] = std::stof(lineTokens[3]);
			verts.push_back(vertPos);
		} else if (lineTokens[0] == "f") {
			std::vector<int> faceIdx;
			for (int i = 0; i < lineTokens.size() - 1; ++i) {
				std::vector<std::string> faceTokens;
				split(lineTokens[i + 1], '/', faceTokens);
				faceIdx.push_back(std::stoi(faceTokens[0]) - 1);
			}
			faces.push_back(faceIdx);
		}
	}

	out.numberofpoints = verts.size();
	out.pointlist = new REAL[out.numberofpoints * 3];
	for (unsigned int i = 0; i < verts.size(); ++i) {
		out.pointlist[3 * i + 0] = verts[i][0];
		out.pointlist[3 * i + 1] = verts[i][1];
		out.pointlist[3 * i + 2] = verts[i][2];
	}

	out.numberoffacets = faces.size();
	out.facetlist = new tetgenio::facet[out.numberoffacets];
	out.facetmarkerlist = new int[out.numberoffacets];
	tetgenio::facet *f;
	tetgenio::polygon *p;
	for (unsigned int i = 0; i < faces.size(); ++i) {
		f = &out.facetlist[i];
		f->numberofpolygons = 1;
		f->polygonlist = new tetgenio::polygon[f->numberofpolygons];
		f->numberofholes = 0;
		f->holelist = NULL;
		p = &f->polygonlist[0];
		p->numberofvertices = faces[i].size();
		p->vertexlist = new int[p->numberofvertices];
		for (unsigned int j = 0; j < p->numberofvertices; ++j) {
			p->vertexlist[j] = faces[i][j];
		}
		out.facetmarkerlist[i] = 0;
	}
	objFile.close();
}

void TastyIO::parseNodeFile(const std::string &inputNodeFile, Particles<float, 3> &ps) {
	std::string text;
	std::ifstream file(inputNodeFile);

	getline(file, text); //skipping over first line

	while(getline(file, text)) //each line of the file has the index of the particle then its x, y, and z coordinates
	{
		std::vector<float> vals;
		unsigned int idx = 0;
		std::vector<std::string> textTokens;
		split(text, ' ', textTokens);
		for (unsigned int i = 0; i < textTokens.size(); ++i) {
			if (textTokens[i].length() > 0) {
				vals.push_back(std::stof(textTokens[i]));
			}
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

void TastyIO::parseEleFile(const std::string &inputEleFile, std::vector<Tetra<float, 3>> &tets, float k, Particles<float, 3> &p) {
	std::string text;
	std::ifstream file(inputEleFile);

	getline(file, text); //skipping over first line

	while(getline(file, text)) //each line of the file has the index of the tetra then the indices of the four particles that make up the tetra
	{
		std::vector<float> vals;
		unsigned int idx = 0;
        std::vector<std::string> textTokens;
        split(text, ' ', textTokens);
        for (unsigned int i = 0; i < textTokens.size(); ++i) {
            if (textTokens[i].length() > 0) {
                vals.push_back(std::stoi(textTokens[i]));
            }
        }

		Tetra<float, 3> t = Tetra<float, 3>(vals[1], vals[2], vals[3], vals[4], k, &p);
		tets.push_back(t);
	}
}