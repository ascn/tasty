#ifndef __TASTYIO_H__
#define __TASTYIO_H__

#include "globalincludes.h"
#include "particles.h"
#include "tetra.h"

namespace TastyIO {

void readOBJ(const std::string &objFilename, tetgenio &out);
void parseNodeFile(const std::string &inputNodeFile, Particles<double, 3> &ps);
void parseEleFile(const std::string &inputEleFile, std::vector<Tetra<double, 3>> &tets, double k, Particles<double, 3> &p);

}

#endif // __TASTYIO_H__