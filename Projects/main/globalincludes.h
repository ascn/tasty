#ifndef __GLOBALINCLUDES_H__
#define __GLOBALINCLUDES_H__

#include <Partio.h>
#include <tetgen.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <cstdlib>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <cmath>
#include <iomanip>

using Vector2i = Eigen::Matrix<int, 2, 1>;
using Vector2f = Eigen::Matrix<float, 2, 1>;
using Vector2d = Eigen::Matrix<double, 2, 1>;
using Vector3i = Eigen::Matrix<int, 3, 1>;
using Vector3f = Eigen::Matrix<float, 3, 1>;
using Vector3d = Eigen::Matrix<double, 3, 1>;
using Mat3f = Eigen::Matrix<float, 3, 3>;
using Mat3d = Eigen::Matrix<double, 3, 3>;
#endif // __GLOBALINCLUDES_H__