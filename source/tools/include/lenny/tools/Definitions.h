#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <vector>

/**
 * Definitions
 */
#define PI 3.14159265358979323846

/**
 * Typedefs
 */
typedef unsigned int uint;

/**
 * Eigen definitions
 */
namespace Eigen {

typedef Eigen::Quaternion<double> QuaternionD;
typedef Eigen::Triplet<double> TripletD;
typedef std::vector<TripletD> TripletDList;
typedef Eigen::SparseMatrix<double> SparseMatrixD;

typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;

}  // namespace Eigen

/**
* Lenny definitions
*/
namespace lenny {
struct Ray {
    Eigen::Vector3d origin, direction;
};
}  // namespace lenny