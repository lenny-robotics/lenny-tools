#pragma once

#include <lenny/tools/Definitions.h>

#include <string>

namespace lenny::tools::utils {
/**
 * Math helpers
 */
#define IS_ZERO(x) (fabs(x) < 1e-10)
#define IS_NAN(x) (x != x)
#define TO_RAD(x) (((x)*PI) / 180.0)
#define TO_DEG(x) (((x)*180.0) / PI)

/**
 * Bound to range
 */
template <typename T>
inline void boundToRange(T& value, const T& min, const T& max) {
    if (value < min)
        value = min;
    if (value > max)
        value = max;
}

/**
 * Math helpers
 */
double safeACos(const double value);
double safeASin(double value);
double getRotationAngle(const Eigen::QuaternionD& q, const Eigen::Vector3d& v);
Eigen::QuaternionD getRotationQuaternion(const double angle, const Eigen::Vector3d& axis);
std::pair<Eigen::Vector3d, Eigen::Vector3d> getOrthogonalVectors(const Eigen::Vector3d& vec);

/**
 * Random numbers
 */
double getRandomNumberInRange(std::pair<double, double> range);

/**
 * Rotation matrices
 */
Eigen::Matrix3d rotX(double angle);
Eigen::Matrix3d rotY(double angle);
Eigen::Matrix3d rotZ(double angle);

/**
 * Write matrix to file
 */
void writeMatrixToFile(const std::string& filePath, const Eigen::MatrixXd& matrix);

/**
 * Add triplets to list
 */
void addTripletDToList(Eigen::TripletDList& triplets, const int row, const int col, const double value);
void addTripletDToList_ignoreUpperElements(Eigen::TripletDList& triplets, const int row, const int col, const double value);
void addTripletDToList_mirrorElements(Eigen::TripletDList& triplets, const int row, const int col, const double value);

/**
 * Read & write helpers
 */
bool checkFileExtension(const std::string& filePath, const std::string& extension);
void createDirectory(const std::string& dir);
std::string getCurrentDateAndTime();
std::string browseFile();

}  // namespace lenny::tools::utils

/**
* Eigen to string
*/
namespace Eigen {

template <typename Type, int Rows, int Cols>
inline std::string to_string(const Eigen::Matrix<Type, Rows, Cols>& matrix) {
    std::stringstream ss;
    ss << matrix;
    return ss.str();
}

template <typename Type>
inline std::string to_string(const Eigen::SparseMatrix<Type>& matrix) {
    std::stringstream ss;
    ss << matrix;
    return ss.str();
}

template <typename Type>
inline std::string to_string(const Eigen::Quaternion<Type>& quaternion) {
    std::stringstream ss;
    ss << quaternion;
    return ss.str();
}

}  // namespace Eigen