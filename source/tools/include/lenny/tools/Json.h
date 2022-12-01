#pragma once

#include <lenny/tools/Logger.h>

#include <iomanip>
#include <nlohmann/json.hpp>

/**
 * Definition
 */
using json = nlohmann::json;

/**
 * Helpers
 */
std::string demangle(const std::string& name);
#define TYPE_ID(x) demangle(std::string(typeid(x).name()))

#define TO_JSON(e, x) j[TYPE_ID(e)][#x] = e.x;
#define FROM_JSON(e, x) e.x = j[TYPE_ID(e)].value(#x, decltype(e.x)());

/**
 * Eigen
 */
namespace Eigen {

//--- Matrix
template <typename _Scalar, int _Rows, int _Cols>
inline void to_json(json& js, const Matrix<_Scalar, _Rows, _Cols>& mat) {
    json data = json::array();
    for (int r = 0; r < mat.rows(); r++)
        for (int c = 0; c < mat.cols(); c++)
            data.emplace_back(mat(r, c));
    js["rows"] = mat.rows();
    js["cols"] = mat.cols();
    js["data"] = data;
}

template <typename _Scalar, int _Rows, int _Cols>
inline void from_json(const json& js, Matrix<_Scalar, _Rows, _Cols>& mat) {
    const json data = js["data"];
    const int rows = js["rows"];
    const int cols = js["cols"];
    mat.resize(rows, cols);
    int iter = 0;
    for (int r = 0; r < mat.rows(); r++)
        for (int c = 0; c < mat.cols(); c++)
            mat(r, c) = data[iter++];
}

//--- Quaternion
template <typename _Scalar>
inline void to_json(json& js, const Quaternion<_Scalar>& quat) {
    js["w"] = quat.w();
    js["x"] = quat.x();
    js["y"] = quat.y();
    js["z"] = quat.z();
}

template <typename _Scalar>
inline void from_json(const json& js, Quaternion<_Scalar>& quat) {
    quat.w() = js["w"];
    quat.x() = js["x"];
    quat.y() = js["y"];
    quat.z() = js["z"];
}

}  // namespace Eigen