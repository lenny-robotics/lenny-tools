#pragma once

#include <lenny/tools/Json.h>

namespace lenny::tools {

class Transformation {
public:
    Transformation() : position(Eigen::Vector3d::Zero()), orientation(Eigen::QuaternionD::Identity()) {}
    Transformation(const Eigen::Vector3d& position, const Eigen::QuaternionD& orientation) : position(position), orientation(orientation) {}
    Transformation(const Eigen::QuaternionD& orientation, const Eigen::Vector3d& position) : position(position), orientation(orientation) {}
    Transformation(const Eigen::Vector3d& position, const Eigen::Matrix3d& orientation) : position(position), orientation(orientation) {}
    Transformation(const Eigen::Matrix3d& orientation, const Eigen::Vector3d& position) : position(position), orientation(orientation) {}
    ~Transformation() = default;

    Eigen::Vector3d getGlobalCoordinatesForPoint(const Eigen::Vector3d& p_local) const;
    Eigen::Vector3d getGlobalCoordinatesForVector(const Eigen::Vector3d& v_local) const;
    Eigen::QuaternionD getGlobalCoordinates(const Eigen::QuaternionD& q_local) const;
    Eigen::Matrix3d getGlobalCoordinates(const Eigen::Matrix3d& m_local) const;
    Transformation getGlobalCoordinates(const Transformation& t_local) const;

    Eigen::Vector3d getLocalCoordinatesForPoint(const Eigen::Vector3d& p_global) const;
    Eigen::Vector3d getLocalCoordinatesForVector(const Eigen::Vector3d& v_global) const;
    Eigen::QuaternionD getLocalCoordinates(const Eigen::QuaternionD& q_global) const;
    Eigen::Matrix3d getLocalCoordinates(const Eigen::Matrix3d& m_global) const;
    Transformation getLocalCoordinates(const Transformation& t_global) const;

    Transformation inverse() const;
    Transformation operator*(const Transformation& other) const;

    bool isApprox(const Transformation& other) const;

    static void to_json(json& j, const Transformation& o);
    static void from_json(const json& j, Transformation& o);

    static std::string to_string(const Transformation& trafo);

public:
    Eigen::Vector3d position;
    Eigen::QuaternionD orientation;
};

inline std::ostream& operator<<(std::ostream& os, const Transformation& trafo) {
    os << "Position: (" << trafo.position.x() << ", " << trafo.position.y() << ", " << trafo.position.z() << ") / Orientation: (" << trafo.orientation.w()
       << ", " << trafo.orientation.x() << ", " << trafo.orientation.y() << ", " << trafo.orientation.z() << ")";
    return os;
}

}  // namespace lenny::tools