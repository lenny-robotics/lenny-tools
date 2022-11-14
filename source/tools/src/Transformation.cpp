#include <lenny/tools/Transformation.h>

namespace lenny::tools {

Eigen::Vector3d Transformation::getGlobalCoordinatesForPoint(const Eigen::Vector3d& p_local) const {
    return position + orientation * p_local;
}

Eigen::Vector3d Transformation::getGlobalCoordinatesForVector(const Eigen::Vector3d& v_local) const {
    return orientation * v_local;
}

Eigen::QuaternionD Transformation::getGlobalCoordinates(const Eigen::QuaternionD& q_local) const {
    return orientation * q_local;
}

Eigen::Matrix3d Transformation::getGlobalCoordinates(const Eigen::Matrix3d& m_local) const {
    return orientation * m_local;
}

Transformation Transformation::getGlobalCoordinates(const Transformation& t_local) const {
    return *this * t_local;
}

Eigen::Vector3d Transformation::getLocalCoordinatesForPoint(const Eigen::Vector3d& p_global) const {
    return orientation.inverse() * (p_global - position);
}

Eigen::Vector3d Transformation::getLocalCoordinatesForVector(const Eigen::Vector3d& v_global) const {
    return orientation.inverse() * v_global;
}

Eigen::QuaternionD Transformation::getLocalCoordinates(const Eigen::QuaternionD& q_global) const {
    return orientation.inverse() * q_global;
}

Eigen::Matrix3d Transformation::getLocalCoordinates(const Eigen::Matrix3d& m_global) const {
    return orientation.inverse() * m_global;
}

Transformation Transformation::getLocalCoordinates(const Transformation& t_global) const {
    return this->inverse() * t_global;
}

Transformation Transformation::inverse() const {
    Transformation trafo;
    trafo.orientation = this->orientation.inverse();
    trafo.position = -trafo.orientation.matrix() * this->position;
    return trafo;
}

Transformation Transformation::operator*(const Transformation& other) const {
    Transformation trafo;
    trafo.orientation = this->orientation * other.orientation;
    trafo.position = this->orientation * other.position + this->position;
    return trafo;
}

bool Transformation::isApprox(const Transformation& other) const {
    return this->position.isApprox(other.position) && this->orientation.matrix().isApprox(other.orientation.matrix());
}

void Transformation::to_json(json& j, const Transformation& o) {
    TO_JSON(o, position);
    TO_JSON(o, orientation);
}

void Transformation::from_json(const json& j, Transformation& o) {
    FROM_JSON(o, position);
    FROM_JSON(o, orientation);
}

std::string Transformation::to_string(const Transformation& trafo) {
    std::stringstream ss;
    ss << trafo;
    return ss.str();
}

}  // namespace lenny::tools