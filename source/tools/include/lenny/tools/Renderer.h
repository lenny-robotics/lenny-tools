#pragma once

#include <lenny/tools/Transformation.h>

#include <memory>

namespace lenny::tools {

class Renderer {
public:
    //--- Pointer
    typedef std::unique_ptr<Renderer> UPtr;

    //--- Constructor
    Renderer() = default;
    virtual ~Renderer() = default;

    //--- Draw functions
    virtual void drawCuboid(const Eigen::Vector3d& COM, const Eigen::QuaternionD& orientation, const Eigen::Vector3d& dimensions,
                            const Eigen::Vector4d& color) const {}
    virtual void drawPlane(const Eigen::Vector3d& COM, const Eigen::QuaternionD& orientation, const Eigen::Vector2d& dimensions,
                           const Eigen::Vector4d& color) const {}
    virtual void drawSphere(const Eigen::Vector3d& position, const double& radius, const Eigen::Vector4d& color) const {}
    virtual void drawEllipsoid(const Eigen::Vector3d& COM, const Eigen::QuaternionD& orientation, const Eigen::Vector3d& dimensions,
                               const Eigen::Vector4d& color) const {}
    virtual void drawCylinder(const Eigen::Vector3d& startPosition, const Eigen::Vector3d& endPosition, const double& radius,
                              const Eigen::Vector4d& color) const {}
    virtual void drawCylinder(const Eigen::Vector3d& COM, const Eigen::QuaternionD& orientation, const double& height, const double& radius,
                              const Eigen::Vector4d& color) const {}
    virtual void drawCone(const Eigen::Vector3d& origin, const Eigen::Vector3d& direction, const double& radius, const Eigen::Vector4d& color) const {}
    virtual void drawArrow(const Eigen::Vector3d& startPosition, const Eigen::Vector3d& direction, const double& radius, const Eigen::Vector4d& color) const {}
    virtual void drawCoordinateSystem(const Eigen::Vector3d& origin, const Eigen::QuaternionD& orientation, const double& length, const double& radius,
                                      double alpha = 1.0) const {}
    virtual void drawCapsule(const Eigen::Vector3d& startPosition, const Eigen::Vector3d& endPosition, const double& radius,
                             const Eigen::Vector4d& color) const {}
    virtual void drawCapsule(const Eigen::Vector3d& COM, const Eigen::QuaternionD& orientation, const double& height, const double& radius,
                             const Eigen::Vector4d& color) const {}
    virtual void drawTetrahedron(const std::array<Eigen::Vector3d, 4>& globalPoints, const Eigen::Vector4d& color) const {}
    virtual void drawLine(const std::vector<Eigen::Vector3d>& linePoints, const double& radius, const Eigen::Vector4d& color) const {}
    virtual void drawTrajectory(const std::vector<Eigen::Vector3d>& trajectoryPoints, const double& radius, const Eigen::Vector4d& color,
                                const bool& showDots) const {}
    virtual void drawSector(const Eigen::Vector3d& center, const Eigen::QuaternionD& orientation, const double& radius,
                            const std::pair<double, double>& angleRange, const Eigen::Vector4d& color) const {}
    virtual void drawRoundedCuboid(const Eigen::Vector3d& COM, const Eigen::QuaternionD& orientation, const Eigen::Vector3d& dimensions, const double& radius,
                                   const Eigen::Vector4d& color) const {}
    virtual void drawRoundedPlane(const Eigen::Vector3d& COM, const Eigen::QuaternionD& orientation, const Eigen::Vector2d& dimensions, const double& radius,
                                  const Eigen::Vector4d& color) const {}

public:
    //--- Static member
    static UPtr I;
};

}  // namespace lenny::tools