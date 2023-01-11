#include <lenny/tools/Model.h>
#include <lenny/tools/Renderer.h>

namespace lenny::tools {

Renderer::UPtr Renderer::I = std::make_unique<Renderer>();

void Renderer::drawCuboid(const Eigen::Vector3d& COM, const Eigen::QuaternionD& orientation, const Eigen::Vector3d& dimensions,
                          const Eigen::Vector4d& color) const {
    static Model cube("gui/data/meshes/cube.obj");
    cube.draw(COM, orientation, dimensions, color.segment(0, 3), color[3]);
}

void Renderer::drawPlane(const Eigen::Vector3d& COM, const Eigen::QuaternionD& orientation, const Eigen::Vector2d& dimensions,
                         const Eigen::Vector4d& color) const {
    drawCuboid(COM, orientation, Eigen::Vector3d(dimensions[0], 1e-3, dimensions[1]), color);
}

void Renderer::drawSphere(const Eigen::Vector3d& position, const double& radius, const Eigen::Vector4d& color) const {
    static Model sphere("gui/data/meshes/sphere.obj");
    sphere.draw(position, Eigen::QuaternionD::Identity(), 2.0 * radius * Eigen::Vector3d::Ones(), color.segment(0, 3), color[3]);
}

void Renderer::drawEllipsoid(const Eigen::Vector3d& COM, const Eigen::QuaternionD& orientation, const Eigen::Vector3d& dimensions,
                             const Eigen::Vector4d& color) const {
    static Model sphere("gui/data/meshes/sphere.obj");
    sphere.draw(COM, orientation, 2.0 * dimensions, color.segment(0, 3), color[3]);
}

void Renderer::drawCylinder(const Eigen::Vector3d& startPosition, const Eigen::Vector3d& endPosition, const double& radius,
                            const Eigen::Vector4d& color) const {
    static Model cylinder("gui/data/meshes/cylinder.obj");

    Eigen::Vector3d dir = endPosition - startPosition;
    double s = dir.norm();
    if (s < 10e-10)
        return;
    Eigen::Vector3d a = dir.normalized();
    Eigen::Vector3d b = Eigen::Vector3d::UnitZ();
    Eigen::Vector3d v = (b.cross(a)).normalized();
    if (v.norm() < 0.5)
        v = Eigen::Vector3d::UnitX();
    double angle = acos(b.dot(a) / (b.norm() * a.norm()));

    cylinder.draw(startPosition, Eigen::QuaternionD(Eigen::AngleAxisd(angle, v)), Eigen::Vector3d(radius, radius, s), color.segment(0, 3), color[3]);
}

void Renderer::drawCylinder(const Eigen::Vector3d& COM, const Eigen::QuaternionD& orientation, const double& height, const double& radius,
                            const Eigen::Vector4d& color) const {
    Eigen::Vector3d startPosition = COM + orientation * Eigen::Vector3d(0.0, height / 2.0, 0.0);
    Eigen::Vector3d endPosition = COM - orientation * Eigen::Vector3d(0.0, height / 2.0, 0.0);
    drawCylinder(startPosition, endPosition, radius, color);
}

void Renderer::drawTetrahedron(const std::array<Eigen::Vector3d, 4>& globalPoints, const Eigen::Vector4d& color) const {
    //Not implemented here, since it does not use a model file
}

void Renderer::drawCone(const Eigen::Vector3d& origin, const Eigen::Vector3d& direction, const double& radius, const Eigen::Vector4d& color) const {
    static Model cone("gui/data/meshes/cone.obj");

    double s = direction.norm();
    if (s < 10e-10)
        return;
    Eigen::Vector3d a = direction.normalized();
    Eigen::Vector3d b = Eigen::Vector3d::UnitY();
    Eigen::Vector3d v = (b.cross(a)).normalized();
    if (v.norm() < 0.5)
        v = Eigen::Vector3d::UnitX();
    double angle = acos(b.dot(a) / (b.norm() * a.norm()));

    cone.draw(origin, Eigen::QuaternionD(Eigen::AngleAxisd(angle, v)), 1e-3 * Eigen::Vector3d(radius, s, radius), color.segment(0, 3), color[3]);
}

void Renderer::drawArrow(const Eigen::Vector3d& startPosition, const Eigen::Vector3d& direction, const double& radius, const Eigen::Vector4d& color) const {
    Eigen::Vector3d dir = direction;
    for (uint i = 0; i < 3; i++)
        if (fabs(dir[i]) < 1e-6)
            dir[i] = 1e-6;

    double coneRadius = 1.5 * radius;
    Eigen::Vector3d coneDir = dir / dir.norm() * coneRadius * 1.5;
    Eigen::Vector3d cyl_endPos = startPosition + dir - coneDir;

    drawCylinder(startPosition, cyl_endPos, radius, color);
    drawCone(cyl_endPos, coneDir, coneRadius, color);
}

void Renderer::drawCoordinateSystem(const Eigen::Vector3d& origin, const Eigen::QuaternionD& orientation, const double& length, const double& radius,
                                    double alpha) const {
    drawArrow(origin, length * orientation.matrix().col(0), radius, Eigen::Vector4d(0.75, 0.0, 0.0, alpha));
    drawArrow(origin, length * orientation.matrix().col(1), radius, Eigen::Vector4d(0.0, 0.75, 0.0, alpha));
    drawArrow(origin, length * orientation.matrix().col(2), radius, Eigen::Vector4d(0.0, 0.0, 0.75, alpha));
}

void Renderer::drawCapsule(const Eigen::Vector3d& startPosition, const Eigen::Vector3d& endPosition, const double& radius, const Eigen::Vector4d& color) const {
    drawCylinder(startPosition, endPosition, radius, color);
    drawSphere(startPosition, radius, color);
    drawSphere(endPosition, radius, color);
}

void Renderer::drawCapsule(const Eigen::Vector3d& COM, const Eigen::QuaternionD& orientation, const double& height, const double& radius,
                           const Eigen::Vector4d& color) const {
    Eigen::Vector3d startPosition = COM + orientation * Eigen::Vector3d(0.0, height / 2.0, 0.0);
    Eigen::Vector3d endPosition = COM - orientation * Eigen::Vector3d(0.0, height / 2.0, 0.0);
    drawCapsule(startPosition, endPosition, radius, color);
}

void Renderer::drawLine(const std::vector<Eigen::Vector3d>& linePoints, const double& radius, const Eigen::Vector4d& color) const {
    for (uint i = 0; i < linePoints.size() - 1; i++)
        drawCylinder(linePoints[i], linePoints[i + 1], radius, color);
}

void Renderer::drawTrajectory(const std::vector<Eigen::Vector3d>& trajectoryPoints, const double& radius, const Eigen::Vector4d& color,
                              const bool& showDots) const {
    drawLine(trajectoryPoints, radius, color);
    if (showDots) {
        const double dotRadius = 2.0 * radius;
        for (uint i = 0; i < trajectoryPoints.size(); i++)
            drawSphere(trajectoryPoints[i], dotRadius, color);
    }
}

void Renderer::drawSector(const Eigen::Vector3d& center, const Eigen::QuaternionD& orientation, const double& radius,
                          const std::pair<double, double>& angleRange, const Eigen::Vector4d& color) const {
    static Model sector("gui/data/meshes/sector.obj");
    for (double angle = angleRange.first; angle < angleRange.second; angle += PI / 180.0) {
        sector.draw(center, orientation * tools::utils::getRotationQuaternion(angle, Eigen::Vector3d::UnitY()), 1e-3 * radius * Eigen::Vector3d::Ones(),
                    color.segment(0, 3), color[3]);
    }
}

void Renderer::drawRoundedCuboid(const Eigen::Vector3d& COM, const Eigen::QuaternionD& orientation, const Eigen::Vector3d& dimensions, const double& radius,
                                 const Eigen::Vector4d& color) const {
    const Transformation trafo(COM, orientation);
    const Eigen::Vector3d P1 = trafo.getGlobalCoordinatesForPoint(0.5 * Eigen::Vector3d(dimensions[0], dimensions[1], dimensions[2]));
    const Eigen::Vector3d P2 = trafo.getGlobalCoordinatesForPoint(0.5 * Eigen::Vector3d(-dimensions[0], dimensions[1], dimensions[2]));
    const Eigen::Vector3d P3 = trafo.getGlobalCoordinatesForPoint(0.5 * Eigen::Vector3d(dimensions[0], -dimensions[1], dimensions[2]));
    const Eigen::Vector3d P4 = trafo.getGlobalCoordinatesForPoint(0.5 * Eigen::Vector3d(dimensions[0], dimensions[1], -dimensions[2]));
    const Eigen::Vector3d P5 = trafo.getGlobalCoordinatesForPoint(0.5 * Eigen::Vector3d(dimensions[0], -dimensions[1], -dimensions[2]));
    const Eigen::Vector3d P6 = trafo.getGlobalCoordinatesForPoint(0.5 * Eigen::Vector3d(-dimensions[0], dimensions[1], -dimensions[2]));
    const Eigen::Vector3d P7 = trafo.getGlobalCoordinatesForPoint(0.5 * Eigen::Vector3d(-dimensions[0], -dimensions[1], dimensions[2]));
    const Eigen::Vector3d P8 = trafo.getGlobalCoordinatesForPoint(0.5 * Eigen::Vector3d(-dimensions[0], -dimensions[1], -dimensions[2]));

    drawCapsule(P1, P2, radius, color);
    drawCapsule(P1, P3, radius, color);
    drawCapsule(P1, P4, radius, color);
    drawCapsule(P8, P5, radius, color);
    drawCapsule(P8, P6, radius, color);
    drawCapsule(P8, P7, radius, color);
    drawCapsule(P2, P6, radius, color);
    drawCapsule(P2, P7, radius, color);
    drawCapsule(P3, P5, radius, color);
    drawCapsule(P3, P7, radius, color);
    drawCapsule(P4, P5, radius, color);
    drawCapsule(P4, P6, radius, color);

    const Eigen::Vector3d drawDim = dimensions + 2.0 * radius * Eigen::Vector3d::Ones();
    drawPlane(trafo.getGlobalCoordinatesForPoint(0.5 * drawDim.cwiseProduct(Eigen::Vector3d::UnitX())),
              orientation * Eigen::QuaternionD(utils::rotZ(-PI / 2.0)), Eigen::Vector2d(dimensions[1], dimensions[2]), color);
    drawPlane(trafo.getGlobalCoordinatesForPoint(0.5 * drawDim.cwiseProduct(-Eigen::Vector3d::UnitX())),
              orientation * Eigen::QuaternionD(utils::rotZ(PI / 2.0)), Eigen::Vector2d(dimensions[1], dimensions[2]), color);
    drawPlane(trafo.getGlobalCoordinatesForPoint(0.5 * drawDim.cwiseProduct(Eigen::Vector3d::UnitY())), orientation * Eigen::QuaternionD(utils::rotX(0.0)),
              Eigen::Vector2d(dimensions[0], dimensions[2]), color);
    drawPlane(trafo.getGlobalCoordinatesForPoint(0.5 * drawDim.cwiseProduct(-Eigen::Vector3d::UnitY())), orientation * Eigen::QuaternionD(utils::rotX(PI)),
              Eigen::Vector2d(dimensions[0], dimensions[2]), color);
    drawPlane(trafo.getGlobalCoordinatesForPoint(0.5 * drawDim.cwiseProduct(Eigen::Vector3d::UnitZ())),
              orientation * Eigen::QuaternionD(utils::rotX(-PI / 2.0)), Eigen::Vector2d(dimensions[0], dimensions[1]), color);
    drawPlane(trafo.getGlobalCoordinatesForPoint(0.5 * drawDim.cwiseProduct(-Eigen::Vector3d::UnitZ())),
              orientation * Eigen::QuaternionD(utils::rotX(PI / 2.0)), Eigen::Vector2d(dimensions[0], dimensions[1]), color);
}

void Renderer::drawRoundedPlane(const Eigen::Vector3d& COM, const Eigen::QuaternionD& orientation, const Eigen::Vector2d& dimensions, const double& radius,
                                const Eigen::Vector4d& color) const {
    const Transformation trafo(COM, orientation);
    Eigen::Vector3d P1 = trafo.getGlobalCoordinatesForPoint(0.5 * Eigen::Vector3d(dimensions[0], 0.0, dimensions[1]));
    Eigen::Vector3d P2 = trafo.getGlobalCoordinatesForPoint(0.5 * Eigen::Vector3d(-dimensions[0], 0.0, dimensions[1]));
    Eigen::Vector3d P3 = trafo.getGlobalCoordinatesForPoint(0.5 * Eigen::Vector3d(dimensions[0], 0.0, -dimensions[1]));
    Eigen::Vector3d P4 = trafo.getGlobalCoordinatesForPoint(0.5 * Eigen::Vector3d(-dimensions[0], 0.0, -dimensions[1]));

    drawCapsule(P1, P2, radius, color);
    drawCapsule(P1, P3, radius, color);
    drawCapsule(P2, P4, radius, color);
    drawCapsule(P3, P4, radius, color);

    const Eigen::Vector2d drawDim = dimensions + 2.0 * radius * Eigen::Vector2d::Ones();
    drawPlane(trafo.getGlobalCoordinatesForPoint(Eigen::Vector3d(0.0, radius, 0.0)), orientation * Eigen::QuaternionD(utils::rotX(0.0)), dimensions, color);
    drawPlane(trafo.getGlobalCoordinatesForPoint(Eigen::Vector3d(0.0, -radius, 0.0)), orientation * Eigen::QuaternionD(utils::rotX(PI)), dimensions, color);
}

}  // namespace lenny::tools