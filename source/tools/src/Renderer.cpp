#include <lenny/tools/Renderer.h>

namespace lenny::tools {

Renderer::UPtr Renderer::I = std::make_unique<Renderer>();

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