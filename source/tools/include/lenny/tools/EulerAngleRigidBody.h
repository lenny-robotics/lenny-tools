#pragma once

#include <lenny/tools/FiniteDifference.h>
#include <lenny/tools/Transformation.h>
#include <lenny/tools/Typedefs.h>

/**
 * COMMENTS:
 * We have an alpha-beta-gamma representation, i.e. we first rotate by alpha, then by beta, and finally by gamma
 * This means that the rotation matrix is computed by: R = R_gamma * R_beta * R_alpha
 * The state is of size 6, where: x = state[0], y = state[1], z = state[2], alpha = state[3], beta = state[4], gamma = state[5]
 */

namespace lenny::tools {

class EulerAngleRigidBody {
public:
    //--- Typedefs
    LENNY_GENERAGE_TYPEDEFS(EulerAngleRigidBody)

    //--- Constructor
    EulerAngleRigidBody(const Eigen::Vector3d& alphaAxis = Eigen::Vector3d::UnitZ(), const Eigen::Vector3d& betaAxis = Eigen::Vector3d::UnitX(),
                        const Eigen::Vector3d& gammaAxis = Eigen::Vector3d::UnitY());
    ~EulerAngleRigidBody() = default;

    //--- Point computations
    Eigen::Vector3d computeGlobalPoint(const Eigen::Vector6d& state, const Eigen::Vector3d& p_local) const;
    void computePointJacobian(Eigen::Matrix<double, 3, 6>& jacobian, const Eigen::Vector6d& state, const Eigen::Vector3d& p_local) const;
    void computePointTensor(Eigen::TensorD& tensor, const Eigen::Vector6d& state, const Eigen::Vector3d& p_local) const;

    bool testPointJacobian(const Eigen::Vector6d& state, const Eigen::Vector3d& p_local) const;
    bool testPointTensor(const Eigen::Vector6d& state, const Eigen::Vector3d& p_local) const;

    //--- Vector computations
    Eigen::Vector3d computeGlobalVector(const Eigen::Vector6d& state, const Eigen::Vector3d& v_local) const;
    void computeVectorJacobian(Eigen::Matrix<double, 3, 6>& jacobian, const Eigen::Vector6d& state, const Eigen::Vector3d& v_local) const;
    void computeVectorTensor(Eigen::TensorD& tensor, const Eigen::Vector6d& state, const Eigen::Vector3d& v_local) const;

    bool testVectorJacobian(const Eigen::Vector6d& state, const Eigen::Vector3d& v_local) const;
    bool testVectorTensor(const Eigen::Vector6d& state, const Eigen::Vector3d& v_local) const;

    //--- Helpers
    tools::Transformation getTransformationFromState(const Eigen::Vector6d& state) const;
    Eigen::Vector6d getStateFromTransformation(const tools::Transformation& transformation) const;

    void setRotationAxes(const Eigen::Vector3d& alphaAxis, const Eigen::Vector3d& betaAxis, const Eigen::Vector3d& gammaAxis);
    void getRotationAxes(Eigen::Vector3d& alphaAxis, Eigen::Vector3d& betaAxis, Eigen::Vector3d& gammaAxis) const;

public:
    static constexpr uint STATE_SIZE = 6;

private:
    static FiniteDifference fd;

private:
    //--- Helpers
    Eigen::Vector3d getPosition(const Eigen::Vector6d& state) const;
    Eigen::QuaternionD getAlphaRotation(const Eigen::Vector6d& state) const;
    Eigen::QuaternionD getBetaRotation(const Eigen::Vector6d& state) const;
    Eigen::QuaternionD getGammaRotation(const Eigen::Vector6d& state) const;
    Eigen::QuaternionD getTotalRotation(const Eigen::Vector6d& state) const;

private:
    //--- Rotation axes
    Eigen::Vector3d alphaAxis;  //Initialized by constructor
    Eigen::Vector3d betaAxis;   //Initialized by constructor
    Eigen::Vector3d gammaAxis;  //Initialized by constructor
};

}  // namespace lenny::tools