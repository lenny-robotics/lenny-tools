#include <lenny/tools/EulerAngleRigidBody.h>

namespace lenny::tools {

tools::FiniteDifference EulerAngleRigidBody::fd = tools::FiniteDifference("EulerAngleRigidBody");

EulerAngleRigidBody::EulerAngleRigidBody(const Eigen::Vector3d& alphaAxis, const Eigen::Vector3d& betaAxis, const Eigen::Vector3d& gammaAxis) {
    setRotationAxes(alphaAxis, betaAxis, gammaAxis);
}

Eigen::Vector3d EulerAngleRigidBody::computeGlobalPoint(const Eigen::Vector6d& state, const Eigen::Vector3d& p_local) const {
    return getPosition(state) + getTotalRotation(state) * p_local;
}

void EulerAngleRigidBody::computePointJacobian(Eigen::Matrix<double, 3, 6>& jacobian, const Eigen::Vector6d& state, const Eigen::Vector3d& p_local) const {
    //Set rotational derivatives
    computeVectorJacobian(jacobian, state, p_local);

    //Set positional derivatives
    jacobian.block(0, 0, 3, 3).setIdentity();
}

void EulerAngleRigidBody::computePointTensor(Eigen::TensorD& tensor, const Eigen::Vector6d& state, const Eigen::Vector3d& p_local) const {
    computeVectorTensor(tensor, state, p_local);
}

bool EulerAngleRigidBody::testPointJacobian(const Eigen::Vector6d& state, const Eigen::Vector3d& p_local) const {
    auto eval = [&](Eigen::VectorXd& point, const Eigen::VectorXd& state) -> void { point = computeGlobalPoint(state, p_local); };
    auto anal = [&](Eigen::MatrixXd& jacobian, const Eigen::VectorXd& state) -> void {
        Eigen::Matrix<double, 3, 6> jac;
        computePointJacobian(jac, state, p_local);
        jacobian = jac;
    };
    return fd.testMatrix(eval, anal, state, "Point Jacobian", 3, true);
}

bool EulerAngleRigidBody::testPointTensor(const Eigen::Vector6d& state, const Eigen::Vector3d& p_local) const {
    auto eval = [&](Eigen::MatrixXd& jacobian, const Eigen::VectorXd& state) -> void {
        Eigen::Matrix<double, 3, 6> jac;
        computePointJacobian(jac, state, p_local);
        jacobian = jac;
    };
    auto anal = [&](Eigen::TensorD& tensor, const Eigen::VectorXd& state) -> void { computePointTensor(tensor, state, p_local); };
    return fd.testTensor(eval, anal, state, "Point Tensor", 3, state.size());
}

Eigen::Vector3d EulerAngleRigidBody::computeGlobalVector(const Eigen::Vector6d& state, const Eigen::Vector3d& v_local) const {
    return getTotalRotation(state) * v_local;
}

void EulerAngleRigidBody::computeVectorJacobian(Eigen::Matrix<double, 3, 6>& jacobian, const Eigen::Vector6d& state, const Eigen::Vector3d& v_local) const {
    //Reset jacobian
    jacobian.resize(3, state.size());
    jacobian.setZero();

    //Set rotational derivatives
    const Eigen::QuaternionD q_a = getAlphaRotation(state);
    const Eigen::QuaternionD q_b = getBetaRotation(state);
    const Eigen::QuaternionD q_c = getGammaRotation(state);

    const Eigen::Vector3d pqa_pa = alphaAxis.cross(q_a * v_local);
    const Eigen::Vector3d pqb_pb = betaAxis.cross(q_b * q_a * v_local);
    const Eigen::Vector3d pqc_pc = gammaAxis.cross(q_c * q_b * q_a * v_local);

    jacobian.block(0, 3, 3, 1) = q_c * q_b * pqa_pa;
    jacobian.block(0, 4, 3, 1) = q_c * pqb_pb;
    jacobian.block(0, 5, 3, 1) = pqc_pc;
}

void EulerAngleRigidBody::computeVectorTensor(Eigen::TensorD& tensor, const Eigen::Vector6d& state, const Eigen::Vector3d& v_local) const {
    //Reset tensor
    tensor.resize(Eigen::Vector3i(3, state.size(), state.size()));
    tensor.setZero();

    //Set rotational derivatives
    const Eigen::QuaternionD q_a = getAlphaRotation(state);
    const Eigen::QuaternionD q_b = getBetaRotation(state);
    const Eigen::QuaternionD q_c = getGammaRotation(state);

    const Eigen::Vector3d pqa_pa = alphaAxis.cross(q_a * v_local);
    const Eigen::Vector3d pqb_pb = betaAxis.cross(q_b * q_a * v_local);
    const Eigen::Vector3d pqc_pc = gammaAxis.cross(q_c * q_b * q_a * v_local);

    const Eigen::Vector3d p2qa_p2a = q_c * q_b * alphaAxis.cross(pqa_pa);
    const Eigen::Vector3d p2qb_p2b = q_c * betaAxis.cross(pqb_pb);
    const Eigen::Vector3d p2qc_p2c = gammaAxis.cross(pqc_pc);
    const Eigen::Vector3d p2qb_pbpa = q_c * betaAxis.cross(q_b * pqa_pa);
    const Eigen::Vector3d p2qc_papc = gammaAxis.cross(q_c * q_b * pqa_pa);
    const Eigen::Vector3d p2qc_pbpc = gammaAxis.cross(q_c * pqb_pb);

    for (int i = 0; i < 3; i++) {
        tensor.addEntry(Eigen::Vector3i(i, 3, 3), p2qa_p2a[i]);
        tensor.addEntry(Eigen::Vector3i(i, 4, 4), p2qb_p2b[i]);
        tensor.addEntry(Eigen::Vector3i(i, 5, 5), p2qc_p2c[i]);

        tensor.addEntry(Eigen::Vector3i(i, 3, 4), p2qb_pbpa[i]);
        tensor.addEntry(Eigen::Vector3i(i, 4, 3), p2qb_pbpa[i]);

        tensor.addEntry(Eigen::Vector3i(i, 5, 3), p2qc_papc[i]);
        tensor.addEntry(Eigen::Vector3i(i, 3, 5), p2qc_papc[i]);

        tensor.addEntry(Eigen::Vector3i(i, 5, 4), p2qc_pbpc[i]);
        tensor.addEntry(Eigen::Vector3i(i, 4, 5), p2qc_pbpc[i]);
    }
}

bool EulerAngleRigidBody::testVectorJacobian(const Eigen::Vector6d& state, const Eigen::Vector3d& v_local) const {
    auto eval = [&](Eigen::VectorXd& vector, const Eigen::VectorXd& state) -> void { vector = computeGlobalVector(state, v_local); };
    auto anal = [&](Eigen::MatrixXd& jacobian, const Eigen::VectorXd& state) -> void {
        Eigen::Matrix<double, 3, 6> jac;
        computeVectorJacobian(jac, state, v_local);
        jacobian = jac;
    };
    return fd.testMatrix(eval, anal, state, "Vector Jacobian", 3, true);
}

bool EulerAngleRigidBody::testVectorTensor(const Eigen::Vector6d& state, const Eigen::Vector3d& v_local) const {
    auto eval = [&](Eigen::MatrixXd& jacobian, const Eigen::VectorXd& state) -> void {
        Eigen::Matrix<double, 3, 6> jac;
        computeVectorJacobian(jac, state, v_local);
        jacobian = jac;
    };
    auto anal = [&](Eigen::TensorD& tensor, const Eigen::Vector6d& state) -> void { computeVectorTensor(tensor, state, v_local); };
    return fd.testTensor(eval, anal, state, "Vector Tensor", 3, state.size());
}

tools::Transformation EulerAngleRigidBody::getTransformationFromState(const Eigen::Vector6d& state) const {
    return tools::Transformation(getPosition(state), getTotalRotation(state));
}

//Decompose the quaternion q as: q = R(c, gamma) * R(b, beta) * R(a, alpha)
inline void computeEulerAnglesFromQuaternion(double& alpha, double& beta, double& gamma, const Eigen::Vector3d& a, const Eigen::Vector3d& b,
                                             const Eigen::Vector3d& c, const Eigen::QuaternionD& q) {
    //Check inputs
    if (!IS_ZERO(a.dot(b)) || !IS_ZERO(b.dot(c)))
        LENNY_LOG_ERROR("Euler axis are not orthogonal!");

    if (!IS_ZERO(a.norm() - 1.0) || !IS_ZERO(b.norm() - 1.0) || !IS_ZERO(c.norm() - 1.0))
        LENNY_LOG_ERROR("Euler axis are not unit vectors!");

    //Do computations
    const Eigen::Vector3d aRot = q * a;
    if (IS_ZERO(a.dot(c))) {
        //The three axes form an orthonormal basis (i.e. Tait-Bryan)... singularity around beta = -PI/2 or PI/2
        const bool circular = a.cross(b).dot(c) > 0.0;
        if (circular) {
            beta = -tools::utils::safeASin(aRot.dot(c));
            gamma = atan2(aRot.dot(b), aRot.dot(a));
        } else {
            beta = tools::utils::safeASin(aRot.dot(c));
            gamma = atan2(-aRot.dot(b), aRot.dot(a));
        }
    } else if (IS_ZERO(a.dot(c) - 1.0)) {
        //These are "proper" euler axes, where the first and the last one are the same... singularity around beta = 0 or PI
        const Eigen::Vector3d lastAxis = a.cross(b);
        beta = tools::utils::safeACos(aRot.dot(a));
        gamma = atan2(aRot.dot(b), -aRot.dot(lastAxis));
    } else {
        //Something went wrong here
        LENNY_LOG_ERROR("Something is completely off with the inputs");
    }
    const Eigen::QuaternionD qLeft = tools::utils::getRotationQuaternion(-beta, b) * tools::utils::getRotationQuaternion(-gamma, c) * q;
    alpha = tools::utils::getRotationAngle(qLeft, a);

    //Convert back for final check
    const Eigen::QuaternionD q_check =
        tools::utils::getRotationQuaternion(gamma, c) * tools::utils::getRotationQuaternion(beta, b) * tools::utils::getRotationQuaternion(alpha, a);
    if (!IS_ZERO((q_check * q.inverse()).vec().norm() / 10e5))
        LENNY_LOG_ERROR("Final conversion check not passed!")
}

Eigen::Vector6d EulerAngleRigidBody::getStateFromTransformation(const tools::Transformation& transformation) const {
    Eigen::Vector6d state;
    state.segment(0, 3) = transformation.position;
    computeEulerAnglesFromQuaternion(state[3], state[4], state[5], alphaAxis, betaAxis, gammaAxis, transformation.orientation);
    return state;
}

void EulerAngleRigidBody::setRotationAxes(const Eigen::Vector3d& alphaAxis, const Eigen::Vector3d& betaAxis, const Eigen::Vector3d& gammaAxis) {
    this->alphaAxis = alphaAxis.normalized();
    this->betaAxis = betaAxis.normalized();
    this->gammaAxis = gammaAxis.normalized();
}

void EulerAngleRigidBody::getRotationAxes(Eigen::Vector3d& alphaAxis, Eigen::Vector3d& betaAxis, Eigen::Vector3d& gammaAxis) const {
    alphaAxis = this->alphaAxis;
    betaAxis = this->betaAxis;
    gammaAxis = this->gammaAxis;
}

Eigen::Vector3d EulerAngleRigidBody::getPosition(const Eigen::Vector6d& state) const {
    return state.segment(0, 3);
}

Eigen::QuaternionD EulerAngleRigidBody::getAlphaRotation(const Eigen::Vector6d& state) const {
    return tools::utils::getRotationQuaternion(state[3], alphaAxis);
}

Eigen::QuaternionD EulerAngleRigidBody::getBetaRotation(const Eigen::Vector6d& state) const {
    return tools::utils::getRotationQuaternion(state[4], betaAxis);
}

Eigen::QuaternionD EulerAngleRigidBody::getGammaRotation(const Eigen::Vector6d& state) const {
    return tools::utils::getRotationQuaternion(state[5], gammaAxis);
}

Eigen::QuaternionD EulerAngleRigidBody::getTotalRotation(const Eigen::Vector6d& state) const {
    return getGammaRotation(state) * getBetaRotation(state) * getAlphaRotation(state);
}

}  // namespace lenny::tools
