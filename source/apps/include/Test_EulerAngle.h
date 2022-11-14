#pragma once

#include <lenny/tools/EulerAngleRigidBody.h>
#include <lenny/tools/Utils.h>

#include <iostream>

void test_eulerangle() {
    using namespace lenny;

    //Set rigid body and state
    tools::EulerAngleRigidBody rigidBody;
    Eigen::VectorXd state(6);
    for (int i = 0; i < 6; i++)
        state[i] = tools::utils::getRandomNumberInRange({-PI, PI});

    //Test transformation
    const tools::Transformation trafo = rigidBody.getTransformationFromState(state);
    const Eigen::VectorXd state_test = rigidBody.getStateFromTransformation(trafo);
    const tools::Transformation trafo_test = rigidBody.getTransformationFromState(state_test);
    if (trafo.position.isApprox(trafo_test.position) && trafo.orientation.matrix().isApprox(trafo_test.orientation.matrix())) {
        std::cout << "Transformation conversion test PASSED" << std::endl;
    } else {
        std::cout << "Transformation conversion test FAILED" << std::endl;
    }

    //Test point computations
    Eigen::Vector3d p_local = Eigen::Vector3d::Random();
    std::cout << rigidBody.computeGlobalPoint(state, p_local).transpose() << std::endl;
    rigidBody.testPointJacobian(state, p_local);
    rigidBody.testPointTensor(state, p_local);

    Eigen::Vector3d v_local = Eigen::Vector3d::Random();
    std::cout << rigidBody.computeGlobalVector(state, v_local).transpose() << std::endl;
    rigidBody.testVectorJacobian(state, v_local);
    rigidBody.testVectorTensor(state, v_local);
}