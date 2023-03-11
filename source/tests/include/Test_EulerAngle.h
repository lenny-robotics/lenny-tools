#pragma once

#include <gtest/gtest.h>
#include <lenny/tools/EulerAngleRigidBody.h>
#include <lenny/tools/Utils.h>

TEST(tools, EulerAngle) {
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
    EXPECT_TRUE(trafo.position.isApprox(trafo_test.position));
    EXPECT_TRUE(trafo.orientation.matrix().isApprox(trafo_test.orientation.matrix()));

    //Test point computations
    const Eigen::Vector3d p_local = Eigen::Vector3d::Random();
    EXPECT_TRUE(rigidBody.testPointJacobian(state, p_local));
    EXPECT_TRUE(rigidBody.testPointTensor(state, p_local));

    //Test vector computations
    const Eigen::Vector3d v_local = Eigen::Vector3d::Random();
    EXPECT_TRUE(rigidBody.testVectorJacobian(state, v_local));
    EXPECT_TRUE(rigidBody.testVectorTensor(state, v_local));
}