#pragma once

#include <gtest/gtest.h>
#include <lenny/tools/Logger.h>
#include <lenny/tools/Transformation.h>

#include <iostream>

TEST(tools, Transformation) {
    using namespace lenny::tools;

    Transformation origin;

    const Eigen::Vector3d p_local = Eigen::Vector3d::Random();
    const Eigen::Vector3d v_local = Eigen::Vector3d::Random();
    const Eigen::QuaternionD q_local = Eigen::QuaternionD::UnitRandom();
    const Eigen::Matrix3d m_local = Eigen::Matrix3d::Random();
    const Transformation t_local(Eigen::Vector3d::Random(), Eigen::QuaternionD::UnitRandom());

    const Eigen::Vector3d p_global = origin.getGlobalCoordinatesForPoint(p_local);
    const Eigen::Vector3d v_global = origin.getGlobalCoordinatesForVector(v_local);
    const Eigen::QuaternionD q_global = origin.getGlobalCoordinates(q_local);
    const Eigen::Matrix3d m_global = origin.getGlobalCoordinates(m_local);
    const Transformation t_global = origin.getGlobalCoordinates(t_local);

    const Transformation t_local_test = origin.getLocalCoordinates(t_global);

    EXPECT_TRUE(p_global.isApprox(p_local));
    EXPECT_TRUE(v_global.isApprox(v_local));
    EXPECT_TRUE(q_global.isApprox(q_local));
    EXPECT_TRUE(m_global.isApprox(m_local));
    EXPECT_TRUE(t_global.isApprox(t_local));

    EXPECT_TRUE(origin.getLocalCoordinatesForPoint(p_global).isApprox(p_local));
    EXPECT_TRUE(origin.getLocalCoordinatesForVector(v_global).isApprox(v_local));
    EXPECT_TRUE(origin.getLocalCoordinates(q_global).isApprox(q_local));
    EXPECT_TRUE(origin.getLocalCoordinates(m_global).isApprox(m_local));
    EXPECT_TRUE(t_local.isApprox(t_local_test));

    std::cout << t_local << std::endl;
    LENNY_LOG_INFO("Transformation: %s\n", Transformation::to_string(t_local).c_str());

    json js;
    Transformation::to_json(js, t_local);
    std::cout << js << std::endl;
    Transformation t_local_test2;
    Transformation::from_json(js, t_local_test2);
    EXPECT_TRUE(t_local_test.isApprox(t_local_test2));
}