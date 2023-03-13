#pragma once

#include <gtest/gtest.h>
#include <lenny/tools/Trajectory.h>

#include <iostream>

TEST(tools, Trajectory) {
    {
        lenny::tools::Trajectory1d trajectory;
        trajectory.addEntry(2.0, -1.0);
        trajectory.addEntry(-2.0, 3.0);
        trajectory.addEntry(1.0, 1.0);
        trajectory.addEntry(-1.0, 1.0);

        std::cout << trajectory << std::endl;

        EXPECT_DOUBLE_EQ(trajectory.getLinearInterpolation(-3.0), 3.0);
        EXPECT_DOUBLE_EQ(trajectory.getLinearInterpolation(-1.5), 2.0);
        EXPECT_DOUBLE_EQ(trajectory.getLinearInterpolation(0.0), 1.0);
        EXPECT_DOUBLE_EQ(trajectory.getLinearInterpolation(1.0), 1.0);
        EXPECT_DOUBLE_EQ(trajectory.getLinearInterpolation(2.0), -1.0);
        EXPECT_DOUBLE_EQ(trajectory.getLinearInterpolation(3.0), -1.0);

        EXPECT_DOUBLE_EQ(trajectory.getSplineInterpolation(-3.0), 3.0);
        EXPECT_DOUBLE_EQ(trajectory.getSplineInterpolation(-1.5), 1.875);
        EXPECT_DOUBLE_EQ(trajectory.getSplineInterpolation(0.0), 1.0);
        EXPECT_DOUBLE_EQ(trajectory.getSplineInterpolation(0.5), 1.1875);
        EXPECT_DOUBLE_EQ(trajectory.getSplineInterpolation(1.0), 1.0);
        EXPECT_DOUBLE_EQ(trajectory.getSplineInterpolation(1.5), 0.125);
        EXPECT_DOUBLE_EQ(trajectory.getSplineInterpolation(2.0), -1.0);
    }

    {
        lenny::tools::TrajectoryXd trajectory;
        trajectory.addEntry(2.0, -1.0 * Eigen::VectorXd::Ones(3));
        trajectory.addEntry(-2.0, 3.0 * Eigen::VectorXd::Ones(3));
        trajectory.addEntry(1.0, 1.0 * Eigen::VectorXd::Ones(3));
        trajectory.addEntry(-1.0, 1.0 * Eigen::VectorXd::Ones(3));

        std::cout << trajectory << std::endl;

        EXPECT_EQ(trajectory.getLinearInterpolation(-3.0), 3.0 * Eigen::VectorXd::Ones(3));
        EXPECT_EQ(trajectory.getLinearInterpolation(-1.5), 2.0 * Eigen::VectorXd::Ones(3));
        EXPECT_EQ(trajectory.getLinearInterpolation(0.0), 1.0 * Eigen::VectorXd::Ones(3));
        EXPECT_EQ(trajectory.getLinearInterpolation(1.0), 1.0 * Eigen::VectorXd::Ones(3));
        EXPECT_EQ(trajectory.getLinearInterpolation(2.0), -1.0 * Eigen::VectorXd::Ones(3));
        EXPECT_EQ(trajectory.getLinearInterpolation(3.0), -1.0 * Eigen::VectorXd::Ones(3));

        EXPECT_EQ(trajectory.getSplineInterpolation(-3.0), 3.0 * Eigen::VectorXd::Ones(3));
        EXPECT_EQ(trajectory.getSplineInterpolation(-1.5), 1.875 * Eigen::VectorXd::Ones(3));
        EXPECT_EQ(trajectory.getSplineInterpolation(0.0), 1.0 * Eigen::VectorXd::Ones(3));
        EXPECT_EQ(trajectory.getSplineInterpolation(0.5), 1.1875 * Eigen::VectorXd::Ones(3));
        EXPECT_EQ(trajectory.getSplineInterpolation(1.0), 1.0 * Eigen::VectorXd::Ones(3));
        EXPECT_EQ(trajectory.getSplineInterpolation(1.5), 0.125 * Eigen::VectorXd::Ones(3));
        EXPECT_EQ(trajectory.getSplineInterpolation(2.0), -1.0 * Eigen::VectorXd::Ones(3));
    }
}