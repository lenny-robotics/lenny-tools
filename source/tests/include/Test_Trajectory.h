#pragma once

#include <lenny/tools/Trajectory.h>

#include <iostream>

void test_trajectory() {
    {
        lenny::tools::Trajectory1d trajectory;
        trajectory.addEntry(2.0, -1.0);
        trajectory.addEntry(-2.0, 3.0);
        trajectory.addEntry(1.0, 1.0);
        trajectory.addEntry(-1.0, 1.0);

        std::cout << trajectory << std::endl;

        auto test = [&](const double& val, const double& testVal) -> void {
            if (fabs(val - testVal) < 1e-5)
                std::cout << "PASSED! -> " << val << " VS " << testVal << std::endl;
            else
                std::cout << "FAILED! -> " << val << " VS " << testVal << std::endl;
        };

        test(trajectory.getLinearInterpolation(-3.0), 3.0);
        test(trajectory.getLinearInterpolation(-1.5), 2.0);
        test(trajectory.getLinearInterpolation(0.0), 1.0);
        test(trajectory.getLinearInterpolation(1.0), 1.0);
        test(trajectory.getLinearInterpolation(2.0), -1.0);
        test(trajectory.getLinearInterpolation(3.0), -1.0);

        test(trajectory.getSplineInterpolation(-3.0), 3.0);
        test(trajectory.getSplineInterpolation(-1.5), 1.875);
        test(trajectory.getSplineInterpolation(0.0), 1.0);
        test(trajectory.getSplineInterpolation(0.5), 1.1875);
        test(trajectory.getSplineInterpolation(1.0), 1.0);
        test(trajectory.getSplineInterpolation(1.5), 0.125);
        test(trajectory.getSplineInterpolation(2.0), -1.0);
    }

    {
        lenny::tools::TrajectoryXd trajectory;
        trajectory.addEntry(2.0, -1.0 * Eigen::VectorXd::Ones(3));
        trajectory.addEntry(-2.0, 3.0 * Eigen::VectorXd::Ones(3));
        trajectory.addEntry(1.0, 1.0 * Eigen::VectorXd::Ones(3));
        trajectory.addEntry(-1.0, 1.0 * Eigen::VectorXd::Ones(3));

        std::cout << trajectory << std::endl;

        auto test_val = [&](const Eigen::VectorXd& val, const Eigen::VectorXd& testVal) -> void {
            if ((val - testVal).norm() < 1e-5)
                std::cout << "PASSED! ->" << val.transpose() << " VS " << testVal.transpose() << std::endl;
            else
                std::cout << "FAILED! -> " << val.transpose() << " VS " << testVal.transpose() << std::endl;
        };

        test_val(trajectory.getLinearInterpolation(-3.0), 3.0 * Eigen::VectorXd::Ones(3));
        test_val(trajectory.getLinearInterpolation(-1.5), 2.0 * Eigen::VectorXd::Ones(3));
        test_val(trajectory.getLinearInterpolation(0.0), 1.0 * Eigen::VectorXd::Ones(3));
        test_val(trajectory.getLinearInterpolation(1.0), 1.0 * Eigen::VectorXd::Ones(3));
        test_val(trajectory.getLinearInterpolation(2.0), -1.0 * Eigen::VectorXd::Ones(3));
        test_val(trajectory.getLinearInterpolation(3.0), -1.0 * Eigen::VectorXd::Ones(3));

        test_val(trajectory.getSplineInterpolation(-3.0), 3.0 * Eigen::VectorXd::Ones(3));
        test_val(trajectory.getSplineInterpolation(-1.5), 1.875 * Eigen::VectorXd::Ones(3));
        test_val(trajectory.getSplineInterpolation(0.0), 1.0 * Eigen::VectorXd::Ones(3));
        test_val(trajectory.getSplineInterpolation(0.5), 1.1875 * Eigen::VectorXd::Ones(3));
        test_val(trajectory.getSplineInterpolation(1.0), 1.0 * Eigen::VectorXd::Ones(3));
        test_val(trajectory.getSplineInterpolation(1.5), 0.125 * Eigen::VectorXd::Ones(3));
        test_val(trajectory.getSplineInterpolation(2.0), -1.0 * Eigen::VectorXd::Ones(3));
    }
}