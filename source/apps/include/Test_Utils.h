#pragma once

#include <gtest/gtest.h>
#include <lenny/tools/Logger.h>
#include <lenny/tools/Utils.h>

#include <iomanip>
#include <iostream>

TEST(tools, Utils) {
    using namespace lenny::tools::utils;

    { //boundToRange
        double value = 1.0;
        boundToRange(value, 0.0, 2.0);
        EXPECT_DOUBLE_EQ(value, 1.0);
        boundToRange(value, 1.5, 2.0);
        EXPECT_DOUBLE_EQ(value, 1.5);
        boundToRange(value, 0.0, 1.0);
        EXPECT_DOUBLE_EQ(value, 1.0);
    }

    { //getRandomNumberInRange
        std::cout << "TEST - getRandomNumberInRange" << std::endl;
        {
            std::map<double, double> hist;
            for (int n = 0; n < 100000; ++n) {
                ++hist[std::round(getRandomNumberInRange({-1.5, 3.5}))];
            }
            for (auto p : hist) {
                std::cout << std::fixed << std::setprecision(1) << std::setw(2) << p.first << ' ' << std::string(p.second / 200, '*') << '\n';
            }
        }
        std::cout << "-------------------------" << std::endl << std::endl;
    }

    { //getRotationAngle
        auto test = [](const double angle, const Eigen::Vector3d axis) -> void {
            const Eigen::QuaternionD q = getRotationQuaternion(angle, axis);
            const double angle_test = getRotationAngle(q, axis);
            EXPECT_LE(fabs(angle_test - angle), 1e-5);
        };

        for (uint i = 0; i < 10; i++)
            test(getRandomNumberInRange({0.0, 1.0}), Eigen::Vector3d::Random().normalized());
    }

    { //getOrthogonalVectors
        auto test = [](const Eigen::Vector3d vec) -> void {
            auto orthoVecs = getOrthogonalVectors(vec);
            EXPECT_LE(orthoVecs.first.dot(vec), 1e-5);
            EXPECT_LE(orthoVecs.second.dot(vec), 1e-5);
        };

        for (uint i = 0; i < 10; i++)
            test(Eigen::Vector3d::Random().normalized());
    }

    { //writeMatrixToFile
        writeMatrixToFile(LENNY_PROJECT_FOLDER "/logs/vec.txt", Eigen::VectorXd::Random(5));
        writeMatrixToFile(LENNY_PROJECT_FOLDER "/logs/mat.txt", Eigen::MatrixXd::Random(2, 3));
    }

    { //checkFileExtension
        EXPECT_TRUE(checkFileExtension("Test.txt", "txt"));
        EXPECT_FALSE(checkFileExtension("Test.bla", "txt"));
    }

    { //createDirectory
        createDirectory(LENNY_PROJECT_FOLDER "/logs/Test");
        createDirectory(LENNY_PROJECT_FOLDER "/logs/Test");
    }

    { //getCurrentDateAndTime
        std::cout << "TEST - getCurrentDateAndTime" << std::endl;
        std::cout << getCurrentDateAndTime() << std::endl;
        std::cout << "-------------------------" << std::endl << std::endl;
    }

//    { //browseFile
//        std::cout << "TEST - browseFile" << std::endl;
//        std::cout << browseFile() << std::endl;
//        std::cout << "-------------------------" << std::endl << std::endl;
//    }

    { //Eigen::to_string
        std::cout << "TEST - Eigen::to_string" << std::endl;
        Eigen::MatrixXd matrix = Eigen::MatrixXd::Random(3, 3);
        std::cout << matrix << std::endl;
        std::cout << "........................." << std::endl;
        std::cout << Eigen::to_string(matrix) << std::endl;
        std::cout << "........................." << std::endl;
        LENNY_LOG_INFO("Matrix:\n%s\n", Eigen::to_string(matrix).c_str());
        std::cout << "-------------------------" << std::endl << std::endl;

        Eigen::QuaternionD quaternion = Eigen::QuaternionD::UnitRandom();
        std::cout << quaternion << std::endl;
        std::cout << "........................." << std::endl;
        std::cout << Eigen::to_string(quaternion) << std::endl;
        std::cout << "........................." << std::endl;
        LENNY_LOG_INFO("Quaternion:\n%s\n", Eigen::to_string(quaternion).c_str());
        std::cout << "-------------------------" << std::endl << std::endl;
    }
}