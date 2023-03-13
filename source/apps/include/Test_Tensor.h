#pragma once

#include <gtest/gtest.h>
#include <lenny/tools/Logger.h>
#include <lenny/tools/Tensor.h>

#include <iostream>

TEST(tools, Tensor) {
    Eigen::TensorD tensor(Eigen::Vector3i(4, 3, 2));
    tensor.addEntry(Eigen::Vector3i(0, 0, 0), 85.0);
    tensor.addEntry(Eigen::Vector3i(1, 0, 0), 15.0);
    tensor.addEntry(Eigen::Vector3i(0, 1, 0), 63.0);
    tensor.addEntry(Eigen::Vector3i(0, 0, 1), 14.0);
    std::cout << "Dimensions: " << tensor.getDimensions().transpose() << std::endl;

    EXPECT_DOUBLE_EQ(tensor.getEntry(Eigen::Vector3i(0, 0, 0)), 85.0);
    EXPECT_DOUBLE_EQ(tensor.getEntry(Eigen::Vector3i(1, 0, 0)), 15.0);
    EXPECT_DOUBLE_EQ(tensor.getEntry(Eigen::Vector3i(0, 1, 0)), 63.0);
    EXPECT_DOUBLE_EQ(tensor.getEntry(Eigen::Vector3i(0, 0, 1)), 14.0);

    std::vector<std::pair<Eigen::Vector3i, double>> entryList;
    tensor.getEntryList(entryList);
    for (const auto& [index, value] : entryList)
        std::cout << index.transpose() << ": " << value << std::endl;

    std::cout << "Number of entries: " << tensor.getNumberOfEntries() << std::endl;
    std::cout << "Norm: " << tensor.norm() << std::endl;
    std::cout << tensor << std::endl;
    LENNY_LOG_INFO("\n%s\n", Eigen::TensorD::to_string(tensor).c_str());

    Eigen::TensorD::writeToFile(LENNY_PROJECT_FOLDER "/logs/Tensor.txt", tensor);

    Eigen::SparseMatrixD mat;
    tensor.multiply(mat, Eigen::Vector4d::Ones());
    std::cout << mat.toDense() << std::endl;
}