#pragma once

#include <lenny/tools/FiniteDifference.h>
#include <lenny/tools/Utils.h>

class FDTest : public lenny::tools::FiniteDifference {
public:
    FDTest() : lenny::tools::FiniteDifference("FDTest") {
        printTestMatricesToFile = true;
    }
    ~FDTest() = default;

    double computeValue(const Eigen::VectorXd& x) const {
        double value = 0.0;
        for (uint i = 0; i < 3; i++)
            for (uint j = 0; j < 3; j++)
                for (uint k = 0; k < 3; k++)
                    value += x[i] * x[j] * x[k];
        return value;
    }

    void computeFirstDerivative(Eigen::VectorXd& dvdx, const Eigen::VectorXd& x) const {
        dvdx.resize(3);
        dvdx.setZero();
        for (uint i = 0; i < 3; i++)
            for (uint j = 0; j < 3; j++)
                for (uint k = 0; k < 3; k++)
                    dvdx[i] += 3.0 * x[j] * x[k];
    }

    void computeSecondDerivative(Eigen::TripletDList& d2vdx2, const Eigen::VectorXd& x) const {
        d2vdx2.clear();
        for (uint i = 0; i < 3; i++)
            for (uint j = 0; j < 3; j++)
                for (uint k = 0; k < 3; k++)
                    lenny::tools::utils::addTripletDToList(d2vdx2, i, j, 6.0 * x[k]);
    }

    void computeSecondDerivative(Eigen::SparseMatrixD& dv2dx2, const Eigen::VectorXd& x) const {
        Eigen::TripletDList entries;
        computeSecondDerivative(entries, x);
        dv2dx2.resize(x.size(), x.size());
        dv2dx2.setFromTriplets(entries.begin(), entries.end());
    }

    void computeSecondDerivative(Eigen::MatrixXd& dv2dx2, const Eigen::VectorXd& x) const {
        Eigen::SparseMatrixD matrix;
        computeSecondDerivative(matrix, x);
        dv2dx2 = matrix.toDense();
    }

    void computeThirdDerivative(Eigen::TensorD& d3vdx3, const Eigen::VectorXd& x) const {
        d3vdx3.resize(Eigen::Vector3i(3, 3, 3));
        for (uint i = 0; i < 3; i++)
            for (uint j = 0; j < 3; j++)
                for (uint k = 0; k < 3; k++)
                    d3vdx3.addEntry(Eigen::Vector3i(i, j, k), 6.0);
    }

    void testFirstDerivativeWithFD(const Eigen::VectorXd& x) const {
        auto eval = [&](const Eigen::VectorXd& x) -> double { return computeValue(x); };
        auto ana = [&](Eigen::VectorXd& dvdx, const Eigen::VectorXd& x) -> void { computeFirstDerivative(dvdx, x); };
        testVector(eval, ana, x, "dvdx");
    }

    void testSecondDerivativeWithFD_TripletList(const Eigen::VectorXd& x) const {
        auto eval = [&](Eigen::VectorXd& dvdx, const Eigen::VectorXd& x) -> void { computeFirstDerivative(dvdx, x); };
        auto ana = [&](Eigen::TripletDList& d2vdx2, const Eigen::VectorXd& x) -> void { computeSecondDerivative(d2vdx2, x); };
        testMatrix(eval, ana, x, "d2vdx2_tripL", x.size(), true);
    }

    void testSecondDerivativeWithFD_SparseMatrix(const Eigen::VectorXd& x) const {
        auto eval = [&](Eigen::VectorXd& dvdx, const Eigen::VectorXd& x) -> void { computeFirstDerivative(dvdx, x); };
        auto ana = [&](Eigen::SparseMatrixD& d2vdx2, const Eigen::VectorXd& x) -> void { computeSecondDerivative(d2vdx2, x); };
        testMatrix(eval, ana, x, "d2vdx2_sMat", x.size(), true);
    }

    void testSecondDerivativeWithFD_DenseMatrix(const Eigen::VectorXd& x) const {
        auto eval = [&](Eigen::VectorXd& dvdx, const Eigen::VectorXd& x) -> void { computeFirstDerivative(dvdx, x); };
        auto ana = [&](Eigen::MatrixXd& d2vdx2, const Eigen::VectorXd& x) -> void { computeSecondDerivative(d2vdx2, x); };
        testMatrix(eval, ana, x, "d2vdx2_dMat", x.size(), true);
    }

    void testThirdDerivativeWithFD_TripletList(const Eigen::VectorXd& x) const {
        auto eval = [&](Eigen::TripletDList& d2vdx2, const Eigen::VectorXd& x) -> void { computeSecondDerivative(d2vdx2, x); };
        auto ana = [&](Eigen::TensorD& d3vdx3, const Eigen::VectorXd& x) -> void { computeThirdDerivative(d3vdx3, x); };
        testTensor(eval, ana, x, "d3vdx3_tripL", x.size(), x.size());
    }

    void testThirdDerivativeWithFD_SparseMatrix(const Eigen::VectorXd& x) const {
        auto eval = [&](Eigen::SparseMatrixD& d2vdx2, const Eigen::VectorXd& x) -> void { computeSecondDerivative(d2vdx2, x); };
        auto ana = [&](Eigen::TensorD& d3vdx3, const Eigen::VectorXd& x) -> void { computeThirdDerivative(d3vdx3, x); };
        testTensor(eval, ana, x, "d3vdx3_sMat", x.size(), x.size());
    }

    void testThirdDerivativeWithFD_DenseMatrix(const Eigen::VectorXd& x) const {
        auto eval = [&](Eigen::MatrixXd& d2vdx2, const Eigen::VectorXd& x) -> void { computeSecondDerivative(d2vdx2, x); };
        auto ana = [&](Eigen::TensorD& d3vdx3, const Eigen::VectorXd& x) -> void { computeThirdDerivative(d3vdx3, x); };
        testTensor(eval, ana, x, "d3vdx3_dMat", x.size(), x.size());
    }
};

void test_finitedifference() {
    Eigen::VectorXd x = Eigen::VectorXd::Random(3);

    FDTest test;
    test.testFirstDerivativeWithFD(x);
    test.testSecondDerivativeWithFD_TripletList(x);
    test.testSecondDerivativeWithFD_SparseMatrix(x);
    test.testSecondDerivativeWithFD_DenseMatrix(x);
    test.testThirdDerivativeWithFD_TripletList(x);
    test.testThirdDerivativeWithFD_SparseMatrix(x);
    test.testThirdDerivativeWithFD_DenseMatrix(x);
}