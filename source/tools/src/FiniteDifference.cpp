#include <lenny/tools/FiniteDifference.h>
#include <lenny/tools/Logger.h>
#include <lenny/tools/Utils.h>

namespace lenny::tools {

void FiniteDifference::estimateVector(Eigen::VectorXd& vec, const Eigen::VectorXd& x, const F_Val& eval) const {
    Eigen::VectorXd x_tmp(x);
    const uint size = (uint)x_tmp.size();
    vec.resize(size);
    vec.setZero();

    double C_P, C_M;
    for (uint i = 0; i < size; i++) {
        double tmpVal = x_tmp(i);
        x_tmp(i) = tmpVal + delta;
        if (f_PreEval)
            f_PreEval(x_tmp);
        C_P = eval(x_tmp);

        x_tmp(i) = tmpVal - delta;
        if (f_PreEval)
            f_PreEval(x_tmp);
        C_M = eval(x_tmp);

        x_tmp(i) = tmpVal;
        vec(i) += (C_P - C_M) / (2.0 * delta);
    }
}

void FiniteDifference::estimateMatrix(Eigen::TripletDList& tripL, const Eigen::VectorXd& x, const F_Vec& eval, const uint& firstDim, bool fullMatrix) const {
    Eigen::VectorXd x_tmp(x);
    const uint size = (uint)x_tmp.size();
    tripL.clear();

    Eigen::VectorXd C_P(firstDim), C_M(firstDim), H_i_col(firstDim);
    for (uint i = 0; i < size; i++) {
        double tmpVal = x_tmp(i);
        x_tmp(i) = tmpVal + delta;
        if (f_PreEval)
            f_PreEval(x_tmp);
        C_P.setZero();
        eval(C_P, x_tmp);

        x_tmp(i) = tmpVal - delta;
        if (f_PreEval)
            f_PreEval(x_tmp);
        C_M.setZero();
        eval(C_M, x_tmp);

        x_tmp(i) = tmpVal;
        H_i_col = (C_P - C_M) / (2.0 * delta);

        uint j = i;
        if (fullMatrix || (size != firstDim))
            j = 0;
        for (; j < firstDim; j++) {
            utils::addTripletDToList(tripL, j, i, H_i_col(j));
        }
    }
}

void FiniteDifference::estimateMatrix(Eigen::SparseMatrixD& sMat, const Eigen::VectorXd& x, const F_Vec& eval, const uint& firstDim, bool fullMatrix) const {
    Eigen::TripletDList tripL;
    estimateMatrix(tripL, x, eval, firstDim, fullMatrix);
    sMat.resize(firstDim, x.size());
    sMat.setFromTriplets(tripL.begin(), tripL.end());
}

void FiniteDifference::estimateMatrix(Eigen::MatrixXd& dMat, const Eigen::VectorXd& x, const F_Vec& eval, const uint& firstDim, bool fullMatrix) const {
    Eigen::SparseMatrixD sMat(firstDim, x.size());
    estimateMatrix(sMat, x, eval, firstDim, fullMatrix);
    dMat = sMat.toDense();
}

void FiniteDifference::estimateTensor(Eigen::TensorD& ten, const Eigen::VectorXd& x, const F_TripL& eval, const uint& firstDim, const uint& secondDim) const {
    auto eval_sMat = [&](Eigen::SparseMatrixD& sMat, const Eigen::VectorXd& x) -> void {
        Eigen::TripletDList tripL;
        eval(tripL, x);
        sMat.resize(firstDim, secondDim);
        sMat.setFromTriplets(tripL.begin(), tripL.end());
    };
    estimateTensor(ten, x, eval_sMat, firstDim, secondDim);
}

void FiniteDifference::estimateTensor(Eigen::TensorD& ten, const Eigen::VectorXd& x, const F_SMat& eval, const uint& firstDim, const uint& secondDim) const {
    Eigen::VectorXd x_tmp(x);
    ten.resize(Eigen::Vector3i(firstDim, secondDim, x_tmp.size()));

    Eigen::SparseMatrixD C_P(firstDim, secondDim), C_M(firstDim, secondDim), H_i(firstDim, secondDim);
    for (uint i = 0; i < x_tmp.size(); i++) {
        double tmpVal = x_tmp(i);
        x_tmp(i) = tmpVal + delta;
        if (f_PreEval)
            f_PreEval(x_tmp);
        C_P.setZero();
        eval(C_P, x_tmp);

        x_tmp(i) = tmpVal - delta;
        if (f_PreEval)
            f_PreEval(x_tmp);
        C_M.setZero();
        eval(C_M, x_tmp);

        x_tmp(i) = tmpVal;
        H_i = (C_P - C_M) / (2.0 * delta);

        for (int k = 0; k < H_i.outerSize(); ++k)
            for (Eigen::SparseMatrixD::InnerIterator it(H_i, k); it; ++it)
                ten.addEntry(Eigen::Vector3i((int)it.row(), (int)it.col(), i), it.value());
    }
}

void FiniteDifference::estimateTensor(Eigen::TensorD& ten, const Eigen::VectorXd& x, const F_DMat& eval, const uint& firstDim, const uint& secondDim) const {
    auto eval_sMat = [&](Eigen::SparseMatrixD& sMat, const Eigen::VectorXd& x) -> void {
        Eigen::MatrixXd dMat = Eigen::MatrixXd::Zero(firstDim, secondDim);
        eval(dMat, x);
        sMat = dMat.sparseView();
    };
    estimateTensor(ten, x, eval_sMat, firstDim, secondDim);
}

bool FiniteDifference::testVector(const F_Val& eval, const F_Vec& ana, const Eigen::VectorXd& x, const std::string& name) const {
    Eigen::VectorXd estimate = Eigen::VectorXd::Zero(x.size());
    Eigen::VectorXd analytic = Eigen::VectorXd::Zero(x.size());

    estimateVector(estimate, x, eval);
    if (f_PreEval)
        f_PreEval(x);
    ana(analytic, x);

    return performCheck(estimate, analytic, name);
}

bool FiniteDifference::testMatrix(const F_Vec& eval, const F_TripL& ana, const Eigen::VectorXd& x, const std::string& name, const uint& firstDim,
                                  const bool& fullMatrix) const {
    const uint secDim = (uint)x.size();

    Eigen::SparseMatrixD estimate(firstDim, secDim);
    Eigen::SparseMatrixD analytic(firstDim, secDim);
    Eigen::TripletDList tripL_tmp;

    tripL_tmp.clear();
    estimateMatrix(tripL_tmp, x, eval, firstDim, fullMatrix);
    estimate.setFromTriplets(tripL_tmp.begin(), tripL_tmp.end());

    tripL_tmp.clear();
    if (f_PreEval)
        f_PreEval(x);
    ana(tripL_tmp, x);
    analytic.setFromTriplets(tripL_tmp.begin(), tripL_tmp.end());

    return performCheck(estimate.toDense(), analytic.toDense(), name);
}

bool FiniteDifference::testMatrix(const F_Vec& eval, const F_SMat& ana, const Eigen::VectorXd& x, const std::string& name, const uint& firstDim,
                                  const bool& fullMatrix) const {
    const uint secDim = (uint)x.size();

    Eigen::SparseMatrixD estimate(firstDim, secDim);
    Eigen::SparseMatrixD analytic(firstDim, secDim);

    estimate.setZero();
    estimateMatrix(estimate, x, eval, firstDim, fullMatrix);

    analytic.setZero();
    if (f_PreEval)
        f_PreEval(x);
    ana(analytic, x);

    return performCheck(estimate.toDense(), analytic.toDense(), name);
}

bool FiniteDifference::testMatrix(const F_Vec& eval, const F_DMat& ana, const Eigen::VectorXd& x, const std::string& name, const uint& firstDim,
                                  const bool& fullMatrix) const {
    const uint secDim = (uint)x.size();

    Eigen::MatrixXd estimate = Eigen::MatrixXd::Zero(firstDim, secDim);
    Eigen::MatrixXd analytic = Eigen::MatrixXd::Zero(firstDim, secDim);

    estimateMatrix(estimate, x, eval, firstDim, fullMatrix);
    if (f_PreEval)
        f_PreEval(x);
    ana(analytic, x);

    return performCheck(estimate, analytic, name);
}

bool FiniteDifference::testTensor(const F_TripL& eval, const F_Ten& ana, const Eigen::VectorXd& x, const std::string& name, const uint& firstDim,
                                  const uint& secondDim) const {
    auto eval_sMat = [&](Eigen::SparseMatrixD& sMat, const Eigen::VectorXd& x) -> void {
        Eigen::TripletDList tripL;
        eval(tripL, x);
        sMat.resize(firstDim, secondDim);
        sMat.setFromTriplets(tripL.begin(), tripL.end());
    };
    return testTensor(eval_sMat, ana, x, name, firstDim, secondDim);
}

bool FiniteDifference::testTensor(const F_SMat& eval, const F_Ten& ana, const Eigen::VectorXd& x, const std::string& name, const uint& firstDim,
                                  const uint& secondDim) const {
    const uint thirdDim = (uint)x.size();

    Eigen::TensorD estimate(Eigen::Vector3i(firstDim, secondDim, thirdDim));
    Eigen::TensorD analytic(Eigen::Vector3i(firstDim, secondDim, thirdDim));

    estimateTensor(estimate, x, eval, firstDim, secondDim);
    if (f_PreEval)
        f_PreEval(x);
    ana(analytic, x);

    return performCheck(estimate, analytic, name);
}

bool FiniteDifference::testTensor(const F_DMat& eval, const F_Ten& ana, const Eigen::VectorXd& x, const std::string& name, const uint& firstDim,
                                  const uint& secondDim) const {
    auto eval_sMat = [&](Eigen::SparseMatrixD& sMat, const Eigen::VectorXd& x) -> void {
        Eigen::MatrixXd dMat = Eigen::MatrixXd::Zero(firstDim, secondDim);
        eval(dMat, x);
        sMat = dMat.sparseView();
    };
    return testTensor(eval_sMat, ana, x, name, firstDim, secondDim);
}

inline void printStart(const std::string& name, const std::string& description, const double& analyticNorm, const double& estimateNorm) {
    LENNY_LOG_PRINT(Logger::BLUE, "-----------------------------------------------------------------------------------------------------------\n");
    LENNY_LOG_PRINT(Logger::DEFAULT, "Test ");
    LENNY_LOG_PRINT(Logger::CYAN, "%s", name.c_str());
    LENNY_LOG_PRINT(Logger::DEFAULT, " of ");
    LENNY_LOG_PRINT(Logger::YELLOW, "%s", description.c_str());
    LENNY_LOG_PRINT(Logger::DEFAULT, " with FD -> norms: Ana: %lf, FD: %lf\n", analyticNorm, estimateNorm);
}

inline void printEnd(const bool& checkSuccessful, const std::string& description) {
    if (checkSuccessful)
        LENNY_LOG_PRINT(Logger::GREEN, "   PASSED\n");
    LENNY_LOG_PRINT(Logger::DEFAULT, "End of check of %s\n", description.c_str());
    LENNY_LOG_PRINT(Logger::BLUE, "-----------------------------------------------------------------------------------------------------------\n");
}

bool FiniteDifference::performCheck(const Eigen::VectorXd& estimate, const Eigen::VectorXd& analytic, const std::string& name) const {
    using tools::Logger;

    if (printMatricesToFile) {
        const std::string estimatePath = LENNY_PROJECT_FOLDER "/logs/Est_" + name + "_" + description + ".m";
        utils::writeMatrixToFile(estimatePath, estimate);

        const std::string analyticPath = LENNY_PROJECT_FOLDER "/logs/Ana_" + name + "_" + description + ".m";
        utils::writeMatrixToFile(analyticPath, analytic);
    }

    if (printCheck)
        printStart(name, description, analytic.norm(), estimate.norm());

    bool checkSuccessful = true;
    for (int i = 0; i < estimate.size(); i++) {
        const double absErr = std::abs(estimate[i] - analytic[i]);
        const double relError = 2.0 * absErr / (eps + std::abs(analytic[i]) + std::abs(estimate[i]));
        if (relError > relTol && absErr > absTol) {
            checkSuccessful = false;
            if (printCheck) {
                LENNY_LOG_PRINT(Logger::RED, "   MISMATCH");
                LENNY_LOG_PRINT(Logger::DEFAULT, " -> Element: %d, Ana val: %lf, Est val: %lf, Error: %lf\n", i, analytic[i], estimate[i], absErr);
            }
        }
    }

    if (printCheck)
        printEnd(checkSuccessful, description);

    return checkSuccessful;
}

bool FiniteDifference::performCheck(const Eigen::MatrixXd& estimate, const Eigen::MatrixXd& analytic, const std::string& name) const {
    using tools::Logger;

    if (printMatricesToFile) {
        const std::string estimatePath = LENNY_PROJECT_FOLDER "/logs/Est_" + name + "_" + description + ".m";
        utils::writeMatrixToFile(estimatePath, estimate);

        const std::string analyticPath = LENNY_PROJECT_FOLDER "/logs/Ana_" + name + "_" + description + ".m";
        utils::writeMatrixToFile(analyticPath, analytic);
    }

    if (printCheck) {
        const double ana_norm = (analytic.size() > 0 && estimate.size() > 0) ? analytic.norm() : 0.0;
        const double est_norm = (analytic.size() > 0 && estimate.size() > 0) ? estimate.norm() : 0.0;
        printStart(name, description, ana_norm, est_norm);
    }

    bool checkSuccessful = true;
    for (int i = 0; i < estimate.rows(); i++) {
        for (int j = 0; j < estimate.cols(); j++) {
            const double absErr = std::abs(estimate.coeff(i, j) - analytic.coeff(i, j));
            const double relError = 2.0 * absErr / (eps + std::abs(estimate.coeff(i, j)) + std::abs(analytic.coeff(i, j)));
            if (relError > relTol && absErr > absTol) {
                checkSuccessful = false;
                if (printCheck) {
                    LENNY_LOG_PRINT(Logger::RED, "   MISMATCH");
                    LENNY_LOG_PRINT(Logger::DEFAULT, " -> Element: (%d, %d), Ana val: %lf, FD val: %lf. Error: %lf\n", i, j, analytic.coeff(i, j),
                                    estimate.coeff(i, j), absErr);
                }
            }
        }
    }

    if (printCheck)
        printEnd(checkSuccessful, description);

    return checkSuccessful;
}

bool FiniteDifference::performCheck(const Eigen::TensorD& estimate, const Eigen::TensorD& analytic, const std::string& name) const {
    using tools::Logger;

    if (printMatricesToFile) {
        const std::string estimatePath = LENNY_PROJECT_FOLDER "/logs/Est_" + name + "_" + description + ".m";
        Eigen::TensorD::writeToFile(estimatePath, estimate);

        const std::string analyticPath = LENNY_PROJECT_FOLDER "/logs/Ana_" + name + "_" + description + ".m";
        Eigen::TensorD::writeToFile(analyticPath, analytic);
    }

    if (printCheck)
        printStart(name, description, analytic.norm(), estimate.norm());

    bool checkSuccessful = true;
    const Eigen::Vector3i dim = estimate.getDimensions();
    for (int i = 0; i < dim[0]; i++) {
        for (int j = 0; j < dim[1]; j++) {
            for (int k = 0; k < dim[2]; k++) {
                const double fdValue = estimate.getEntry(Eigen::Vector3i(i, j, k));
                const double anaValue = analytic.getEntry(Eigen::Vector3i(i, j, k));
                const double absErr = std::abs(fdValue - anaValue);
                const double relError = 2.0 * absErr / (eps + std::abs(fdValue) + std::abs(anaValue));
                if (relError > relTol && absErr > absTol) {
                    checkSuccessful = false;
                    if (printCheck) {
                        LENNY_LOG_PRINT(Logger::RED, "   MISMATCH");
                        LENNY_LOG_PRINT(Logger::DEFAULT, " -> Element: (%d, %d, %d), Ana val: %lf, FD val: %lf. Error: %lf\n", i, j, k, anaValue, fdValue,
                                        absErr);
                    }
                }
            }
        }
    }

    if (printCheck)
        printEnd(checkSuccessful, description);

    return checkSuccessful;
}

}  // namespace lenny::tools