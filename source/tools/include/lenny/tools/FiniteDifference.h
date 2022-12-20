#pragma once

#include <lenny/tools/Tensor.h>

#include <functional>

namespace lenny::tools {

class FiniteDifference {
public:
    //--- Typedefs
    typedef std::function<double(const Eigen::VectorXd& x)> F_Val;
    typedef std::function<void(Eigen::VectorXd& vec, const Eigen::VectorXd& x)> F_Vec;
    typedef std::function<void(Eigen::TripletDList& tripL, const Eigen::VectorXd& x)> F_TripL;
    typedef std::function<void(Eigen::SparseMatrixD& sMat, const Eigen::VectorXd& x)> F_SMat;
    typedef std::function<void(Eigen::MatrixXd& dMat, const Eigen::VectorXd& x)> F_DMat;
    typedef std::function<void(Eigen::TensorD& ten, const Eigen::VectorXd& x)> F_Ten;
    typedef std::function<void(const Eigen::VectorXd& x)> F_PreEval;

    //--- Constructor
    FiniteDifference(const std::string& description) : description(description) {}
    ~FiniteDifference() = default;

    //--- Estimators
    void estimateVector(Eigen::VectorXd& vec, const Eigen::VectorXd& x, const F_Val& eval) const;
    void estimateMatrix(Eigen::TripletDList& tripL, const Eigen::VectorXd& x, const F_Vec& eval, const uint& firstDim, bool fullMatrix) const;
    void estimateMatrix(Eigen::SparseMatrixD& sMat, const Eigen::VectorXd& x, const F_Vec& eval, const uint& firstDim, bool fullMatrix) const;
    void estimateMatrix(Eigen::MatrixXd& dMat, const Eigen::VectorXd& x, const F_Vec& eval, const uint& firstDim, bool fullMatrix) const;
    void estimateTensor(Eigen::TensorD& ten, const Eigen::VectorXd& x, const F_TripL& eval, const uint& firstDim, const uint& secondDim) const;
    void estimateTensor(Eigen::TensorD& ten, const Eigen::VectorXd& x, const F_SMat& eval, const uint& firstDim, const uint& secondDim) const;
    void estimateTensor(Eigen::TensorD& ten, const Eigen::VectorXd& x, const F_DMat& eval, const uint& firstDim, const uint& secondDim) const;

    //--- Testers
    bool testVector(const F_Val& eval, const F_Vec& ana, const Eigen::VectorXd& x, const std::string& name) const;
    bool testMatrix(const F_Vec& eval, const F_TripL& ana, const Eigen::VectorXd& x, const std::string& name, const uint& firstDim,
                    const bool& fullMatrix) const;
    bool testMatrix(const F_Vec& eval, const F_SMat& ana, const Eigen::VectorXd& x, const std::string& name, const uint& firstDim,
                    const bool& fullMatrix) const;
    bool testMatrix(const F_Vec& eval, const F_DMat& ana, const Eigen::VectorXd& x, const std::string& name, const uint& firstDim,
                    const bool& fullMatrix) const;
    bool testTensor(const F_TripL& eval, const F_Ten& ana, const Eigen::VectorXd& x, const std::string& name, const uint& firstDim,
                    const uint& secondDim) const;
    bool testTensor(const F_SMat& eval, const F_Ten& ana, const Eigen::VectorXd& x, const std::string& name, const uint& firstDim, const uint& secondDim) const;
    bool testTensor(const F_DMat& eval, const F_Ten& ana, const Eigen::VectorXd& x, const std::string& name, const uint& firstDim, const uint& secondDim) const;

    //--- Printers
    bool performCheck(const Eigen::VectorXd& estimate, const Eigen::VectorXd& analytic, const std::string& name) const;
    bool performCheck(const Eigen::MatrixXd& estimate, const Eigen::MatrixXd& analytic, const std::string& name) const;
    bool performCheck(const Eigen::TensorD& estimate, const Eigen::TensorD& analytic, const std::string& name) const;

public:
    //--- Members
    std::string description;  //Set by constructor
    F_PreEval f_PreEval = nullptr;
    bool printMatricesToFile = false;
    bool printCheck = true;

    //--- Tolerances
    double delta = 1e-6;
    double relTol = 1e-4;
    double absTol = 1e-6;
    double eps = 1e-10;
};

}  // namespace lenny::tools