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

    //--- Constructor
    FiniteDifference(const std::string& description) : description(description) {}
    virtual ~FiniteDifference() = default;

    //--- Estimators
    virtual void estimateVector(Eigen::VectorXd& vec, const Eigen::VectorXd& x, const F_Val& eval) const;
    virtual void estimateMatrix(Eigen::TripletDList& tripL, const Eigen::VectorXd& x, const F_Vec& eval, const uint& firstDim, bool fullMatrix) const;
    virtual void estimateMatrix(Eigen::SparseMatrixD& sMat, const Eigen::VectorXd& x, const F_Vec& eval, const uint& firstDim, bool fullMatrix) const;
    virtual void estimateMatrix(Eigen::MatrixXd& dMat, const Eigen::VectorXd& x, const F_Vec& eval, const uint& firstDim, bool fullMatrix) const;
    virtual void estimateTensor(Eigen::TensorD& ten, const Eigen::VectorXd& x, const F_TripL& eval, const uint& firstDim, const uint& secondDim) const;
    virtual void estimateTensor(Eigen::TensorD& ten, const Eigen::VectorXd& x, const F_SMat& eval, const uint& firstDim, const uint& secondDim) const;
    virtual void estimateTensor(Eigen::TensorD& ten, const Eigen::VectorXd& x, const F_DMat& eval, const uint& firstDim, const uint& secondDim) const;

    //--- Testers
    virtual bool testVector(const F_Val& eval, const F_Vec& ana, const Eigen::VectorXd& x, const std::string& name) const;
    virtual bool testMatrix(const F_Vec& eval, const F_TripL& ana, const Eigen::VectorXd& x, const std::string& name, const uint& firstDim,
                            const bool& fullMatrix) const;
    virtual bool testMatrix(const F_Vec& eval, const F_SMat& ana, const Eigen::VectorXd& x, const std::string& name, const uint& firstDim,
                            const bool& fullMatrix) const;
    virtual bool testMatrix(const F_Vec& eval, const F_DMat& ana, const Eigen::VectorXd& x, const std::string& name, const uint& firstDim,
                            const bool& fullMatrix) const;
    virtual bool testTensor(const F_TripL& eval, const F_Ten& ana, const Eigen::VectorXd& x, const std::string& name, const uint& firstDim,
                            const uint& secondDim) const;
    virtual bool testTensor(const F_SMat& eval, const F_Ten& ana, const Eigen::VectorXd& x, const std::string& name, const uint& firstDim,
                            const uint& secondDim) const;
    virtual bool testTensor(const F_DMat& eval, const F_Ten& ana, const Eigen::VectorXd& x, const std::string& name, const uint& firstDim,
                            const uint& secondDim) const;

    //--- Helpers
    virtual void setFDCheckIsBeingApplied(bool isBeingApplied) const;
    virtual void preFDEvaluation(const Eigen::VectorXd& x) const {}

    //--- Printers
    bool performFDCheck(const Eigen::VectorXd& estimate, const Eigen::VectorXd& analytic, const std::string& name) const;
    bool performFDCheck(const Eigen::MatrixXd& estimate, const Eigen::MatrixXd& analytic, const std::string& name) const;
    bool performFDCheck(const Eigen::TensorD& estimate, const Eigen::TensorD& analytic, const std::string& name) const;

public:
    std::string description;  //Set by constructor
    bool printTestMatricesToFile = false;

    //--- FD tolerances
    double deltaFD = 1e-6;
    double relTolFD = 1e-4;
    double absTolFD = 1e-6;
    double epsFD = 1e-10;
    bool printFDCheck = true;

protected:
    mutable bool fdCheckIsBeingApplied = false;
};

}  // namespace lenny::tools