#pragma once

#include <lenny/tools/Definitions.h>

namespace Eigen {

template <typename T>
class Tensor {
public:
    //--- Constructor
    Tensor();
    Tensor(const Vector3i& dim);
    ~Tensor() = default;

    //--- Getters
    Vector3i getDimensions() const;

    //This is pretty inefficient and should only be used for debugging
    T getEntry(const Vector3i& index) const;
    void getEntryList(std::vector<std::pair<Vector3i, T>>& entryList) const;
    int getNumberOfEntries() const;

    //--- Setters
    void addEntry(const Vector3i& index, const T& value);
    void setZero();
    void resize(const Vector3i& dim);

    //--- Math
    void multiply(SparseMatrix<T>& mat, const Matrix<T, -1, 1>& vec) const;

    //Norm here is the squared average of all entries
    T norm() const;

    //--- Helpers
    void checkIndex(const Vector3i& index) const;
    //Returns true if there are non-zero entries
    bool getMatrixForOuterIndex(SparseMatrix<T>& mat, uint outerIndex) const;
    static void writeToFile(const std::string& filePath, const Tensor<T>& tensor);
    static std::string to_string(const Tensor<T>& tensor);

private:
    /*
        dim[0]: col of triplet
        dim[1]: row of triplet
        dim[2]: outer size of std::vector of entries
    */
    Vector3i dim = Vector3i::Zero();
    std::vector<std::vector<Triplet<T>>> entries;
    int numEntries = 0;
};

//--- Ostream
template <typename T>
std::ostream& operator<<(std::ostream& os, const Tensor<T>& tensor) {
    const Vector3i dim = tensor.getDimensions();
    os << "--- Start of tensor ---" << std::endl;
    os << "--- Dimensions: " << dim[0] << " x " << dim[1] << " x " << dim[2] << std::endl;
    for (int i = 0; i < dim[2]; i++) {
        os << "--- Outer dim: " << i << std::endl;
        SparseMatrix<T> mat;
        tensor.getMatrixForOuterIndex(mat, i);
        os << mat.toDense() << std::endl;
    }
    os << "--- End of tensor ---";
    return os;
}

//Typedef
typedef Tensor<double> TensorD;

}  // namespace Eigen
