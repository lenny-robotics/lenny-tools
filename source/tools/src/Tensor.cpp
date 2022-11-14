#include <lenny/tools/Logger.h>
#include <lenny/tools/Tensor.h>

#include <fstream>

namespace Eigen {

template <typename T>
Tensor<T>::Tensor() {
    dim.setZero();
    entries.clear();
    numEntries = 0;
}

template <typename T>
Tensor<T>::Tensor(const Vector3i& dim) {
    resize(dim);
}

template <typename T>
Vector3i Tensor<T>::getDimensions() const {
    return dim;
}

template <typename T>
T Tensor<T>::getEntry(const Vector3i& index) const {
    checkIndex(index);
    double value = 0.0;
    for (const Triplet<T>& triplet : entries[index[2]])
        if (triplet.col() == index[0] && triplet.row() == index[1])
            value += triplet.value();
    return value;
}

template <typename T>
void Tensor<T>::getEntryList(std::vector<std::pair<Eigen::Vector3i, T>>& entryList) const {
    entryList.clear();
    for (int i = 0; i < (int)entries.size(); i++)
        for (const Triplet<T>& entry : entries[i])
            entryList.push_back({Vector3i(entry.col(), entry.row(), i), entry.value()});
}

template <typename T>
int Tensor<T>::getNumberOfEntries() const {
    return numEntries;
}

template <typename T>
void Tensor<T>::addEntry(const Vector3i& index, const T& value) {
    if (IS_ZERO(value))
        return;

    checkIndex(index);
    entries[index[2]].push_back(Triplet<T>(index[1], index[0], value));
    numEntries++;
}

template <typename T>
void Tensor<T>::setZero() {
    entries = std::vector<std::vector<Triplet<T>>>(dim[2], std::vector<Triplet<T>>{});
    numEntries = 0;
}

template <typename T>
void Tensor<T>::resize(const Vector3i& dim) {
    this->dim = dim;
    setZero();
}

template <typename T>
void Tensor<T>::checkIndex(const Vector3i& index) const {
    if (index[0] >= dim[0] || index[1] >= dim[1] || index[2] >= dim[2] || index[0] < 0.0 || index[1] < 0.0 || index[2] < 0.0) {
        using lenny::tools::Logger;
        for (int i = 0; i < 3; i++)
            LENNY_LOG_PRINT(Logger::RED, "#%d: Index %d VS Dimension %d\n", i, index[i], dim[i]);
        LENNY_LOG_ERROR("Index exceeds dimensions!");
    }
}

template <typename T>
bool Tensor<T>::getMatrixForOuterIndex(SparseMatrix<T>& mat, uint outerIndex) const {
    if (outerIndex >= (uint)dim[2])
        LENNY_LOG_ERROR("OuterIndex is out of range!");

    mat.resize(dim[1], dim[0]);
    if (entries[outerIndex].size() == 0) {
        mat.setZero();
        return false;
    }
    mat.setFromTriplets(entries[outerIndex].begin(), entries[outerIndex].end());
    return true;
}

template <typename T>
void Tensor<T>::multiply(SparseMatrix<T>& mat, const Matrix<T, -1, 1>& vec) const {
    if (dim[0] != vec.size())
        LENNY_LOG_ERROR("Error in dimensions");

    mat.resize(dim[1], dim[2]);
    mat.setZero();
    const SparseVector<T> vec_sparse = vec.sparseView();
    SparseMatrix<T> mat_tmp;
    for (int i = 0; i < dim[2]; i++) {
        bool hasNonZeros = getMatrixForOuterIndex(mat_tmp, i);
        if (hasNonZeros)
            mat.col(i) = mat_tmp * vec_sparse;
    }
}

template <typename T>
T Tensor<T>::norm() const {
    double sum = 0.0;
    double count = 0.0;
    for (const std::vector<Triplet<T>>& entry : entries) {
        for (const Triplet<T>& triplet : entry) {
            sum += triplet.value() * triplet.value();
            count++;
        }
    }
    if (count > 0.0)
        sum /= count;
    return sum;
}

template <typename T>
void Tensor<T>::writeToFile(const std::string& filePath, const Tensor<T>& tensor) {
    std::ofstream file(filePath);
    if (file.is_open()) {
        const Vector3i dim = tensor.getDimensions();
        file << "Dimensions: " << dim.transpose() << std::endl;
        for (int i = 0; i < dim[2]; i++) {
            file << "Outer dim: " << i << std::endl;
            SparseMatrix<T> mat;
            tensor.getMatrixForOuterIndex(mat, i);
            file << mat.toDense() << std::endl;
        }
    } else {
        LENNY_LOG_WARNING("File could not be opened! Abort...");
    }
    file.close();
}

template <typename T>
std::string Tensor<T>::to_string(const Tensor<T>& tensor) {
    std::stringstream ss;
    ss << tensor;
    return ss.str();
}

template class Tensor<double>;

}  // namespace Eigen