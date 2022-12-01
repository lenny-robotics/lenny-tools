#include <lenny/tools/Logger.h>
#include <lenny/tools/Utils.h>
#include <sys/stat.h>

#include <ctime>
#include <fstream>

#if WIN32
#include <windows.h>
#endif

namespace lenny::tools::utils {

double getRandomNumberInRange(std::pair<double, double> range) {
    return range.first + (rand() / (RAND_MAX / (range.second - range.first)));
}

double safeACos(const double value) {
    if (value < -1.0)
        return PI;
    else if (value > 1.0)
        return 0.0;
    return acos(value);
}

double safeASin(double value) {
    boundToRange(value, -1.0, 1.0);
    return asin(value);
}

double getRotationAngle(const Eigen::QuaternionD& q, const Eigen::Vector3d& v) {
    const double dotProd = q.vec().dot(v);
    const double sign = (dotProd < 0.0) ? -1.0 : 1.0;
    double result = sign * 2.0 * safeACos(q.w());
    if (result > PI)
        result -= 2.0 * PI;
    if (result < -PI)
        result += 2.0 * PI;
    return result;
}

Eigen::QuaternionD getRotationQuaternion(const double angle, const Eigen::Vector3d& axis) {
    return Eigen::QuaternionD(Eigen::AngleAxisd(angle, axis));
}

std::pair<Eigen::Vector3d, Eigen::Vector3d> getOrthogonalVectors(const Eigen::Vector3d& vec) {
    const double x_check = vec.x() * vec.x() / vec.squaredNorm();
    const Eigen::Vector3d vec_tmp = (x_check < 0.5) ? Eigen::Vector3d::UnitX() : Eigen::Vector3d::UnitY();
    const Eigen::Vector3d b = vec.cross(vec_tmp).normalized();
    const Eigen::Vector3d a = b.cross(vec).normalized();
    return {a, b};
}

Eigen::Matrix3d rotX(double angle) {
    return Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitX()).matrix();
}

Eigen::Matrix3d rotY(double angle) {
    return Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitY()).matrix();
}

Eigen::Matrix3d rotZ(double angle) {
    return Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()).matrix();
}

void writeMatrixToFile(const std::string& filePath, const Eigen::MatrixXd& matrix) {
    std::ofstream file(filePath);
    if (file.is_open()) {
        file << matrix;
    } else {
        LENNY_LOG_WARNING("File could not be opened! Abort...");
    }
    file.close();
}

void addTripletDToList(Eigen::TripletDList& triplets, const int row, const int col, const double value) {
    if (!IS_ZERO(value))
        triplets.emplace_back(row, col, value);
}

void addTripletDToList_ignoreUpperElements(Eigen::TripletDList& triplets, const int row, const int col, const double value) {
    if (row >= col)
        addTripletDToList(triplets, row, col, value);
}

void addTripletDToList_mirrorElements(Eigen::TripletDList& triplets, const int row, const int col, const double value) {
    addTripletDToList(triplets, row, col, value);
    if (row != col)
        addTripletDToList(triplets, col, row, value);
}

bool checkFileExtension(const std::string& filePath, const std::string& extension) {
    const std::string fpExt = filePath.substr(filePath.find_last_of('.') + 1);
    return (fpExt == extension);
}

void createDirectory(const std::string& dir) {
#if WIN32
    const std::string stemp = std::string(dir.begin(), dir.end());
    LPCSTR sw = stemp.c_str();
    if (CreateDirectory(sw, nullptr)) {
        // Directory created
    } else if (ERROR_ALREADY_EXISTS == GetLastError()) {
        // Directory already exists
    } else {
        // Something went wrong
        LENNY_LOG_WARNING("Directory `%s` could not be created!", dir.c_str());
    }
#else
    //Check if directory exists
    struct stat buffer;
    if (stat(dir.c_str(), &buffer) == 0)
        return;

    //If not, create directory
    const int dir_err = mkdir(dir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    if (dir_err == -1)
        LENNY_LOG_WARNING("Directory `%s` could not be created!", dir.c_str());
#endif
}

std::string getCurrentDateAndTime() {
    time_t now = time(nullptr);
    char name[80];
    struct tm tstruct = *localtime(&now);
    strftime(name, sizeof(name), "%Y-%m-%d_%X", &tstruct);
    std::string name_str = std::string(name);
    std::replace(name_str.begin(), name_str.end(), ':', '-');
    return name_str;
}

std::string browseFile() {
#if WIN32
    const int bufferSize = MAX_PATH;
    char currentDir[bufferSize];
    if (!GetCurrentDirectory(bufferSize, currentDir))
        LENNY_LOG_WARNING("Unable to GET current directory")

    OPENFILENAME ofn;
    char szFile[100];

    ZeroMemory(&ofn, sizeof(ofn));
    ofn.lStructSize = sizeof(ofn);
    ofn.hwndOwner = nullptr;
    ofn.lpstrFile = szFile;
    ofn.lpstrFile[0] = '\0';
    ofn.nMaxFile = sizeof(szFile);
    ofn.lpstrFilter = "All\0*.*\0Text\0*.TXT\0";
    ofn.nFilterIndex = 1;
    ofn.lpstrFileTitle = nullptr;
    ofn.nMaxFileTitle = 0;
    ofn.lpstrInitialDir = nullptr;
    ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST;
    GetOpenFileName(&ofn);

    if (!SetCurrentDirectory(currentDir))
        LENNY_LOG_WARNING("Unable to SET current directory")

    return std::string(szFile);
#else
    std::string openString = "zenity --file-selection --filename=";
    FILE* f = popen(openString.c_str(), "r");
    char file[1024];
    char* fg = fgets(file, 1024, f);
    std::string filePath = std::string(file);
    filePath.erase(std::remove(filePath.begin(), filePath.end(), '\n'), filePath.end());
    return filePath;
#endif
}

}  // namespace lenny::tools::utils