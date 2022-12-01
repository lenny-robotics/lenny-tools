#include <lenny/tools/Json.h>

#if WIN32
std::string demangle(const std::string& name) {
    std::stringstream ss(name);
    std::istream_iterator<std::string> begin(ss), end;
    const auto list = std::vector<std::string>(begin, end);
    return list.back();
}
#else
#include <cxxabi.h>

std::string demangle(const std::string& name) {
    char buf[1024];
    size_t size = 1024;
    int status;
    char* res = abi::__cxa_demangle(name.c_str(), buf, &size, &status);
    return std::string(res);
}
#endif