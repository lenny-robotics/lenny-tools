#include <lenny/tools/Json.h>

#if WIN32
const char* demangle(const char* name) {
    return name;
}
#else
#include <cxxabi.h>

const char* demangle(const char* name) {
    char buf[1024];
    size_t size = 1024;
    int status;
    char* res = abi::__cxa_demangle(name, buf, &size, &status);
    return res;
}
#endif