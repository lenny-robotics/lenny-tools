#include <cxxabi.h>
#include <lenny/tools/Json.h>

#include <iostream>

const char* demangle(const char* name) {
    char buf[1024];
    size_t size = 1024;
    int status;
    char* res = abi::__cxa_demangle(name, buf, &size, &status);
    return res;
}